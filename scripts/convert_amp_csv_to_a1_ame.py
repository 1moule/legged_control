#!/usr/bin/env python3
"""Convert legged_control AMP CSV logs to LeggedGym-Ex A1 AME motion txt files.

The A1 AME task in LeggedGym-Ex reads JSON .txt files from:
  resources/reference_motion/unitree_a1/a1_motion/*.txt

Each frame layout is:
  root_pos(3), root_quat_xyzw(4), dof_pos(12), foot_pos_base(12),
  root_lin_vel_b(3), root_ang_vel_b(3), dof_vel(12), foot_vel_base(12)

The source CSV logger stores joints in OCS2 order:
  [LF, LH, RF, RH] x [HAA, HFE, KFE]

This script exports A1 AME order:
  dof_pos/dof_vel: [FR, FL, RR, RL] x [hip, thigh, calf]
  foot positions:  [FL, FR, RL, RR]
"""

import argparse
import csv
import glob
import json
import math
import os
from typing import Dict, Iterable, List, Tuple

import numpy as np


PROJECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_OUTPUT_DIR = os.path.join(PROJECT_DIR, "amp_dataset_a1_ame")

SOURCE_JOINT_ORDER = ("LF", "LH", "RF", "RH")
A1_DOF_ORDER = ("FR", "FL", "RR", "RL")
A1_FOOT_ORDER = ("FL", "FR", "RL", "RR")
LEG_ALIAS_TO_SOURCE = {
    "FL": "LF",
    "FR": "RF",
    "RL": "LH",
    "RR": "RH",
}
JOINTS_PER_LEG = 3

# A1 URDF dimensions used to estimate key body positions relative to base.
HIP_X = {"FL": 0.1805, "FR": 0.1805, "RL": -0.1805, "RR": -0.1805}
HIP_Y = {"FL": 0.0470, "FR": -0.0470, "RL": 0.0470, "RR": -0.0470}
THIGH_Y = {"FL": 0.0838, "FR": -0.0838, "RL": 0.0838, "RR": -0.0838}
THIGH_LENGTH = 0.2
CALF_LENGTH = 0.2


def source_joint_indices(target_leg_order: Iterable[str]) -> List[int]:
    indices = []
    for target_leg in target_leg_order:
        source_leg = LEG_ALIAS_TO_SOURCE[target_leg]
        start = SOURCE_JOINT_ORDER.index(source_leg) * JOINTS_PER_LEG
        indices.extend([start, start + 1, start + 2])
    return indices


DOF_INDICES = source_joint_indices(A1_DOF_ORDER)
FOOT_DOF_INDICES = {
    leg: source_joint_indices((leg,)) for leg in A1_FOOT_ORDER
}


def read_numeric_csv(path: str) -> Tuple[List[str], List[Dict[str, float]]]:
    rows = []
    with open(path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        header = list(reader.fieldnames or [])
        for raw_row in reader:
            row = {}
            for key, value in raw_row.items():
                if key == "gait_name" or value is None or value == "":
                    continue
                try:
                    row[key] = float(value)
                except ValueError:
                    continue
            rows.append(row)
    return header, rows


def require_columns(header: Iterable[str], columns: Iterable[str], csv_file: str) -> None:
    header_set = set(header)
    missing = [name for name in columns if name not in header_set]
    if missing:
        raise RuntimeError(f"{csv_file} is missing required columns: {', '.join(missing)}")


def take_columns(row: Dict[str, float], prefix: str, count: int) -> np.ndarray:
    return np.array([row[f"{prefix}{i}"] for i in range(count)], dtype=np.float32)


def quat_xyzw_from_ypr(yaw: float, pitch: float, roll: float) -> List[float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / norm, y / norm, z / norm, w / norm]


def rot_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float32)


def rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float32)


def foot_position_in_base(leg: str, joint_angles: np.ndarray) -> np.ndarray:
    haa, hfe, kfe = [float(value) for value in joint_angles]
    position = np.array([HIP_X[leg], HIP_Y[leg], 0.0], dtype=np.float32)
    rotation = rot_x(haa)
    position = position + rotation @ np.array([0.0, THIGH_Y[leg], 0.0], dtype=np.float32)
    rotation = rotation @ rot_y(hfe)
    position = position + rotation @ np.array([0.0, 0.0, -THIGH_LENGTH], dtype=np.float32)
    rotation = rotation @ rot_y(kfe)
    position = position + rotation @ np.array([0.0, 0.0, -CALF_LENGTH], dtype=np.float32)
    return position.astype(np.float32)


def build_motion_arrays(rows: List[Dict[str, float]], source_fps: float, target_fps: float) -> Dict[str, np.ndarray]:
    root_pos = []
    root_rot = []
    root_lin_vel = []
    root_ang_vel = []
    dof_pos = []
    dof_vel = []
    foot_pos = []

    for row in rows:
        source_q = take_columns(row, "q", 12)
        source_dq = take_columns(row, "dq", 12)
        dof_pos.append(source_q[DOF_INDICES])
        dof_vel.append(source_dq[DOF_INDICES])

        root_pos.append([row["base_px"], row["base_py"], row["base_pz"]])
        root_rot.append(quat_xyzw_from_ypr(row["base_yaw"], row["base_pitch"], row["base_roll"]))
        root_lin_vel.append([row["root_lin_vel_bx"], row["root_lin_vel_by"], row["root_lin_vel_bz"]])
        root_ang_vel.append([row["root_ang_vel_bx"], row["root_ang_vel_by"], row["root_ang_vel_bz"]])

        foot_frame = []
        for leg in A1_FOOT_ORDER:
            foot_frame.extend(foot_position_in_base(leg, source_q[FOOT_DOF_INDICES[leg]]))
        foot_pos.append(foot_frame)

    arrays = {
        "root_pos": np.asarray(root_pos, dtype=np.float32),
        "root_rot": np.asarray(root_rot, dtype=np.float32),
        "dof_pos": np.asarray(dof_pos, dtype=np.float32),
        "foot_pos": np.asarray(foot_pos, dtype=np.float32),
        "root_lin_vel": np.asarray(root_lin_vel, dtype=np.float32),
        "root_ang_vel": np.asarray(root_ang_vel, dtype=np.float32),
        "dof_vel": np.asarray(dof_vel, dtype=np.float32),
    }

    arrays["foot_vel"] = finite_difference(arrays["foot_pos"], source_fps)
    if abs(source_fps - target_fps) > 1e-6:
        arrays = resample_arrays(arrays, source_fps, target_fps)
    return arrays


def finite_difference(values: np.ndarray, fps: float) -> np.ndarray:
    if len(values) <= 1:
        return np.zeros_like(values)
    return np.gradient(values, 1.0 / fps, axis=0).astype(np.float32)


def resample_arrays(arrays: Dict[str, np.ndarray], source_fps: float, target_fps: float) -> Dict[str, np.ndarray]:
    n_source = len(next(iter(arrays.values())))
    n_target = max(2, int(round(n_source * target_fps / source_fps)))
    source_t = np.arange(n_source, dtype=np.float32) / source_fps
    target_t = np.arange(n_target, dtype=np.float32) / target_fps
    target_t = np.minimum(target_t, source_t[-1])

    resampled = {}
    for key, value in arrays.items():
        out = np.empty((n_target, value.shape[1]), dtype=np.float32)
        for dim in range(value.shape[1]):
            out[:, dim] = np.interp(target_t, source_t, value[:, dim])
        resampled[key] = out

    # Keep quaternions normalized after interpolation.
    quat = resampled["root_rot"]
    norm = np.linalg.norm(quat, axis=1, keepdims=True)
    norm[norm < 1e-8] = 1.0
    resampled["root_rot"] = quat / norm
    resampled["foot_vel"] = finite_difference(resampled["foot_pos"], target_fps)
    return resampled


def build_frames(arrays: Dict[str, np.ndarray]) -> List[List[float]]:
    stacked = np.concatenate(
        [
            arrays["root_pos"],
            arrays["root_rot"],
            arrays["dof_pos"],
            arrays["foot_pos"],
            arrays["root_lin_vel"],
            arrays["root_ang_vel"],
            arrays["dof_vel"],
            arrays["foot_vel"],
        ],
        axis=1,
    ).astype(np.float32)
    return [[round(float(value), 6) for value in frame] for frame in stacked]


def output_path_for(csv_file: str, output_dir: str, prefix: str) -> str:
    name = os.path.splitext(os.path.basename(csv_file))[0]
    return os.path.join(output_dir, f"{prefix}{name}.txt")


def write_motion_txt(path: str, frames: List[List[float]], target_fps: float, motion_weight: float) -> None:
    motion = {
        "LoopMode": "Wrap",
        "FrameDuration": round(1.0 / target_fps, 8),
        "EnableCycleOffsetPosition": True,
        "EnableCycleOffsetRotation": True,
        "MotionWeight": motion_weight,
        "Frames": frames,
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(motion, handle, indent=0)
        handle.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert legged_control AMP CSV logs to LeggedGym-Ex A1 AME .txt motions.")
    parser.add_argument("--input_dir", default=os.path.join(PROJECT_DIR, "amp_data"),
                        help="Directory containing AMP CSV logs.")
    parser.add_argument("--output_dir", default=DEFAULT_OUTPUT_DIR,
                        help="Output directory. Copy generated txt files to LeggedGym-Ex resources/reference_motion/unitree_a1/a1_motion.")
    parser.add_argument("--output_prefix", default="nmpc_",
                        help="Prefix for generated txt files.")
    parser.add_argument("--source_fps", type=float, default=50.0,
                        help="CSV logging frequency.")
    parser.add_argument("--target_fps", type=float, default=50.0,
                        help="Output motion frequency.")
    parser.add_argument("--min_sequence_length", type=int, default=50,
                        help="Skip CSV files shorter than this many frames.")
    parser.add_argument("--motion_weight", type=float, default=1.0,
                        help="MotionWeight field written to each txt file.")
    parser.add_argument("--overwrite", action="store_true",
                        help="Overwrite existing output txt files.")
    args = parser.parse_args()

    csv_files = sorted(glob.glob(os.path.join(args.input_dir, "*.csv")))
    if not csv_files:
        raise RuntimeError(f"No CSV files found in {args.input_dir}")

    required_columns = [
        "base_px", "base_py", "base_pz",
        "base_yaw", "base_pitch", "base_roll",
        "root_lin_vel_bx", "root_lin_vel_by", "root_lin_vel_bz",
        "root_ang_vel_bx", "root_ang_vel_by", "root_ang_vel_bz",
    ]
    required_columns += [f"q{i}" for i in range(12)]
    required_columns += [f"dq{i}" for i in range(12)]

    converted = []
    skipped = []
    for csv_file in csv_files:
        header, rows = read_numeric_csv(csv_file)
        if len(rows) < args.min_sequence_length:
            skipped.append((csv_file, f"too short: {len(rows)} frames"))
            continue
        require_columns(header, required_columns, csv_file)

        out_path = output_path_for(csv_file, args.output_dir, args.output_prefix)
        if os.path.exists(out_path) and not args.overwrite:
            skipped.append((csv_file, f"exists: {out_path}"))
            continue

        arrays = build_motion_arrays(rows, args.source_fps, args.target_fps)
        frames = build_frames(arrays)
        write_motion_txt(out_path, frames, args.target_fps, args.motion_weight)
        converted.append((csv_file, out_path, len(frames)))

    print(f"Converted {len(converted)} file(s).")
    for csv_file, out_path, frames in converted:
        print(f"  {os.path.basename(csv_file)} -> {out_path} ({frames} frames, 61 dims)")

    if skipped:
        print(f"Skipped {len(skipped)} file(s):")
        for csv_file, reason in skipped:
            print(f"  {os.path.basename(csv_file)}: {reason}")

    print("\nFor LeggedGym-Ex a1_ame, place generated txt files under:")
    print("  <LeggedGym-Ex>/resources/reference_motion/unitree_a1/a1_motion/")
    print("A1 AME default config globs:")
    print("  resources/reference_motion/unitree_a1/a1_motion/*.txt")


if __name__ == "__main__":
    main()
