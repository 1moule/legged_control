#!/usr/bin/env python3

import argparse
import json
import os
import re
import threading
import time

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule, mpc_observation
from std_msgs.msg import Bool, String


MODE_NAME_TO_NUMBER = {
    "FLY": 0,
    "RH": 1,
    "LH": 2,
    "LH_RH": 3,
    "RF": 4,
    "RF_RH": 5,
    "RF_LH": 6,
    "RF_LH_RH": 7,
    "LF": 8,
    "LF_RH": 9,
    "LF_LH": 10,
    "LF_LH_RH": 11,
    "LF_RF": 12,
    "LF_RF_RH": 13,
    "LF_RF_LH": 14,
    "STANCE": 15,
}


DEFAULT_SEQUENCES = [
    ("vx_sweep", [
        ("stance_zero", 0.0, 0.0, 0.0, 5.0),
        ("vx_pos_0.3", 0.3, 0.0, 0.0, 5.0),
        ("vx_pos_0.6", 0.6, 0.0, 0.0, 5.0),
        ("vx_pos_1.0", 1.0, 0.0, 0.0, 5.0),
        ("vx_neg_0.3", -0.3, 0.0, 0.0, 5.0),
        ("vx_neg_0.6", -0.6, 0.0, 0.0, 5.0),
        ("vx_neg_1.0", -1.0, 0.0, 0.0, 5.0),
    ]),
    ("vy_sweep", [
        ("vy_pos_0.3", 0.0, 0.3, 0.0, 5.0),
        ("vy_pos_0.6", 0.0, 0.6, 0.0, 5.0),
        ("vy_pos_1.0", 0.0, 1.0, 0.0, 5.0),
        ("vy_neg_0.3", 0.0, -0.3, 0.0, 5.0),
        ("vy_neg_0.6", 0.0, -0.6, 0.0, 5.0),
        ("vy_neg_1.0", 0.0, -1.0, 0.0, 5.0),
    ]),
    ("yaw_sweep", [
        ("yaw_pos_0.4", 0.0, 0.0, 0.4, 5.0),
        ("yaw_pos_0.7", 0.0, 0.0, 0.7, 5.0),
        ("yaw_pos_1.0", 0.0, 0.0, 1.0, 5.0),
        ("yaw_neg_0.4", 0.0, 0.0, -0.4, 5.0),
        ("yaw_neg_0.7", 0.0, 0.0, -0.7, 5.0),
        ("yaw_neg_1.0", 0.0, 0.0, -1.0, 5.0),
    ]),
    ("xy_diagonal", [
        ("x_pos_y_pos", 0.35, 0.20, 0.0, 5.0),
        ("x_pos_y_pos_fast", 0.85, 0.50, 0.0, 5.0),
        ("x_pos_y_neg", 0.35, -0.20, 0.0, 5.0),
        ("x_pos_y_neg_fast", 0.85, -0.50, 0.0, 5.0),
        ("x_neg_y_pos", -0.25, 0.18, 0.0, 5.0),
        ("x_neg_y_pos_fast", -0.85, 0.50, 0.0, 5.0),
        ("x_neg_y_neg", -0.25, -0.18, 0.0, 5.0),
        ("x_neg_y_neg_fast", -0.85, -0.50, 0.0, 5.0),
    ]),
    ("xy_yaw_mix", [
        ("x_pos_y_pos_yaw_pos", 0.30, 0.15, 0.45, 5.0),
        ("x_pos_y_pos_yaw_pos_fast", 0.70, 0.35, 1.00, 5.0),
        ("x_pos_y_neg_yaw_neg", 0.30, -0.15, -0.45, 5.0),
        ("x_pos_y_neg_yaw_neg_fast", 0.70, -0.35, -1.00, 5.0),
        ("x_neg_y_pos_yaw_neg", -0.20, 0.15, -0.35, 5.0),
        ("x_neg_y_pos_yaw_neg_fast", -0.70, 0.35, -1.00, 5.0),
        ("x_neg_y_neg_yaw_pos", -0.20, -0.15, 0.35, 5.0),
        ("x_neg_y_neg_yaw_pos_fast", -0.70, -0.35, 1.00, 5.0),
    ]),
]


def extract_block(text, name):
    match = re.search(r"(^|\n)\s*" + re.escape(name) + r"\s*\{", text)
    if not match:
        return None

    start = text.find("{", match.start())
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1:index]
    return None


def indexed_values(block):
    values = []
    for line in block.splitlines():
        line = line.split(";", 1)[0].strip()
        match = re.search(r"\[\s*\d+\s*\]\s+([A-Za-z0-9_.+-]+)", line)
        if match:
            values.append(match.group(1))
    return values


def load_gaits(gait_file):
    with open(gait_file, "r", encoding="utf-8") as handle:
        text = handle.read()

    list_block = extract_block(text, "list")
    if list_block is None:
        raise RuntimeError("Could not find gait list in {}".format(gait_file))

    gaits = {}
    for gait_name in indexed_values(list_block):
        gait_block = extract_block(text, gait_name)
        if gait_block is None:
            continue

        mode_block = extract_block(gait_block, "modeSequence")
        time_block = extract_block(gait_block, "switchingTimes")
        if mode_block is None or time_block is None:
            continue

        modes = []
        for mode_name in indexed_values(mode_block):
            if mode_name not in MODE_NAME_TO_NUMBER:
                raise RuntimeError("Unknown mode '{}' in gait '{}'".format(mode_name, gait_name))
            modes.append(MODE_NAME_TO_NUMBER[mode_name])

        gaits[gait_name] = ([float(value) for value in indexed_values(time_block)], modes)
    return gaits


def parse_segment(value):
    parts = value.split(":")
    if len(parts) == 4:
        name, vx, wz, duration = parts
        vy = 0.0
    elif len(parts) == 5:
        name, vx, vy, wz, duration = parts
    else:
        raise argparse.ArgumentTypeError("segment must be name:vx:vy:wz:duration or name:vx:wz:duration")

    try:
        return name, float(vx), float(vy), float(wz), float(duration)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("segment vx, vy, wz and duration must be numeric") from exc


class RampCommandPublisher:
    def __init__(self, rate_hz, linear_ramp_rate, yaw_ramp_rate):
        self._publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._rate_hz = rate_hz
        self._linear_ramp_rate = linear_ramp_rate
        self._yaw_ramp_rate = yaw_ramp_rate
        self._lock = threading.Lock()
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_wz = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True

    def start(self):
        self._thread.start()

    def stop(self):
        self.set_target(0.0, 0.0, 0.0, reset=True)
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.publish_once(0.0, 0.0, 0.0)

    def set_target(self, vx, vy, wz, reset=False):
        with self._lock:
            self._target_vx = vx
            self._target_vy = vy
            self._target_wz = wz
            if reset:
                self._vx = vx
                self._vy = vy
                self._wz = wz

    def publish_once(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._publisher.publish(msg)

    @staticmethod
    def _approach(current, target, delta):
        if current < target:
            return min(current + delta, target)
        if current > target:
            return max(current - delta, target)
        return current

    def _run(self):
        rate = rospy.Rate(self._rate_hz)
        linear_delta = self._linear_ramp_rate / self._rate_hz
        yaw_delta = self._yaw_ramp_rate / self._rate_hz
        while not rospy.is_shutdown() and not self._stop.is_set():
            with self._lock:
                self._vx = self._approach(self._vx, self._target_vx, linear_delta)
                self._vy = self._approach(self._vy, self._target_vy, linear_delta)
                self._wz = self._approach(self._wz, self._target_wz, yaw_delta)
                vx = self._vx
                vy = self._vy
                wz = self._wz
            self.publish_once(vx, vy, wz)
            rate.sleep()


class AutoAmpCollector:
    def __init__(self, args):
        self.args = args
        self.robot_name = args.robot_name
        self.gait_topic = self.robot_name + "_mpc_mode_schedule"
        self.observation_topic = self.robot_name + "_mpc_observation"
        self.last_observation_wall_time = 0.0

        self.command = RampCommandPublisher(args.cmd_rate, args.linear_ramp_rate, args.yaw_ramp_rate)
        self.gait_pub = rospy.Publisher(self.gait_topic, mode_schedule, queue_size=1, latch=True)
        self.amp_pub = rospy.Publisher("/amp/enable_logging", Bool, queue_size=1, latch=True)
        self.gait_name_pub = rospy.Publisher("/amp/gait_name", String, queue_size=1, latch=True)
        self.switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        self.observation_sub = rospy.Subscriber(self.observation_topic, mpc_observation, self._observation_callback)

        self.gait_file = args.gait_file or rospy.get_param("/gaitCommandFile")
        self.amp_log_dir = args.amp_log_dir or rospy.get_param("/amp_log_dir", "amp_data")
        self.amp_log_prefix = args.amp_log_prefix or rospy.get_param("/amp_log_prefix", "motion")
        self.gaits = load_gaits(self.gait_file)
        self.current_gait = None

        if args.gait not in self.gaits:
            raise RuntimeError("Gait '{}' was not found in {}".format(args.gait, self.gait_file))
        if args.static_gait not in self.gaits:
            raise RuntimeError("Static gait '{}' was not found in {}".format(args.static_gait, self.gait_file))

    def _observation_callback(self, _msg):
        self.last_observation_wall_time = time.monotonic()

    def wait_for_connections(self, publisher, name, timeout, required=False):
        start = time.monotonic()
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
            if time.monotonic() - start > timeout:
                message = "Timed out waiting for subscriber on {}".format(name)
                if required:
                    raise RuntimeError(message)
                rospy.logwarn(message)
                return 0
            rate.sleep()
        return publisher.get_num_connections()

    def wait_for_recent_observation(self, timeout, max_age=1.0):
        start = time.monotonic()
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            if self.last_observation_wall_time > 0.0 and time.monotonic() - self.last_observation_wall_time <= max_age:
                return
            if time.monotonic() - start > timeout:
                raise RuntimeError("Timed out waiting for recent {}".format(self.observation_topic))
            rate.sleep()

    def switch_on_controller(self):
        rospy.loginfo("Waiting for controller_manager/switch_controller ...")
        rospy.wait_for_service("/controller_manager/switch_controller", timeout=self.args.init_timeout)
        request = SwitchControllerRequest()
        request.start_controllers = ["controllers/joint_state_controller", "controllers/legged_controller"]
        request.stop_controllers = []
        request.strictness = SwitchControllerRequest.BEST_EFFORT
        request.start_asap = False
        request.timeout = 0.0
        response = self.switch_controller(request)
        if not response.ok:
            raise RuntimeError("switch_controller returned false")
        rospy.loginfo("Controller switch request accepted")

    def publish_gait(self, gait_name, duration=None):
        event_times, modes = self.gaits[gait_name]
        msg = mode_schedule()
        msg.eventTimes = event_times
        msg.modeSequence = modes
        self.command.set_target(0.0, 0.0, 0.0, reset=True)
        connections = self.wait_for_connections(self.gait_pub, self.gait_topic, self.args.connection_timeout, required=True)
        publish_duration = self.args.gait_publish_duration if duration is None else max(duration, self.args.gait_publish_duration)
        end_time = time.monotonic() + publish_duration
        rate = rospy.Rate(1.0 / self.args.gait_publish_period)
        while not rospy.is_shutdown() and time.monotonic() < end_time:
            self.gait_pub.publish(msg)
            rate.sleep()
        self.current_gait = gait_name
        self.gait_name_pub.publish(String(gait_name))
        rospy.loginfo(
            "Published gait '%s' on %s to %d subscriber(s), eventTimes=%s, modeSequence=%s",
            gait_name,
            rospy.resolve_name(self.gait_topic),
            connections,
            event_times,
            modes,
        )

    def gait_for_segment(self, vx, vy, wz):
        if abs(vx) <= self.args.static_velocity_epsilon and abs(vy) <= self.args.static_velocity_epsilon and abs(wz) <= self.args.static_velocity_epsilon:
            return self.args.static_gait
        return self.args.gait

    def ensure_gait_for_segment(self, gait_name):
        if self.current_gait != gait_name:
            self.publish_gait(gait_name)
            settle_time = self.args.static_gait_settle if gait_name == self.args.static_gait else self.args.gait_settle
            rospy.sleep(settle_time)
            self.wait_for_recent_observation(timeout=self.args.observation_timeout)

    def set_amp_recording(self, enabled):
        self.wait_for_connections(self.amp_pub, "/amp/enable_logging", self.args.connection_timeout, required=enabled)
        for _ in range(3):
            self.amp_pub.publish(Bool(enabled))
            rospy.sleep(0.1)

    def run_segment(self, sequence_index, segment_index, name, vx, vy, wz, duration):
        gait_name = self.gait_for_segment(vx, vy, wz)
        rospy.loginfo(
            "Sequence %02d segment %02d: %s, gait=%s, vx=%.2f, vy=%.2f, wz=%.2f, duration=%.1f",
            sequence_index,
            segment_index,
            name,
            gait_name,
            vx,
            vy,
            wz,
            duration,
        )
        self.ensure_gait_for_segment(gait_name)
        self.command.set_target(vx, vy, wz)
        rospy.sleep(self.args.segment_settle)
        self.wait_for_recent_observation(timeout=self.args.observation_timeout)

        wall_start = time.time()
        self.gait_name_pub.publish(String(gait_name))
        rospy.sleep(duration)
        wall_end = time.time()
        rospy.sleep(self.args.segment_pause)

        return {
            "sequence_index": sequence_index,
            "segment_index": segment_index,
            "name": name,
            "gait": gait_name,
            "vx": vx,
            "vy": vy,
            "wz": wz,
            "record_duration": duration,
            "wall_start": wall_start,
            "wall_end": wall_end,
        }

    def run_sequence(self, sequence_index, name, segments):
        expected_file = "{}_{:04d}.csv".format(self.amp_log_prefix, sequence_index)
        rospy.loginfo("Sequence %02d: %s, expected file=%s", sequence_index, name, expected_file)
        wall_start = time.time()
        self.set_amp_recording(True)
        recorded_segments = []
        try:
            for segment_index, segment in enumerate(segments):
                if rospy.is_shutdown():
                    break
                recorded_segments.append(self.run_segment(sequence_index, segment_index, *segment))
        finally:
            self.set_amp_recording(False)
            self.command.set_target(0.0, 0.0, 0.0)
            rospy.sleep(self.args.sequence_pause)

        return {
            "index": sequence_index,
            "name": name,
            "expected_file": expected_file,
            "wall_start": wall_start,
            "wall_end": time.time(),
            "record_duration": sum(float(segment["record_duration"]) for segment in recorded_segments),
            "segments": recorded_segments,
        }

    def write_manifest(self, sequences):
        manifest = {
            "robot_name": self.robot_name,
            "gait": self.args.gait,
            "static_gait": self.args.static_gait,
            "gait_file": self.gait_file,
            "amp_log_dir": self.amp_log_dir,
            "amp_log_prefix": self.amp_log_prefix,
            "files_are_sequences": True,
            "total_requested_record_seconds": sum(float(sequence["record_duration"]) for sequence in sequences),
            "sequences": sequences,
        }
        os.makedirs(os.path.dirname(os.path.abspath(self.args.manifest)), exist_ok=True)
        with open(self.args.manifest, "w", encoding="utf-8") as handle:
            json.dump(manifest, handle, indent=2, sort_keys=True)
            handle.write("\n")
        rospy.loginfo("Wrote manifest: %s", self.args.manifest)

    def run(self):
        sequences = [("custom_segments", self.args.segment)] if self.args.segment else DEFAULT_SEQUENCES
        recorded_sequences = []
        self.command.start()
        try:
            self.set_amp_recording(False)
            self.switch_on_controller()
            self.wait_for_recent_observation(timeout=self.args.init_timeout)
            if self.args.pre_record_trot_duration > 0.0:
                rospy.loginfo(
                    "Pre-record warmup: publish gait=%s, then hold it for %.1f s",
                    self.args.gait,
                    self.args.pre_record_trot_duration,
                )
                self.publish_gait(self.args.gait)
                self.wait_for_recent_observation(timeout=self.args.observation_timeout)
                rospy.sleep(self.args.pre_record_trot_duration)
            rospy.loginfo("Pre-record settle: gait=%s for %.1f s", self.args.static_gait, self.args.post_init_stance)
            self.publish_gait(self.args.static_gait)
            rospy.sleep(self.args.post_init_stance)

            for sequence_index, sequence in enumerate(sequences):
                if rospy.is_shutdown():
                    break
                recorded_sequences.append(self.run_sequence(sequence_index, *sequence))
                self.write_manifest(recorded_sequences)
        finally:
            self.set_amp_recording(False)
            self.command.stop()
            if recorded_sequences:
                self.write_manifest(recorded_sequences)


def main():
    parser = argparse.ArgumentParser(description="Automatically collect AMP data by publishing gait and velocity commands.")
    parser.add_argument("--robot-name", default="legged_robot")
    parser.add_argument("--gait", default="trot")
    parser.add_argument("--static-gait", default="stance")
    parser.add_argument("--static-velocity-epsilon", type=float, default=1e-4)
    parser.add_argument("--gait-file", default="")
    parser.add_argument("--amp-log-dir", default="")
    parser.add_argument("--amp-log-prefix", default="")
    parser.add_argument("--manifest", default=os.path.abspath("amp_data/auto_amp_manifest.json"))
    parser.add_argument("--segment", action="append", type=parse_segment,
                        help="Override default schedule. Format: name:vx:vy:wz:duration")
    parser.add_argument("--pre-record-trot-duration", type=float, default=1.0,
                        help="Switch to the dynamic gait for this many seconds before settling in stance and recording.")
    parser.add_argument("--post-init-stance", type=float, default=3.0)
    parser.add_argument("--gait-settle", type=float, default=3.0)
    parser.add_argument("--static-gait-settle", type=float, default=2.0)
    parser.add_argument("--segment-settle", type=float, default=1.0)
    parser.add_argument("--segment-pause", type=float, default=0.4)
    parser.add_argument("--sequence-pause", type=float, default=1.0)
    parser.add_argument("--cmd-rate", type=float, default=20.0)
    parser.add_argument("--linear-ramp-rate", type=float, default=0.5)
    parser.add_argument("--yaw-ramp-rate", type=float, default=0.8)
    parser.add_argument("--gait-publish-duration", type=float, default=1.0)
    parser.add_argument("--gait-publish-period", type=float, default=0.1)
    parser.add_argument("--connection-timeout", type=float, default=5.0)
    parser.add_argument("--observation-timeout", type=float, default=10.0)
    parser.add_argument("--init-timeout", type=float, default=60.0)
    args = parser.parse_args()

    rospy.init_node("auto_amp_data_collector")
    AutoAmpCollector(args).run()


if __name__ == "__main__":
    main()
