# AMP 数据采集使用说明

本文档说明原版 `legged_control` 中新增的 AMP 数据采集功能如何使用。

## 行为说明

- 控制器启动后不会自动开始采集。
- 只有启用 AMP logger 后，发布 `/amp/enable_logging true` 才会写入数据。
- 自动采集脚本会主动打开录制、发送 gait 和速度指令，结束或 Ctrl-C 时会关闭录制并发送零速度。

## 重新编译

修改后需要重新编译 `legged_controllers`：

```bash
catkin build legged_controllers
source devel/setup.bash
```

如果你的工作区使用 `catkin_make`，也可以按原工作区方式编译。

## 自动采集

先启动仿真或硬件，例如仿真：

```bash
export ROBOT_TYPE=go1
roslaunch legged_unitree_description empty_world.launch
```

然后运行自动采集：

```bash
roslaunch legged_controllers auto_amp_data_collection.launch \
  robot_type:=go1 \
  amp_log_dir:=/home/guanlin/amp_proj/legged_control/amp_data/auto_run \
  amp_log_prefix:=motion
```

自动采集流程：

1. 加载 controller，并启用 AMP logger。
2. 自动 switch 启动 `controllers/legged_controller`。
3. 先发布 `trot` gait，并在确认 gait 话题已有控制器订阅者后持续发布一小段时间。
4. 保持 `trot` 1 秒，避免 `stance` 太快覆盖还没被 MPC 消费的 gait 命令。
5. 再进入 `stance`，默认稳定 3 秒。
6. 发布 `/amp/enable_logging true` 开始录制。
7. 自动发送 `/cmd_vel` 速度序列。
8. 结束后发布 `/amp/enable_logging false`，并发送零速度。

常用参数：

```bash
roslaunch legged_controllers auto_amp_data_collection.launch \
  robot_type:=go1 \
  gait:=trot \
  static_gait:=stance \
  pre_record_trot_duration:=1.0 \
  gait_publish_duration:=1.0 \
  amp_log_dir:=/tmp/amp_data \
  amp_log_prefix:=motion
```

## 默认速度序列

默认会采集以下动作，每段 5 秒：

- 静止站立：`stance_zero`
- 前后平移：`vx = +/-0.3, +/-0.6, +/-1.0 m/s`
- 左右平移：`vy = +/-0.3, +/-0.6, +/-1.0 m/s`
- 原地旋转：`wz = +/-0.4, +/-0.7, +/-1.0 rad/s`
- 斜向平移：慢速和快速组合，合成平移速度不超过 `1.0 m/s`
- 平移加旋转：慢速和快速组合，快速段旋转速度到 `1.0 rad/s`

如果只想采指定片段，可以直接运行脚本：

```bash
rosrun legged_controllers auto_amp_data_collector.py \
  --segment forward_slow:0.3:0.0:0.0:5.0 \
  --segment forward_fast:1.0:0.0:0.0:5.0
```

`--segment` 格式为：

```text
name:vx:vy:wz:duration
```

## 手动录制

如果你只想手动开关录制，先启动 controller 时打开 AMP logger：

```bash
roslaunch legged_controllers load_controller.launch \
  robot_type:=go1 \
  enable_amp_logging:=true \
  amp_log_dir:=/tmp/amp_data \
  amp_log_prefix:=manual
```

开始录制：

```bash
rostopic pub /amp/enable_logging std_msgs/Bool "data: true"
```

停止录制：

```bash
rostopic pub /amp/enable_logging std_msgs/Bool "data: false"
```

可选设置当前 gait 名称，写入 CSV 的 `gait_name` 字段：

```bash
rostopic pub /amp/gait_name std_msgs/String "data: 'trot'"
```

## 输出文件

CSV 会写入 `amp_log_dir`：

```text
motion_0000.csv
motion_0001.csv
...
collection_manifest.json
```

CSV 字段包括：

- `time`
- base 位置和姿态：`base_px, base_py, base_pz, base_yaw, base_pitch, base_roll`
- base body-frame 速度：`root_lin_vel_b*`, `root_ang_vel_b*`
- 关节位置：`q0 ... q11`
- 关节速度：`dq0 ... dq11`
- 足端接触：`contact0 ... contact3`
- `gait_name`

## 转换为 LeggedGym-Ex A1 AME 数据

新增脚本可以把本项目采集的 AMP CSV 转为 `LeggedGym-Ex` 中 `a1_ame` 可直接读取的 motion txt：

```bash
python3 scripts/convert_amp_csv_to_a1_ame.py \
  --input_dir /home/guanlin/amp_proj/legged_control/amp_data/auto_run \
  --output_dir /tmp/a1_ame_motion \
  --output_prefix nmpc_
```

默认输出目录是当前项目下：

```text
amp_dataset_a1_ame/
```

在训练机器上，把生成的 `nmpc_*.txt` 放到你的 LeggedGym-Ex 项目目录：

```text
<LeggedGym-Ex>/resources/reference_motion/unitree_a1/a1_motion/
```

因为 `a1_ame_config.py` 会读取：

```text
resources/reference_motion/unitree_a1/a1_motion/*.txt
```

所以放进去后，正常训练 `a1_ame` 时会自动加载这些新 motion。默认不会覆盖已有同名导出文件；如果需要覆盖，加：

```bash
--overwrite
```

转换后的每帧为 LeggedGym-Ex AMPLoader 使用的 61 维格式：

```text
root_pos(3), root_quat_xyzw(4), dof_pos(12), foot_pos_base(12),
root_lin_vel_b(3), root_ang_vel_b(3), dof_vel(12), foot_vel_base(12)
```

脚本会做两类适配：

- 关节顺序：从采集 CSV 的 `[LF, LH, RF, RH]` 转为 A1 AME 的 `[FR, FL, RR, RL]`
- 足端 key body：按 A1 的 `[FL_foot, FR_foot, RL_foot, RR_foot]` 输出，并用 A1 URDF 尺寸从关节角做简化 FK 估算足端相对 base 的位置和速度

训练命令示例：

```bash
cd /path/to/LeggedGym-Ex
python legged_gym/scripts/train.py --task a1_ame --headless
```

## 注意事项

- 自动采集 launch 不启动 Gazebo，只负责加载 controller 和运行采集脚本。
- 如果使用 `rosrun auto_amp_data_collector.py`，需要先用 `enable_amp_logging:=true` 启动 `load_controller.launch`。
- 自动采集 launch 会设置 `start_gait_keyboard:=false`，避免交互式 gait 命令节点和自动脚本同时发布 gait。
- 自动脚本现在会等待 `legged_robot_mpc_mode_schedule` 有订阅者；如果没有连到 `GaitReceiver`，会直接报错而不是继续录制错误数据。
- 运行时终端应看到类似 `Published gait 'trot' on /legged_robot_mpc_mode_schedule to 1 subscriber(s)` 的日志；控制器终端也应看到 `[GaitReceiver]: Setting new gait after time ...`。
- 如果机器人在 `1.0 m/s` 平移或 `1.0 rad/s` 旋转下不稳定，先用 `--segment` 自定义低速片段验证。
- 转换脚本不依赖固定的 LeggedGym-Ex 路径，适合在其他系统上运行；用 `--output_dir` 指定导出目录即可。
