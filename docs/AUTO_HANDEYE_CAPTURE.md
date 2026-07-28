# UR5 + Gemini 335 自动手眼标定

本目录提供一套可复现的眼在手（eye-in-hand）自动采集与离线求解流程。默认外参文件为
`calibration/extrinsics/handeye/handeye_tool0_camera_color_optical.yaml`；当前提交的结果由 24 个样本、Park 方法求得，固定板位移一致性均值为 0.669 mm。

`auto_handeye_capture.py` 使用已实测的 UR5 与 Gemini 335 接口：

- RGB：`/camera/color/image_raw`、`/camera/color/camera_info`
- 关节：`/joint_states`
- TF：`base -> tool0`
- 轨迹 Action：`/scaled_joint_trajectory_controller/follow_joint_trajectory`

## 克隆后的前置条件

1. 使用 ROS 2 Humble，并构建本仓库中的 `ros2_ws`。
2. 启动 Gemini 335，使 RGB 图像与 CameraInfo 出现在上面的 topic 名称。
3. 启动 UR ROS 2 driver；在 UR 示教器加载并运行 **External Control**。控制器
   `/scaled_joint_trajectory_controller` 必须是 `active`。
4. 使用与当前硬件相同的 ChArUco 板：`6x6`、方格 `40 mm`、marker `30 mm`、
   `DICT_6X6_1000`、起始 ID `233`。

每个终端先加载 ROS 基础环境；如果你把 UR driver、Orbbec driver 或其他依赖构建在 overlay workspace 中，也要加载对应的 `install/setup.bash`：

```bash
source /opt/ros/humble/setup.bash
# 若当前机器使用本地 overlay：
source /path/to/your_ws/install/setup.bash
```

## 非运动检查

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

python3 calibration/extrinsics/handeye/tools/auto_handeye_capture.py inspect
python3 calibration/extrinsics/handeye/tools/auto_handeye_capture.py board-check
python3 calibration/extrinsics/handeye/tools/auto_handeye_capture.py dry-run \
  --trajectory calibration/extrinsics/handeye/auto_handeye_trajectory_test3.yaml
```

`dry-run` 只检查已示教关节、图像/关节接口和轨迹 Action，不会发送目标。
当前项目没有可用的 UR5 MoveIt 配置，因此它不能提供碰撞规划结论。

也可单独确认控制器：

```bash
ros2 control list_controllers
```

## 自动采集（会移动真实机械臂）

确认工位无碰撞、相机和标定板安装牢固、示教器已运行 External Control 后，执行：

```bash
python3 calibration/extrinsics/handeye/tools/auto_handeye_capture.py execute \
  --trajectory calibration/extrinsics/handeye/auto_handeye_trajectory_test3.yaml \
  --execute --capture \
  --output-root calibration/extrinsics/handeye/sessions
```

程序会要求输入 `EXECUTE`。随后机械臂先运动到 `start_pose`，再依次执行 24 个已示教的关节空间位姿；
速度上限、到位容差和等待时间由 `auto_handeye_capture.yaml` 的 `safety` 段控制。轨迹没有经过 MoveIt 碰撞规划，
应保持在可随时按下急停或停止 External Control 的位置操作。

每次采集会创建一个新的 `sessions/auto_capture_YYYYMMDD_HHMMSS/` 目录；不要覆盖历史 session。

## 离线求解与验证

将下面的 `session_dir` 改为刚完成的 session。命令不会驱动机械臂：

```bash
session_dir=calibration/extrinsics/handeye/sessions/auto_capture_YYYYMMDD_HHMMSS
intrinsics=calibration/rgb_intrinsics/results/rgb_intrinsics_gemini335_1920x1080.yaml

python3 calibration/extrinsics/handeye/tools/prepare_auto_handeye_samples.py \
  --session "$session_dir" --intrinsics "$intrinsics"

python3 calibration/extrinsics/handeye/tools/handeye_solve.py \
  --samples "$session_dir/samples.jsonl" \
  --output "$session_dir/handeye_result_park.yaml" --method park

python3 calibration/extrinsics/handeye/tools/handeye_validate_samples.py \
  --samples "$session_dir/samples.jsonl" \
  --result "$session_dir/handeye_result_park.yaml" \
  --output "$session_dir/base_target_validation.csv"
```

确认结果满足项目精度要求后，将 `handeye_result_park.yaml` 中的内容复制到
`calibration/extrinsics/handeye/handeye_tool0_camera_color_optical.yaml`，并重启正在运行的验证或监控程序。

## 人工同步采集验证

```bash
python3 calibration/extrinsics/handeye/tools/auto_handeye_capture.py capture
```

每次按 Enter 时，程序保存当前图像及按图像 ROS 时间戳查询的 `base -> tool0` TF。
时间差超过 `auto_handeye_capture.yaml` 中的 `tf_max_delta_sec` 会拒绝保存。

## 安全边界

- `inspect`、`board-check`、`dry-run` 和离线求解不发送机器人运动；`execute --execute` 会发送真实关节轨迹。
- 自动轨迹执行必须使用单独的 `--execute` 入口，并在提示时输入 `EXECUTE`；首次实际运行前必须确认所有目标、速度、加速度、碰撞风险和停止方法。
- 遇到控制器错误、保护停止或急停时，不得自动恢复机器人。
