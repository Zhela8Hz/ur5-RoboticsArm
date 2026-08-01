# ur5-RoboticsArm
机械臂点胶__手眼标定

## 当前状态

- 当前工作目标：UR5 + Orbbec Gemini335 的相机内参、RTDE 位姿读取和手眼标定路径验证。
- 当前 RGB 相机分辨率：Gemini335 `1920x1080 @ 30 fps`，ROS 实测约 `14 Hz`。
- 当前 Gemini335 RGB 内参文件：`calibration/rgb_intrinsics/results/rgb_intrinsics_gemini335_1920x1080.yaml`。
- 当前默认 ChArUco 大板：`6x6 squares`、方格 `40 mm`、marker `30 mm`、`DICT_6X6_1000`、`start_id=233`。
- 当前默认 RGB 手眼外参：`calibration/extrinsics/handeye/handeye_tool0_camera_color_optical.yaml`。
- 当前已验证路径：Gemini335 图像采集、ChArUco 检测、UR ROS 2 joint/TF 读取、自动关节轨迹采集、Park 手眼求解和 live validation。

## 2026-08-01 动态修正量采样记录

- 现象：`+5 mm`（tool X、左乘）的虚拟误差实验曾输出 `[-2.622, +3.823, -1.383] mm`，偏离理论逆修正 `[-5, 0, 0] mm`。
- 原因：总修正量同时包含原始 `X0` 的物理残差与人为注入量；不能假设未注入时的基线修正为零。该次总修正加回注入的 `+5 mm X` 后为约 `[+2.378, +3.823, -1.383] mm`，表明它与基线残差叠加一致。
- 修改：保留动态运动采样的 `3 cm/s、3 deg/s` 门槛；删除会代数恒等恢复已知注入量的“仅注入修正量”。每第 4 个合格动态样本改为不参与拟合的独立 holdout，只有 holdout 误差下降、求解条件数合格且修正连续稳定后才显示可用。不会发布 TF、写入 X0 或驱动机械臂。
- 当前尚未完成：当前安装状态下的 RGB-D alignment validation、TCP、夹爪工具轴映射和静态目标 base 坐标误差验证。
- 当前总日志：`docs/HANDEYE_TOTAL_LOG.txt`。

注意：DaBai 相机时代的 RGB 手眼外参 session `calibration/extrinsics/handeye/sessions/20260630_195009` 仅作为历史结果保留，不再作为默认标定结果使用。

## 目录

- `scripts/`：相机和 UR5 启动/检查脚本。
- `calibration/rgb_intrinsics/`：RGB 相机内参标定脚本与结果。
- `calibration/depth_intrinsics/`：IR / 深度侧内参标定脚本、采集目录与结果目录。
- `calibration/extrinsics/handeye/`：手眼外参采集、求解、验证脚本和会话结果。
- `configs/robot/`：机器人相关配置。
- `docs/`：操作记录和说明文档。

## 自动手眼标定

UR5 + Gemini335 的自动采集脚本、24 位姿轨迹和默认外参位于
`calibration/extrinsics/handeye/`。完整的环境要求、非运动检查、真实机械臂采集、
离线求解和验证步骤见 [docs/AUTO_HANDEYE_CAPTURE.md](docs/AUTO_HANDEYE_CAPTURE.md)。

当前默认外参为 `calibration/extrinsics/handeye/handeye_tool0_camera_color_optical.yaml`，
由 24 样本 Park 方法求得，固定板位移一致性均值为 0.669 mm。自动采集会驱动真实 UR5，
必须先确认工位无碰撞并在示教器运行 External Control。

## 常用入口

以下命令默认从仓库根目录执行。

启动 Gemini335 RGB 相机：

```bash
./scripts/camera/start_dabai_camera.sh
```

启动 UR5 driver：

```bash
./scripts/robot/start_ur5_driver.sh
```

检查 UR5 状态：

```bash
./scripts/robot/check_ur5_state.sh
```

使用 RTDE 采集一帧手眼样本：

```bash
python3 calibration/extrinsics/handeye/tools/handeye_capture_rtde.py
```

重新采集 RGB 手眼外参：

```bash
./calibration/extrinsics/handeye/scripts/start_handeye_retake_capture.sh
```

求解最新手眼外参：

```bash
./calibration/extrinsics/handeye/scripts/solve_latest_handeye_retake.sh
```

在线验证最新手眼外参：

```bash
./calibration/extrinsics/handeye/scripts/start_handeye_live_validate.sh
```

打开实时视频偏差监控窗口：

```bash
./calibration/extrinsics/handeye/scripts/start_handeye_realtime_monitor.sh
```

带人为漂移打开实时监控窗口：

```bash
HANDEYE_DRIFT_X_M=0.005 \
./calibration/extrinsics/handeye/scripts/start_handeye_realtime_monitor.sh
```

模拟手眼漂移并记录 validation 样本：

```bash
HANDEYE_DRIFT_X_M=0.005 \
HANDEYE_LIVE_OUTPUT=/tmp/live_validation_drift_x5mm.jsonl \
./calibration/extrinsics/handeye/scripts/start_handeye_live_validate.sh
```

使用 Ceres 自动估计漂移修正量：

```bash
cmake -S calibration/extrinsics/handeye/tools -B /tmp/handeye_ceres_build
cmake --build /tmp/handeye_ceres_build
/tmp/handeye_ceres_build/optimize_handeye_drift_ceres \
  --samples /tmp/live_validation_drift_x5mm.jsonl \
  --handeye calibration/extrinsics/handeye/handeye_tool0_camera_color_optical.yaml \
  --applied-drift-xyz-m 0.005 0 0 \
  --applied-drift-rpy-deg 0 0 0
```

如果 CMake 报找不到 `CeresConfig.cmake`，需要先安装 Ceres 开发包；本项目不会自动执行 `sudo` 安装。

## 关键文档

- `docs/SETUP_GEMINI335.md`：从 GitHub 克隆后配置 Gemini335 + UR5 环境的说明。
- `docs/HANDEYE_TOTAL_LOG.txt`：当前 RGB 手眼标定总日志和有效结果说明。
- `docs/HANDEYE_20260707_UR_DRIVER_CHECK_LOG.md`：2026-07-07 UR driver、RTDE 和 Gemini335 验证的单独记录。
- `docs/HANDEYE_OPERATION.md`：RGB 手眼标定操作步骤。

## 分支建议

- `calibration_dabai`：保留 DaBai 相机时期的标定和记录。
- `calibration_gemini335`：保存当前 Gemini335 相机、RTDE 手眼采集路径和 2026-07-07 验证记录。
