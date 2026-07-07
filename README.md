# ur5-RoboticsArm
机械臂点胶__手眼标定

## 当前状态

- 当前工作目标：UR5 + Orbbec Gemini335 的相机内参、RTDE 位姿读取和手眼标定路径验证。
- 当前 RGB 相机分辨率：Gemini335 `1920x1080 @ 30 fps`，ROS 实测约 `14 Hz`。
- 当前 Gemini335 RGB 内参文件：`calibration/rgb_intrinsics/results/rgb_intrinsics_gemini335_1920x1080.yaml`。
- 当前使用的 ChArUco 小板：`6x6 squares`、方格 `25 mm`、marker `18 mm`、`DICT_6X6_1000`、`start_id=233`。
- 当前已验证路径：Gemini335 图像采集、ChArUco 检测、`ur_rtde` 读取 UR5 TCP pose / joints、单帧 `T_base_tool` + `T_camera_target` 采集流程。
- 当前尚未完成：Gemini335 相机固定件安装后的正式多姿态手眼采集、求解和精度验证。
- 当前总日志：`docs/HANDEYE_TOTAL_LOG.txt`。

注意：DaBai 相机时代的有效 RGB 手眼外参 session 为 `calibration/extrinsics/handeye/sessions/20260630_195009`，外参文件为 `calibration/extrinsics/handeye/sessions/latest/handeye_result.yaml`。这些结果只对应当时的 DaBai 相机和末端安装状态，换成 Gemini335 后不能直接用于当前机器人视觉操作。

## 目录

- `scripts/`：相机和 UR5 启动/检查脚本。
- `calibration/rgb_intrinsics/`：RGB 相机内参标定脚本与结果。
- `calibration/depth_intrinsics/`：IR / 深度侧内参标定脚本、采集目录与结果目录。
- `calibration/extrinsics/handeye/`：手眼外参采集、求解、验证脚本和会话结果。
- `configs/robot/`：机器人相关配置。
- `docs/`：操作记录和说明文档。

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

## 关键文档

- `docs/HANDEYE_TOTAL_LOG.txt`：当前 RGB 手眼标定总日志和有效结果说明。
- `docs/HANDEYE_20260707_UR_DRIVER_CHECK_LOG.md`：2026-07-07 UR driver、RTDE 和 Gemini335 验证的单独记录。
- `docs/HANDEYE_OPERATION.md`：RGB 手眼标定操作步骤。

## 分支建议

- `calibration_dabai`：保留 DaBai 相机时期的标定和记录。
- `calibration_gemini335`：保存当前 Gemini335 相机、RTDE 手眼采集路径和 2026-07-07 验证记录。
