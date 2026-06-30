# ur5-RoboticsArm
机械臂点胶

## 当前状态

- 当前 RGB 相机分辨率：`640x360 @ 10 fps`。
- 当前使用的 ChArUco 小板：`6x6 squares`、方格 `25 mm`、marker `18 mm`、`DICT_6X6_1000`、`start_id=233`。
- 当前有效 RGB 手眼外参 session：`calibration/extrinsics/handeye/sessions/20260630_195009`。
- 当前有效外参文件：`calibration/extrinsics/handeye/sessions/latest/handeye_result.yaml`。
- 当前总日志：`docs/HANDEYE_TOTAL_LOG.txt`。

注意：`2026-06-26` 的手眼结果在 RGB 相机/末端拆装重装后已经失效，只保留为历史归档，不能用于当前机器人视觉操作。

## 目录

- `scripts/`：相机和 UR5 启动/检查脚本。
- `calibration/rgb_intrinsics/`：RGB 相机内参标定脚本与结果。
- `calibration/depth_intrinsics/`：IR / 深度侧内参标定脚本、采集目录与结果目录。
- `calibration/extrinsics/handeye/`：手眼外参采集、求解、验证脚本和会话结果。
- `configs/robot/`：机器人相关配置。
- `docs/`：操作记录和说明文档。

## 常用入口

启动 RGB 相机：

```bash
cd /home/z/Apps-my
./scripts/camera/start_dabai_camera.sh
```

启动 UR5 driver：

```bash
cd /home/z/Apps-my
./scripts/robot/start_ur5_driver.sh
```

检查 UR5 状态：

```bash
cd /home/z/Apps-my
./scripts/robot/check_ur5_state.sh
```

重新采集 RGB 手眼外参：

```bash
cd /home/z/Apps-my
./calibration/extrinsics/handeye/scripts/start_handeye_retake_capture.sh
```

求解最新手眼外参：

```bash
cd /home/z/Apps-my
./calibration/extrinsics/handeye/scripts/solve_latest_handeye_retake.sh
```

在线验证最新手眼外参：

```bash
cd /home/z/Apps-my
./calibration/extrinsics/handeye/scripts/start_handeye_live_validate.sh
```

## 关键文档

- `docs/HANDEYE_TOTAL_LOG.txt`：当前 RGB 手眼标定总日志和有效结果说明。
- `docs/HANDEYE_OPERATION.md`：RGB 手眼标定操作步骤。
