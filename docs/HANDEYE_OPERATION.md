# RGB Hand-Eye Calibration Operation

当前流程用于 eye-in-hand：相机固定在 UR5 末端，ChArUco 板固定在工位上。

当前脚本已改为动态解析仓库根目录：优先使用 `git rev-parse --show-toplevel`，失败时再按脚本自身位置做相对路径回退。因此下面命令只要求先 `cd <your cloned repo>`，不再依赖 `/home/z/Apps-my` 这类硬编码绝对路径。

注意：如果 RGB 相机、相机支架、末端夹具或任何影响 `tool0 -> camera_color_optical_frame` 刚性关系的部件被拆卸重装，旧手眼外参立即失效，必须重新采集、求解和验证。2026-06-26 的结果已经因拆装重装失效，只能作为历史数值参考。

## 0. 环境检查

```bash
cd <your cloned repo>
./calibration/extrinsics/handeye/tools/handeye_check_env.sh
```

## 1. 启动相机

终端 1：

```bash
cd <your cloned repo>
./scripts/camera/start_dabai_camera.sh
```

终端 2 检查图像：

```bash
source /opt/ros/humble/setup.bash
source ./ros2_ws/install/setup.bash
export ROS_LOG_DIR="$PWD/.ros-log"
ros2 topic hz /camera/color/image_raw
```

应接近 `10 Hz`。

## 2. 启动 UR5

终端 3：

```bash
cd <your cloned repo>
./scripts/robot/start_ur5_driver.sh
```

在示教器上打开包含 External Control 节点的程序，并按 Play。

终端 4 检查状态：

```bash
cd <your cloned repo>
./scripts/robot/check_ur5_state.sh
```

如果 `tool0` 查不到，先确认实际末端 frame 名称。

## 3. 采集样本

终端 5：

```bash
cd <your cloned repo>
./calibration/extrinsics/handeye/scripts/start_handeye_retake_capture.sh
```

`start_handeye_retake_capture.sh` 会新建时间戳 session，并把 `calibration/extrinsics/handeye/sessions/latest` 指向本次新 session。重做标定时优先使用这个脚本，避免新旧样本混在 `handeye_samples` 目录里。

每次移动机械臂到一个新姿态后：

1. 停稳机械臂。
2. 确认 ChArUco 板完整、清晰、不要贴边。
3. 在采集终端按 Enter。
4. 看到 `OK sample_XXX` 后再移动到下一个姿态。

建议采集 `15-25` 组。姿态必须包含明显的多轴旋转，不要只做平移或始终正对标定板。

采集输出：

```text
calibration/extrinsics/handeye/sessions/<YYYYMMDD_HHMMSS>/samples.jsonl
calibration/extrinsics/handeye/sessions/<YYYYMMDD_HHMMSS>/sample_XXX_raw.png
calibration/extrinsics/handeye/sessions/<YYYYMMDD_HHMMSS>/sample_XXX_overlay.png
```

## 4. 求解外参

采集完成后：

```bash
cd <your cloned repo>
./calibration/extrinsics/handeye/scripts/solve_latest_handeye_retake.sh
```

输出：

```text
calibration/extrinsics/handeye/sessions/latest/handeye_result.yaml
calibration/extrinsics/handeye/sessions/latest/base_target_validation.csv
```

核心结果是：

```text
T_tool_camera
```

含义：`tool0 -> camera_color_optical_frame` 的外参。

## 5. 质量判断

`solve_latest_handeye_retake.sh` 会打印固定标定板一致性：

```text
translation_m_mean
translation_m_max
rotation_deg_mean
rotation_deg_max
```

粗略判断：

- `translation_m_mean < 0.01` 通常可继续验证。
- `translation_m_mean > 0.02` 通常说明样本姿态、图像检测、TF frame 或相机固定有问题。
- `rotation_deg_mean` 越小越好；若超过数度，应重新检查样本。

## 6. 在线验证

求解通过后，保持相机、UR5 driver 和固定 ChArUco 板不变，运行：

```bash
cd <your cloned repo>
./calibration/extrinsics/handeye/scripts/start_handeye_live_validate.sh
```

移动机械臂到几个不同姿态，观察固定板在 `base` 坐标系下的散布。建议目标：

- `scatter_mean < 5 mm`
- `scatter_max < 10 mm`
- `rot_mean < 1-2 deg`

## 7. 常见问题

如果采集显示 `ChArUco pose failed`：

- 标定板太远、太斜、反光或不完整。
- 图像模糊，机械臂未停稳。
- 标定板参数不一致。当前小板脚本参数是 `6x6`、`25 mm` square、`18 mm` marker、`DICT_6X6_1000`、`start_id=233`。

如果采集显示 `TF lookup failed`：

- UR driver 没启动成功。
- 示教器 External Control 没有 Play。
- `tool0` frame 名称不对。
- 可用 `ros2 run tf2_tools view_frames` 查实际 frame。
