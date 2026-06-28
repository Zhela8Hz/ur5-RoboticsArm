# RGB Hand-Eye Calibration Operation

当前流程用于 eye-in-hand：相机固定在 UR5 末端，ChArUco 板固定在工位上。

## 0. 环境检查

```bash
cd /home/z/Apps-my
./calibration/extrinsics/handeye/tools/handeye_check_env.sh
```

## 1. 启动相机

终端 1：

```bash
cd /home/z/Apps-my
./scripts/camera/start_dabai_camera.sh
```

终端 2 检查图像：

```bash
source /opt/ros/humble/setup.bash
source /home/z/Apps-my/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
ros2 topic hz /camera/color/image_raw
```

应接近 `10 Hz`。

## 2. 启动 UR5

终端 3：

```bash
cd /home/z/Apps-my
./scripts/robot/start_ur5_driver.sh
```

在示教器上打开包含 External Control 节点的程序，并按 Play。

终端 4 检查状态：

```bash
cd /home/z/Apps-my
./scripts/robot/check_ur5_state.sh
```

如果 `tool0` 查不到，先确认实际末端 frame 名称。

## 3. 采集样本

终端 5：

```bash
cd /home/z/Apps-my
./calibration/extrinsics/handeye/scripts/start_handeye_capture.sh
```

每次移动机械臂到一个新姿态后：

1. 停稳机械臂。
2. 确认 ChArUco 板完整、清晰、不要贴边。
3. 在采集终端按 Enter。
4. 看到 `OK sample_XXX` 后再移动到下一个姿态。

建议采集 `15-25` 组。姿态必须包含明显的多轴旋转，不要只做平移或始终正对标定板。

采集输出：

```text
/home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/samples.jsonl
/home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/sample_XXX_raw.png
/home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/sample_XXX_overlay.png
```

## 4. 求解外参

采集完成后：

```bash
cd /home/z/Apps-my
./calibration/extrinsics/handeye/scripts/solve_handeye.sh
```

输出：

```text
/home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/handeye_result.yaml
```

核心结果是：

```text
T_tool_camera
```

含义：`tool0 -> camera_color_optical_frame` 的外参。

## 5. 质量判断

`solve_handeye.sh` 会打印固定标定板一致性：

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

## 6. 常见问题

如果采集显示 `ChArUco pose failed`：

- 标定板太远、太斜、反光或不完整。
- 图像模糊，机械臂未停稳。
- 标定板参数不一致。当前脚本参数是 `6x6`、`25 mm` square、`18 mm` marker、`DICT_6X6_1000`、`start_id=233`。

如果采集显示 `TF lookup failed`：

- UR driver 没启动成功。
- 示教器 External Control 没有 Play。
- `tool0` frame 名称不对。
- 可用 `ros2 run tf2_tools view_frames` 查实际 frame。
