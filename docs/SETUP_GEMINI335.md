# Gemini335 + UR5 环境配置说明

本文用于说明别人从 GitHub 克隆 `calibration_gemini335` 分支后，需要补齐哪些系统环境和硬件配置，才能运行当前仓库里的 Gemini335 相机、UR5 driver、RTDE 手眼采集脚本。

当前仓库保存的是项目脚本、机器人配置、标定结果和日志，不包含完整的 ROS2 workspace、Orbbec driver 源码或系统依赖安装包。

## 1. 基础环境

已验证环境：

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- UR5 机器人 IP：`192.168.1.10`
- 本机有线网卡 IP：`192.168.1.20`
- Orbbec Gemini335，USB3 连接

建议从仓库根目录执行脚本：

```bash
cd ~/Apps-my
git switch calibration_gemini335
```

如果仓库放在其他路径，脚本一般仍可工作，因为多数脚本会通过 `git rev-parse --show-toplevel` 或 `Path(__file__)` 动态定位项目根目录。

## 2. 需要安装的软件

需要提前准备：

- ROS2 Humble 基础环境
- `ur_robot_driver`
- `orbbec_camera`
- `cv_bridge`
- OpenCV Python 绑定，且需要 `cv2.aruco`
- `numpy`
- `PyYAML`
- `ur_rtde`

Python 侧最小依赖可参考：

```bash
python3 -m pip install --user ur_rtde numpy pyyaml
```

如果当前 Python 的 OpenCV 没有 `aruco` 模块，需要安装带 contrib 的 OpenCV。注意不要随意覆盖系统 ROS 正在使用的 OpenCV；优先确认当前环境：

```bash
python3 - <<'PY'
import cv2
print(cv2.__version__)
print(hasattr(cv2, "aruco"))
PY
```

## 3. ROS2 workspace 要求

当前仓库没有上传 `ros2_ws/`，但部分脚本默认依赖：

```text
ros2_ws/install/setup.bash
ros2_ws/src/charuco_camera_calibration/
```

如果要使用旧的 ROS2 ChArUco 内参采集脚本，需要准备对应 workspace 并完成构建。

当前 Gemini335 + RTDE 的单帧采集脚本主要依赖系统 ROS2 环境、`orbbec_camera`、`cv_bridge` 和 `ur_rtde`，不依赖 UR driver 发布 TF。

## 4. 硬件网络配置

UR5 机器人侧：

- Robot IP：`192.168.1.10`
- External Control / Host IP 应指向运行 ROS2 driver 的电脑 IP：`192.168.1.20`

电脑侧：

- 与 UR5 直连或同网段连接
- 本机有线网卡设置为 `192.168.1.20`
- 能 ping 通机器人：

```bash
ping 192.168.1.10
```

RTDE 只读验证：

```bash
python3 - <<'PY'
from rtde_receive import RTDEReceiveInterface
r = RTDEReceiveInterface("192.168.1.10")
print("connected:", r.isConnected())
print("tcp:", r.getActualTCPPose())
print("q:", r.getActualQ())
r.disconnect()
PY
```

## 5. 相机验证

启动 Gemini335 RGB：

```bash
./scripts/camera/start_dabai_camera.sh
```

另开终端检查 topic：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```

当前脚本默认参数：

- color topic：`/camera/color/image_raw`
- 分辨率：`1920x1080`
- FPS 参数：`30`
- format：`MJPG`

## 6. UR5 driver 验证

启动 UR5 driver：

```bash
./scripts/robot/start_ur5_driver.sh
```

检查状态：

```bash
./scripts/robot/check_ur5_state.sh
```

注意：当前 Gemini335 + RTDE 采集路径不强制依赖 UR driver 成功启动；只要 RTDE 可连接并且相机图像正常，就可以采集 `T_base_tool` 和 `T_camera_target` 样本。

## 7. Gemini335 + RTDE 单帧采集

确认：

- Gemini335 RGB topic 正常发布
- UR5 已上电且 RTDE 可读
- 当前 UR TCP offset 为 0，即 x/y/z/rx/ry/rz 全为 0
- ChArUco 板在相机画面内

运行：

```bash
python3 calibration/extrinsics/handeye/tools/handeye_capture_rtde.py
```

默认输出目录：

```text
calibration/extrinsics/handeye/sessions/handeye_rtde_probe
```

默认使用内参：

```text
calibration/rgb_intrinsics/results/rgb_intrinsics_gemini335_1920x1080.yaml
```

采集结果包括：

- `samples.jsonl`
- `sample_XXX_raw.png`
- `sample_XXX_overlay.png`
- `T_base_tool`
- `T_camera_target`
- RTDE TCP pose 和 joints

## 8. 当前分支里的已知限制

- Gemini335 相机固定件尚未完成，因此还没有正式多姿态手眼标定结果。
- DaBai 时代的 `handeye_result.yaml` 只能作为历史结果，不能直接用于 Gemini335。
- `scripts/camera/start_dabai_camera.sh` 名称仍保留旧命名，但当前内容已经改为 Gemini335 RGB 启动参数。
- `ros2_ws/` 没有纳入 GitHub，需要使用者自行准备。
- 历史 `samples.jsonl` 中可能记录了 `/home/z/Apps-my/...` 绝对图片路径，这是旧采集数据的记录，不影响当前脚本动态定位项目路径。

## 9. 推荐验证顺序

1. `git switch calibration_gemini335`
2. `python3 -m pip install --user ur_rtde numpy pyyaml`
3. 确认 `python3` 可 `import cv2`、`cv2.aruco`、`rclpy`、`cv_bridge`
4. `ping 192.168.1.10`
5. 用 RTDE 只读脚本确认 UR5 pose 可读
6. 启动 Gemini335 RGB 并检查 `/camera/color/image_raw`
7. 运行 `handeye_capture_rtde.py` 采集单帧
8. 固定件完成后，采集多姿态样本并运行手眼求解
