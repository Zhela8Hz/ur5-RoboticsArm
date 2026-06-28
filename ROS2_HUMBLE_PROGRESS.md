# ROS 2 Humble 与 DaBai DCW 相机标定进度

最后更新：2026-06-22（Asia/Shanghai）  
状态：DaBai DCW 已完成真实设备验证；RGB 内参已标定完成，可进入基于 RGB 的机械臂手眼标定准备。IR 内参标定尚未完成，不阻塞 RGB 手眼标定。

## 2026-06-22 实施记录

### 相机驱动与硬件验证

- 设备已确认：Orbbec DaBai DCW，USB ID `2bc5:0659`，序列号 `CH2TC42007T`，固件 `RD2460`。
- 系统预装的 SDK v2 驱动不适配该型号；已在 `/home/z/Apps-my/ros2_ws` 构建官方 `OrbbecSDK_ROS2 v1.5.15` overlay。
- 该 overlay 使用 `OrbbecSDK v1.10.35`，与 OrbbecViewer 已验证可用的 SDK v1.10 同代。
- 真实设备启动验证通过：RGB、深度流可启动；当前 USB 链路为 USB 2.0。

### USB 2.0 降级运行配置（2026-06-23，已实测）

- 适用场景：设备协商为 USB 2.0（`480M`）时。不要假设 USB 3.x 可用；启动日志应显示 `usb connect type: USB2.0`。
- USB 2.0 下验证通过的最高 RGB-D 组合：RGB `1920×1080 @ 30 fps`、`MJPG`，深度 `1024×768 @ 10 fps`、`Y11`。禁用点云后连续运行约 23 秒，两个流均成功启动并正常停止，无传输错误。
- 推荐启动参数：

  ```bash
  color_width:=1920 color_height:=1080 color_fps:=30 color_format:=MJPG \
  depth_width:=1024 depth_height:=768 depth_fps:=10 depth_format:=Y11 \
  enable_point_cloud:=false
  ```

- 已验证不可用：深度 `1280×800 @ 10 fps`（`Y11`）在 USB 2.0 下被驱动拒绝，报错 `No matched video stream profile found`；这是 USB 2.0 profile 限制，不是线材故障。
- 不要把 `RGB888` 或 `BGRA` 的 `1920×1080 @ 30 fps` 作为 USB 2.0 线上传输格式；该数据量超过 USB 2.0 实际吞吐。必须使用 `MJPG`，再由主机端按需解码/转换。
- 常规标定配置仍可使用 RGB `640×360 @ 10 fps` + Depth `640×360 @ 10 fps`，带宽余量更大。

### RGB 内参标定（完成）

- 标定板：ChArUco，`6×6 squares`，方格 `25 mm`，marker `18 mm`，`DICT_6X6_1000`，`start_id=233`。
- 图像模式：RGB `640×360 @ 10 fps`。
- 有效采集：25 张。
- RMS 重投影误差：`0.2232757 px`，结果可用。
- 输出文件：`/home/z/Apps-my/rgb_intrinsics_640x360.yaml`。
- RGB 内参：`fx=349.3258006`，`fy=348.7786042`，`cx=320.1061644`，`cy=174.3310865`。

### IR / 深度侧状态

- 旧结果是 IR `640×480` 内参，不是深度图或 RGB；其 RMS 为 `1.0457 px`，仅可作粗略参考。
- 已为 IR 标定准备脚本，并支持 `start_id=233`、原始 IR PNG 与检测叠加图保存。
- IR 流 `640×480 @ 10 fps` 已成功发布。当前难点是标定板在工作距离内覆盖视野不足、以及 IR 散斑影响，导致大多数帧可插值 ChArUco 角点较少。
- 已将 IR 启动脚本设为 `enable_ldp:=false` 以降低深度投影器散斑；若需完成高质量 IR 内参，建议使用更大物理尺寸的同布局 ChArUco 板或外部均匀 IR 补光。
- IR/深度内参不阻塞以 RGB 图像为基础的手眼标定；后续若需 RGB-D 对齐、点云抓取或深度测量，再完成 IR 内参、RGB-IR 外参与深度尺度验证。

### 新 IR 标定板建议（0.5 m 及以上距离）

- 当前 `150 mm` 标定板在 `0.5 m` 处预计仅投影约 `108 px` 宽，`18 mm` marker 约 `13 px`，不利于受散斑影响的 IR 检测。
- 决定使用更大板：`6×6 squares`，方格 `50 mm`，marker `36 mm`，棋盘实际尺寸 `300×300 mm`，`DICT_6X6_1000`，`start_id=233`。
- 成品四周保留 `15–20 mm` 白边，整体约 `330–340 mm` 见方，贴在平整硬质背板上。
- 按旧 IR 焦距粗估，该板在 `0.5 m` 处投影约 `217 px` 宽，单个 marker 约 `26 px`；在 `0.7 m` 处约 `19 px`。相比旧板可显著提高检测稳定性。
- 使用新板进行 IR 内参标定时，将参数改为 `square_length_m=0.050`、`marker_length_m=0.036`；RGB 已完成的 `640×360` 内参不需要重做，手眼标定也可使用这张新板。

### 当前可用脚本

- RGB 相机：`/home/z/Apps-my/start_dabai_camera.sh`
- RGB 标定：`/home/z/Apps-my/start_rgb_calibration.sh`
- IR 相机：`/home/z/Apps-my/start_dabai_ir_camera.sh`
- IR 标定：`/home/z/Apps-my/start_ir_calibration.sh`
- 采集服务：`/home/z/Apps-my/capture_rgb_calibration.sh`
- 计算服务：`/home/z/Apps-my/finish_rgb_calibration.sh`

### 下一步

使用已完成的 RGB 内参进行机械臂手眼标定：对每个采样姿态同步保存 RGB 图像中 ChArUco 的 `T_camera_target` 与机械臂导出的 `T_base_tool`，采集约 15–25 组包含多轴旋转的有效配对数据，再求解手眼外参。

## 当前目标与边界

- 系统：Ubuntu 22.04.5 LTS（Jammy），bash，x86_64。
- ROS：仅 ROS 2 Humble Desktop，不安装 ROS 1 或其他 ROS 2 发行版。
- 当前范围：DaBai DCW 相机驱动、彩色相机内参、ChArUco 位姿估计的准备工作。
- 不配置 UR 驱动、导航、机械臂外部控制或任何真实机械臂运动。
- 不执行 `apt upgrade`。已发生的 4 个系统库更新是安装 Orbbec 驱动的依赖，已在用户明确同意后由 APT 完成。

## 已完成

| 类别 | 状态 |
| --- | --- |
| ROS 官方软件源 | `ros2-apt-source 1.2.0~jammy` 已安装，ROS Jammy 源可用 |
| ROS 运行环境 | `ros-humble-desktop 0.10.0-1jammy.20260423.142311` 已安装 |
| 构建与依赖工具 | `colcon`、`rosdep 0.26.0`、`vcs 0.3.0`、`cmake 3.22.1`、`pip3 22.0.2` 已安装 |
| ROS 基础验证 | `rclpy`、`tf2_ros` Python 导入成功；`rviz2` 包可发现 |
| rosdep | `sudo rosdep init` 已完成；`rosdep update` 尚未完成，因为 Codex 沙箱不能访问 GitHub |
| Orbbec | `ros-humble-orbbec-camera 2.7.6` 及其依赖已安装 |
| 视觉依赖 | `python3-opencv 4.5.4`、OpenCV `aruco`、`cv_bridge`、`image_transport` 已可用 |
| 工作空间 | `~/ros2_ws/src` 已创建 |
| 标定包 | `charuco_camera_calibration` 已创建、构建，并通过 `ros2 pkg` 与 `ros2 run` 启动验证 |

## 工作空间与标定包

工作空间：`~/ros2_ws`。本次创建的包：

```text
~/ros2_ws/src/charuco_camera_calibration
```

标定包可订阅 RGB 或 IR 图像：检测 ChArUco 角点、采集有效观测、计算内参并保存 YAML。它不含 UR 通信、机器人状态订阅或运动指令。

每个新终端必须先加载：

```bash
source ~/ros2_ws/setup_charuco_calibration.bash
```

该脚本会加载 `/opt/ros/humble` 和当前工作空间。`~/.bashrc` 尚未修改。

验证包是否可用：

```bash
ros2 pkg prefix charuco_camera_calibration
ros2 pkg executables charuco_camera_calibration
```

启动标定节点：

```bash
ros2 run charuco_camera_calibration charuco_intrinsics
```

节点服务：

```text
/capture
/calibrate
```

默认输出文件：

```text
~/.ros/camera_info/dabai_dcw_charuco.yaml
```

## 标定操作说明（已完成 RGB；IR 可选）

RGB 标定使用工作区中的 `start_dabai_camera.sh`、`start_rgb_calibration.sh`、`capture_rgb_calibration.sh` 和 `finish_rgb_calibration.sh`。IR 标定使用相应的 `start_dabai_ir_camera.sh` 与 `start_ir_calibration.sh`。实体 ChArUco 板参数必须与脚本参数一致：`6×6`、方格 `25 mm`、marker `18 mm`、`DICT_6X6_1000`、`start_id=233`。

## 后续手眼标定（暂不配置）

eye-in-hand 需要每个采样姿态的两份数据：

```text
T_base_tool       # 机械臂基座 -> TCP/末端，由机械臂侧提供
T_camera_target   # 相机 -> 固定 ChArUco 板，由相机侧计算
```

采集约 15–25 组、包含多轴旋转的有效配对数据后，求解 `T_tool_camera`。若机械臂负责人能导出 TCP 位姿 CSV，即可离线完成，不必由本工作空间控制机械臂。

eye-to-hand 则相机固定在工位、标定板固定在末端，求解相机相对基座的外参；需要重新采集数据，不能复用 eye-in-hand 结果。

## 恢复提示

重新启动 Codex 后，发送：

```text
读取 /home/z/Apps-my/ROS2_HUMBLE_PROGRESS.md，并继续其中的 ROS 2 Humble 配置任务。
```
