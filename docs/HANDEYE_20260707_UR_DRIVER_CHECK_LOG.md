# 2026-07-07 UR5 hand-eye path check log

## Goal

在相机固定件尚未打印完成前，先验证手眼标定软件路径是否能跑通，包括：

- UR5 网络连通；
- `ur_robot_driver` 启动链路；
- `/joint_states` 和 `base -> tool0` TF 前置条件；
- 后续 `handeye_capture.py` 所需的机器人侧数据是否可用。

本次没有执行真实机器人运动，没有执行 Git 写操作，没有修改机器人控制程序。

## Step 1: Network check

仓库脚本 `scripts/robot/start_ur5_driver.sh` 中配置：

```text
robot_ip:=192.168.1.10
reverse_ip:=192.168.1.20
kinematics_params_file:=configs/robot/ur5_actual_calibration.yaml
```

`ping 192.168.1.10` 成功：

```text
3 packets transmitted, 3 received, 0% packet loss
rtt min/avg/max/mdev = 0.131/0.162/0.182/0.022 ms
```

结论：电脑到 UR5 控制柜网络连通正常。

## Step 2: UR driver startup attempts

多次运行：

```bash
./scripts/robot/start_ur5_driver.sh
```

已确认正常部分：

```text
Connected: Universal Robots Dashboard Server
Successfully connected to Dashboard Server at 192.168.1.10. Robot has version 3.15.8.0
Negotiated RTDE protocol version to 2.
Received URControl version 3.15.8.0
Setting up RTDE communication with frequency 125.000000
```

失败原文：

```text
[ur_ros2_control_node-1] [FATAL] [URPositionHardwareInterface]: Could not get configuration package within timeout, are you connected to the robot?(Configured timeout: 1 sec)
[ur_ros2_control_node-1] [INFO] [resource_manager]: Failed to 'configure' hardware 'ur5'
[ur_ros2_control_node-1] terminate called after throwing an instance of 'std::runtime_error'
[ur_ros2_control_node-1]   what():  Failed to set the initial state of the component : ur5 to active
```

结论：Dashboard 和 RTDE 初步通信正常，但 `ur_ros2_control_node` 没能完成机器人 configuration package 握手，因此 controller manager 未启动成功，`/joint_states` 和真实 `base -> tool0` TF 尚不可用。

## Local network configuration

本机有线网卡：

```text
enp7s0: 192.168.1.20/24
```

到机器人路由：

```text
192.168.1.10 dev enp7s0 src 192.168.1.20
```

结论：`reverse_ip:=192.168.1.20` 与本机有线网卡和路由匹配。

## Dashboard read-only queries

使用 Dashboard TCP 端口 `29999` 做只读查询：

```bash
printf 'robotmode\n' | timeout 5 nc 192.168.1.10 29999
printf 'safetymode\n' | timeout 5 nc 192.168.1.10 29999
printf 'running\n' | timeout 5 nc 192.168.1.10 29999
printf 'get loaded program\n' | timeout 5 nc 192.168.1.10 29999
printf 'programState\n' | timeout 5 nc 192.168.1.10 29999
```

查询结果：

```text
Robotmode: RUNNING
Safetymode: NORMAL
Program running: false
No program loaded
STOPPED null
URSoftware 3.15.8.106339
```

结论：机器人已上电且安全状态正常，但示教器当前没有加载并运行包含 `External Control` 节点的 UR 程序。

## Current diagnosis

当前 `ur_robot_driver` 失败的主要原因不是：

- 机器人 IP 不通；
- 本机 `reverse_ip` 配置错误；
- Dashboard 完全不可达；
- `EtherNet/IP` 或 `PROFINET IO` 适配器禁用。

更可能的原因是：

- 示教器没有加载 `External Control` 程序；
- 程序没有运行到 `External Control` 节点；
- `External Control` URCap 未安装或未启用；
- `External Control` 节点中的 Host IP 未正确指向 `192.168.1.20`。

## Impact on hand-eye path check

会影响：

- `/joint_states`；
- `base -> tool0` TF；
- `handeye_capture.py` 中的机器人位姿采集；
- 完整手眼标定路径验证。

不会影响：

- 单独验证 Gemini335 相机启动；
- `/camera/color/image_raw`；
- `/camera/color/camera_info`；
- ChArUco 图像检测入口。

## Next steps

在示教器上完成以下操作后，再重试 UR driver：

1. 加载或创建包含 `External Control` 节点的 `.urp` 程序。
2. 确认 `External Control` Host IP 为 `192.168.1.20`。
3. 按 Play，使程序处于运行状态。
4. 确认没有 popup、pause、protective stop、safeguard stop 或 emergency stop。
5. 重新运行：

```bash
cd /home/z/Apps-my
./scripts/robot/start_ur5_driver.sh
```

如果成功，应继续检查：

```bash
cd /home/z/Apps-my
./scripts/robot/check_ur5_state.sh
```
