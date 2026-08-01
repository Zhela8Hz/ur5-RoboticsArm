# 动态手眼 SE(3) 修正

程序：`calibration/extrinsics/handeye/tools/dynamic_handeye_se3_correction.py`。
它是独立程序，不改写 `handeye_tool0_camera_color_optical.yaml`，不发布 TF，不发送任何机器人命令。

## 坐标系与矩阵顺序

```text
base -- T_base_tool --> tool -- X0=T_tool_camera --> camera -- T_camera_board --> board
```

固定板约束为：

```text
T_base_board(i) = T_base_tool(i) × DeltaX_tool × X0 × T_camera_board(i)
```

`DeltaX_tool` 在 `tool` 坐标系表达，且**左乘**：

```text
X_current = DeltaX_tool × X0
```

所有旋转计算使用旋转矩阵、单位四元数和 SO(3) 最短旋转向量；不使用欧拉角。四元数在计算时归一化并使用 `w >= 0` 的符号约定。

注入自测的语义是：以 `E_tool × X0` 作为被污染模型，而合成观测仍由可信 `X0` 产生；因此理论修正为 `inverse(E_tool)`，通过完整 SE(3) 矩阵比较，而非逐项取负。

## 运行

```bash
cd /home/z/Apps-my
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 calibration/extrinsics/handeye/tools/dynamic_handeye_se3_correction.py
```

运行前先执行离线自测试：

```bash
python3 calibration/extrinsics/handeye/tools/dynamic_handeye_se3_correction.py --self-test
python3 -m unittest handeye_pose_selection.tests.test_dynamic_handeye_se3_correction
```

程序不创建图形界面，所有运行状态直接输出到终端：`Error injection`、`Current correction`、`Valid samples`、处理帧数、丢帧数，以及当前实时检测到的 `Current ChArUco corners`。按 `Ctrl-C` 停止程序。

## 配置与数据

配置文件为 `calibration/extrinsics/handeye/dynamic_handeye_se3_correction.yaml`。其中包含话题、TF、内参、板参数、时间同步、运动/关键帧限制和优化参数。

默认大板为 `6×6`、方格 `40 mm`、marker `30 mm`、`DICT_6X6_1000`、起始 ID `233`，与原自动标定配置一致。当前环境若使用不同相机话题、帧名或板，请只修改该新配置文件。

每次真实运行在 `calibration/extrinsics/handeye/sessions/dynamic_se3_<UTC时间>/` 创建新 session，保存原始/可选标注图像、逐帧 JSONL、配置快照、日志和每 5 组有效数据的优化 YAML；不会覆盖原始 session 或 `X0`。

图像使用其 ROS 时间戳，在 `/tf` 历史中查找前后 `base→tool0`，并进行平移线性插值、四元数 SLERP。无时间包围时拒绝，绝不读取最新 TF。TF 包围误差 `≤5 ms` 理想，`≤10 ms` 可参与优化，`10–20 ms` 仅保存，`>20 ms` 拒绝。

## 完整矩阵示例

无噪声 `Tx=+5 mm` 左乘 tool-frame 注入：

```text
E_tool = [[1,0,0,+0.005], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
DeltaX_theory = inverse(E_tool)
              = [[1,0,0,-0.005], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
```

实际离线报告中该项估计平移误差为约 `5.10e-09 mm`，旋转误差约 `2.96e-10 deg`。

自测试还会将左乘/tool（实际运行约定）、右乘/camera、`tool→camera` 与 `camera→tool` 互逆、以及 `T × inverse(T)` 和 `inverse(T) × T` 写入报告，防止比较坐标系或矩阵顺序不一致。

## 重要限制

这是低速小幅运动采集的“修正建议器”。若优化达到 `20 mm` 或 `5 deg` 边界、雅可比秩不足、条件数过大或求解失败，界面显示“当前修正量：暂不可用”，但保留原始数据和上一次成功报告。默认只优化平移；确认单轴测试后，才将配置 `optimization.mode` 改为 `full_se3`。
