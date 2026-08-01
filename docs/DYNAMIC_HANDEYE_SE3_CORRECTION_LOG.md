# 动态手眼 SE(3) 修正程序说明日志

- 新程序：`calibration/extrinsics/handeye/tools/dynamic_handeye_se3_correction.py`
- 核心模块：`calibration/extrinsics/handeye/tools/dynamic_handeye_se3_core.py`
- 配置：`calibration/extrinsics/handeye/dynamic_handeye_se3_correction.yaml`
- 原动态程序快照：`calibration/extrinsics/handeye/tools/handeye_ceres_correction_monitor.pre_dynamic_backup.py`
- 原程序未被覆盖或修改；原始手眼结果 `X0` 只读加载。

实现采用 `X_current = DeltaX_tool × X0`。`DeltaX_tool` 为 tool 坐标系左乘修正；固定 ChArUco 板通过各帧 `T_base_tool × X_current × T_camera_board` 的一致性估计。

离线自测试已运行：无漂移、Tx/Ty/Tz +5 mm、Rx/Ry/Rz +1 deg 均通过。完整报告位于 `calibration/extrinsics/handeye/dynamic_handeye_se3_self_test_report.yaml`。

## 2026-08-01：显示界面导致桌面卡死的排查与修复

### 现象与现场证据

- 最后一次会话目录为 `calibration/extrinsics/handeye/sessions/dynamic_se3_20260801_104826/`，本地文件时间为 `2026-08-01 18:48:26 +0800`。
- 该会话已完成 `base -> tool0` TF 启动检查并创建会话文件，但 `run.log` 与 `frames.jsonl` 均为 `0` 字节；没有记录到已处理图像帧或优化结果。
- 在此之前没有执行 `auto_handeye_capture.py execute`、ROS trajectory action、关节目标或其他机器人运动命令；故卡死与机械臂实际运动无关。
- 上一次启动会话的系统日志中，GNOME Shell 在 `18:40` 至 `18:52` 间记录了 `876` 条 `Can't update stage views actor ... because it needs an allocation.`。
- 同一时间段 X11 输入服务记录 `SYN_DROPPED` 和 `your system is too slow`，表示桌面事件处理已经严重滞后并发生输入丢失。
- 没有发现 OOM、内核 lockup 或 NVIDIA Xid 记录。重启后的检查中，NVIDIA 内核模块已加载，但 `nvidia-smi` 返回 `NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver.`；这说明图形驱动/X11 链路仍需后续单独观察。

### 原因判断

原程序在 OpenCV GUI 循环中持续把 `1920x1080` 相机图像缩放到大画布并调用 `imshow`。该负载与当时已经异常的 GNOME/X11/NVIDIA 显示链路叠加，导致桌面和输入服务失去响应。此判断只针对显示链路；`+5 mm` tool-frame 左乘注入、TF 检查和优化计算没有证据表明造成系统卡死。

### 已实施修改

- 完全移除 `cv2.namedWindow`、`cv2.imshow`、`cv2.waitKey` 和大画布渲染；运行程序不再创建图形化界面。
- 运行状态改为纯终端输出，默认 `2 Hz`，保留原界面中的：`Error injection`、`Current correction`、`Valid samples`、处理帧数和最新帧替换计数。
- 新增 `Current ChArUco corners`，显示当前实时检测到的 ChArUco 角点数量；同时输出 `Current frame status` 说明接受或拒绝原因。
- 保持 `frames.jsonl`、优化结果 YAML，以及可选原始/标注图像保存行为不变。
- 会话创建后立即向 `run.log` 写入启动参数；每处理 30 帧写入一次处理数、有效样本数、丢帧数和最新拒绝原因，避免异常关机后没有运行线索。
- 配置项由 `live.max_display_rate_hz` 改为 `live.max_status_rate_hz`；默认值为 `2.0`。

### 验证结果

- Python 语法检查通过。
- 两份 YAML 配置解析通过。
- 使用 `dynamic_handeye_se3_xplus5mm_experiment.yaml` 的离线自测通过，输出为 `all_passed: True`。
- 未运行真实设备、未执行机器人运动、未使用 sudo、未执行 Git 写操作。

### 后续建议

先在纯文字模式下运行，确认相机、TF 和数据处理稳定；如果仍发生整机卡死，再针对 NVIDIA 驱动与 X11/GNOME 链路进行系统级排查。该系统级排查不应通过本程序修改或自动执行 sudo 操作。

### TF 启动检查后续修正

后续现场验证已确认 `/joint_states` 正常，且 `tf2_echo base tool0` 能连续返回有效变换。新程序最初的 TF 预检却仍超时，原因是它手工订阅 `/tf` 并只筛选一条直接的 `base -> tool0` 消息；UR5 的 TF 可以由多个关节链变换组成，`tf2_echo` 会正确解析该链，但手工筛选可能得不到匹配消息。

修正后，新程序与此前成功运行的 `handeye_capture.py`、`handeye_live_validate.py` 和 Ceres monitor 一致：使用 `tf2_ros.Buffer()` 和 `TransformListener()`，启动时以 `lookup_transform(base, tool0, Time())` 检查链路，处理图像时以图像 ROS 时间戳调用 `lookup_transform(base, tool0, image_stamp)`。不会以最新 TF 替代图像时间戳查询，也不会发送任何机器人命令。

## 2026-08-01：位姿 2 到 5 脚本的 Ctrl-C 停止修复

现场执行 `run_handeye_poses_1_to_5.py --execute` 时，终端 `Ctrl-C` 退出后，已被控制器接受的 `FollowJointTrajectory` 仍可能继续执行；重新启动 External Control 后也可能恢复该目标。现场已通过示教器急停和停止 External Control 将机械臂停住，之后 UR 驱动未继续运行。

原因是原脚本没有在退出前可靠取消控制器端 action；此外 `rclpy.init()` 的默认 SIGINT 处理可能先关闭 ROS context，使 Python 清理逻辑无法再发送取消请求。

修复内容位于 `calibration/extrinsics/handeye/tools/run_handeye_poses_1_to_5.py`：

- 以 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` 禁用 rclpy 的默认 SIGINT 接管；
- Python 捕获 `Ctrl-C` 后，在 ROS context 仍有效时调用 `cancel_goal_async()`；
- 只有 controller 接受取消且 action result 返回终态，才销毁 action client；
- 若 3 秒内未确认停止，脚本保留客户端并要求用户先在示教器停止/急停，再按 Enter 重试取消；
- 因此 `Ctrl-C` 已不再被视为物理紧急停止的替代品，示教器停止/急停仍是最高优先级。

已完成 ROS API 参数与 Python 语法检查；未在真实设备上测试此停止逻辑，也没有发送新的机器人动作。

## 2026-08-01：位姿运动与低速判定阈值同步

用户指定 TCP 线速度限制为 `3 cm/s`（`0.030 m/s`）和角速度限制为 `3 deg/s`。`run_handeye_poses_1_to_5.py` 的名义 TCP 时长计算已使用这两个数值；动态修正程序的默认配置与 `xplus5mm` 实验配置也将 `motion.max_tcp_linear_speed_m_s` 和 `motion.max_tcp_angular_speed_deg_s` 同步为相同值。关节速度与加速度限制仍为 `0.05 rad/s`、`0.05 rad/s²`，实际每段时长取全部关节与 TCP 限制中的最大值。

## 2026-08-01：动态修正有效样本门槛调整

最新 `dynamic_se3_20260801_123825` session 的前 40 帧均未成为有效样本。统计显示：24 帧因 `insufficient or non-unique ChArUco corners` 被拒绝，其中 16 帧检测到 22 个角点；而已完成图像质量计算的帧重投影 RMS 为约 `0.25–0.37 px`、清晰度为约 `450–558`、边缘距离为约 `115–291 px`，均满足其他质量门槛。历史成功手眼工具使用 20 个角点作为最低门槛。

因此，默认配置和 `xplus5mm` 实验配置的 `quality.min_charuco_corners` 已从 `23` 放宽为 `20`。TF 时间戳查询失败没有被放宽：另有 16 帧被拒绝的原因是图像时间戳比最新 `base -> tool0` TF 超前约 1–5 秒；在机械臂运动期间用旧 TF 代替该图像时间戳会使手眼修正失真，因此仍需保持严格拒绝并确保 UR TF 流持续发布。

后续文字输出显示同类 TF 超前错误持续出现。根因确认在本程序：主循环曾按 `max_status_rate_hz=2` 的节奏休眠，每轮只处理一次 ROS 回调，导致 tf2 buffer 消费 `/tf` 的速度远低于 UR TF 发布频率，最新可用 TF 落后图像数秒。已新增独立的 `live.max_ros_spin_rate_hz=100`；图像处理仍为 `5 Hz`、文字输出仍为 `2 Hz`，但 ROS 回调不再被终端输出节流。图像时间戳 TF 查询规则保持不变。

后续 session 的 `sync_time_error_sec` 已恢复为 `0.0`，但有效样本仍为零。拒绝统计显示，静止时的微小 TF 抖动被 `motion direction discontinuity` 误判为运动方向反转；因此新增 `motion.direction_change_min_translation_m=0.0005`，位移小于 `0.5 mm` 时不再比较方向。`near duplicate pose` 保持不变，避免在静止姿态重复计样本。实际角速度曾达到约 `4.93 deg/s`，超过用户指定的 `3 deg/s`，该 `TCP speed limit` 拒绝保留不放宽。位姿 2–5 只有 4 个不同姿态，故 translation-only 优化的 `min_samples` 与 `update_every_valid_samples` 同步由 5 调整为 4；雅可比秩、条件数和修正幅度保护仍保留。

## 2026-08-01：动态运动采样的修正量解释与输出改进

用户确认目标是**机械臂运动状态下采样**，因此不应以“连续静止”筛掉动态帧；此前临时加入的静止采样门槛已撤销。两个动态修正配置恢复为与运动脚本一致的 `3 cm/s、3 deg/s` 动态门槛，图像仍严格按其 ROS 时间戳查询 TF。

`dynamic_se3_20260801_131320` 在 24 个有效样本时输出总修正 `[-2.622, +3.823, -1.383] mm`。该结果的秩满足 translation-only 要求，条件数约 `42.6`，不是求解器不可解。它不能直接与 `[-5, 0, 0] mm` 比较，因为总修正同时包含原始手眼外参 `X0` 的物理残差。将 `+5 mm X` 注入加回该总修正，得到约 `[+2.378, +3.823, -1.383] mm` 的未注入基线残差；两者关系与“基线修正叠加虚拟注入逆修正”一致。

为直接验证注入效果，程序现在对同一批动态样本额外以原始 `X0` 求一次基线修正，并输出 `Injection-only correction (baseline-referenced)`：计算为 `inverse(DeltaX_baseline) × DeltaX_injected`。对本实验它应接近 `[-5, 0, 0] mm`，而不会被真实基线误差混淆。优化结果 YAML 也保存基线修正和该相对修正。未执行任何机器人运动、未发布 TF、未改写 X0；旧 session 保留不变。

## 2026-08-01：移除代数恒等式输出并增加独立留出验证

进一步审查确认，`Injection-only correction (baseline-referenced)` 不是独立测量结果。对同一批数据，若未注入解为 `DeltaX_baseline`、注入矩阵为 `E`，则 `DeltaX_baseline × inverse(E)` 会产生完全相同的板位姿预测；因此两次求解相除会天然恢复 `inverse(E)`。该输出容易被误认为相机从运动数据中独立识别出 `-5 mm`，现已删除。

程序改为确定性训练/留出拆分：每第 4 个质量合格的动态运动样本标记为 `holdout`，永不进入优化，其余样本标记为 `training`。候选修正仅由 training 样本求解，再固定候选修正和 training 求得的板位姿，在 holdout 样本上计算修正前后 RMS。只有同时满足以下条件，`Current correction` 才会显示为可用：至少 4 个 holdout 样本；训练条件数不超过 `500`；holdout 修正后 RMS 不超过 `1 mm`；修正后/修正前 RMS 比不超过 `0.8`；相邻候选变化不超过 `1 mm` 和 `0.2 deg`；连续 2 次更新稳定。

同时修正关键帧去重错误：原逻辑使用 `all(...)`，只会在新姿态同时接近全部历史样本时拒绝，导致重复轨迹区域被重复加权；现改为接近任一历史关键帧即拒绝。终端保留 `Valid samples`，并新增 `Training samples`、`Holdout samples`、holdout RMS 和验证状态。旧 session 不改写，新逻辑仅对重启后建立的新 session 生效。

使用 `dynamic_se3_20260801_133232` 离线回放时，24 个 training、7 个 holdout 的候选修正为约 `[-2.939, +5.245, +3.823] mm`，holdout RMS 约 `0.366 mm`、修正后/前比约 `0.126`、条件数约 `55.6`；但只完成一次稳定更新，因此按新规则仍显示 unavailable，避免过早报告尚未充分稳定的修正量。离线自测试与新增单元测试通过；未执行真实设备运动。

## 2026-08-01：现场运行中容易出现的误判和报错

本节记录 `2026-08-01 22:16 CST` 前后的现场排查结论，便于后续复现动态修正和手眼验证时快速判断问题。所有操作均为只读相机、TF、topic 和本地日志检查；未发送机器人运动命令，未使用 sudo，未执行 Git 写操作。

### 1. ROS2 topic 存在但收不到实际消息

现象：

```text
/camera/color/image_raw
/joint_states
/tf
```

`ros2 topic list` 可以看到 topic，但 `ros2 topic echo /camera/color/image_raw --once`、`ros2 topic echo /joint_states --once` 或 `ros2 topic hz ...` 超时没有输出；`ros2 node list` 也可能为空。

原因判断：

这不等于相机或机器人一定坏了。Codex 当前命令可能运行在沙箱/命名空间里，ROS graph 能看到 topic 名称，但订阅不到真实 DDS 数据。现场已验证：在沙箱内一次性读取图像失败，但使用外部只读权限运行相同 ROS2 订阅逻辑后可以成功读取图像和 TF。

处理方式：

- 先用普通终端确认 `ros2 topic echo /camera/color/image_raw --once` 是否能收到图像。
- 如果 Codex 沙箱内收不到，但普通终端能收到，应在明确授权后让 Codex 以沙箱外只读方式订阅 ROS2 数据。
- 该操作只读取图像和 TF，不会造成机器人运动；但仍应明确说明“不发送 trajectory/action/service/URScript”。

### 2. 图像 10 秒内未收到

英文报错原文：

```text
RuntimeError: No image received from /camera/color/image_raw within 10 s
```

中文解释：

脚本已经订阅 `/camera/color/image_raw`，但在超时时间内没有收到任何 `sensor_msgs/msg/Image`。常见原因包括：相机 launch 未真正发布图像、ROS2 QoS/域环境不一致、驱动卡住、或者 Codex 沙箱无法消费 DDS 数据。

处理方式：

- 检查 `ros2 topic info -v /camera/color/image_raw` 是否有 publisher。
- 检查 `ros2 topic hz /camera/color/image_raw` 是否真的有频率。
- 如果 topic 有 publisher 但 `hz` 没输出，优先怀疑发布端无数据或当前终端 DDS 环境/沙箱有问题。
- 不要把这个错误解释成手眼标定误差。

### 3. 刚启动就按图像时间戳查 TF，可能查到过去外推错误

英文报错原文：

```text
RuntimeError: TF lookup failed within 3.0 s: Lookup would require extrapolation into the past.  Requested time 1785593513.533048 but the earliest data is at time 1785593513.578007, when looking up transform from frame [tool0] to frame [base]
```

中文解释：

这个错误发生在 TF buffer 刚创建后立刻收到图像：图像时间戳比本进程 TF buffer 中最早缓存的 TF 还早约几十毫秒，因此 `lookup_transform(base, tool0, image_stamp)` 无法回查到那一刻。它通常不是机器人 TF 没发布，也不是标定错误。

处理方式：

- 启动节点后先预热 TF buffer，再开始接受图像样本。
- 动态修正程序应继续坚持“图像时间戳匹配 TF”，不要直接用最新 TF 替代运动中的图像时间戳。
- 如果只是刚启动第一帧失败，可以丢弃第一帧并等待后续帧。

### 4. 图像时间戳比最新 TF 更新，属于未来外推错误

英文报错原文示例：

```text
Current frame status: TF lookup failed: Lookup would require extrapolation into the future.  Requested time 1785588210.530505 but the latest data is at time 1785588204.514011, when looking up transform from frame [tool0] to frame [base]
```

中文解释：

图像时间戳比 TF buffer 里最新的 `base -> tool0` 时间还新。若差值达到数百毫秒到数秒，运动中强行用旧 TF 会把修正量算歪。

处理方式：

- 如果差值小于约 1 秒且持续变化，可显示为 `waiting for future TF`，等待 TF 到达。
- 如果长期卡住，检查 UR driver、`/joint_states`、`/tf` 频率和程序是否被终端输出节流。
- 之前已将动态修正程序的 ROS spin 和文字输出解耦：图像处理/终端输出可以低频，但 ROS 回调需要高频消费 TF。

### 5. 标定板挪动后，不要用旧固定板基准判断手眼误差

若 Charuco 板在不同日期或不同实验之间被移动、旋转或重新摆放过，则当前单帧反推的 `T_base_target` 不能直接与旧 session 的固定板平均位置比较。该差异主要反映“标定板实际摆放位置变了”，不是手眼标定误差。

真正可用于评估手眼标定一致性的，是在同一个固定板位置下、多个机械臂停止位姿反推出来的 `T_base_target` 散布。若只想看当前单帧图像质量，应查看 ChArUco pose 的重投影误差，而不是拿当前板位姿和旧基准相减。

处理方式：

- 单帧只能检查：是否能检测到 Charuco、是否能取得图像时间戳 TF、当前 ChArUco 重投影 RMS 是多少。
- 要看当前手眼标定误差，应固定 Charuco 板不动，在多个停止位姿采 15-25 帧，比较 `T_base_target` 的平移/旋转散布。
- 动态修正实验要验证 `+5 mm` 注入，则需要无注入动态基线 session 和注入动态 session 分开采集，再比较两个独立 session 的相对修正；不能用同一批样本自算基线。

## 2026-08-01：基线参与计算后的注入误差验证结论

今天的动态手眼修正实验确认了一个关键结论：**注入实验不能只看注入后的总修正量，必须把独立采集的无注入动态基线加入对比计算**。原因是总修正量同时包含真实系统动态残差和人为注入误差的反向修正；只有计算“注入 session 的修正量相对无注入基线 session 的变化量”，才能判断注入误差是否被检测出来。

这里的基线不是同一批数据里代数构造出来的 `Injection-only correction`。该输出此前已删除，因为同一批样本同时求无注入和注入结果会形成代数恒等式，不能作为独立证据。有效做法是：

1. 跑一轮 `--no-injection`，得到独立无注入动态基线；
2. 跑一轮注入配置，得到注入后的总修正；
3. 用 `inverse(DeltaX_no_injection) * DeltaX_injected` 或等价的平移差值比较相对变化。

本日关键数据如下：

- 无注入动态基线：`dynamic_se3_20260801_142459/results/correction_020.yaml`
  - 候选修正：`[+2.501, +4.093, -0.761] mm`
  - holdout RMS：`2.020 mm -> 0.247 mm`
  - 验证状态：`waiting for stable correction updates (1/2)`
- `tool X +5 mm` 注入：`dynamic_se3_20260801_134944/results/correction_020.yaml`
  - 总候选修正：`[-2.564, +4.017, -0.532] mm`
  - 相对无注入基线：`[-5.066, -0.076, +0.228] mm`
  - 结论：X 轴注入被检测到，方向和数值均接近预期 `[-5, 0, 0] mm`；但该 session 仍停在 `1/2` 稳定验证。
- `tool Y -5 mm` 注入：`dynamic_se3_20260801_144040/results/correction_020.yaml`
  - 总候选修正：`[+2.565, +8.504, -2.922] mm`
  - 相对无注入基线：`[+0.064, +4.411, -2.161] mm`
  - 结论：Y 方向符号正确，`Y -5 mm` 注入应由 `Y +5 mm` 修正抵消；现场数据检测到约 `+4.4 mm` 的 Y 变化，但 Z 方向混入约 `-2.2 mm`，因此该次结果还不够干净，也同样停在 `1/2` 稳定验证。

同时完成的简要修改：

- `dynamic_handeye_se3_correction.py` 增加 `--no-injection`，可在不改 YAML 的情况下跑无注入基线；
- 终端输出新增 `Candidate correction (not yet validated)`，即使未通过 `2/2` 稳定验证，也显示当前候选修正量；
- 注入配置已从 `tool X +5 mm` 改为 `tool Y -5 mm`，用于后续 Y 轴实验；
- 单帧图像质量检查改看 ChArUco 重投影误差；标定板挪动后，不再使用旧固定板位置差值判断手眼误差。

本日 Codex 未执行任何机器人运动命令，未使用 sudo，未执行 Git 写操作。真实机械臂运动均由用户自行在终端执行。
