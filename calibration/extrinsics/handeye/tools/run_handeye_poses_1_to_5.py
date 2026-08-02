#!/usr/bin/env python3
"""从当前位姿低速移动到已示教的位姿 2，再依次运行到位姿 10。

默认只进行接口、关节顺序和当前位置检查。只有传入 ``--execute`` 才会向真实机器人发送轨迹。

该程序不使用位姿 1。它从当前机器人关节状态出发，依次发送位姿 2 到 10。
"""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_TRAJECTORY = PROJECT_ROOT / 'calibration/extrinsics/handeye/auto_handeye_trajectory_test3.yaml'
ACTION_NAME = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
JOINT_STATE_TOPIC = '/joint_states'
BASE_FRAME = 'base'
TOOL_FRAME = 'tool0'
FIRST_POSE_ID = 2
LAST_POSE_ID = 10

# 低速安全指标。零速、零加速度端点的五次时间标度轨迹峰值分别为
# 1.875*delta/T 和 (10/sqrt(3))*delta/T^2。
MAX_JOINT_SPEED_RAD_S = 0.05
MAX_JOINT_ACCELERATION_RAD_S2 = 0.05
MIN_SEGMENT_DURATION_SEC = 8.0
# 由记录的 base -> tool0 位姿计算的名义 TCP 速度上限。关节轨迹控制器
# 不提供严格的笛卡尔速度约束，因此它们用于保守地延长每一段时长。
MAX_TCP_LINEAR_SPEED_M_S = 0.030
MAX_TCP_ANGULAR_SPEED_RAD_S = math.radians(3.0)
GOAL_TOLERANCE_RAD = 0.02
ACTION_TIMEOUT_MARGIN_SEC = 5.0


class RobotState(Node):
    def __init__(self):
        super().__init__('run_handeye_poses_1_to_5')
        self.latest_joint_state = None
        self.active_goal = None
        self.active_result_future = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_joint_state, 10)

    def _on_joint_state(self, message):
        self.latest_joint_state = message

    def wait_for_joint_state(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while self.latest_joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint_state is None:
            raise RuntimeError(f'Timed out waiting for {JOINT_STATE_TOPIC}.')

    def cancel_active_goal(self):
        """Cancel a trajectory accepted by the controller before this client exits."""
        if self.active_goal is None:
            return True
        print('Cancelling active trajectory goal...', flush=True)
        try:
            future = self.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result() if future.done() else None
            if response is None or not response.goals_canceling:
                print('WARNING: trajectory cancellation request timed out; use the teach pendant stop/EMERGENCY STOP.', flush=True)
                return False
            print('Active trajectory cancellation accepted; waiting for controller stop.', flush=True)
            if self.active_result_future is not None:
                rclpy.spin_until_future_complete(self, self.active_result_future, timeout_sec=3.0)
                if not self.active_result_future.done():
                    print('WARNING: controller did not confirm stop within 3 s; use the teach pendant stop/EMERGENCY STOP.', flush=True)
                    return False
            print('Controller reported the active trajectory stopped.', flush=True)
            self.active_goal = None
            self.active_result_future = None
            return True
        except Exception as exc:
            print(f'WARNING: failed to request trajectory cancellation: {exc}; use the teach pendant stop/EMERGENCY STOP.', flush=True)
            return False


def confirm_active_goal_stopped(node):
    """Never discard a live action client while its trajectory is unconfirmed."""
    while not node.cancel_active_goal():
        try:
            input('Trajectory stop is unconfirmed. Stop External Control or press EMERGENCY STOP, then press Enter here to retry cancellation: ')
        except KeyboardInterrupt:
            print('\nStop still unconfirmed; the client remains active. Use the teach pendant stop/EMERGENCY STOP.', flush=True)


def load_poses(trajectory_path):
    with trajectory_path.open('r', encoding='utf-8') as stream:
        trajectory = yaml.safe_load(stream)
    waypoints = trajectory.get('waypoints')
    if not isinstance(waypoints, list):
        raise RuntimeError(f'{trajectory_path} has no waypoint list.')
    selected = {item.get('id'): item for item in waypoints if isinstance(item, dict)}
    pose_ids = range(FIRST_POSE_ID, LAST_POSE_ID + 1)
    missing = [pose_id for pose_id in pose_ids if pose_id not in selected]
    if missing:
        raise RuntimeError(f'{trajectory_path} is missing pose IDs: {missing}.')
    poses = [selected[pose_id] for pose_id in pose_ids]
    joint_names = poses[0].get('joint_names')
    if not isinstance(joint_names, list) or not joint_names:
        raise RuntimeError(f'Pose {FIRST_POSE_ID} has no valid joint_names.')
    for pose_id, pose in zip(pose_ids, poses):
        if pose.get('joint_names') != joint_names:
            raise RuntimeError(f'Pose {pose_id} joint_names differ from pose {FIRST_POSE_ID}.')
        positions = pose.get('joint_positions_rad')
        if not isinstance(positions, list) or len(positions) != len(joint_names):
            raise RuntimeError(f'Pose {pose_id} has invalid joint_positions_rad.')
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in positions):
            raise RuntimeError(f'Pose {pose_id} has non-finite joint positions.')
    return poses


def positions_in_order(joint_state, expected_names):
    received = {name: position for name, position in zip(joint_state.name, joint_state.position)}
    missing = [name for name in expected_names if name not in received]
    if missing:
        raise RuntimeError(f'Live JointState is missing joints: {missing}.')
    return [received[name] for name in expected_names]


def maximum_joint_error(actual, expected):
    return max(abs(value - target) for value, target in zip(actual, expected))


def target_transform(pose):
    transform = pose.get('T_base_tool')
    if not isinstance(transform, dict):
        raise RuntimeError(f"Pose {pose.get('id')} has no T_base_tool transform.")
    translation = transform.get('translation_m')
    quaternion = transform.get('quaternion_xyzw')
    if not isinstance(translation, dict) or not isinstance(quaternion, dict):
        raise RuntimeError(f"Pose {pose.get('id')} has an invalid T_base_tool transform.")
    position = [translation.get(axis) for axis in ('x', 'y', 'z')]
    orientation = [quaternion.get(axis) for axis in ('x', 'y', 'z', 'w')]
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in position + orientation):
        raise RuntimeError(f"Pose {pose.get('id')} has non-finite T_base_tool values.")
    return position, orientation


def live_transform(node, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    last_error = None
    while time.monotonic() < deadline:
        try:
            transform = node.tf_buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return [translation.x, translation.y, translation.z], [rotation.x, rotation.y, rotation.z, rotation.w]
        except TransformException as exc:
            last_error = exc
            rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f'Timed out waiting for TF {BASE_FRAME} -> {TOOL_FRAME}: {last_error}')


def cartesian_delta(current_position, current_orientation, target_position, target_orientation):
    linear_m = math.sqrt(sum((current - target) ** 2 for current, target in zip(current_position, target_position)))
    current_norm = math.sqrt(sum(value * value for value in current_orientation))
    target_norm = math.sqrt(sum(value * value for value in target_orientation))
    if current_norm == 0.0 or target_norm == 0.0:
        raise RuntimeError('Received a zero-norm tool orientation quaternion.')
    dot = abs(sum(current * target for current, target in zip(current_orientation, target_orientation)))
    angular_rad = 2.0 * math.acos(min(1.0, max(-1.0, dot / (current_norm * target_norm))))
    return linear_m, angular_rad


def duration_for_delta(max_delta, linear_m, angular_rad):
    """Return a duration satisfying the configured quintic-profile limits."""
    return max(
        MIN_SEGMENT_DURATION_SEC,
        1.875 * max_delta / MAX_JOINT_SPEED_RAD_S,
        math.sqrt((10.0 / math.sqrt(3.0)) * max_delta / MAX_JOINT_ACCELERATION_RAD_S2),
        linear_m / MAX_TCP_LINEAR_SPEED_M_S,
        angular_rad / MAX_TCP_ANGULAR_SPEED_RAD_S,
    )


def send_pose(node, client, pose, expected_names, timeout_sec):
    pose_id = pose['id']
    current = positions_in_order(node.latest_joint_state, expected_names)
    target = list(pose['joint_positions_rad'])
    if 'wrist_3_joint' in expected_names:
        wrist_3_index = expected_names.index('wrist_3_joint')
        target[wrist_3_index] += 2.0 * math.pi * round((current[wrist_3_index] - target[wrist_3_index]) / (2.0 * math.pi))
    max_delta = maximum_joint_error(current, target)
    # A controller may reject a degenerate trajectory whose target is already
    # the current joint state.  This occurs when a previous run stopped at one
    # of the taught poses; skipping it is both safer and semantically correct.
    if max_delta <= GOAL_TOLERANCE_RAD:
        print(f'SKIP pose {pose_id}: already at target (max_joint_delta={max_delta:.4f}rad <= {GOAL_TOLERANCE_RAD:.4f}rad); no trajectory sent.')
        return
    current_position, current_orientation = live_transform(node, timeout_sec)
    target_position, target_orientation = target_transform(pose)
    linear_m, angular_rad = cartesian_delta(current_position, current_orientation, target_position, target_orientation)
    duration_sec = duration_for_delta(max_delta, linear_m, angular_rad)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(expected_names)
    start_point = JointTrajectoryPoint()
    start_point.positions = list(current)
    start_point.velocities = [0.0] * len(current)
    start_point.accelerations = [0.0] * len(current)
    point = JointTrajectoryPoint()
    point.positions = list(target)
    point.velocities = [0.0] * len(target)
    point.accelerations = [0.0] * len(target)
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int(round((duration_sec % 1.0) * 1_000_000_000))
    if point.time_from_start.nanosec == 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec = 0
    # 显式发送当前状态作为 t=0 起点，保证该段可按零速、零加速度的
    # 五次插值执行；duration_for_delta 的限制据此成立。
    goal.trajectory.points = [start_point, point]

    print(
        f'Sending pose {pose_id}: duration={duration_sec:.2f}s, '
        f'max_joint_delta={max_delta:.4f}rad, '
        f'tcp_delta={linear_m:.4f}m/{angular_rad:.4f}rad, '
        f'limits speed={MAX_JOINT_SPEED_RAD_S:.2f}rad/s '
        f'acceleration={MAX_JOINT_ACCELERATION_RAD_S2:.2f}rad/s^2 '
        f'tcp_linear={MAX_TCP_LINEAR_SPEED_M_S:.3f}m/s '
        f'tcp_angular={math.degrees(MAX_TCP_ANGULAR_SPEED_RAD_S):.1f}deg/s'
    )
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError(
            f'Pose {pose_id} goal was rejected. The action server is reachable, but the controller did not accept this goal; '
            'check that External Control is running and `ros2 control list_controllers` shows scaled_joint_trajectory_controller as active.'
        )
    node.active_goal = goal_handle

    result_future = goal_handle.get_result_async()
    node.active_result_future = result_future
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=duration_sec + ACTION_TIMEOUT_MARGIN_SEC)
    if not result_future.done():
        confirm_active_goal_stopped(node)
        raise RuntimeError(f'Pose {pose_id} exceeded execution timeout; cancellation was requested.')
    result = result_future.result()
    node.active_goal = None
    node.active_result_future = None
    if result is None or result.status != 4:
        raise RuntimeError(f'Pose {pose_id} failed with action status {None if result is None else result.status}.')

    rclpy.spin_once(node, timeout_sec=0.1)
    final_positions = positions_in_order(node.latest_joint_state, expected_names)
    final_error = maximum_joint_error(final_positions, target)
    if final_error > GOAL_TOLERANCE_RAD:
        raise RuntimeError(f'Pose {pose_id} final joint error {final_error:.4f}rad exceeds {GOAL_TOLERANCE_RAD:.4f}rad.')
    print(f'OK pose {pose_id}: max_joint_error_rad={final_error:.4f}')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', type=Path, default=DEFAULT_TRAJECTORY)
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    parser.add_argument('--execute', action='store_true', help='Required before any real robot motion.')
    return parser.parse_args()


def main():
    args = parse_args()
    if args.timeout_sec <= 0:
        raise ValueError('--timeout-sec must be positive.')
    trajectory_path = args.trajectory.resolve()
    poses = load_poses(trajectory_path)
    expected_names = poses[0]['joint_names']
    # Do not let rclpy shut down the ROS context before Python handles Ctrl-C.
    # The still-live context is required to cancel an accepted trajectory goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RobotState()
    client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)
    try:
        node.wait_for_joint_state(args.timeout_sec)
        live_transform(node, args.timeout_sec)
        print(f'Loaded {trajectory_path}')
        if not client.wait_for_server(timeout_sec=args.timeout_sec):
            raise RuntimeError(f'Trajectory Action server unavailable: {ACTION_NAME}')
        print(f'Validated live JointState, TF {BASE_FRAME} -> {TOOL_FRAME}, and trajectory Action server.')
        if not args.execute:
            print('Dry run complete. No trajectory was sent. Re-run with --execute only after a separate motion authorization.')
            return 0
        print('WARNING: This directly sends joint trajectories without MoveIt collision planning.')
        print(f'Executing poses {FIRST_POSE_ID} through {LAST_POSE_ID} because --execute was provided.')
        for pose in poses:
            send_pose(node, client, pose, expected_names, args.timeout_sec)
        print(f'Completed low-speed motion from the current pose through pose {LAST_POSE_ID}.')
        return 0
    except KeyboardInterrupt:
        print('\nCtrl-C received: cancelling the active trajectory before exit.', flush=True)
        confirm_active_goal_stopped(node)
        return 130
    finally:
        confirm_active_goal_stopped(node)
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
