#!/usr/bin/env python3
"""Safe, runtime-adaptive teaching support for fixed-path hand-eye capture.

This first module never sends a robot motion command.  It discovers live ROS
interfaces and records manually taught joint positions with their TF metadata.
"""

import argparse
import copy
import csv
import math
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
import cv2
import numpy as np
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener


PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = PROJECT_ROOT / 'calibration/extrinsics/handeye/auto_handeye_capture.yaml'
IMAGE_TYPE = 'sensor_msgs/msg/Image'
CAMERA_INFO_TYPE = 'sensor_msgs/msg/CameraInfo'
JOINT_STATE_TYPE = 'sensor_msgs/msg/JointState'
TRAJECTORY_ACTION_TYPE = 'control_msgs/action/FollowJointTrajectory'


def load_yaml(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'YAML root must be a mapping: {path}')
    return data


def save_yaml(path, data):
    Path(path).write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8'
    )


def as_ros_time(stamp):
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


def transform_dict(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        'translation_m': {'x': float(translation.x), 'y': float(translation.y), 'z': float(translation.z)},
        'quaternion_xyzw': {
            'x': float(rotation.x), 'y': float(rotation.y), 'z': float(rotation.z), 'w': float(rotation.w),
        },
        'stamp_sec': as_ros_time(transform.header.stamp),
    }


class InterfaceProbe(Node):
    def __init__(self, config):
        super().__init__('auto_handeye_interface_probe')
        self.config = config
        self.latest_joint_state = None
        self.latest_image = None
        self.camera_info = None
        self.lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscriptions_started = set()

    def graph_topics(self):
        return dict(self.get_topic_names_and_types())

    def graph_actions(self):
        return dict(get_action_names_and_types(self))

    @staticmethod
    def choose_topic(topics, expected_type, configured, preferred_suffixes):
        if configured != 'auto':
            types = topics.get(configured, [])
            if expected_type not in types:
                raise RuntimeError(f'Configured topic {configured} does not publish {expected_type}.')
            return configured
        candidates = sorted(name for name, types in topics.items() if expected_type in types)
        if not candidates:
            return None
        for suffix in preferred_suffixes:
            matches = [name for name in candidates if name.endswith(suffix)]
            if len(matches) == 1:
                return matches[0]
        return candidates[0] if len(candidates) == 1 else None

    def discover(self):
        topics = self.graph_topics()
        actions = self.graph_actions()
        interfaces = self.config['interfaces']
        image_topic = self.choose_topic(topics, IMAGE_TYPE, interfaces['image_topic'], ('/color/image_raw', '/image_raw'))
        camera_info_topic = self.choose_topic(
            topics, CAMERA_INFO_TYPE, interfaces['camera_info_topic'], ('/color/camera_info', '/camera_info')
        )
        joint_state_topic = self.choose_topic(topics, JOINT_STATE_TYPE, interfaces['joint_state_topic'], ('/joint_states',))
        configured_action = interfaces['trajectory_action']
        if configured_action == 'auto':
            action_candidates = sorted(
                name for name, types in actions.items() if TRAJECTORY_ACTION_TYPE in types
            )
            trajectory_action = action_candidates[0] if len(action_candidates) == 1 else None
        else:
            types = actions.get(configured_action, [])
            if TRAJECTORY_ACTION_TYPE not in types:
                raise RuntimeError(
                    f'Configured action {configured_action} does not provide {TRAJECTORY_ACTION_TYPE}.'
                )
            trajectory_action = configured_action
        return {
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'joint_state_topic': joint_state_topic,
            'trajectory_action': trajectory_action,
            'topic_candidates': topics,
            'action_candidates': actions,
        }

    def subscribe(self, discovered):
        if discovered['joint_state_topic'] and 'joint' not in self.subscriptions_started:
            self.create_subscription(JointState, discovered['joint_state_topic'], self._on_joint, 20)
            self.subscriptions_started.add('joint')
        if discovered['image_topic'] and 'image' not in self.subscriptions_started:
            self.create_subscription(Image, discovered['image_topic'], self._on_image, 5)
            self.subscriptions_started.add('image')
        if discovered['camera_info_topic'] and 'camera_info' not in self.subscriptions_started:
            self.create_subscription(CameraInfo, discovered['camera_info_topic'], self._on_camera_info, 5)
            self.subscriptions_started.add('camera_info')

    def _on_joint(self, message):
        if message.name and message.position:
            with self.lock:
                self.latest_joint_state = message

    def _on_image(self, message):
        with self.lock:
            self.latest_image = message

    def _on_camera_info(self, message):
        with self.lock:
            self.camera_info = message

    def wait_for(self, required, timeout_sec):
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            with self.lock:
                present = {
                    'joint_state': self.latest_joint_state is not None,
                    'image': self.latest_image is not None,
                    'camera_info': self.camera_info is not None,
                }
            if all(present[key] for key in required):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def current_waypoint(self, base_frame, tool_frame, note):
        with self.lock:
            joint_state = copy.deepcopy(self.latest_joint_state)
        if joint_state is None:
            raise RuntimeError('No JointState has been received.')
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame, tool_frame, rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as exc:
            raise RuntimeError(f'TF lookup {base_frame} -> {tool_frame} failed: {exc}') from exc
        return {
            'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
            'joint_names': list(joint_state.name),
            'joint_positions_rad': [float(value) for value in joint_state.position],
            'joint_velocities_rad_s': [float(value) for value in joint_state.velocity],
            'joint_state_stamp_sec': as_ros_time(joint_state.header.stamp),
            'base_frame': base_frame,
            'tool_frame': tool_frame,
            'T_base_tool': transform_dict(transform),
            'note': note,
        }


def validate_trajectory(trajectory, current_joint_names):
    if not isinstance(trajectory.get('start_pose'), dict):
        raise RuntimeError('Trajectory has no start_pose.')
    waypoints = trajectory.get('waypoints')
    if not isinstance(waypoints, list) or not waypoints:
        raise RuntimeError('Trajectory has no capture waypoints.')
    expected_names = list(current_joint_names)
    for label, waypoint in [('start_pose', trajectory['start_pose'])] + [
        (f'waypoint {index}', waypoint) for index, waypoint in enumerate(waypoints, start=1)
    ]:
        names = waypoint.get('joint_names')
        positions = waypoint.get('joint_positions_rad')
        if names != expected_names:
            raise RuntimeError(f'{label} joint_names do not match live JointState names.')
        if not isinstance(positions, list) or len(positions) != len(expected_names):
            raise RuntimeError(f'{label} has an invalid joint_positions_rad length.')
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in positions):
            raise RuntimeError(f'{label} contains non-finite joint positions.')
    return len(waypoints)


def detect_charuco(image, camera_info, config):
    board_config = config['charuco']
    dictionary_id = getattr(cv2.aruco, board_config['dictionary_id'], None)
    if dictionary_id is None:
        raise RuntimeError(f"Unsupported ArUco dictionary: {board_config['dictionary_id']}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard_create(
        board_config['squares_x'], board_config['squares_y'],
        board_config['square_length_m'], board_config['marker_length_m'], dictionary,
    )
    board.setIds(np.arange(board_config['start_id'], board_config['start_id'] + len(board.ids), dtype=np.int32))
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    board_ids = set(int(marker_id) for marker_id in board.ids.flatten())
    detected_ids = [] if ids is None else [int(marker_id) for marker_id in ids.flatten()]
    target_ids = [marker_id for marker_id in detected_ids if marker_id in board_ids]
    duplicate_target_ids = sorted({marker_id for marker_id in target_ids if target_ids.count(marker_id) > 1})
    count, _, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board,
        np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3),
        np.asarray(camera_info.d, dtype=np.float64).reshape(-1, 1),
    )
    count = int(count or 0)
    return {
        'marker_count': 0 if ids is None else int(len(ids)),
        'target_marker_count': len(target_ids),
        'duplicate_target_ids': duplicate_target_ids,
        'corner_count': count,
        'detected': (
            charuco_ids is not None
            and count >= board_config['min_corners']
            and not duplicate_target_ids
        ),
    }


def print_discovery(discovered):
    print('Detected interfaces:')
    for key in ('image_topic', 'camera_info_topic', 'joint_state_topic', 'trajectory_action'):
        print(f'  {key}: {discovered[key] or "UNRESOLVED"}')
    if not discovered['trajectory_action']:
        actions = [name for name, types in discovered['action_candidates'].items() if TRAJECTORY_ACTION_TYPE in types]
        print(f'  FollowJointTrajectory candidates: {actions or "none"}')


def require_frames(config):
    interfaces = config['interfaces']
    if interfaces['base_frame'] == 'auto' or interfaces['tool_frame'] == 'auto':
        raise RuntimeError(
            'base_frame and tool_frame cannot be auto during teaching. Set the frames confirmed by tf2_echo.'
        )
    return interfaces['base_frame'], interfaces['tool_frame']


def discover_with_retry(node, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    last_error = None
    while time.monotonic() < deadline:
        try:
            return node.discover()
        except RuntimeError as exc:
            last_error = exc
            rclpy.spin_once(node, timeout_sec=0.1)
    if last_error is not None:
        raise last_error
    return node.discover()


def command_inspect(args):
    config = load_yaml(args.config)
    rclpy.init()
    node = InterfaceProbe(config)
    try:
        discovered = discover_with_retry(node, args.timeout_sec)
        node.subscribe(discovered)
        node.wait_for(('image', 'camera_info'), args.timeout_sec)
        print_discovery(discovered)
        with node.lock:
            if node.camera_info:
                print(
                    f"  camera_info: {node.camera_info.width}x{node.camera_info.height}, "
                    f"frame={node.camera_info.header.frame_id}"
                )
            if node.latest_image:
                print(f'  latest_image_stamp_sec: {as_ros_time(node.latest_image.header.stamp):.9f}')
            if node.latest_joint_state:
                print(f"  joints: {', '.join(node.latest_joint_state.name)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_teach(args):
    config = load_yaml(args.config)
    base_frame, tool_frame = require_frames(config)
    trajectory_path = Path(args.trajectory).resolve()
    if trajectory_path.exists():
        trajectory = load_yaml(trajectory_path)
    else:
        trajectory = {'format_version': 1, 'start_pose': None, 'waypoints': []}
    if not isinstance(trajectory.get('waypoints'), list) or 'start_pose' not in trajectory:
        raise RuntimeError('Invalid trajectory YAML: expected start_pose and waypoints.')

    rclpy.init()
    node = InterfaceProbe(config)
    spin_thread = None
    try:
        discovered = discover_with_retry(node, args.timeout_sec)
        node.subscribe(discovered)
        if not discovered['joint_state_topic']:
            raise RuntimeError('No unique JointState topic found; configure interfaces.joint_state_topic.')
        if not node.wait_for(('joint_state',), args.timeout_sec):
            raise RuntimeError('Timed out waiting for JointState.')
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        print_discovery(discovered)
        print('Commands: Enter=record, d=delete last waypoint, l=list, s=save, q=quit')
        while True:
            command = input('teach> ').strip().lower()
            if command in {'q', 'quit'}:
                break
            if command in {'l', 'list'}:
                print(f"start_pose: {'recorded' if trajectory['start_pose'] else 'not recorded'}")
                for index, waypoint in enumerate(trajectory['waypoints'], start=1):
                    print(f"  {index}: {waypoint.get('note', '')} {waypoint['joint_positions_rad']}")
                continue
            if command in {'d', 'delete'}:
                if trajectory['waypoints']:
                    removed = trajectory['waypoints'].pop()
                    print(f"Deleted waypoint {removed.get('id', len(trajectory['waypoints']) + 1)}.")
                else:
                    print('No capture waypoint to delete; start_pose is retained.')
                continue
            if command in {'s', 'save'}:
                save_yaml(trajectory_path, trajectory)
                print(f'Saved {trajectory_path}')
                continue
            if command:
                print('Unknown command.')
                continue
            note = input('note (optional)> ').strip()
            waypoint = node.current_waypoint(base_frame, tool_frame, note)
            if trajectory['start_pose'] is None:
                trajectory['start_pose'] = waypoint
                print('Recorded start_pose.')
            else:
                existing_ids = [item.get('id', 0) for item in trajectory['waypoints']]
                waypoint['id'] = max(existing_ids, default=0) + 1
                trajectory['waypoints'].append(waypoint)
                print(f"Recorded waypoint {waypoint['id']}.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
    save_yaml(trajectory_path, trajectory)
    print(f'Saved {trajectory_path}')


def command_dry_run(args):
    config = load_yaml(args.config)
    trajectory = load_yaml(args.trajectory)
    rclpy.init()
    node = InterfaceProbe(config)
    client = None
    try:
        discovered = discover_with_retry(node, args.timeout_sec)
        node.subscribe(discovered)
        if not discovered['joint_state_topic']:
            raise RuntimeError('No unique JointState topic found; configure interfaces.joint_state_topic.')
        if not discovered['trajectory_action']:
            raise RuntimeError('No unique FollowJointTrajectory Action found; configure interfaces.trajectory_action.')
        if not node.wait_for(('joint_state',), args.timeout_sec):
            raise RuntimeError('Timed out waiting for live JointState.')
        with node.lock:
            joint_names = list(node.latest_joint_state.name)
        waypoint_count = validate_trajectory(trajectory, joint_names)
        client = ActionClient(node, FollowJointTrajectory, discovered['trajectory_action'])
        if not client.wait_for_server(timeout_sec=args.timeout_sec):
            raise RuntimeError(f"Trajectory Action server unavailable: {discovered['trajectory_action']}")
        print_discovery(discovered)
        print(f'Validated start_pose and {waypoint_count} capture waypoints against {len(joint_names)} live joints.')
        print('No trajectory was sent. Collision/planning validation is unavailable because no project MoveIt config was found.')
    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def command_board_check(args):
    config = load_yaml(args.config)
    board_config = config['charuco']
    dictionary_id = getattr(cv2.aruco, board_config['dictionary_id'], None)
    if dictionary_id is None:
        raise RuntimeError(f"Unsupported ArUco dictionary: {board_config['dictionary_id']}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard_create(
        board_config['squares_x'], board_config['squares_y'],
        board_config['square_length_m'], board_config['marker_length_m'], dictionary,
    )
    board.setIds(np.arange(board_config['start_id'], board_config['start_id'] + len(board.ids), dtype=np.int32))
    rclpy.init()
    node = InterfaceProbe(config)
    bridge = CvBridge()
    last_stamp = None
    stable_frames = 0
    try:
        discovered = discover_with_retry(node, args.timeout_sec)
        node.subscribe(discovered)
        if not node.wait_for(('image', 'camera_info'), args.timeout_sec):
            raise RuntimeError('Timed out waiting for Image and CameraInfo.')
        print_discovery(discovered)
        print('Checking ChArUco quality. Press Ctrl-C to stop.')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            with node.lock:
                image_message = copy.deepcopy(node.latest_image)
                camera_info = copy.deepcopy(node.camera_info)
            if image_message is None or camera_info is None:
                continue
            stamp = as_ros_time(image_message.header.stamp)
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
            gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            result = detect_charuco(image, camera_info, config)
            count = result['corner_count']
            if result['detected']:
                stable_frames += 1
            else:
                stable_frames = 0
            status = 'READY' if stable_frames >= board_config['stable_frames'] else 'WAIT'
            print(
                f'{status} stamp={stamp:.6f} markers={result["marker_count"]} '
                f'corners={count} duplicate_target_ids={result["duplicate_target_ids"]} '
                f'stable={stable_frames}/{board_config["stable_frames"]}',
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def command_capture(args):
    config = load_yaml(args.config)
    output_root = Path(args.output_root or config['storage']['output_root']).resolve()
    run_dir = output_root / datetime.now().strftime('run_%Y%m%d_%H%M%S')
    image_dir = run_dir / 'images'
    image_dir.mkdir(parents=True, exist_ok=False)
    save_yaml(run_dir / 'config_used.yaml', config)
    rclpy.init()
    node = InterfaceProbe(config)
    bridge = CvBridge()
    records = []
    base_frame, tool_frame = require_frames(config)
    try:
        discovered = discover_with_retry(node, args.timeout_sec)
        node.subscribe(discovered)
        if not node.wait_for(('image', 'joint_state'), args.timeout_sec):
            raise RuntimeError('Timed out waiting for Image and JointState.')
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        print_discovery(discovered)
        print('Press Enter to save one stationary image/TF pair; q then Enter to quit.')
        while True:
            if input(f'capture {len(records) + 1}> ').strip().lower() in {'q', 'quit', 'exit'}:
                break
            with node.lock:
                image_message = copy.deepcopy(node.latest_image)
                joint_state = copy.deepcopy(node.latest_joint_state)
                camera_info = copy.deepcopy(node.camera_info)
            stamp = rclpy.time.Time.from_msg(image_message.header.stamp)
            try:
                transform = node.tf_buffer.lookup_transform(
                    base_frame, tool_frame, stamp, timeout=Duration(seconds=0.5)
                )
            except TransformException as exc:
                print(f'ERR TF lookup at image timestamp failed: {exc}')
                continue
            delta_sec = abs(as_ros_time(transform.header.stamp) - as_ros_time(image_message.header.stamp))
            if delta_sec > config['safety']['tf_max_delta_sec']:
                print(f'ERR TF/image time delta {delta_sec:.6f}s exceeds limit.')
                continue
            image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
            board_result = detect_charuco(image, camera_info, config)
            if not board_result['detected']:
                print(f"ERR ChArUco rejected: markers={board_result['marker_count']} corners={board_result['corner_count']}")
                continue
            index = len(records) + 1
            filename = f'pose_{index:03d}.png'
            if not cv2.imwrite(str(image_dir / filename), image):
                raise RuntimeError(f'Failed to write {filename}.')
            records.append({
                'pose_id': index, 'image_filename': f'images/{filename}',
                'image_timestamp': as_ros_time(image_message.header.stamp),
                'tf_time_delta_sec': delta_sec, 'T_base_tool': transform_dict(transform),
                'joint_names': list(joint_state.name),
                'joint_positions_rad': [float(value) for value in joint_state.position],
                'board_detection': board_result,
            })
            print(f'OK pose_{index:03d}: image/TF delta={delta_sec:.6f}s')
    finally:
        if 'spin_thread' in locals():
            rclpy.shutdown()
            spin_thread.join(timeout=2.0)
        else:
            rclpy.shutdown()
        node.destroy_node()
    save_yaml(run_dir / 'poses.yaml', {'records': records})
    with (run_dir / 'poses.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['pose_id', 'image_filename', 'image_timestamp', 'tf_time_delta_sec', 'joint_positions_rad', 'marker_count', 'corner_count'])
        writer.writerows([[row['pose_id'], row['image_filename'], row['image_timestamp'], row['tf_time_delta_sec'], row['joint_positions_rad'], row['board_detection']['marker_count'], row['board_detection']['corner_count']] for row in records])
    print(f'Saved {len(records)} pairs in {run_dir}')


def command_execute(args):
    if not args.execute:
        raise RuntimeError('Refusing motion: rerun with --execute after reviewing the taught trajectory.')
    config = load_yaml(args.config)
    trajectory = load_yaml(args.trajectory)
    rclpy.init()
    node = InterfaceProbe(config)
    client = None
    active_goal = None
    records = []
    run_dir = None
    try:
        discovered = discover_with_retry(node, args.timeout_sec)
        node.subscribe(discovered)
        if not node.wait_for(('joint_state',), args.timeout_sec):
            raise RuntimeError('Timed out waiting for live JointState.')
        with node.lock:
            joint_names = list(node.latest_joint_state.name)
        validate_trajectory(trajectory, joint_names)
        client = ActionClient(node, FollowJointTrajectory, discovered['trajectory_action'])
        if not client.wait_for_server(timeout_sec=args.timeout_sec):
            raise RuntimeError('Trajectory Action server unavailable.')
        print_discovery(discovered)
        print('WARNING: MoveIt collision planning is unavailable. Only execute if the taught path is manually verified clear.')
        if input('Type EXECUTE to move to start_pose and all waypoints: ').strip() != 'EXECUTE':
            print('Cancelled before any trajectory was sent.')
            return
        bridge = CvBridge()
        if args.capture:
            run_dir = Path(args.output_root).resolve() / datetime.now().strftime('auto_capture_%Y%m%d_%H%M%S')
            (run_dir / 'images').mkdir(parents=True, exist_ok=False)
            save_yaml(run_dir / 'config_used.yaml', config)
            print(f'Capture output: {run_dir}')
        for label, target in [('start_pose', trajectory['start_pose'])] + [
            (f"waypoint_{item['id']}", item) for item in trajectory['waypoints']
        ]:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(target['joint_names'])
            point = JointTrajectoryPoint()
            with node.lock:
                current = list(node.latest_joint_state.position)
            commanded_positions = list(target['joint_positions_rad'])
            if 'wrist_3_joint' in goal.trajectory.joint_names:
                wrist_3_index = goal.trajectory.joint_names.index('wrist_3_joint')
                commanded_positions[wrist_3_index] += 2.0 * math.pi * round(
                    (current[wrist_3_index] - commanded_positions[wrist_3_index]) / (2.0 * math.pi)
                )
            point.positions = commanded_positions
            max_delta = max(abs(actual - expected) for actual, expected in zip(current, point.positions))
            duration_sec = max(
                float(config['safety']['motion_duration_sec']),
                max_delta / float(config['safety']['max_joint_speed_rad_s']),
            )
            point.time_from_start.sec = int(duration_sec)
            point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
            goal.trajectory.points = [point]
            print(f'Sending {label} with duration={duration_sec:.2f}s, max_joint_delta={max_delta:.3f}rad')
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future)
            active_goal = future.result()
            if active_goal is None or not active_goal.accepted:
                raise RuntimeError(f'{label} goal was rejected.')
            result_future = active_goal.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=duration_sec + 5.0)
            if not result_future.done():
                cancel_future = active_goal.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
                raise RuntimeError(f'{label} exceeded its execution timeout; cancellation was requested.')
            result = result_future.result()
            if result is None or result.status != 4:
                raise RuntimeError(f'{label} execution failed with action status {None if result is None else result.status}.')
            with node.lock:
                current = list(node.latest_joint_state.position)
            maximum_error = max(abs(actual - expected) for actual, expected in zip(current, commanded_positions))
            if maximum_error > config['safety']['goal_tolerance_rad']:
                raise RuntimeError(f'{label} final joint error {maximum_error:.4f} rad exceeds tolerance.')
            print(f'OK {label}: max_joint_error_rad={maximum_error:.4f}')
            if args.capture:
                settle_deadline = time.monotonic() + float(config['safety']['settle_duration_sec'])
                while time.monotonic() < settle_deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                stable = 0
                captured = False
                last_failure = 'no image/TF sample received'
                for attempt in range(config['storage']['board_retry_count'] * config['charuco']['stable_frames'] * 4):
                    rclpy.spin_once(node, timeout_sec=0.2)
                    with node.lock:
                        image_message = copy.deepcopy(node.latest_image)
                        camera_info = copy.deepcopy(node.camera_info)
                        joint_state = copy.deepcopy(node.latest_joint_state)
                    if image_message is None or camera_info is None:
                        last_failure = 'missing Image or CameraInfo'
                        continue
                    image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
                    board_result = detect_charuco(image, camera_info, config)
                    stable = stable + 1 if board_result['detected'] else 0
                    if stable < config['charuco']['stable_frames']:
                        last_failure = f"ChArUco corners={board_result['corner_count']} below stable threshold"
                        continue
                    image_time = rclpy.time.Time.from_msg(image_message.header.stamp)
                    try:
                        transform = node.tf_buffer.lookup_transform(
                            config['interfaces']['base_frame'], config['interfaces']['tool_frame'], image_time,
                            timeout=Duration(seconds=0.5),
                        )
                    except TransformException:
                        try:
                            transform = node.tf_buffer.lookup_transform(
                                config['interfaces']['base_frame'], config['interfaces']['tool_frame'],
                                rclpy.time.Time(), timeout=Duration(seconds=0.2),
                            )
                        except TransformException:
                            stable = 0
                            last_failure = 'TF unavailable for image time and latest time'
                            continue
                    delta_sec = abs(as_ros_time(transform.header.stamp) - as_ros_time(image_message.header.stamp))
                    if delta_sec > config['safety']['tf_max_delta_sec']:
                        max_velocity = max((abs(value) for value in joint_state.velocity), default=float('inf'))
                        if (
                            max_velocity > config['safety']['still_velocity_rad_s']
                            or delta_sec > config['safety']['stationary_tf_max_delta_sec']
                        ):
                            stable = 0
                            last_failure = f'TF/image delta={delta_sec:.6f}s exceeds allowed stationary limit'
                            continue
                    pose_id = len(records) + 1
                    image_name = f'pose_{pose_id:03d}.png'
                    if not cv2.imwrite(str(run_dir / 'images' / image_name), image):
                        raise RuntimeError(f'Failed to save {image_name}.')
                    records.append({
                        'pose_id': pose_id, 'waypoint_label': label, 'image_filename': f'images/{image_name}',
                        'image_timestamp': as_ros_time(image_message.header.stamp), 'tf_time_delta_sec': delta_sec,
                        'T_base_tool': transform_dict(transform), 'joint_names': list(joint_state.name),
                        'joint_positions_rad': [float(value) for value in joint_state.position],
                        'board_detection': board_result,
                    })
                    print(f"CAPTURED {label}: corners={board_result['corner_count']} tf_delta={delta_sec:.6f}s")
                    captured = True
                    break
                if not captured:
                    message = f'{label} capture failed: {last_failure}.'
                    if args.continue_on_capture_failure:
                        print(f'WARN {message} Continuing quality survey.')
                    else:
                        raise RuntimeError(message)
            active_goal = None
    except KeyboardInterrupt:
        if active_goal is not None:
            cancel_future = active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
        raise RuntimeError('Execution interrupted; active goal cancellation was requested.')
    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if run_dir is not None:
        save_yaml(run_dir / 'poses.yaml', {'records': records})
        with (run_dir / 'poses.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['pose_id', 'waypoint_label', 'image_filename', 'image_timestamp', 'tf_time_delta_sec', 'joint_positions_rad', 'marker_count', 'corner_count'])
            writer.writerows([[row['pose_id'], row['waypoint_label'], row['image_filename'], row['image_timestamp'], row['tf_time_delta_sec'], row['joint_positions_rad'], row['board_detection']['marker_count'], row['board_detection']['corner_count']] for row in records])
        print(f'Saved {len(records)} capture records in {run_dir}')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    subparsers = parser.add_subparsers(dest='command', required=True)
    subparsers.add_parser('inspect', help='Discover live ROS interfaces without commanding the robot.')
    teach = subparsers.add_parser('teach', help='Record manually taught joint waypoints; never commands motion.')
    teach.add_argument('--trajectory', type=Path, required=True)
    dry_run = subparsers.add_parser('dry-run', help='Validate taught joints and Action availability; never commands motion.')
    dry_run.add_argument('--trajectory', type=Path, required=True)
    subparsers.add_parser('board-check', help='Check live ChArUco visibility without saving or moving.')
    capture = subparsers.add_parser('capture', help='Manually save image/TF pairs without robot motion.')
    capture.add_argument('--output-root', type=Path)
    execute = subparsers.add_parser('execute', help='Directly execute taught joints without MoveIt collision planning.')
    execute.add_argument('--trajectory', type=Path, required=True)
    execute.add_argument('--execute', action='store_true', help='Required acknowledgement before sending any trajectory.')
    execute.add_argument('--capture', action='store_true', help='Capture a quality-gated image/TF pair after every target.')
    execute.add_argument('--continue-on-capture-failure', action='store_true', help='For a quality survey, continue after a failed capture.')
    execute.add_argument('--output-root', type=Path, default=PROJECT_ROOT / 'calibration/extrinsics/handeye/sessions')
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        if args.command == 'inspect':
            command_inspect(args)
        elif args.command == 'teach':
            command_teach(args)
        elif args.command == 'dry-run':
            command_dry_run(args)
        elif args.command == 'capture':
            command_capture(args)
        elif args.command == 'execute':
            command_execute(args)
        else:
            command_board_check(args)
    except (RuntimeError, ValueError, OSError, yaml.YAMLError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
