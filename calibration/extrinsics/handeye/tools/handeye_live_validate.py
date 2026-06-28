#!/usr/bin/env python3
"""Live validation for an eye-in-hand hand-eye calibration."""

import argparse
import json
import math
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener


def load_intrinsics(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    dist = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
    return camera_matrix, dist


def load_handeye(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return np.array(data['transform']['matrix'], dtype=np.float64)


def make_charuco_board(squares_x, squares_y, square_length, marker_length, dictionary_name, start_id):
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard_create(
        int(squares_x), int(squares_y), float(square_length), float(marker_length), dictionary
    )
    board.setIds(np.arange(start_id, start_id + len(board.ids), dtype=np.int32))
    return dictionary, board


def quat_to_rot(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def transform_to_matrix(transform_msg):
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def rot_angle_deg(r_a, r_b):
    delta = r_a.T @ r_b
    value = max(-1.0, min(1.0, (np.trace(delta) - 1.0) / 2.0))
    return float(np.degrees(np.arccos(value)))


class LiveValidator(Node):
    def __init__(self, args):
        super().__init__('handeye_live_validate')
        self.args = args
        self.bridge = CvBridge()
        self.latest = None
        self.lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_matrix, self.dist_coeffs = load_intrinsics(args.intrinsics)
        self.t_tool_camera = load_handeye(args.handeye)
        self.dictionary, self.board = make_charuco_board(
            args.squares_x,
            args.squares_y,
            args.square_length_m,
            args.marker_length_m,
            args.dictionary_id,
            args.start_id,
        )
        self.create_subscription(Image, args.image_topic, self.on_image, 10)
        self.get_logger().info(f'Listening image: {args.image_topic}')
        self.get_logger().info(f'Using hand-eye result: {args.handeye}')

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return
        if image.ndim == 2:
            gray = image if image.dtype == np.uint8 else cv2.normalize(
                image, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        with self.lock:
            self.latest = (msg.header.stamp, gray)

    def estimate_camera_target(self, gray):
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        if ids is None or len(ids) == 0:
            return None, 0
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board, self.camera_matrix, self.dist_coeffs
        )
        if charuco_ids is None or count < self.args.min_charuco_corners:
            return None, int(count or 0)
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.board,
            self.camera_matrix,
            self.dist_coeffs,
            None,
            None,
        )
        if not ok:
            return None, int(count)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3], _ = cv2.Rodrigues(rvec)
        matrix[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        return matrix, int(count)

    def sample(self):
        with self.lock:
            latest = self.latest
        if latest is None:
            return None, 'No image received yet.'
        _, gray = latest
        t_camera_target, corners = self.estimate_camera_target(gray)
        if t_camera_target is None:
            return None, f'ChArUco pose failed; corners={corners}'
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.args.tf_timeout_sec),
            )
        except TransformException as exc:
            return None, f'TF lookup failed: {exc}'
        t_base_tool = transform_to_matrix(tf_msg)
        t_base_target = t_base_tool @ self.t_tool_camera @ t_camera_target
        return {'corners': corners, 'T_base_target': t_base_target}, 'OK'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--intrinsics', default='/home/z/Apps-my/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml')
    parser.add_argument('--handeye', default='/home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/handeye_result_gt20_no32.yaml')
    parser.add_argument('--base-frame', default='base')
    parser.add_argument('--tool-frame', default='tool0')
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=6)
    parser.add_argument('--square-length-m', type=float, default=0.025)
    parser.add_argument('--marker-length-m', type=float, default=0.018)
    parser.add_argument('--dictionary-id', default='DICT_6X6_1000')
    parser.add_argument('--start-id', type=int, default=233)
    parser.add_argument('--min-charuco-corners', type=int, default=20)
    parser.add_argument('--tf-timeout-sec', type=float, default=0.5)
    parser.add_argument('--output', default='/home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/live_validation.jsonl')
    args = parser.parse_args()

    rclpy.init()
    node = LiveValidator(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    samples = []
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    print('Move robot to different stopped poses. Press Enter to validate; q to quit.', flush=True)
    try:
        while True:
            text = input(f'validate sample_{len(samples) + 1:03d}> ').strip().lower()
            if text in {'q', 'quit', 'exit'}:
                break
            result, message = node.sample()
            if result is None:
                print('ERR ' + message, flush=True)
                continue
            pose = result['T_base_target']
            samples.append(pose)
            record = {
                'sample_id': len(samples),
                'corners': result['corners'],
                'T_base_target': pose.tolist(),
            }
            with output.open('a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
            positions = np.array([s[:3, 3] for s in samples])
            mean_position = positions.mean(axis=0)
            trans_errors = np.linalg.norm(positions - mean_position, axis=1)
            rot_errors = np.array([rot_angle_deg(samples[0][:3, :3], s[:3, :3]) for s in samples])
            print(
                f"OK corners={result['corners']} "
                f"target_xyz=[{pose[0,3]:.4f}, {pose[1,3]:.4f}, {pose[2,3]:.4f}] "
                f"scatter_mean={trans_errors.mean()*1000:.2f}mm "
                f"scatter_max={trans_errors.max()*1000:.2f}mm "
                f"rot_mean={rot_errors.mean():.2f}deg "
                f"rot_max={rot_errors.max():.2f}deg",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    sys.exit(main())
