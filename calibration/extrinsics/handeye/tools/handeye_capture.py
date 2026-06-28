#!/usr/bin/env python3
"""Interactive RGB ChArUco + UR TF sample capture for eye-in-hand calibration."""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener


def load_intrinsics(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    dist = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
    return data, camera_matrix, dist


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
    if norm == 0:
        raise ValueError('zero-length quaternion')
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


def matrix_to_list(matrix):
    return [[float(v) for v in row] for row in matrix]


class HandeyeCapture(Node):
    def __init__(self, args):
        super().__init__('handeye_capture')
        self.args = args
        self.bridge = CvBridge()
        self.latest = None
        self.lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.intrinsics_data, self.camera_matrix, self.dist_coeffs = load_intrinsics(args.intrinsics)
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
        self.get_logger().info(f'Looking up TF: {args.base_frame} -> {args.tool_frame}')

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return
        if image.ndim == 2:
            display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            gray = image if image.dtype == np.uint8 else cv2.normalize(
                image, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
        else:
            display = image.copy()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        with self.lock:
            self.latest = (msg.header.stamp, gray, display)

    def estimate_target_pose(self, gray, display):
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        overlay = display.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
        if ids is None or len(ids) == 0:
            return None, overlay, 0

        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board, self.camera_matrix, self.dist_coeffs
        )
        if charuco_ids is not None and count > 0:
            cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids)
        if charuco_ids is None or count < self.args.min_charuco_corners:
            return None, overlay, int(count or 0)

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
            return None, overlay, int(count)

        cv2.drawFrameAxes(
            overlay,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            self.args.axis_length_m,
        )
        target_in_camera = np.eye(4, dtype=np.float64)
        target_in_camera[:3, :3], _ = cv2.Rodrigues(rvec)
        target_in_camera[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        return target_in_camera, overlay, int(count)

    def capture_once(self, sample_id):
        with self.lock:
            latest = self.latest
        if latest is None:
            return False, 'No image received yet.'

        stamp, gray, display = latest
        target_in_camera, overlay, corner_count = self.estimate_target_pose(gray, display)
        if target_in_camera is None:
            return False, f'ChArUco pose failed; detected corners: {corner_count}.'

        if self.args.use_latest_tf:
            tf_time = Time()
        else:
            tf_time = Time.from_msg(stamp)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.tool_frame,
                tf_time,
                timeout=Duration(seconds=self.args.tf_timeout_sec),
            )
        except TransformException as exc:
            return False, f'TF lookup failed: {exc}'

        tool_in_base = transform_to_matrix(tf_msg)
        raw_path = self.args.output_dir / f'sample_{sample_id:03d}_raw.png'
        overlay_path = self.args.output_dir / f'sample_{sample_id:03d}_overlay.png'
        cv2.imwrite(str(raw_path), display)
        cv2.imwrite(str(overlay_path), overlay)

        record = {
            'sample_id': sample_id,
            'stamp_sec': int(stamp.sec) + int(stamp.nanosec) * 1e-9,
            'image_topic': self.args.image_topic,
            'base_frame': self.args.base_frame,
            'tool_frame': self.args.tool_frame,
            'camera_frame': self.args.camera_frame,
            'target_frame': 'charuco_board',
            'charuco_corners': corner_count,
            'T_base_tool': matrix_to_list(tool_in_base),
            'T_camera_target': matrix_to_list(target_in_camera),
            'raw_image': str(raw_path),
            'overlay_image': str(overlay_path),
        }
        with self.args.samples_file.open('a', encoding='utf-8') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')
        return True, f'sample_{sample_id:03d}: corners={corner_count}, saved {overlay_path}'


def next_sample_id(samples_file):
    if not samples_file.exists():
        return 1
    last = 0
    for line in samples_file.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        last = max(last, int(json.loads(line)['sample_id']))
    return last + 1


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--intrinsics', default='/home/z/Apps-my/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml')
    parser.add_argument('--output-dir', type=Path, default=Path('/home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples'))
    parser.add_argument('--samples-file', type=Path, default=None)
    parser.add_argument('--base-frame', default='base')
    parser.add_argument('--tool-frame', default='tool0')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=6)
    parser.add_argument('--square-length-m', type=float, default=0.025)
    parser.add_argument('--marker-length-m', type=float, default=0.018)
    parser.add_argument('--dictionary-id', default='DICT_6X6_1000')
    parser.add_argument('--start-id', type=int, default=233)
    parser.add_argument('--min-charuco-corners', type=int, default=8)
    parser.add_argument('--axis-length-m', type=float, default=0.05)
    parser.add_argument('--tf-timeout-sec', type=float, default=0.5)
    parser.add_argument('--use-latest-tf', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.samples_file is None:
        args.samples_file = args.output_dir / 'samples.jsonl'

    rclpy.init()
    node = HandeyeCapture(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sample_id = next_sample_id(args.samples_file)
    print('Press Enter to capture one stopped robot pose. Type q then Enter to quit.', flush=True)
    print(f'Samples file: {args.samples_file}', flush=True)
    try:
        while True:
            text = input(f'capture sample_{sample_id:03d}> ').strip().lower()
            if text in {'q', 'quit', 'exit'}:
                break
            ok, message = node.capture_once(sample_id)
            print(('OK  ' if ok else 'ERR ') + message, flush=True)
            if ok:
                sample_id += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    sys.exit(main())
