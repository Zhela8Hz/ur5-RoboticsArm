#!/usr/bin/env python3
"""Interactive ChArUco intrinsics capture from a ROS image topic."""

import argparse
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


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


def make_detector_params():
    params = cv2.aruco.DetectorParameters_create()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 101
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.03
    params.minCornerDistanceRate = 0.03
    params.minDistanceToBorder = 1
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    params.errorCorrectionRate = 0.8
    return params


def normalize_gray8(image):
    if image.dtype == np.uint8:
        return image
    data = image.astype(np.float32)
    vmin, vmax = np.percentile(data, 1), np.percentile(data, 99)
    if vmax <= vmin:
        return np.zeros(image.shape, dtype=np.uint8)
    return np.clip((data - vmin) / (vmax - vmin) * 255, 0, 255).astype(np.uint8)


class InteractiveCharucoIntrinsics(Node):
    def __init__(self, args):
        super().__init__('interactive_charuco_intrinsics')
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.image_size = None
        self.capture_attempt = 0
        self.charuco_corners = []
        self.charuco_ids = []
        self.dictionary, self.board = make_charuco_board(
            args.squares_x,
            args.squares_y,
            args.square_length_m,
            args.marker_length_m,
            args.dictionary_id,
            args.start_id,
        )
        self.detector_params = make_detector_params()
        self.create_subscription(Image, args.image_topic, self.on_image, 10)

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return
        if image.ndim == 2:
            raw_image = image
            gray = normalize_gray8(image)
        else:
            raw_image = image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        with self.lock:
            self.latest = (gray, raw_image)
            self.image_size = (gray.shape[1], gray.shape[0])

    def wait_for_image(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self.lock:
                if self.latest is not None:
                    return True
            time.sleep(0.05)
        return False

    def capture_once(self):
        with self.lock:
            latest = None if self.latest is None else (self.latest[0].copy(), self.latest[1].copy())
        if latest is None:
            return False, 'No image received yet.'

        gray, raw_image = latest
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params
        )
        marker_count = 0 if ids is None else len(ids)

        self.capture_attempt += 1
        capture_dir = Path(self.args.capture_dir).expanduser()
        capture_dir.mkdir(parents=True, exist_ok=True)
        stem = f'capture_{self.capture_attempt:03d}'
        raw_path = capture_dir / f'{stem}_raw.png'
        overlay_path = capture_dir / f'{stem}_overlay.png'
        cv2.imwrite(str(raw_path), raw_image)
        overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        charuco_count = 0
        charuco_corners = None
        charuco_ids = None
        if ids is not None and marker_count > 0:
            cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
            count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board
            )
            charuco_count = int(count or 0)
            if charuco_ids is not None and charuco_count > 0:
                cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids)

        cv2.imwrite(str(overlay_path), overlay)
        accepted = charuco_ids is not None and charuco_count > self.args.min_charuco_corners
        if accepted:
            self.charuco_corners.append(charuco_corners)
            self.charuco_ids.append(charuco_ids)
            return True, (
                f'ACCEPTED {len(self.charuco_corners):02d}: '
                f'markers={marker_count}, charuco_corners={charuco_count} '
                f'(>{self.args.min_charuco_corners}); saved {raw_path} and {overlay_path}'
            )
        return False, (
            f'REJECTED: markers={marker_count}, charuco_corners={charuco_count} '
            f'(need >{self.args.min_charuco_corners}); saved {raw_path} and {overlay_path}'
        )

    def calibrate(self):
        if len(self.charuco_corners) < self.args.min_samples:
            return False, (
                f'Only {len(self.charuco_corners)} accepted samples; '
                f'need at least {self.args.min_samples}.'
            )
        error, camera_matrix, distortion, _, _ = cv2.aruco.calibrateCameraCharuco(
            self.charuco_corners, self.charuco_ids, self.board, self.image_size, None, None
        )
        path = Path(self.args.output_file).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'image_width': self.image_size[0],
            'image_height': self.image_size[1],
            'camera_name': self.args.camera_name,
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(distortion.ravel()),
                'data': distortion.ravel().tolist(),
            },
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': camera_matrix.ravel().tolist()},
            'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': np.hstack((camera_matrix, np.zeros((3, 1)))).ravel().tolist(),
            },
            'reprojection_error_px': float(error),
            'accepted_samples': len(self.charuco_corners),
            'min_charuco_corners_strictly_greater_than': int(self.args.min_charuco_corners),
        }
        path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
        return True, f'Calibration saved to {path}; reprojection error: {error:.3f}px'


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=6)
    parser.add_argument('--square-length-m', type=float, default=0.025)
    parser.add_argument('--marker-length-m', type=float, default=0.018)
    parser.add_argument('--dictionary-id', default='DICT_6X6_1000')
    parser.add_argument('--start-id', type=int, default=233)
    parser.add_argument('--camera-name', default='gemini335_color')
    parser.add_argument('--min-charuco-corners', type=int, default=20)
    parser.add_argument('--target-samples', type=int, default=20)
    parser.add_argument('--min-samples', type=int, default=12)
    parser.add_argument('--capture-dir', default='/tmp/gemini335_rgb_intrinsics_interactive')
    parser.add_argument(
        '--output-file',
        default='calibration/rgb_intrinsics/results/rgb_intrinsics_gemini335_1920x1080_interactive.yaml',
    )
    parser.add_argument('--image-timeout-sec', type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = InteractiveCharucoIntrinsics(args)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        print(f'Waiting for images on {args.image_topic} ...', flush=True)
        if not node.wait_for_image(args.image_timeout_sec):
            raise SystemExit(f'No image received on {args.image_topic} within {args.image_timeout_sec:.1f}s')
        print('Ready.', flush=True)
        print('Press Enter to capture. Type c + Enter to calibrate, s + Enter for status, q + Enter to quit.', flush=True)
        print(f'Only frames with ChArUco corners > {args.min_charuco_corners} are accepted.', flush=True)

        while True:
            accepted_count = len(node.charuco_corners)
            prompt = f'[{accepted_count}/{args.target_samples}] capture> '
            command = input(prompt).strip().lower()
            if command in ('q', 'quit', 'exit'):
                print('Quit without calibrating.', flush=True)
                break
            if command in ('s', 'status'):
                print(f'accepted_samples={accepted_count}, target_samples={args.target_samples}', flush=True)
                continue
            if command in ('c', 'calibrate', 'finish'):
                ok, message = node.calibrate()
                print(message, flush=True)
                if ok:
                    break
                continue
            if command:
                print('Unknown command. Press Enter to capture, c to calibrate, s for status, q to quit.', flush=True)
                continue

            accepted, message = node.capture_once()
            print(message, flush=True)
            if accepted and len(node.charuco_corners) >= args.target_samples:
                print('Target sample count reached. Type c + Enter to calculate intrinsics.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
