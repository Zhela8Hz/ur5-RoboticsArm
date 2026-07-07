#!/usr/bin/env python3
"""Live ChArUco detection check from a ROS image topic."""

import argparse
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def load_intrinsics(path):
    if not path:
        return None, None
    intrinsics_path = Path(path)
    if not intrinsics_path.exists():
        raise FileNotFoundError(f'Intrinsics file not found: {intrinsics_path}')
    data = yaml.safe_load(intrinsics_path.read_text(encoding='utf-8'))
    camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
    return camera_matrix, dist_coeffs


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


class CharucoLiveCheck(Node):
    def __init__(self, args):
        super().__init__('charuco_live_check')
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.frame_index = 0
        self.use_clahe = args.clahe
        self.invert = args.invert
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.detector_params = make_detector_params()
        self.camera_matrix, self.dist_coeffs = load_intrinsics(args.intrinsics)
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

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return
        if image.ndim == 2:
            gray = normalize_gray8(image)
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            display = image.copy()
            gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
        with self.lock:
            self.latest = (gray, display)

    def preprocess(self, gray):
        processed = gray
        if self.use_clahe:
            processed = self.clahe.apply(processed)
        if self.invert:
            processed = cv2.bitwise_not(processed)
        return processed

    def render_latest(self):
        with self.lock:
            latest = self.latest
        if latest is None:
            return None, 0, 0, False

        gray, _ = latest
        processed = self.preprocess(gray)
        overlay = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)
        corners, ids, _ = cv2.aruco.detectMarkers(
            processed, self.dictionary, parameters=self.detector_params
        )
        marker_count = 0 if ids is None else len(ids)
        charuco_count = 0
        pose_ok = False

        if ids is not None and marker_count > 0:
            cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
            count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, processed, self.board, self.camera_matrix, self.dist_coeffs
            )
            charuco_count = int(count or 0)
            if charuco_ids is not None and charuco_count > 0:
                cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids)
            if (
                charuco_ids is not None
                and charuco_count >= self.args.min_charuco_corners
                and self.camera_matrix is not None
                and self.dist_coeffs is not None
            ):
                pose_ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners,
                    charuco_ids,
                    self.board,
                    self.camera_matrix,
                    self.dist_coeffs,
                    None,
                    None,
                )
                if pose_ok:
                    cv2.drawFrameAxes(
                        overlay,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        self.args.axis_length_m,
                    )

        status = 'GOOD' if charuco_count >= self.args.min_charuco_corners else 'LOW'
        color = (0, 180, 0) if status == 'GOOD' else (0, 0, 255)
        text = (
            f'markers={marker_count} charuco={charuco_count} status={status} '
            f'pose={int(pose_ok)} clahe={int(self.use_clahe)} inv={int(self.invert)}'
        )
        cv2.rectangle(overlay, (8, 8), (620, 42), (255, 255, 255), -1)
        cv2.putText(overlay, text, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        return overlay, marker_count, charuco_count, pose_ok

    def save_overlay(self, overlay):
        self.args.output_dir.mkdir(parents=True, exist_ok=True)
        self.frame_index += 1
        path = self.args.output_dir / f'charuco_live_check_{self.frame_index:03d}.png'
        cv2.imwrite(str(path), overlay)
        print(f'Saved {path}', flush=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--intrinsics', default=str(PROJECT_ROOT / 'calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml'))
    parser.add_argument('--output-dir', type=Path, default=PROJECT_ROOT / 'calibration/rgb_intrinsics/live_check')
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=6)
    parser.add_argument('--square-length-m', type=float, default=0.025)
    parser.add_argument('--marker-length-m', type=float, default=0.018)
    parser.add_argument('--dictionary-id', default='DICT_6X6_1000')
    parser.add_argument('--start-id', type=int, default=233)
    parser.add_argument('--min-charuco-corners', type=int, default=20)
    parser.add_argument('--axis-length-m', type=float, default=0.05)
    parser.add_argument('--clahe', action='store_true')
    parser.add_argument('--invert', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CharucoLiveCheck(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('Press s to save overlay, c to toggle CLAHE, i to toggle invert, q to quit.', flush=True)
    try:
        while rclpy.ok():
            overlay, _, _, _ = node.render_latest()
            if overlay is None:
                overlay = np.full((360, 640, 3), 255, dtype=np.uint8)
                cv2.putText(overlay, 'Waiting for image...', (30, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
            cv2.imshow('charuco_live_check', overlay)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                node.save_overlay(overlay)
            if key == ord('c'):
                node.use_clahe = not node.use_clahe
                print(f'CLAHE={int(node.use_clahe)}', flush=True)
            if key == ord('i'):
                node.invert = not node.invert
                print(f'invert={int(node.invert)}', flush=True)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    sys.exit(main())
