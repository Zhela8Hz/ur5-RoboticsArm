#!/usr/bin/env python3
"""Validate camera intrinsics with a ChArUco board."""

import argparse
import math
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


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


def board_corners(board):
    corners = getattr(board, 'chessboardCorners', None)
    if corners is None and hasattr(board, 'getChessboardCorners'):
        corners = board.getChessboardCorners()
    if corners is None:
        raise RuntimeError('OpenCV CharucoBoard does not expose chessboard corners')
    return np.asarray(corners, dtype=np.float32)


def load_intrinsics_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    dist = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
    return camera_matrix, dist


class SdkIntrinsicsValidator(Node):
    def __init__(self, args):
        super().__init__('validate_sdk_intrinsics_charuco')
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image = None
        if args.intrinsics_yaml:
            self.camera_matrix, self.dist_coeffs = load_intrinsics_yaml(args.intrinsics_yaml)
        else:
            self.camera_matrix = None
            self.dist_coeffs = None
        self.dictionary, self.board = make_charuco_board(
            args.squares_x,
            args.squares_y,
            args.square_length_m,
            args.marker_length_m,
            args.dictionary_id,
            args.start_id,
        )
        self.object_corners = board_corners(self.board)
        self.detector_params = make_detector_params()
        self.create_subscription(Image, args.image_topic, self.on_image, 10)
        if not args.intrinsics_yaml:
            self.create_subscription(CameraInfo, args.camera_info_topic, self.on_camera_info, 10)

    def on_camera_info(self, msg):
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        with self.lock:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return
        if image.ndim == 2:
            gray = normalize_gray8(image)
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        with self.lock:
            self.latest_image = gray

    def evaluate_latest(self):
        with self.lock:
            gray = None if self.latest_image is None else self.latest_image.copy()
            camera_matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
            dist_coeffs = None if self.dist_coeffs is None else self.dist_coeffs.copy()

        if gray is None:
            return None, 'waiting for image'
        if camera_matrix is None or dist_coeffs is None:
            return None, 'waiting for camera_info'

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)
        marker_count = 0 if ids is None else len(ids)
        if ids is None or marker_count == 0:
            return None, 'no ArUco markers detected'

        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board, camera_matrix, dist_coeffs
        )
        count = int(count or 0)
        if charuco_ids is None or count < self.args.min_charuco_corners:
            return None, f'only {count} ChArUco corners; need {self.args.min_charuco_corners}'

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.board, camera_matrix, dist_coeffs, None, None
        )
        if not ok:
            return None, 'pose estimation failed'

        object_points = self.object_corners[charuco_ids.flatten()].reshape(-1, 1, 3)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        residuals = projected.reshape(-1, 2) - charuco_corners.reshape(-1, 2)
        per_corner = np.linalg.norm(residuals, axis=1)
        rmse = math.sqrt(float(np.mean(np.sum(residuals * residuals, axis=1))))
        result = {
            'markers': marker_count,
            'charuco': count,
            'rmse_px': rmse,
            'mean_px': float(np.mean(per_corner)),
            'max_px': float(np.max(per_corner)),
            'image_width': int(gray.shape[1]),
            'image_height': int(gray.shape[0]),
        }
        return result, 'ok'


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/color/camera_info')
    parser.add_argument('--intrinsics-yaml', default='')
    parser.add_argument('--samples', type=int, default=10)
    parser.add_argument('--timeout-sec', type=float, default=60.0)
    parser.add_argument('--sample-interval-sec', type=float, default=0.7)
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=6)
    parser.add_argument('--square-length-m', type=float, default=0.025)
    parser.add_argument('--marker-length-m', type=float, default=0.018)
    parser.add_argument('--dictionary-id', default='DICT_6X6_1000')
    parser.add_argument('--start-id', type=int, default=233)
    parser.add_argument('--min-charuco-corners', type=int, default=20)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = SdkIntrinsicsValidator(args)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    valid = []
    last_sample = 0.0
    deadline = time.monotonic() + args.timeout_sec
    print('Move the ChArUco board through the camera view until enough samples are collected.', flush=True)
    try:
        while len(valid) < args.samples and time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_sample < args.sample_interval_sec:
                time.sleep(0.05)
                continue
            last_sample = now
            result, status = node.evaluate_latest()
            if result is None:
                print(f'WAIT {status}', flush=True)
                continue
            valid.append(result)
            print(
                f'OK {len(valid):02d}/{args.samples}: '
                f'{result["image_width"]}x{result["image_height"]} '
                f'markers={result["markers"]} charuco={result["charuco"]} '
                f'rmse={result["rmse_px"]:.3f}px mean={result["mean_px"]:.3f}px max={result["max_px"]:.3f}px',
                flush=True,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)

    if not valid:
        raise SystemExit('No valid ChArUco samples collected.')

    rmses = np.array([item['rmse_px'] for item in valid], dtype=np.float64)
    means = np.array([item['mean_px'] for item in valid], dtype=np.float64)
    maxes = np.array([item['max_px'] for item in valid], dtype=np.float64)
    print('Summary:')
    print(f'  valid_samples: {len(valid)}')
    print(f'  rmse_px_mean: {float(np.mean(rmses)):.3f}')
    print(f'  rmse_px_median: {float(np.median(rmses)):.3f}')
    print(f'  rmse_px_max: {float(np.max(rmses)):.3f}')
    print(f'  mean_px_mean: {float(np.mean(means)):.3f}')
    print(f'  max_px_max: {float(np.max(maxes)):.3f}')


if __name__ == '__main__':
    main()
