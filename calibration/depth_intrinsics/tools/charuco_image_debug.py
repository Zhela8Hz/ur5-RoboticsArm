#!/usr/bin/env python3
"""Grab RGB/IR ROS images and compare ChArUco/chessboard preprocessing variants."""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


PROJECT_ROOT = Path(__file__).resolve().parents[3]


class ImageGrabber(Node):
    def __init__(self, topics):
        super().__init__('charuco_image_debug')
        self.bridge = CvBridge()
        self.frames = {}
        self.topics = topics
        for name, topic in topics.items():
            self.create_subscription(
                Image,
                topic,
                lambda msg, key=name, topic_name=topic: self.on_image(key, topic_name, msg),
                qos_profile_sensor_data,
            )

    def on_image(self, key, topic, msg):
        if key in self.frames:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.frames[key] = {
            'topic': topic,
            'encoding': msg.encoding,
            'width': msg.width,
            'height': msg.height,
            'image': image,
        }


def to_gray(image):
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def percentile_norm(gray, low=1.0, high=99.0):
    if gray.dtype == np.uint8:
        return gray.copy()
    data = gray.astype(np.float32)
    vmin, vmax = np.percentile(data, low), np.percentile(data, high)
    if vmax <= vmin:
        return np.zeros(gray.shape, dtype=np.uint8)
    return np.clip((data - vmin) / (vmax - vmin) * 255, 0, 255).astype(np.uint8)


def minmax_norm(gray):
    if gray.dtype == np.uint8:
        return gray.copy()
    return cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


def make_detector_params():
    params = cv2.aruco.DetectorParameters_create()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 51
    params.adaptiveThreshWinSizeStep = 8
    params.minMarkerPerimeterRate = 0.005
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.03
    params.minCornerDistanceRate = 0.02
    params.minDistanceToBorder = 1
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    params.errorCorrectionRate = 0.9
    return params


def make_board(args):
    dictionary_id = getattr(cv2.aruco, args.dictionary_id)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x,
        args.squares_y,
        args.square_length_m,
        args.marker_length_m,
        dictionary,
    )
    board.setIds(np.arange(args.start_id, args.start_id + len(board.ids), dtype=np.int32))
    return dictionary, board


def detect_charuco(gray8, dictionary, board, detector_params):
    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray8,
        dictionary,
        parameters=detector_params,
    )
    marker_count = 0 if ids is None else len(ids)
    charuco_count = 0
    charuco_corners = None
    charuco_ids = None
    if ids is not None and marker_count > 0:
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners,
            ids,
            gray8,
            board,
        )
        charuco_count = int(count or 0)

    vis = cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
    if ids is not None and marker_count > 0:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    if charuco_ids is not None and charuco_count > 0:
        cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)
    return marker_count, charuco_count, len(rejected), vis


def detect_plain_chessboard(gray8, pattern_size):
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ret, corners = cv2.findChessboardCornersSB(gray8, pattern_size, flags=flags)
    vis = cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
    if ret:
        cv2.drawChessboardCorners(vis, pattern_size, corners, ret)
    return bool(ret), 0 if corners is None else len(corners), vis


def build_variants(gray):
    base = percentile_norm(gray, 1.0, 99.0)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe_base = clahe.apply(base)
    gamma = np.clip((base.astype(np.float32) / 255.0) ** 0.65 * 255, 0, 255).astype(np.uint8)
    blurred = cv2.GaussianBlur(base, (3, 3), 0)
    return [
        ('minmax', minmax_norm(gray)),
        ('p1_99', base),
        ('p0_5_99_5', percentile_norm(gray, 0.5, 99.5)),
        ('clahe', clahe_base),
        ('inv', cv2.bitwise_not(base)),
        ('clahe_inv', cv2.bitwise_not(clahe_base)),
        ('gamma065', gamma),
        ('blur_clahe', clahe.apply(blurred)),
    ]


def save_and_detect(name, frame, args, dictionary, board, detector_params):
    output_dir = args.output_dir
    image = frame['image']
    gray = to_gray(image)

    print(
        f'source={name} topic={frame["topic"]} encoding={frame["encoding"]} '
        f'width={frame["width"]} height={frame["height"]} dtype={image.dtype} '
        f'shape={image.shape} min={int(gray.min())} max={int(gray.max())} '
        f'mean={float(gray.mean()):.2f}',
        flush=True,
    )

    raw_path = output_dir / f'{args.tag}_raw_{name}.png'
    cv2.imwrite(str(raw_path), image)
    print(f'saved_raw={raw_path}', flush=True)

    best = None
    for variant_name, gray8 in build_variants(gray):
        norm_path = output_dir / f'{args.tag}_{name}_{variant_name}.png'
        cv2.imwrite(str(norm_path), gray8)

        markers, charuco_count, rejected, charuco_vis = detect_charuco(
            gray8,
            dictionary,
            board,
            detector_params,
        )
        charuco_path = output_dir / f'{args.tag}_{name}_{variant_name}_charuco_result.png'
        cv2.imwrite(str(charuco_path), charuco_vis)

        chess_ret, chess_count, chess_vis = detect_plain_chessboard(
            gray8,
            args.pattern_size,
        )
        chess_path = output_dir / f'{args.tag}_{name}_{variant_name}_chessboard_result.png'
        cv2.imwrite(str(chess_path), chess_vis)

        print(
            f'{name} variant={variant_name}: '
            f'aruco_markers={markers} charuco_corners={charuco_count} rejected={rejected} '
            f'chessboard_ret={chess_ret} chessboard_corners={chess_count}',
            flush=True,
        )
        score = (charuco_count, markers, chess_count if chess_ret else 0)
        if best is None or score > best[0]:
            best = (score, variant_name)

    print(f'{name} best_variant={best[1]} score={best[0]}', flush=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rgb-topic', default='/camera/color/image_raw')
    parser.add_argument('--ir-topic', default='/camera/ir/image_raw')
    parser.add_argument('--output-dir', type=Path, default=PROJECT_ROOT / 'calibration/depth_intrinsics/probes')
    parser.add_argument('--timeout-sec', type=float, default=8.0)
    parser.add_argument('--tag', default='debug')
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=6)
    parser.add_argument('--square-length-m', type=float, default=0.025)
    parser.add_argument('--marker-length-m', type=float, default=0.018)
    parser.add_argument('--dictionary-id', default='DICT_6X6_1000')
    parser.add_argument('--start-id', type=int, default=233)
    parser.add_argument('--pattern-size', type=int, nargs=2, default=(5, 5))
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.pattern_size = tuple(args.pattern_size)

    print(
        f'charuco board: squares_x={args.squares_x} squares_y={args.squares_y} '
        f'square_size={args.square_length_m} marker_size={args.marker_length_m} '
        f'dictionary={args.dictionary_id} start_id={args.start_id}',
        flush=True,
    )
    print(f'pattern_size for plain chessboard: {args.pattern_size}', flush=True)

    rclpy.init()
    node = ImageGrabber({'rgb': args.rgb_topic, 'ir': args.ir_topic})
    deadline = time.time() + args.timeout_sec
    while rclpy.ok() and time.time() < deadline and len(node.frames) < 2:
        rclpy.spin_once(node, timeout_sec=0.1)
    frames = dict(node.frames)
    node.destroy_node()
    rclpy.shutdown()

    dictionary, board = make_board(args)
    detector_params = make_detector_params()
    for name in ('rgb', 'ir'):
        if name not in frames:
            print(f'{name}: no frame received', flush=True)
            continue
        save_and_detect(name, frames[name], args, dictionary, board, detector_params)


if __name__ == '__main__':
    main()
