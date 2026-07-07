#!/usr/bin/env python3
"""Capture one IR frame and report ChArUco detection statistics."""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


PROJECT_ROOT = Path(__file__).resolve().parents[3]


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


class Probe(Node):
    def __init__(self, args):
        super().__init__('ir_charuco_probe')
        self.args = args
        self.bridge = CvBridge()
        self.image = None
        self.create_subscription(Image, args.image_topic, self.on_image, 10)

    def on_image(self, msg):
        if self.image is None:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def to_gray8(image):
    if image.ndim == 2:
        gray = image
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if gray.dtype == np.uint8:
        return gray
    data = gray.astype(np.float32)
    vmin, vmax = np.percentile(data, 1), np.percentile(data, 99)
    if vmax <= vmin:
        return np.zeros(gray.shape, dtype=np.uint8)
    return np.clip((data - vmin) / (vmax - vmin) * 255, 0, 255).astype(np.uint8)


def detect(gray, args):
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

    clahe = cv2.createCLAHE(clipLimit=args.clahe_clip, tileGridSize=(8, 8))
    variants = [
        ('raw', gray),
        ('clahe', clahe.apply(gray)),
        ('inv', cv2.bitwise_not(gray)),
        ('clahe_inv', cv2.bitwise_not(clahe.apply(gray))),
    ]
    detector_params = make_detector_params()
    best = None
    rows = []
    for name, image in variants:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            image, dictionary, parameters=detector_params
        )
        marker_count = 0 if ids is None else len(ids)
        charuco_count = 0
        if ids is not None and marker_count > 0:
            count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, image, board
            )
            charuco_count = int(count or 0)
        ids_list = [] if ids is None else ids.flatten().tolist()
        row = (name, image, corners, ids, marker_count, charuco_count, ids_list, len(rejected))
        rows.append(row)
        if best is None or (charuco_count, marker_count) > (best[5], best[4]):
            best = row
    return rows, best


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/camera/ir/image_raw')
    parser.add_argument('--output-dir', type=Path, default=PROJECT_ROOT / 'calibration/depth_intrinsics/probes')
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=6)
    parser.add_argument('--square-length-m', type=float, default=0.025)
    parser.add_argument('--marker-length-m', type=float, default=0.018)
    parser.add_argument('--dictionary-id', default='DICT_6X6_1000')
    parser.add_argument('--start-id', type=int, default=233)
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    parser.add_argument('--tag', default='probe')
    parser.add_argument('--clahe-clip', type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Probe(args)
    end_time = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while rclpy.ok() and node.image is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > end_time:
            print('No image received.', flush=True)
            node.destroy_node()
            rclpy.shutdown()
            return 2

    raw = node.image
    gray = to_gray8(raw)
    rows, best = detect(gray, args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / f'{args.tag}_raw.png'
    overlay_path = args.output_dir / f'{args.tag}_overlay.png'
    cv2.imwrite(str(raw_path), raw)

    name, image, corners, ids, marker_count, charuco_count, ids_list, rejected = best
    overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if ids is not None and marker_count > 0:
        cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
    cv2.imwrite(str(overlay_path), overlay)

    print(f'image_shape={gray.shape} dtype={raw.dtype} min={int(gray.min())} max={int(gray.max())} mean={float(gray.mean()):.1f}')
    for row in rows:
        print(
            f'variant={row[0]} markers={row[4]} charuco={row[5]} '
            f'ids={row[6]} rejected={row[7]}'
        )
    print(f'best={name} markers={marker_count} charuco={charuco_count}')
    print(f'saved_raw={raw_path}')
    print(f'saved_overlay={overlay_path}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
