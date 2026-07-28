#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def matrix_from_pose(pose):
    transform = np.eye(4, dtype=np.float64)
    translation = pose['translation_m']
    quaternion = pose['quaternion_xyzw']
    quaternion_array = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']], dtype=np.float64)
    rotation, _ = cv2.Rodrigues(np.zeros((3, 1)))
    x, y, z, w = quaternion_array / np.linalg.norm(quaternion_array)
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    transform[:3, :3] = rotation
    transform[:3, 3] = [translation['x'], translation['y'], translation['z']]
    return transform


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--intrinsics', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='Output JSONL path; defaults to <session>/samples.jsonl.')
    args = parser.parse_args()
    config = yaml.safe_load((args.session / 'config_used.yaml').read_text())
    poses = yaml.safe_load((args.session / 'poses.yaml').read_text())['records']
    intrinsics = yaml.safe_load(args.intrinsics.read_text())
    camera = np.array(intrinsics['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    distortion = np.array(intrinsics['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
    charuco = config['charuco']
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, charuco['dictionary_id']))
    board = cv2.aruco.CharucoBoard_create(charuco['squares_x'], charuco['squares_y'], charuco['square_length_m'], charuco['marker_length_m'], dictionary)
    board.setIds(np.arange(charuco['start_id'], charuco['start_id'] + len(board.ids), dtype=np.int32))
    output = args.output or args.session / 'samples.jsonl'
    valid = 0
    with output.open('w', encoding='utf-8') as stream:
        for record in poses:
            image = cv2.imread(str(args.session / record['image_filename']))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            board_ids = set(int(marker_id) for marker_id in board.ids.flatten())
            detected_ids = [] if ids is None else [int(marker_id) for marker_id in ids.flatten()]
            target_ids = [marker_id for marker_id in detected_ids if marker_id in board_ids]
            duplicate_target_ids = sorted({marker_id for marker_id in target_ids if target_ids.count(marker_id) > 1})
            if duplicate_target_ids:
                print(f"skip pose_id={record['pose_id']}: duplicate target marker IDs {duplicate_target_ids}")
                continue
            count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board, camera, distortion)
            if charuco_ids is None or int(count or 0) < charuco['min_corners']:
                continue
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, board, camera, distortion, None, None)
            if not ok:
                continue
            target = np.eye(4, dtype=np.float64)
            target[:3, :3], _ = cv2.Rodrigues(rvec)
            target[:3, 3] = np.asarray(tvec).reshape(3)
            stream.write(json.dumps({'sample_id': record['pose_id'], 'tool_frame': 'tool0', 'camera_frame': 'camera_color_optical_frame', 'charuco_corners': int(count), 'T_base_tool': matrix_from_pose(record['T_base_tool']).tolist(), 'T_camera_target': target.tolist(), 'raw_image': record['image_filename']}) + '\n')
            valid += 1
    print(f'valid_samples={valid} output={output}')


if __name__ == '__main__':
    main()
