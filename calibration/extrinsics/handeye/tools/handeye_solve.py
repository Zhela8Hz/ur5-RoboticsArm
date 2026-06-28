#!/usr/bin/env python3
"""Solve eye-in-hand extrinsics from handeye_capture.py JSONL samples."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[4]


METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def load_samples(path):
    samples = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if line.strip():
            samples.append(json.loads(line))
    return samples


def as_matrix(value):
    matrix = np.array(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f'expected 4x4 matrix, got {matrix.shape}')
    return matrix


def invert_transform(matrix):
    inv = np.eye(4, dtype=np.float64)
    inv[:3, :3] = matrix[:3, :3].T
    inv[:3, 3] = -inv[:3, :3] @ matrix[:3, 3]
    return inv


def rot_angle_deg(matrix):
    trace = np.trace(matrix[:3, :3])
    value = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return float(np.degrees(np.arccos(value)))


def translation_norm(matrix):
    return float(np.linalg.norm(matrix[:3, 3]))


def solve(samples, method):
    r_gripper2base = []
    t_gripper2base = []
    r_target2cam = []
    t_target2cam = []
    for sample in samples:
        t_base_tool = as_matrix(sample['T_base_tool'])
        t_camera_target = as_matrix(sample['T_camera_target'])
        r_gripper2base.append(t_base_tool[:3, :3])
        t_gripper2base.append(t_base_tool[:3, 3].reshape(3, 1))
        r_target2cam.append(t_camera_target[:3, :3])
        t_target2cam.append(t_camera_target[:3, 3].reshape(3, 1))

    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=method,
    )
    t_tool_camera = np.eye(4, dtype=np.float64)
    t_tool_camera[:3, :3] = r_cam2gripper
    t_tool_camera[:3, 3] = np.asarray(t_cam2gripper, dtype=np.float64).reshape(3)
    return t_tool_camera


def estimate_residuals(samples, t_tool_camera):
    # For a fixed board, T_base_target_i = T_base_tool_i * T_tool_camera * T_camera_target_i
    # should be nearly constant across all samples.
    base_targets = []
    for sample in samples:
        t_base_tool = as_matrix(sample['T_base_tool'])
        t_camera_target = as_matrix(sample['T_camera_target'])
        base_targets.append(t_base_tool @ t_tool_camera @ t_camera_target)
    reference = base_targets[0]
    trans = []
    rot = []
    for transform in base_targets[1:]:
        delta = invert_transform(reference) @ transform
        trans.append(translation_norm(delta))
        rot.append(rot_angle_deg(delta))
    if not trans:
        return {'translation_m_mean': 0.0, 'translation_m_max': 0.0, 'rotation_deg_mean': 0.0, 'rotation_deg_max': 0.0}
    return {
        'translation_m_mean': float(np.mean(trans)),
        'translation_m_max': float(np.max(trans)),
        'rotation_deg_mean': float(np.mean(rot)),
        'rotation_deg_max': float(np.max(rot)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', default=str(PROJECT_ROOT / 'calibration/extrinsics/handeye/sessions/handeye_samples/samples.jsonl'))
    parser.add_argument('--output', default=str(PROJECT_ROOT / 'calibration/extrinsics/handeye/sessions/handeye_samples/handeye_result.yaml'))
    parser.add_argument('--method', choices=sorted(METHODS), default='tsai')
    args = parser.parse_args()

    samples = load_samples(args.samples)
    if len(samples) < 8:
        raise SystemExit(f'Need at least 8 samples; got {len(samples)}. Prefer 15-25.')

    t_tool_camera = solve(samples, METHODS[args.method])
    residuals = estimate_residuals(samples, t_tool_camera)
    result = {
        'calibration_type': 'eye_in_hand',
        'method': args.method,
        'sample_count': len(samples),
        'transform': {
            'parent_frame': samples[0].get('tool_frame', 'tool0'),
            'child_frame': samples[0].get('camera_frame', 'camera_color_optical_frame'),
            'name': 'T_tool_camera',
            'matrix': [[float(v) for v in row] for row in t_tool_camera],
            'translation_xyz_m': [float(v) for v in t_tool_camera[:3, 3]],
            'rotation_matrix': [[float(v) for v in row] for row in t_tool_camera[:3, :3]],
        },
        'fixed_board_consistency': residuals,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(result, sort_keys=False), encoding='utf-8')

    print(f'Saved: {output}')
    print('T_tool_camera:')
    print(np.array2string(t_tool_camera, precision=8, suppress_small=False))
    print('Fixed-board consistency:')
    for key, value in residuals.items():
        print(f'  {key}: {value:.6f}')


if __name__ == '__main__':
    main()
