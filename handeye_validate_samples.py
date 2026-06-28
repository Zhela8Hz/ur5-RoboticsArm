#!/usr/bin/env python3
"""Validate a fixed ChArUco board pose using solved eye-in-hand extrinsics."""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


def as_matrix(value):
    matrix = np.array(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f'expected 4x4 matrix, got {matrix.shape}')
    return matrix


def load_samples(path):
    samples = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if line.strip():
            samples.append(json.loads(line))
    return samples


def rot_angle_deg(r_a, r_b):
    delta = r_a.T @ r_b
    value = max(-1.0, min(1.0, (np.trace(delta) - 1.0) / 2.0))
    return float(np.degrees(np.arccos(value)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', default='/home/z/Apps-my/handeye_samples/samples.jsonl')
    parser.add_argument('--result', default='/home/z/Apps-my/handeye_samples/handeye_result.yaml')
    parser.add_argument('--output', default='/home/z/Apps-my/handeye_samples/base_target_validation.csv')
    args = parser.parse_args()

    samples = load_samples(args.samples)
    result = yaml.safe_load(Path(args.result).read_text(encoding='utf-8'))
    t_tool_camera = as_matrix(result['transform']['matrix'])

    poses = []
    for sample in samples:
        t_base_tool = as_matrix(sample['T_base_tool'])
        t_camera_target = as_matrix(sample['T_camera_target'])
        poses.append((sample, t_base_tool @ t_tool_camera @ t_camera_target))

    positions = np.array([pose[:3, 3] for _, pose in poses])
    mean_position = positions.mean(axis=0)
    distances = np.linalg.norm(positions - mean_position, axis=1)
    reference_rotation = poses[0][1][:3, :3]
    rotation_errors = np.array([rot_angle_deg(reference_rotation, pose[:3, :3]) for _, pose in poses])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='utf-8') as f:
        f.write('sample_id,charuco_corners,x_m,y_m,z_m,translation_error_m,rotation_error_deg,overlay_image\n')
        for (sample, pose), dist, rot in zip(poses, distances, rotation_errors):
            f.write(
                f"{sample['sample_id']},{sample.get('charuco_corners', '')},"
                f"{pose[0,3]:.9f},{pose[1,3]:.9f},{pose[2,3]:.9f},"
                f"{dist:.9f},{rot:.9f},{sample.get('overlay_image', '')}\n"
            )

    print(f'Saved: {output}')
    print(f'Samples: {len(samples)}')
    print('Estimated fixed board position in base frame, mean xyz [m]:')
    print(np.array2string(mean_position, precision=8, suppress_small=False))
    print('Translation scatter around mean [m]:')
    print(f'  mean: {float(distances.mean()):.6f}')
    print(f'  max : {float(distances.max()):.6f}')
    print('Rotation scatter relative to first sample [deg]:')
    print(f'  mean: {float(rotation_errors.mean()):.6f}')
    print(f'  max : {float(rotation_errors.max()):.6f}')
    worst = sorted(zip(distances, rotation_errors, poses), reverse=True)[:5]
    print('Worst samples by translation scatter:')
    for dist, rot, (sample, _) in worst:
        print(
            f"  sample_{sample['sample_id']:03d}: "
            f"trans={float(dist):.6f} m, rot={float(rot):.6f} deg, "
            f"corners={sample.get('charuco_corners', '')}"
        )


if __name__ == '__main__':
    main()
