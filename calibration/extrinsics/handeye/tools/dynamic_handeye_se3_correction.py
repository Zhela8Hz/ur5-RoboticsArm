#!/usr/bin/env python3
"""Live fixed-board SE(3) hand-eye correction advisor (virtual only).

X0 is T_tool_camera.  The sole correction convention is
X_current = DeltaX_tool @ X0.  DeltaX is left-multiplied and expressed in the
tool frame.  The program never publishes a TF, commands a robot, or writes X0.
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

from dynamic_handeye_se3_core import (evaluate_correction_rms, inverse, matrix_record, pose_distance,
                                      rotvec_shortest, solve_correction, transform, validate_transform)

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = ROOT / 'calibration/extrinsics/handeye/dynamic_handeye_se3_correction.yaml'


def resolve_paths(config):
    for key in ('intrinsics', 'original_handeye', 'output_root', 'self_test_report'):
        value = Path(config['paths'][key])
        config['paths'][key] = str(value if value.is_absolute() else ROOT / value)
    return config


def load_config(path):
    config = resolve_paths(yaml.safe_load(Path(path).read_text(encoding='utf-8')))
    if config['apply'].get('publish_tf') or config['apply'].get('overwrite_original_handeye'):
        raise ValueError('This program intentionally forbids TF publication and overwriting X0.')
    injection = config['injection']
    if injection['frame'] != 'tool' or injection['multiplication'] != 'left':
        raise ValueError('Only a tool-frame left-multiplied injection is supported by this program.')
    live = config.setdefault('live', {})
    live.setdefault('image_queue_depth', 1)
    live.setdefault('max_process_rate_hz', 5.0)
    live.setdefault('max_status_rate_hz', 2.0)
    live.setdefault('max_ros_spin_rate_hz', 100.0)
    live.setdefault('max_tf_wait_sec', 1.5)
    live.setdefault('startup_tf_timeout_sec', 5.0)
    if int(live['image_queue_depth']) < 1:
        raise ValueError('live.image_queue_depth must be at least 1.')
    if float(live['max_process_rate_hz']) <= 0.0:
        raise ValueError('live.max_process_rate_hz must be positive.')
    if float(live['max_status_rate_hz']) <= 0.0:
        raise ValueError('live.max_status_rate_hz must be positive.')
    if float(live['max_ros_spin_rate_hz']) <= 0.0:
        raise ValueError('live.max_ros_spin_rate_hz must be positive.')
    if float(live['max_tf_wait_sec']) <= 0.0:
        raise ValueError('live.max_tf_wait_sec must be positive.')
    if float(live['startup_tf_timeout_sec']) <= 0.0:
        raise ValueError('live.startup_tf_timeout_sec must be positive.')
    validation = config.setdefault('validation', {})
    validation.setdefault('holdout_every_n_samples', 4)
    validation.setdefault('min_holdout_samples', 4)
    validation.setdefault('max_condition_number', 500.0)
    validation.setdefault('max_holdout_rms_m', 0.001)
    validation.setdefault('max_after_before_ratio', 0.8)
    validation.setdefault('max_update_translation_m', 0.001)
    validation.setdefault('max_update_rotation_deg', 0.2)
    validation.setdefault('required_stable_updates', 2)
    if int(validation['holdout_every_n_samples']) < 2:
        raise ValueError('validation.holdout_every_n_samples must be at least 2.')
    if int(validation['min_holdout_samples']) < 1:
        raise ValueError('validation.min_holdout_samples must be at least 1.')
    if int(validation['required_stable_updates']) < 1:
        raise ValueError('validation.required_stable_updates must be at least 1.')
    return config


def apply_no_injection_override(config):
    """Disable virtual error injection for this run without editing the YAML."""
    injection = config['injection']
    injection['enabled'] = False
    injection['translation_mm'] = [0.0, 0.0, 0.0]
    injection['rotation_rotvec_deg'] = [0.0, 0.0, 0.0]
    return config


def load_intrinsics(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return (np.asarray(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3),
            np.asarray(data['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1))


def load_x0(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    x0 = np.asarray(data['transform']['matrix'], dtype=np.float64)
    if data['transform'].get('parent_frame') != 'tool0' or not validate_transform(x0):
        raise ValueError('Original hand-eye must be a valid T_tool_camera matrix.')
    return x0


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def matrix_from_components(translation, quaternion_xyzw):
    quat = np.asarray(quaternion_xyzw, dtype=np.float64)
    if not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-12:
        raise ValueError('invalid TF quaternion')
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rotation.from_quat(quat / np.linalg.norm(quat)).as_matrix()
    out[:3, 3] = translation
    return out


def interpolate_tf(before, after, image_ns):
    """Linear translation + quaternion SLERP, never a latest-TF fallback."""
    before_ns, before_pose = before
    after_ns, after_pose = after
    if after_ns < before_ns or image_ns < before_ns or image_ns > after_ns:
        raise ValueError('TF bracket does not cover image timestamp')
    ratio = 0.0 if after_ns == before_ns else (image_ns - before_ns) / (after_ns - before_ns)
    position = (1.0 - ratio) * before_pose[:3, 3] + ratio * after_pose[:3, 3]
    rotations = Rotation.from_matrix(np.stack((before_pose[:3, :3], after_pose[:3, :3])))
    rotation = Slerp([0.0, 1.0], rotations)([ratio]).as_matrix()[0]
    out = np.eye(4, dtype=np.float64); out[:3, :3] = rotation; out[:3, 3] = position
    return out, float(ratio), max(image_ns - before_ns, after_ns - image_ns) / 1e9


def make_board(config):
    spec = config['board']
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec['dictionary_id']))
    board = cv2.aruco.CharucoBoard_create(spec['squares_x'], spec['squares_y'], spec['square_length_m'], spec['marker_length_m'], dictionary)
    board.setIds(np.arange(spec['start_id'], spec['start_id'] + len(board.ids), dtype=np.int32))
    return dictionary, board


def annotation(image, corners, ids, charuco_corners=None, charuco_ids=None):
    output = image.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(output, corners, ids)
    if charuco_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(output, charuco_corners, charuco_ids)
    return output


class DynamicCorrectionNode:
    """ROS-bound live collector; imports are delayed so --self-test is offline."""
    def __init__(self, config):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.time import Time
        from sensor_msgs.msg import Image
        from tf2_ros import Buffer, TransformException, TransformListener

        class _Node(Node):
            pass
        self.rclpy, self.node, self.config = rclpy, _Node('dynamic_handeye_se3_correction'), config
        self.bridge = CvBridge(); self.image_queue = deque(maxlen=int(config['live']['image_queue_depth'])); self.last_camera_image = None; self.last_overlay = None; self.pending_frame = None
        self.last_synced = None; self.valid = []; self.training_samples = []; self.holdout_samples = []
        self.last_result = None; self.last_candidate_result = None; self.last_validation = None
        self.stable_update_count = 0; self.correction_available = False; self.frame_counter = 0; self.last_record = None; self.last_corner_count = None; self.last_runtime_status = 'waiting for image'
        self.dropped_queue_frames = 0; self.processed_frames = 0; self.last_process_monotonic = 0.0
        self.dictionary, self.board = make_board(config)
        self.k, self.d = load_intrinsics(config['paths']['intrinsics']); self.x0 = load_x0(config['paths']['original_handeye'])
        inj = config['injection']; self.injection = transform(np.asarray(inj['translation_mm']) / 1000.0, np.radians(inj['rotation_rotvec_deg']))
        self.x_input = self.injection @ self.x0 if inj['enabled'] else self.x0.copy()
        self.session = None
        self.log = logging.getLogger(f'dynamic_se3_{id(self)}'); self.log.setLevel(logging.INFO)
        self.records = None
        image_qos = QoSProfile(depth=int(config['live']['image_queue_depth']), reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.node.create_subscription(Image, config['interfaces']['image_topic'], self.on_image, image_qos)
        # Match the established hand-eye tools: tf2 resolves the full UR link
        # chain, even when base->tool0 is not a single raw /tf message.
        self._Time = Time
        self._Duration = Duration
        self._TransformException = TransformException
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

    def close(self):
        if self.records is not None:
            self.records.close()
        self.node.destroy_node()

    def start_session(self):
        """Create persistent output only after the TF preflight has passed."""
        if self.session is not None:
            return
        self.session = Path(self.config['paths']['output_root']) / datetime.now(timezone.utc).strftime('dynamic_se3_%Y%m%d_%H%M%S')
        self.session.mkdir(parents=True, exist_ok=False); (self.session / 'images').mkdir()
        (self.session / 'results').mkdir(); yaml.safe_dump(self.config, (self.session / 'config_used.yaml').open('w', encoding='utf-8'), sort_keys=False)
        handler = logging.FileHandler(self.session / 'run.log', encoding='utf-8')
        handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s')); self.log.addHandler(handler)
        self.records = (self.session / 'frames.jsonl').open('a', encoding='utf-8')
        self.log.info('session started; process_rate_hz=%s status_rate_hz=%s ros_spin_rate_hz=%s image_queue_depth=%s',
                      self.config['live']['max_process_rate_hz'], self.config['live']['max_status_rate_hz'],
                      self.config['live']['max_ros_spin_rate_hz'], self.config['live']['image_queue_depth'])

    def on_image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            if len(self.image_queue) == self.image_queue.maxlen:
                self.dropped_queue_frames += 1
            self.last_camera_image = image
            self.image_queue.append((image, stamp_ns(message.header.stamp)))
        except Exception as exc:
            self.log.warning('image conversion rejected: %s', exc)

    def has_live_tf(self):
        """Use the same latest-TF preflight as the proven hand-eye tools."""
        try:
            self.tf_buffer.lookup_transform(self.config['interfaces']['base_frame'],
                                            self.config['interfaces']['tool_frame'],
                                            self._Time(), timeout=self._Duration(seconds=0.0))
            return True
        except self._TransformException:
            return False

    def startup_status(self):
        return (f'Waiting for TF {self.config["interfaces"]["base_frame"]} -> '
                f'{self.config["interfaces"]["tool_frame"]}')

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        overlay = annotation(image, corners, ids)
        result = {'image_quality_ok': False, 'charuco_ok': False, 'corner_count': 0, 'ids': [], 'reprojection_rms_px': None, 'blur_metric': float(cv2.Laplacian(gray, cv2.CV_64F).var()), 'edge_margin_px': None, 'reason': ''}
        if ids is None or len(ids) == 0:
            result['reason'] = 'no marker ids'; return overlay, result, None
        marker_ids = [int(x) for x in ids.reshape(-1)]
        valid_ids = set(range(self.config['board']['start_id'], self.config['board']['start_id'] + len(self.board.ids)))
        if len(marker_ids) != len(set(marker_ids)) or any(i not in valid_ids for i in marker_ids):
            result['reason'] = 'marker IDs are duplicate or outside configured board'; return overlay, result, None
        count, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board, self.k, self.d)
        count = int(count or 0); result['corner_count'] = count; result['ids'] = [] if ci is None else [int(x) for x in ci.reshape(-1)]
        overlay = annotation(image, corners, ids, cc, ci)
        valid_corner_ids = set(range(len(self.board.chessboardCorners)))
        if ci is None or count < self.config['quality']['min_charuco_corners'] or len(set(result['ids'])) != count or any(i not in valid_corner_ids for i in result['ids']):
            result['reason'] = 'insufficient or non-unique ChArUco corners'; return overlay, result, None
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(cc, ci, self.board, self.k, self.d, None, None)
        if not ok:
            result['reason'] = 'camera->board pose failed'; return overlay, result, None
        object_points = np.asarray([self.board.chessboardCorners[int(i)] for i in ci.reshape(-1)], dtype=np.float64)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.k, self.d)
        result['reprojection_rms_px'] = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - cc.reshape(-1, 2)) ** 2, axis=1))))
        xs, ys = cc.reshape(-1, 2).T; h, w = gray.shape
        result['edge_margin_px'] = float(min(xs.min(), ys.min(), w - 1 - xs.max(), h - 1 - ys.max()))
        if result['blur_metric'] < self.config['quality']['min_laplacian_variance']:
            result['reason'] = 'blur threshold'; return overlay, result, None
        if result['edge_margin_px'] < self.config['quality']['min_edge_margin_px']:
            result['reason'] = 'board near image edge'; return overlay, result, None
        if result['reprojection_rms_px'] > self.config['quality']['max_reprojection_rms_px']:
            result['reason'] = 'reprojection RMS threshold'; return overlay, result, None
        pose = np.eye(4); pose[:3, :3], _ = cv2.Rodrigues(rvec); pose[:3, 3] = tvec.reshape(3)
        result['image_quality_ok'] = result['charuco_ok'] = True; result['reason'] = 'image quality accepted'
        cv2.drawFrameAxes(overlay, self.k, self.d, rvec, tvec, 0.05)
        return overlay, result, pose

    def motion_quality(self, pose, image_ns):
        if self.last_synced is None:
            self.last_synced = (pose, image_ns, None, None, None); return True, {'linear_m_s': 0.0, 'angular_rad_s': 0.0, 'sync_translation_error_m': 0.0, 'sync_rotation_error_rad': 0.0}, ''
        previous, previous_ns, previous_linear, previous_angular, previous_direction = self.last_synced
        dt = (image_ns - previous_ns) / 1e9
        if dt <= 0.0:
            return False, {}, 'non-monotonic image timestamp'
        translation, angle = pose_distance(previous, pose); linear, angular = translation / dt, angle / dt
        reason = ''
        limit = self.config['motion']
        if linear > limit['max_tcp_linear_speed_m_s'] or angular > math.radians(limit['max_tcp_angular_speed_deg_s']): reason = 'TCP speed limit'
        if previous_linear is not None and abs(linear - previous_linear) / dt > limit['max_linear_acceleration_m_s2']: reason = 'linear acceleration limit'
        if previous_angular is not None and abs(angular - previous_angular) / dt > math.radians(limit['max_angular_acceleration_deg_s2']): reason = 'angular acceleration limit'
        # At rest, sub-millimetre TF noise has an arbitrary direction.  It
        # must not be mistaken for a reversal of a deliberate TCP movement.
        direction_min = float(limit.get('direction_change_min_translation_m', 0.0005))
        direction = None if translation < direction_min else (pose[:3, 3] - previous[:3, 3]) / translation
        if direction is not None and previous_direction is not None and float(np.dot(direction, previous_direction)) < limit['direction_change_cosine_min']:
            reason = 'motion direction discontinuity'
        self.last_synced = (pose, image_ns, linear, angular, direction)
        return not reason, {'linear_m_s': linear, 'angular_rad_s': angular, 'sync_translation_error_m': 0.0, 'sync_rotation_error_rad': 0.0}, reason

    def distinct(self, pose):
        cfg = self.config['selection']
        if self.valid:
            t, r = pose_distance(self.valid[-1]['T_base_tool'], pose)
            if t < cfg['min_new_translation_m'] and math.degrees(r) < cfg['min_new_rotation_deg']: return False, 'keyframe change below threshold'
        # Reject a pose that duplicates any earlier accepted keyframe.  Using
        # all(...) here allowed repeated trajectory regions to be overweighted.
        if self.valid and any(pose_distance(item['T_base_tool'], pose)[0] < cfg['near_duplicate_translation_m'] and math.degrees(pose_distance(item['T_base_tool'], pose)[1]) < cfg['near_duplicate_rotation_deg'] for item in self.valid):
            return False, 'near duplicate pose'
        return True, ''

    def save_record(self, record, image, overlay):
        if self.session is None or self.records is None:
            raise RuntimeError('Persistent session was not started after TF preflight.')
        index = record['frame_id']; ext = self.config['storage']['image_extension']
        if self.config['storage']['save_all_images']:
            raw = self.session / 'images' / f'{index:06d}_raw.{ext}'; cv2.imwrite(str(raw), image); record['image_path'] = str(raw.relative_to(self.session))
        if self.config['storage']['save_annotated_images']:
            annotated = self.session / 'images' / f'{index:06d}_annotated.{ext}'; cv2.imwrite(str(annotated), overlay); record['annotated_image_path'] = str(annotated.relative_to(self.session))
        self.records.write(json.dumps(record, ensure_ascii=False) + '\n'); self.records.flush()

    def optimize_if_due(self):
        every = self.config['optimization']['update_every_valid_samples']
        if len(self.training_samples) < self.config['optimization']['min_samples'] or len(self.training_samples) % every:
            return
        result = solve_correction(self.training_samples, self.x_input, self.config)
        cfg = self.config['validation']
        validation = {'passed': False, 'reason': result.reason, 'holdout_before_rms': None,
                      'holdout_after_rms': None, 'after_before_ratio': None,
                      'update_translation_m': None, 'update_rotation_deg': None,
                      'stable_update_count': self.stable_update_count}
        previous_candidate = self.last_candidate_result
        if result.success:
            self.last_candidate_result = result
            if len(self.holdout_samples) < int(cfg['min_holdout_samples']):
                validation['reason'] = (f'waiting for independent holdout samples '
                                        f'({len(self.holdout_samples)}/{int(cfg["min_holdout_samples"])})')
                self.stable_update_count = 0
            else:
                weight = float(self.config['optimization']['rotation_weight_m_per_rad'])
                before = evaluate_correction_rms(self.holdout_samples, self.x_input, np.eye(4), result.board, weight)
                after = evaluate_correction_rms(self.holdout_samples, self.x_input, result.delta, result.board, weight)
                ratio = math.inf if before is None or before <= 0.0 or after is None else after / before
                validation.update({'holdout_before_rms': before, 'holdout_after_rms': after,
                                   'after_before_ratio': ratio})
                if previous_candidate is not None:
                    update_t, update_r = pose_distance(previous_candidate.delta, result.delta)
                    validation['update_translation_m'] = update_t
                    validation['update_rotation_deg'] = math.degrees(update_r)
                failure = ''
                if result.condition is None or result.condition > float(cfg['max_condition_number']):
                    failure = 'training condition number exceeds validation limit'
                elif after is None or after > float(cfg['max_holdout_rms_m']):
                    failure = 'independent holdout RMS exceeds validation limit'
                elif ratio > float(cfg['max_after_before_ratio']):
                    failure = 'independent holdout improvement is insufficient'
                elif previous_candidate is None:
                    failure = 'waiting for a second correction update to check stability'
                elif (validation['update_translation_m'] > float(cfg['max_update_translation_m'])
                      or validation['update_rotation_deg'] > float(cfg['max_update_rotation_deg'])):
                    failure = 'correction update is not stable'
                if failure:
                    self.stable_update_count = 0
                    validation['reason'] = failure
                else:
                    self.stable_update_count += 1
                    validation['stable_update_count'] = self.stable_update_count
                    required = int(cfg['required_stable_updates'])
                    if self.stable_update_count >= required:
                        validation['passed'] = True
                        validation['reason'] = 'independent holdout and stability checks passed'
                    else:
                        validation['reason'] = f'waiting for stable correction updates ({self.stable_update_count}/{required})'
        self.correction_available = bool(result.success and validation['passed'])
        if self.correction_available:
            self.last_result = result
        self.last_validation = validation
        report = {'valid_count': len(self.valid), 'training_count': len(self.training_samples),
                  'holdout_count': len(self.holdout_samples), 'status': result.reason,
                  'success': result.success, 'solver_success': result.success,
                  'validated_available': self.correction_available,
                  'hit_bound': result.hit_bound,
                  'rank': result.rank, 'condition': result.condition, 'before_rms': result.before_rms, 'after_rms': result.after_rms,
                  'X0_T_tool_camera': matrix_record(self.x0), 'DeltaX_tool': None if result.delta is None else matrix_record(result.delta),
                  'X_current': None if result.delta is None else matrix_record(result.delta @ self.x_input),
                  'T_base_board': None if result.board is None else matrix_record(result.board),
                  'independent_validation': validation}
        with (self.session / 'results' / f'correction_{len(self.training_samples):03d}.yaml').open('w', encoding='utf-8') as stream: yaml.safe_dump(report, stream, sort_keys=False)
        self.log.info('optimization training=%s holdout=%s status=%s rank=%s condition=%s validation=%s',
                      len(self.training_samples), len(self.holdout_samples), result.reason,
                      result.rank, result.condition, validation['reason'])

    def process(self):
        if self.pending_frame is None and not self.image_queue:
            return self.last_overlay if self.last_overlay is not None else (self.last_camera_image if self.last_camera_image is not None else np.zeros((720, 1280, 3), dtype=np.uint8))
        min_interval = 1.0 / float(self.config['live']['max_process_rate_hz'])
        if self.pending_frame is None and time.monotonic() - self.last_process_monotonic < min_interval:
            return self.last_overlay if self.last_overlay is not None else self.last_camera_image
        if self.pending_frame is None:
            image, image_ns = self.image_queue.popleft()
            self.last_process_monotonic = time.monotonic()
            overlay, quality, camera_board = self.detect(image)
            self.pending_frame = {'image': image, 'image_ns': image_ns, 'overlay': overlay, 'quality': quality,
                                  'camera_board': camera_board, 'wait_started': time.monotonic()}
            self.last_corner_count = quality['corner_count']
            self.last_runtime_status = quality['reason']
        pending = self.pending_frame
        image, image_ns, overlay, quality, camera_board = (pending['image'], pending['image_ns'], pending['overlay'],
                                                     pending['quality'], pending['camera_board'])
        record = {'frame_id': self.frame_counter + 1, 'image_timestamp_ns': image_ns, 'image_quality_ok': quality['image_quality_ok'], 'sync_ok': False, 'valid_for_optimization': False, 'corner_count': quality['corner_count'], 'charuco_ids': quality['ids'], 'reprojection_rms_px': quality['reprojection_rms_px'], 'blur_metric': quality['blur_metric'], 'edge_margin_px': quality['edge_margin_px'], 'reject_reason': quality['reason']}
        if quality['image_quality_ok']:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.config['interfaces']['base_frame'], self.config['interfaces']['tool_frame'],
                    self._Time(nanoseconds=image_ns), timeout=self._Duration(seconds=0.01))
                base_tool = matrix_from_components(
                    (tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z),
                    (tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w))
                tf_ns = stamp_ns(tf_msg.header.stamp)
                time_error = abs(tf_ns - image_ns) / 1e9
                record.update({'clock_domain': self.config['interfaces']['expected_clock_domain'], 'tf_query_timestamp_ns': image_ns, 'tf_before_timestamp_ns': tf_ns, 'tf_after_timestamp_ns': tf_ns, 'tf_interpolation_ratio': 0.0, 'sync_time_error_sec': time_error, 'T_base_tool_before': matrix_record(base_tool), 'T_base_tool_after': matrix_record(base_tool), 'T_base_tool': matrix_record(base_tool), 'T_camera_board': matrix_record(camera_board)})
                if time_error > self.config['sync']['save_only_max_time_error_sec']:
                    record['reject_reason'] = 'TF time bracket exceeds save-only range'
                else:
                    record['sync_ok'] = True
                    motion_ok, motion, reason = self.motion_quality(base_tool, image_ns)
                    motion['sync_translation_error_m'] = motion['linear_m_s'] * time_error
                    motion['sync_rotation_error_rad'] = motion['angular_rad_s'] * time_error
                    record.update(motion)
                    if time_error > self.config['sync']['optimize_max_time_error_sec']: reason = 'TF synchronization is save-only'
                    distinct, distinct_reason = self.distinct(base_tool)
                    if motion_ok and not reason and distinct:
                        sample = {'T_base_tool': base_tool, 'T_camera_board': camera_board, 'record': record}
                        self.valid.append(sample)
                        holdout_every = int(self.config['validation']['holdout_every_n_samples'])
                        is_holdout = len(self.valid) % holdout_every == 0
                        record['sample_role'] = 'holdout' if is_holdout else 'training'
                        record['valid_for_optimization'] = not is_holdout
                        record['valid_for_independent_validation'] = is_holdout
                        record['reject_reason'] = ''
                        (self.holdout_samples if is_holdout else self.training_samples).append(sample)
                        self.optimize_if_due()
                    else: record['reject_reason'] = reason or distinct_reason
            except self._TransformException as exc:
                # tf2 requires a transform at or after the requested image
                # timestamp.  Keep this one image while the live TF stream
                # catches up; never substitute an older/latest transform.
                elapsed = time.monotonic() - pending['wait_started']
                if 'extrapolation into the future' in str(exc) and elapsed < float(self.config['live']['max_tf_wait_sec']):
                    self.last_runtime_status = f'waiting for future TF ({elapsed:.2f}s)'
                    self.last_overlay = overlay
                    return overlay
                record['reject_reason'] = f'TF lookup failed: {exc}'
        self.frame_counter += 1; self.processed_frames += 1
        self.save_record(record, image, overlay)
        self.last_record = record
        self.last_overlay = overlay
        self.last_runtime_status = record['reject_reason'] or 'image quality accepted'
        self.pending_frame = None
        if self.frame_counter % 30 == 0:
            self.log.info('processed=%s valid=%s dropped_latest=%s last_reject=%s',
                          self.processed_frames, len(self.valid), self.dropped_queue_frames,
                          record['reject_reason'])
        return overlay

    def text_status(self):
        """Return the former on-screen status groups as terminal text."""
        inj = self.config['injection']
        injection = (f"Error injection: {'Yes' if inj['enabled'] else 'No'}\n"
                     f"Translation: [{inj['translation_mm'][0]:+.3f}, {inj['translation_mm'][1]:+.3f}, {inj['translation_mm'][2]:+.3f}] mm\n"
                     f"Rotation vector: [{inj['rotation_rotvec_deg'][0]:+.3f}, {inj['rotation_rotvec_deg'][1]:+.3f}, {inj['rotation_rotvec_deg'][2]:+.3f}] deg\n"
                     f"Frame: {inj['frame']}\nComposition: {inj['multiplication']}")
        def correction_block(title, result):
            d = result.delta; rv = np.degrees(rotvec_shortest(d[:3, :3])); t = d[:3, 3] * 1000.0
            condition = 'unavailable' if result.condition is None else f'{result.condition:.1f}'
            return (f"{title}:\nTranslation: [{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}] mm\n"
                    f"Rotation vector: [{rv[0]:+.3f}, {rv[1]:+.3f}, {rv[2]:+.3f}] deg\n"
                    f"Frame: tool\nSolver status: {result.reason}; condition: {condition}")
        if self.last_result is None or not self.correction_available:
            reason = 'waiting for first optimization' if self.last_validation is None else self.last_validation['reason']
            correction = f'Current correction: unavailable\nValidation: {reason}'
            if self.last_candidate_result is not None and self.last_candidate_result.delta is not None:
                correction += '\n' + correction_block('Candidate correction (not yet validated)', self.last_candidate_result)
        else:
            correction = correction_block('Current correction', self.last_result)
        validation_text = 'Independent holdout: unavailable'
        if self.last_validation is not None:
            before = self.last_validation['holdout_before_rms']; after = self.last_validation['holdout_after_rms']
            before_text = 'unavailable' if before is None else f'{before * 1000.0:.3f} mm'
            after_text = 'unavailable' if after is None else f'{after * 1000.0:.3f} mm'
            validation_text = (f'Independent holdout: {len(self.holdout_samples)} samples\n'
                               f'RMS before/after: {before_text} / {after_text}\n'
                               f'Status: {self.last_validation["reason"]}')
        record = self.last_record
        corners = 'waiting for image' if self.last_corner_count is None else str(self.last_corner_count)
        quality = self.last_runtime_status if record is None or self.pending_frame is not None else (record['reject_reason'] or 'image quality accepted')
        return ('\n'.join(item for item in (injection, correction, validation_text,
                           f'Valid samples: {len(self.valid)}',
                           f'Training samples: {len(self.training_samples)}',
                           f'Holdout samples: {len(self.holdout_samples)}',
                           f'Processed frames: {self.processed_frames}',
                           f'Dropped latest-frame replacements: {self.dropped_queue_frames}',
                           f'Current ChArUco corners: {corners}',
                           f'Current frame status: {quality}',
                           '-' * 72) if item))

def synthetic_samples(x_truth, count=24):
    board = transform([0.42, -0.18, 0.31], [0.2, -0.1, 0.15]); samples = []
    for i in range(count):
        a = transform([0.02 * math.sin(i), 0.018 * math.cos(1.7*i), 0.015 * math.sin(.7*i)], [.08*math.sin(.3*i), .07*math.cos(.5*i), .05*math.sin(.8*i)])
        c = inverse(x_truth) @ inverse(a) @ board
        samples.append({'T_base_tool': a, 'T_camera_board': c})
    return samples


def run_self_test(config, report_path):
    tests = [('no_drift', np.zeros(3), np.zeros(3))] + [(name, trans, rot) for name, trans, rot in (
        ('Tx', [0.005, 0, 0], [0, 0, 0]), ('Ty', [0, 0.005, 0], [0, 0, 0]), ('Tz', [0, 0, 0.005], [0, 0, 0]),
        ('Rx', [0, 0, 0], [math.radians(1), 0, 0]), ('Ry', [0, 0, 0], [0, math.radians(1), 0]), ('Rz', [0, 0, 0], [0, 0, math.radians(1)]))]
    x0 = transform([0.04, -0.03, 0.09], [0.2, -0.1, 0.3]); results = []
    for name, t, r in tests:
        injected = transform(t, r); x_input = injected @ x0; cfg = json.loads(json.dumps(config)); cfg['optimization']['mode'] = 'full_se3'; cfg['optimization']['min_jacobian_rank'] = 6
        solved = solve_correction(synthetic_samples(x0), x_input, cfg); expected = inverse(injected)
        estimate = solved.delta if solved.success else np.eye(4); translation_error, rotation_error = pose_distance(expected, estimate)
        inverse_ok = validate_transform(injected @ inverse(injected)) and validate_transform(inverse(injected) @ injected)
        passed = solved.success and inverse_ok and translation_error <= 1e-5 and math.degrees(rotation_error) <= 1e-3
        results.append({'name': name, 'passed': passed, 'injection_matrix': matrix_record(injected), 'expected_correction_matrix': matrix_record(expected), 'estimated_correction_matrix': matrix_record(estimate), 'translation_error_mm': translation_error * 1000.0, 'rotation_error_deg': math.degrees(rotation_error), 'solver_status': solved.reason})
    # These checks make the non-selected conventions explicit.  The live
    # program deliberately supports only left/tool, but all order/inverse
    # identities are verified so a future convention change cannot silently
    # compare errors in the wrong frame.
    x = transform([.04, -.03, .09], [.2, -.1, .3]); error = transform([.005, -.002, .001], [.01, -.02, .03])
    x_left_bad, x_left_corrected = error @ x, inverse(error) @ (error @ x)
    x_right_bad, x_right_corrected = x @ error, (x @ error) @ inverse(error)
    x_camera_tool = inverse(x); x_camera_tool_roundtrip = inverse(x_camera_tool)
    checks = [
        {'name': 'left_tool_injection_and_inverse_correction', 'passed': np.allclose(x_left_corrected, x, atol=1e-10), 'injected_matrix': matrix_record(x_left_bad), 'corrected_matrix': matrix_record(x_left_corrected)},
        {'name': 'right_camera_injection_and_inverse_correction', 'passed': np.allclose(x_right_corrected, x, atol=1e-10), 'injected_matrix': matrix_record(x_right_bad), 'corrected_matrix': matrix_record(x_right_corrected)},
        {'name': 'tool_camera_camera_tool_inverse_roundtrip', 'passed': np.allclose(x_camera_tool_roundtrip, x, atol=1e-10), 'tool_camera': matrix_record(x), 'camera_tool': matrix_record(x_camera_tool)},
        {'name': 'both_inverse_products_identity', 'passed': validate_transform(x @ inverse(x)) and validate_transform(inverse(x) @ x), 'matrix': matrix_record(x)},
    ]
    report = {'convention': 'X_current = DeltaX_tool @ X_input; injection is left/tool and expected correction is inverse(injection)', 'all_passed': all(item['passed'] for item in results) and all(item['passed'] for item in checks), 'tests': results, 'order_and_inverse_checks': checks}
    with Path(report_path).open('w', encoding='utf-8') as stream: yaml.safe_dump(report, stream, sort_keys=False)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--no-injection', action='store_true',
                        help='Disable virtual error injection for this run without editing the YAML config.')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--self-test-report', type=Path)
    args = parser.parse_args(); config = load_config(args.config)
    if args.no_injection:
        apply_no_injection_override(config)
    if args.self_test:
        report_path = args.self_test_report or Path(config['paths']['self_test_report']); report = run_self_test(config, report_path)
        print(f"self-test report: {report_path}\nall_passed: {report['all_passed']}"); return 0 if report['all_passed'] else 1
    import rclpy
    rclpy.init(); live = DynamicCorrectionNode(config)
    try:
        deadline = time.monotonic() + float(config['live']['startup_tf_timeout_sec'])
        while rclpy.ok() and not live.has_live_tf() and time.monotonic() < deadline:
            rclpy.spin_once(live.node, timeout_sec=0.1)
        if not live.has_live_tf():
            raise RuntimeError(f'{live.startup_status()} was not received within {config["live"]["startup_tf_timeout_sec"]} s; refusing to start text monitoring.')
        live.start_session()
        print('Dynamic hand-eye SE3 correction: text mode (no GUI). Press Ctrl-C to stop.', flush=True)
        last_console_status = 0.0
        while rclpy.ok():
            loop_started = time.monotonic()
            # ROS callback throughput must be independent of terminal output.
            # At 2 Hz, tf2 consumed /tf too slowly and its latest transform
            # lagged camera timestamps by seconds, causing future extrapolation.
            rclpy.spin_once(live.node, timeout_sec=0.01)
            live.process()
            now = time.monotonic()
            if now - last_console_status >= 1.0 / float(config['live']['max_status_rate_hz']):
                print(live.text_status(), flush=True)
                last_console_status = now
            time.sleep(max(0.0, 1.0 / float(config['live']['max_ros_spin_rate_hz']) - (time.monotonic() - loop_started)))
    finally:
        live.close()
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    raise SystemExit(main())
