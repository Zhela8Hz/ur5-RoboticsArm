"""SE(3) math and deterministic fixed-board correction solver.

Convention used by the dynamic monitor:
    X0 = T_tool_camera
    X_current = DeltaX_tool @ X0
    T_base_board_i = T_base_tool_i @ X_current @ T_camera_board_i

DeltaX is therefore left-multiplied and expressed in ``tool``.  No Euler
angles are used by this module.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def transform(translation_m=None, rotvec_rad=None):
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rotation.from_rotvec(np.zeros(3) if rotvec_rad is None else rotvec_rad).as_matrix()
    out[:3, 3] = np.zeros(3) if translation_m is None else np.asarray(translation_m, dtype=np.float64)
    return out


def inverse(matrix):
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = matrix[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ matrix[:3, 3]
    return out


def validate_transform(matrix, atol=1e-7):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        return False
    rotation = matrix[:3, :3]
    return bool(np.allclose(rotation.T @ rotation, np.eye(3), atol=atol) and abs(np.linalg.det(rotation) - 1.0) <= atol
                and np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=atol))


def rotvec_shortest(rotation):
    """SO(3) logarithm, with a normalized quaternion and w >= 0 convention."""
    quat = Rotation.from_matrix(rotation).as_quat()  # xyzw
    quat /= np.linalg.norm(quat)
    if quat[3] < 0.0:
        quat *= -1.0
    return Rotation.from_quat(quat).as_rotvec()


def pose_log(matrix):
    return np.r_[matrix[:3, 3], rotvec_shortest(matrix[:3, :3])]


def rotation_angle_rad(a, b):
    return float(np.linalg.norm(rotvec_shortest(a[:3, :3].T @ b[:3, :3])))


def pose_distance(a, b):
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), rotation_angle_rad(a, b)


def matrix_record(matrix):
    return np.asarray(matrix, dtype=np.float64).tolist()


@dataclass(frozen=True)
class CorrectionResult:
    success: bool
    reason: str
    delta: np.ndarray | None
    board: np.ndarray | None
    before_rms: float | None
    after_rms: float | None
    rank: int | None
    condition: float | None
    hit_bound: bool
    sample_count: int


def _board_from_param(params):
    return transform(params[:3], params[3:6])


def _delta_from_param(params, mode):
    if mode == 'translation_only':
        return transform(params[:3], np.zeros(3))
    return transform(params[:3], params[3:6])


def _params_from_matrix(matrix):
    return np.r_[matrix[:3, 3], rotvec_shortest(matrix[:3, :3])]


def _residual(samples, x_input, mode, rotation_weight_m_per_rad, fixed_board=None):
    """Residuals of inv(T_base_board) * A_i * Delta * X * C_i in se(3)."""
    def fn(params):
        delta_size = 3 if mode == 'translation_only' else 6
        delta = _delta_from_param(params[:delta_size], mode)
        board = fixed_board if fixed_board is not None else _board_from_param(params[delta_size:delta_size + 6])
        residuals = []
        for sample in samples:
            predicted = sample['T_base_tool'] @ delta @ x_input @ sample['T_camera_board']
            error = pose_log(inverse(board) @ predicted)
            residuals.extend(error[:3])
            residuals.extend(error[3:] * rotation_weight_m_per_rad)
        return np.asarray(residuals, dtype=np.float64)
    return fn


def evaluate_correction_rms(samples, x_input, delta, board, rotation_weight_m_per_rad):
    """Evaluate a fixed correction and board pose without fitting the samples."""
    if (not samples or not validate_transform(x_input) or not validate_transform(delta)
            or not validate_transform(board)):
        return None
    residuals = []
    for sample in samples:
        if not validate_transform(sample['T_base_tool']) or not validate_transform(sample['T_camera_board']):
            return None
        predicted = sample['T_base_tool'] @ delta @ x_input @ sample['T_camera_board']
        error = pose_log(inverse(board) @ predicted)
        residuals.extend(error[:3])
        residuals.extend(error[3:] * float(rotation_weight_m_per_rad))
    values = np.asarray(residuals, dtype=np.float64)
    return float(np.sqrt(np.mean(values ** 2)))


def solve_correction(samples, x_input, config):
    """Estimate a left/tool-frame correction and optional fixed base->board pose."""
    mode = config['optimization']['mode']
    if mode not in ('translation_only', 'full_se3'):
        return CorrectionResult(False, 'invalid optimization mode', None, None, None, None, None, None, False, len(samples))
    if len(samples) < config['optimization']['min_samples']:
        return CorrectionResult(False, 'insufficient samples', None, None, None, None, None, None, False, len(samples))
    if not validate_transform(x_input) or any(not validate_transform(s['T_base_tool']) or not validate_transform(s['T_camera_board']) for s in samples):
        return CorrectionResult(False, 'invalid SE(3) input', None, None, None, None, None, None, False, len(samples))
    weight = float(config['optimization']['rotation_weight_m_per_rad'])
    fixed_board = config['optimization'].get('known_base_board_matrix')
    if fixed_board is not None:
        fixed_board = np.asarray(fixed_board, dtype=np.float64)
        if not validate_transform(fixed_board):
            return CorrectionResult(False, 'invalid configured base->board', None, None, None, None, None, None, False, len(samples))
    delta_size = 3 if mode == 'translation_only' else 6
    initial_delta = np.zeros(delta_size)
    if fixed_board is None:
        initial_board = samples[0]['T_base_tool'] @ x_input @ samples[0]['T_camera_board']
        initial = np.r_[initial_delta, _params_from_matrix(initial_board)]
    else:
        initial = initial_delta
    residual = _residual(samples, x_input, mode, weight, fixed_board)
    before = float(np.sqrt(np.mean(residual(initial) ** 2)))
    translation_bound = float(config['optimization']['max_translation_m'])
    rotation_bound = math.radians(float(config['optimization']['max_rotation_deg']))
    low = np.full_like(initial, -np.inf); high = np.full_like(initial, np.inf)
    low[:3], high[:3] = -translation_bound, translation_bound
    if mode == 'full_se3':
        low[3:6], high[3:6] = -rotation_bound, rotation_bound
    try:
        result = least_squares(
            residual, initial, bounds=(low, high), loss=config['optimization']['loss'],
            f_scale=float(config['optimization']['huber_f_scale_m']), x_scale='jac', max_nfev=int(config['optimization']['max_nfev']),
        )
    except Exception as exc:
        return CorrectionResult(False, f'optimizer exception: {exc}', None, None, before, None, None, None, False, len(samples))
    delta = _delta_from_param(result.x[:delta_size], mode)
    board = fixed_board if fixed_board is not None else _board_from_param(result.x[delta_size:delta_size + 6])
    singular = np.linalg.svd(result.jac, compute_uv=False)
    rank = int(np.linalg.matrix_rank(result.jac, tol=float(config['optimization']['jacobian_rank_tolerance'])))
    condition = float(singular[0] / singular[-1]) if len(singular) and singular[-1] > 0.0 else math.inf
    delta_rot = float(np.linalg.norm(rotvec_shortest(delta[:3, :3])))
    hit_bound = (np.any(np.abs(delta[:3, 3]) >= translation_bound * 0.999) or delta_rot >= rotation_bound * 0.999)
    after = float(np.sqrt(np.mean(residual(result.x) ** 2)))
    min_rank = int(config['optimization']['min_jacobian_rank'])
    max_condition = float(config['optimization']['max_condition_number'])
    if hit_bound:
        return CorrectionResult(False, 'correction reached configured bound', None, board, before, after, rank, condition, True, len(samples))
    if rank < min_rank or condition > max_condition:
        return CorrectionResult(False, 'insufficient observability', None, board, before, after, rank, condition, False, len(samples))
    if not result.success:
        return CorrectionResult(False, f'optimizer failed: {result.message}', None, board, before, after, rank, condition, False, len(samples))
    return CorrectionResult(True, 'ok', delta, board, before, after, rank, condition, False, len(samples))
