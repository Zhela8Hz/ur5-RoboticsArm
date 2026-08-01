import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'calibration/extrinsics/handeye/tools'))
from dynamic_handeye_se3_core import (evaluate_correction_rms, inverse, pose_distance,
                                      solve_correction, transform, validate_transform)
from dynamic_handeye_se3_correction import DynamicCorrectionNode, interpolate_tf, synthetic_samples


def config():
    return {'optimization': {'mode': 'full_se3', 'loss': 'linear', 'huber_f_scale_m': .001, 'rotation_weight_m_per_rad': .05,
            'min_samples': 5, 'max_translation_m': .02, 'max_rotation_deg': 5., 'max_nfev': 300,
            'jacobian_rank_tolerance': 1e-8, 'min_jacobian_rank': 6, 'max_condition_number': 1e10, 'known_base_board_matrix': None}}


class DynamicHandeyeSE3Tests(unittest.TestCase):
    def test_interpolation_translation_and_rotation(self):
        a = transform([0, 0, 0], [0, 0, 0]); b = transform([1, 0, 0], [0, 0, math.pi])
        output, ratio, error = interpolate_tf((100, a), (300, b), 200)
        self.assertAlmostEqual(ratio, .5); self.assertEqual(error, 1e-7)
        self.assertTrue(np.allclose(output[:3, 3], [0.5, 0, 0]))
        self.assertAlmostEqual(np.linalg.norm(output[:3, :3] @ np.array([1., 0, 0]) - np.array([0., 1., 0])), 0., places=7)

    def test_inverse_is_se3_identity(self):
        matrix = transform([.01, -.02, .03], [.2, -.1, .3])
        self.assertTrue(validate_transform(matrix @ inverse(matrix)))
        self.assertTrue(validate_transform(inverse(matrix) @ matrix))

    def test_left_tool_correction_recovers_inverse_injection(self):
        x0 = transform([.04, -.03, .09], [.2, -.1, .3]); injected = transform([.005, 0, 0], [0, 0, 0])
        result = solve_correction(synthetic_samples(x0), injected @ x0, config())
        self.assertTrue(result.success, result.reason)
        translation, rotation = pose_distance(inverse(injected), result.delta)
        self.assertLessEqual(translation, 1e-5); self.assertLessEqual(math.degrees(rotation), .001)

    def test_holdout_evaluation_does_not_fit_holdout_samples(self):
        x0 = transform([.04, -.03, .09], [.2, -.1, .3]); injected = transform([.005, 0, 0])
        samples = synthetic_samples(x0)
        training = [sample for index, sample in enumerate(samples, start=1) if index % 4]
        holdout = [sample for index, sample in enumerate(samples, start=1) if not index % 4]
        result = solve_correction(training, injected @ x0, config())
        self.assertTrue(result.success, result.reason)
        before = evaluate_correction_rms(holdout, injected @ x0, np.eye(4), result.board, .05)
        after = evaluate_correction_rms(holdout, injected @ x0, result.delta, result.board, .05)
        self.assertIsNotNone(before); self.assertIsNotNone(after)
        self.assertGreater(before, 1e-4)
        self.assertLess(after, 1e-8)

    def test_near_duplicate_of_any_prior_pose_is_rejected(self):
        node = DynamicCorrectionNode.__new__(DynamicCorrectionNode)
        node.config = {'selection': {'min_new_translation_m': .0005, 'min_new_rotation_deg': .1,
                                     'near_duplicate_translation_m': .00025,
                                     'near_duplicate_rotation_deg': .05}}
        first = transform([0, 0, 0]); last = transform([.02, 0, 0])
        node.valid = [{'T_base_tool': first}, {'T_base_tool': last}]
        distinct, reason = node.distinct(transform([.0001, 0, 0]))
        self.assertFalse(distinct)
        self.assertEqual(reason, 'near duplicate pose')


if __name__ == '__main__':
    unittest.main()
