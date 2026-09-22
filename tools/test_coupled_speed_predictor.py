#!/usr/bin/env python3
"""Frozen one-step response oracle and bounded counterfactual regression tests."""
from pathlib import Path
import unittest
import numpy as np

from predict_speed_limit_response import motor_torque, predict


class CoupledPredictorTests(unittest.TestCase):
    def test_motor_envelope(self):
        torque, available = motor_torque(np.array((10., 10., -10., 10.)),
            np.array((0., 7.48, 7.48, -7.48)), 1.471, 7.48)
        np.testing.assert_allclose(available, (1.471, 0., 1.471, 1.471))
        np.testing.assert_allclose(torque, (1.471, 0., -1.471, 1.471))

    def test_frozen_response_and_counterfactual_boundaries(self):
        path = Path(__file__).resolve().parents[1] / "docs/contact-snapshots/speed-limit-governed-kinematics-v1.json"
        result = predict(path)
        self.assertLess(result["baseline_free_velocity_error"], 1e-10)
        self.assertLess(result["baseline_torque_error"], 1e-10)
        self.assertFalse(result["runtime_activation"])
        for candidate in result["candidates"]:
            shift = np.array(candidate["target_shift_rad"])
            np.testing.assert_array_equal(shift[3:], np.zeros(15))
            self.assertAlmostEqual(shift[1] + shift[2], 0.0)
            self.assertFalse(candidate["physical_gates_passed"])
        self.assertGreater(result["candidates"][0]["free_max_angular_speed"], 10.0)
        self.assertLess(result["candidates"][-1]["free_max_angular_speed"], 10.0)


if __name__ == "__main__":
    unittest.main()
