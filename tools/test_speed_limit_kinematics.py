#!/usr/bin/env python3
"""Regression tests for diagnostic-only servo reversal analysis."""
import unittest
import json
from pathlib import Path
import tempfile
import numpy as np

from audit_speed_limit_kinematics import audit, critical_rate


class CriticalRateTests(unittest.TestCase):
    def test_initial_velocity_and_acceleration(self):
        error, incoming, requested, omega = 0.212542, 3.25503, -6.36063, 25.0
        self.assertAlmostEqual(critical_rate(error, incoming, requested, omega, 0.0), incoming)
        epsilon = 1e-7
        acceleration = (critical_rate(error, incoming, requested, omega, epsilon)
                        - critical_rate(error, incoming, requested, omega, -epsilon)) / (2 * epsilon)
        self.assertAlmostEqual(acceleration, omega * omega * error - 2 * omega * incoming, places=6)

    def test_cancelled_requests_do_not_cancel_actual_rates(self):
        omega = 25.0
        errors = (0.277782, 0.212542)
        incoming = (3.62076, 3.25503)
        scales = (0.25, 0.5, 0.850353106, 1.0)
        times = np.array((0.0, 0.005, 0.020, 0.050, 0.100, 0.200))
        references = None
        for scale in scales:
            rates = (7.47998251 * scale, -7.47998251 * scale)
            summed = sum(critical_rate(e, v, r, omega, times) for e, v, r in zip(errors, incoming, rates))
            if references is None:
                references = summed
            np.testing.assert_allclose(summed, references, atol=1e-12, rtol=1e-12)
        self.assertGreater(references[1], 6.0)  # Still reinforces after one control tick.
        self.assertGreater(references[4], 1.0)  # Cancellation is not instantaneous even at 100 ms.
        self.assertLess(references[-1], 0.3)

    def test_follows_constant_ramp_at_long_times(self):
        self.assertAlmostEqual(critical_rate(0.5, 3.0, -6.0, 25.0, 10.0), -6.0)


class FrozenAuditValidationTests(unittest.TestCase):
    def test_rejects_invalid_snapshots(self):
        fixture = Path(__file__).resolve().parents[1] / "docs/contact-snapshots/speed-limit-governed-kinematics-v1.json"
        mutations = (
            lambda d: d["kinematics"]["mass"][0].pop(),
            lambda d: d["kinematics"]["v_in"].__setitem__(0, float("nan")),
            lambda d: d["kinematics"]["wires"][0].__setitem__("wire", 1),
            lambda d: d["kinematics"]["wires"][0].__setitem__("target_rate", float("inf")),
            lambda d: d["winner"].__setitem__("winner_w", 0.0),
            lambda d: d["kinematics"].__setitem__("angular_frame", "LOCAL"),
        )
        with tempfile.TemporaryDirectory(prefix="hexapod-speed-audit-test-") as temporary:
            path = Path(temporary) / "invalid.json"
            for mutate in mutations:
                data = json.loads(fixture.read_text())
                mutate(data)
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    audit(path)


if __name__ == "__main__":
    unittest.main()
