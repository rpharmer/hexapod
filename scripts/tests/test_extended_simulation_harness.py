#!/usr/bin/env python3
"""Unit tests for extended_simulation_harness log parsing and budget checks."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.extended_simulation_harness import (
    DriftBudgets,
    aggregate,
    compute_cycle_summary,
    validate_budgets,
)


class ExtendedSimulationHarnessTests(unittest.TestCase):
    def write_log(self, text: str) -> Path:
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        path = Path(tmpdir.name) / "cycle.log"
        path.write_text(text, encoding="utf-8")
        return path

    def test_compute_cycle_summary_extracts_bias_counters_and_command_envelopes(self) -> None:
        log = self.write_log(
            """
[info] runtime.metrics loops=100 freshness_rejects_total=2 est_roll_rad=0.01 est_pitch_rad=-0.02 joint_cmd_diag={peak_velocity_radps:4.0} leg_target_diag={peak_foot_vel_mps:1.2}
[info] runtime.metrics loops=200 freshness_rejects_total=6 est_roll_rad=0.03 est_pitch_rad=-0.04 joint_cmd_diag={peak_velocity_radps:7.5} leg_target_diag={peak_foot_vel_mps:1.6}
            """.strip()
        )

        summary = compute_cycle_summary(1, log)
        self.assertEqual(summary.runtime_metrics_lines, 2)
        self.assertGreater(summary.estimator_bias_drift_rad, 0.0)
        self.assertAlmostEqual(summary.freshness_reject_growth_per_1k_loops, 30.0)
        self.assertGreater(summary.cmd_peak_joint_vel_drift_radps, 0.0)
        self.assertGreater(summary.cmd_peak_foot_vel_drift_mps, 0.0)

    def test_aggregate_and_validate_budgets(self) -> None:
        cycle_1 = self.write_log(
            """
[info] runtime.metrics loops=100 freshness_rejects_total=1 est_roll_rad=0.01 est_pitch_rad=0.01 joint_cmd_diag={peak_velocity_radps:3.0} leg_target_diag={peak_foot_vel_mps:0.8}
[info] runtime.metrics loops=200 freshness_rejects_total=1 est_roll_rad=0.01 est_pitch_rad=0.01 joint_cmd_diag={peak_velocity_radps:3.1} leg_target_diag={peak_foot_vel_mps:0.9}
            """.strip()
        )
        cycle_2 = self.write_log(
            """
[info] runtime.metrics loops=100 freshness_rejects_total=10 est_roll_rad=0.20 est_pitch_rad=0.22 joint_cmd_diag={peak_velocity_radps:16.0} leg_target_diag={peak_foot_vel_mps:5.0}
[info] runtime.metrics loops=200 freshness_rejects_total=20 est_roll_rad=0.22 est_pitch_rad=0.25 joint_cmd_diag={peak_velocity_radps:18.0} leg_target_diag={peak_foot_vel_mps:5.5}
            """.strip()
        )

        summary = aggregate([
            compute_cycle_summary(1, cycle_1),
            compute_cycle_summary(2, cycle_2),
        ])
        failures = validate_budgets(
            summary,
            DriftBudgets(
                max_estimator_bias_drift_rad=0.05,
                max_counter_growth_rate_drift_per_1k_loops=2.0,
                max_cmd_peak_joint_vel_drift_radps=4.0,
                max_cmd_peak_foot_vel_drift_mps=1.0,
            ),
        )
        self.assertGreaterEqual(len(failures), 3)


if __name__ == "__main__":
    unittest.main()
