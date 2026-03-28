#!/usr/bin/env python3
"""Unit tests for scenario_regression_harness log parsing."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.scenario_regression_harness import analyze_log


class ScenarioRegressionHarnessTests(unittest.TestCase):
    def write_log(self, text: str) -> Path:
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        path = Path(tmpdir.name) / "scenario.log"
        path.write_text(text, encoding="utf-8")
        return path

    def test_extracts_runtime_metrics_and_tip_over_streaks(self) -> None:
        log = self.write_log(
            """
[info] runtime.metrics loops=10 joint_cmd_diag={peak_velocity_radps:4.2} leg_target_diag={peak_foot_vel_mps:0.9}
[info] [diag] mode=WALK fault=NONE loops=1
[info] [diag] mode=WALK fault=TIP_OVER loops=2
[info] [diag] mode=WALK fault=TIP_OVER loops=3
[info] [diag] mode=WALK fault=NONE loops=4
[info] runtime.metrics loops=20 joint_cmd_diag={peak_velocity_radps:6.8} leg_target_diag={peak_foot_vel_mps:1.3}
[info] [diag] mode=WALK fault=TIP_OVER loops=5
            """.strip()
        )

        stats = analyze_log(log)

        self.assertEqual(stats.runtime_metrics_lines, 2)
        self.assertAlmostEqual(stats.peak_foot_vel_mps, 1.3)
        self.assertAlmostEqual(stats.peak_velocity_radps, 6.8)
        self.assertEqual(stats.tip_over_events, 3)
        self.assertEqual(stats.max_tip_over_streak, 2)

    def test_handles_logs_without_metrics(self) -> None:
        log = self.write_log("[info] [diag] mode=IDLE fault=NONE loops=1\n")
        stats = analyze_log(log)
        self.assertEqual(stats.runtime_metrics_lines, 0)
        self.assertEqual(stats.peak_foot_vel_mps, 0.0)
        self.assertEqual(stats.peak_velocity_radps, 0.0)
        self.assertEqual(stats.tip_over_events, 0)
        self.assertEqual(stats.max_tip_over_streak, 0)


if __name__ == "__main__":
    unittest.main()
