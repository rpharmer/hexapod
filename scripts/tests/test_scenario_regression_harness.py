#!/usr/bin/env python3
"""Unit tests for scenario_regression_harness log parsing."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from scripts.scenario_regression_harness import (
    ScenarioFingerprint,
    analyze_log,
    high_frequency_energy_ratio,
    build_fingerprint,
    load_fingerprint_store,
    resolve_thresholds,
    save_fingerprint_store,
    validate_thresholds,
)


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
        self.assertEqual(stats.foot_velocity_samples, 2)
        self.assertEqual(stats.joint_velocity_samples, 2)
        self.assertAlmostEqual(stats.peak_foot_vel_mps, 1.3)
        self.assertAlmostEqual(stats.peak_velocity_radps, 6.8)
        self.assertGreaterEqual(stats.foot_high_frequency_energy_ratio, 0.0)
        self.assertLessEqual(stats.foot_high_frequency_energy_ratio, 1.0)
        self.assertGreaterEqual(stats.joint_high_frequency_energy_ratio, 0.0)
        self.assertLessEqual(stats.joint_high_frequency_energy_ratio, 1.0)
        self.assertEqual(stats.tip_over_events, 3)
        self.assertEqual(stats.max_tip_over_streak, 2)
        self.assertEqual(stats.fault_timeline_hash, "bffbfb4a2c65")
        self.assertEqual(stats.peak_command_tuple, (1.3, 6.8))
        self.assertEqual(stats.final_status_summary, "mode=WALK;fault=TIP_OVER;loops=5")

    def test_handles_logs_without_metrics(self) -> None:
        log = self.write_log("[info] [diag] mode=IDLE fault=NONE loops=1\n")
        stats = analyze_log(log)
        self.assertEqual(stats.runtime_metrics_lines, 0)
        self.assertEqual(stats.foot_velocity_samples, 0)
        self.assertEqual(stats.joint_velocity_samples, 0)
        self.assertEqual(stats.peak_foot_vel_mps, 0.0)
        self.assertEqual(stats.peak_velocity_radps, 0.0)
        self.assertEqual(stats.foot_high_frequency_energy_ratio, 0.0)
        self.assertEqual(stats.joint_high_frequency_energy_ratio, 0.0)
        self.assertEqual(stats.tip_over_events, 0)
        self.assertEqual(stats.max_tip_over_streak, 0)
        self.assertEqual(stats.fault_timeline_hash, "c627c09c14e5")
        self.assertEqual(stats.final_status_summary, "mode=IDLE;fault=NONE;loops=1")

    def test_fingerprint_store_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            store_path = Path(tmpdir) / "fingerprints.json"
            store = {
                "scenarios/example.toml": ScenarioFingerprint(
                    fault_timeline_hash="abc123",
                    peak_command_tuple=(0.1, 2.3),
                    final_status_summary="mode=WALK;fault=NONE;loops=42",
                )
            }

            save_fingerprint_store(store_path, store)
            loaded = load_fingerprint_store(store_path)
            self.assertEqual(loaded, store)

    def test_build_fingerprint_uses_compact_fields(self) -> None:
        stats = analyze_log(self.write_log("[info] [diag] mode=IDLE fault=NONE loops=1\n"))
        fingerprint = build_fingerprint(stats)
        self.assertEqual(
            fingerprint,
            ScenarioFingerprint(
                fault_timeline_hash="c627c09c14e5",
                peak_command_tuple=(0.0, 0.0),
                final_status_summary="mode=IDLE;fault=NONE;loops=1",
            ),
        )

    def test_threshold_profile_defaults_and_overrides(self) -> None:
        strict = resolve_thresholds(
            SimpleNamespace(
                threshold_profile="strict",
                max_peak_foot_vel_mps=None,
                max_peak_velocity_radps=None,
                max_foot_high_frequency_energy_ratio=None,
                max_joint_high_frequency_energy_ratio=None,
                tip_over_persist_streak=None,
            )
        )
        self.assertAlmostEqual(strict.max_peak_foot_vel_mps, 12.2)
        self.assertAlmostEqual(strict.max_peak_velocity_radps, 25.0)
        self.assertAlmostEqual(strict.max_foot_high_frequency_energy_ratio, 0.2)
        self.assertAlmostEqual(strict.max_joint_high_frequency_energy_ratio, 0.2)
        self.assertEqual(strict.tip_over_persist_streak, 2)

        current_with_override = resolve_thresholds(
            SimpleNamespace(
                threshold_profile="current-baseline",
                max_peak_foot_vel_mps=13.0,
                max_peak_velocity_radps=None,
                max_foot_high_frequency_energy_ratio=0.12,
                max_joint_high_frequency_energy_ratio=None,
                tip_over_persist_streak=4,
            )
        )
        self.assertAlmostEqual(current_with_override.max_peak_foot_vel_mps, 13.0)
        self.assertAlmostEqual(current_with_override.max_peak_velocity_radps, 120.0)
        self.assertAlmostEqual(current_with_override.max_foot_high_frequency_energy_ratio, 0.12)
        self.assertAlmostEqual(current_with_override.max_joint_high_frequency_energy_ratio, 0.35)
        self.assertEqual(current_with_override.tip_over_persist_streak, 4)

    def test_validate_thresholds_rejects_non_finite_or_non_positive_values(self) -> None:
        with self.assertRaises(SystemExit):
            validate_thresholds(
                resolve_thresholds(
                    SimpleNamespace(
                        threshold_profile="current-baseline",
                        max_peak_foot_vel_mps=float("nan"),
                        max_peak_velocity_radps=None,
                        max_foot_high_frequency_energy_ratio=None,
                        max_joint_high_frequency_energy_ratio=None,
                        tip_over_persist_streak=None,
                    )
                )
            )

        with self.assertRaises(SystemExit):
            validate_thresholds(
                resolve_thresholds(
                    SimpleNamespace(
                        threshold_profile="current-baseline",
                        max_peak_foot_vel_mps=None,
                        max_peak_velocity_radps=0.0,
                        max_foot_high_frequency_energy_ratio=None,
                        max_joint_high_frequency_energy_ratio=None,
                        tip_over_persist_streak=None,
                    )
                )
            )

        with self.assertRaises(SystemExit):
            validate_thresholds(
                resolve_thresholds(
                    SimpleNamespace(
                        threshold_profile="current-baseline",
                        max_peak_foot_vel_mps=None,
                        max_peak_velocity_radps=None,
                        max_foot_high_frequency_energy_ratio=1.1,
                        max_joint_high_frequency_energy_ratio=None,
                        tip_over_persist_streak=None,
                    )
                )
            )

    def test_high_frequency_energy_ratio_detects_jittery_sequence(self) -> None:
        smooth = [0.0, 0.4, 0.8, 1.1, 1.5, 1.8, 2.2, 2.6]
        jittery = [0.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0]

        smooth_ratio = high_frequency_energy_ratio(smooth)
        jittery_ratio = high_frequency_energy_ratio(jittery)

        self.assertLess(smooth_ratio, jittery_ratio)
        self.assertGreater(jittery_ratio, 0.5)


if __name__ == "__main__":
    unittest.main()
