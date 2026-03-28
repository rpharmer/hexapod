#!/usr/bin/env python3
"""Run a scenario and enforce log-based regression checks."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


FAULT_RE = re.compile(r"\bfault=([A-Z_]+)")
MODE_RE = re.compile(r"\bmode=([A-Z_]+)")
PEAK_FOOT_RE = re.compile(r"peak_foot_vel_mps:([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")
PEAK_JOINT_RE = re.compile(r"peak_velocity_radps:([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")


@dataclass
class LogStats:
    runtime_metrics_lines: int = 0
    peak_foot_vel_mps: float = 0.0
    peak_velocity_radps: float = 0.0
    foot_velocity_samples: int = 0
    joint_velocity_samples: int = 0
    foot_high_frequency_energy_ratio: float = 0.0
    joint_high_frequency_energy_ratio: float = 0.0
    tip_over_events: int = 0
    max_tip_over_streak: int = 0
    fault_timeline_hash: str = ""
    peak_command_tuple: tuple[float, float] = (0.0, 0.0)
    final_status_summary: str = "unknown"


@dataclass(frozen=True)
class ThresholdPreset:
    max_peak_foot_vel_mps: float
    max_peak_velocity_radps: float
    max_foot_high_frequency_energy_ratio: float
    max_joint_high_frequency_energy_ratio: float
    tip_over_persist_streak: int


@dataclass(frozen=True)
class ScenarioFingerprint:
    fault_timeline_hash: str
    peak_command_tuple: tuple[float, float]
    final_status_summary: str


THRESHOLD_PRESETS: dict[str, ThresholdPreset] = {
    "current-baseline": ThresholdPreset(
        max_peak_foot_vel_mps=15.0,
        max_peak_velocity_radps=120.0,
        max_foot_high_frequency_energy_ratio=0.35,
        max_joint_high_frequency_energy_ratio=0.35,
        tip_over_persist_streak=3,
    ),
    "strict": ThresholdPreset(
        max_peak_foot_vel_mps=12.2,
        max_peak_velocity_radps=25.0,
        max_foot_high_frequency_energy_ratio=0.2,
        max_joint_high_frequency_energy_ratio=0.2,
        tip_over_persist_streak=2,
    ),
}


def resolve_thresholds(args: argparse.Namespace) -> ThresholdPreset:
    preset = THRESHOLD_PRESETS[args.threshold_profile]
    return ThresholdPreset(
        max_peak_foot_vel_mps=(
            args.max_peak_foot_vel_mps
            if args.max_peak_foot_vel_mps is not None
            else preset.max_peak_foot_vel_mps
        ),
        max_peak_velocity_radps=(
            args.max_peak_velocity_radps
            if args.max_peak_velocity_radps is not None
            else preset.max_peak_velocity_radps
        ),
        max_foot_high_frequency_energy_ratio=(
            args.max_foot_high_frequency_energy_ratio
            if args.max_foot_high_frequency_energy_ratio is not None
            else preset.max_foot_high_frequency_energy_ratio
        ),
        max_joint_high_frequency_energy_ratio=(
            args.max_joint_high_frequency_energy_ratio
            if args.max_joint_high_frequency_energy_ratio is not None
            else preset.max_joint_high_frequency_energy_ratio
        ),
        tip_over_persist_streak=(
            args.tip_over_persist_streak
            if args.tip_over_persist_streak is not None
            else preset.tip_over_persist_streak
        ),
    )


def validate_thresholds(thresholds: ThresholdPreset) -> None:
    if not math.isfinite(thresholds.max_peak_foot_vel_mps) or thresholds.max_peak_foot_vel_mps <= 0:
        raise SystemExit("--max-peak-foot-vel-mps must be a finite value > 0")
    if not math.isfinite(thresholds.max_peak_velocity_radps) or thresholds.max_peak_velocity_radps <= 0:
        raise SystemExit("--max-peak-velocity-radps must be a finite value > 0")
    if (
        not math.isfinite(thresholds.max_foot_high_frequency_energy_ratio)
        or thresholds.max_foot_high_frequency_energy_ratio < 0
        or thresholds.max_foot_high_frequency_energy_ratio > 1
    ):
        raise SystemExit("--max-foot-high-frequency-energy-ratio must be in [0, 1]")
    if (
        not math.isfinite(thresholds.max_joint_high_frequency_energy_ratio)
        or thresholds.max_joint_high_frequency_energy_ratio < 0
        or thresholds.max_joint_high_frequency_energy_ratio > 1
    ):
        raise SystemExit("--max-joint-high-frequency-energy-ratio must be in [0, 1]")
    if thresholds.tip_over_persist_streak < 1:
        raise SystemExit("--tip-over-persist-streak must be >= 1")


def high_frequency_energy_ratio(samples: list[float]) -> float:
    """Compute high-band spectral energy ratio from a real-valued sequence.

    A simple DFT is used and non-DC bins are partitioned into equal low/mid/high
    normalized-frequency buckets. The returned ratio is high_band / total_non_dc.
    """
    count = len(samples)
    if count < 4:
        return 0.0

    mean = sum(samples) / count
    centered = [sample - mean for sample in samples]
    nyquist_bin = count // 2
    if nyquist_bin < 2:
        return 0.0

    low_energy = 0.0
    mid_energy = 0.0
    high_energy = 0.0

    for k in range(1, nyquist_bin + 1):
        re = 0.0
        im = 0.0
        for n, value in enumerate(centered):
            angle = 2.0 * math.pi * k * n / count
            re += value * math.cos(angle)
            im -= value * math.sin(angle)
        power = re * re + im * im
        normalized = k / nyquist_bin
        if normalized <= (1.0 / 3.0):
            low_energy += power
        elif normalized <= (2.0 / 3.0):
            mid_energy += power
        else:
            high_energy += power

    total_energy = low_energy + mid_energy + high_energy
    if total_energy <= 0:
        return 0.0
    return high_energy / total_energy


def analyze_log(log_path: Path) -> LogStats:
    stats = LogStats()
    tip_over_streak = 0
    foot_velocity_samples: list[float] = []
    joint_velocity_samples: list[float] = []
    fault_timeline: list[str] = []
    final_mode = "UNKNOWN"
    final_fault = "UNKNOWN"
    final_loops = "?"

    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "runtime.metrics" in line:
                stats.runtime_metrics_lines += 1

                match = PEAK_FOOT_RE.search(line)
                if match:
                    sample = float(match.group(1))
                    stats.peak_foot_vel_mps = max(stats.peak_foot_vel_mps, sample)
                    foot_velocity_samples.append(sample)

                match = PEAK_JOINT_RE.search(line)
                if match:
                    sample = float(match.group(1))
                    stats.peak_velocity_radps = max(stats.peak_velocity_radps, sample)
                    joint_velocity_samples.append(sample)

            fault_match = FAULT_RE.search(line)
            if not fault_match:
                continue

            mode_match = MODE_RE.search(line)
            if mode_match:
                final_mode = mode_match.group(1)
            final_fault = fault_match.group(1)
            loops_match = re.search(r"\bloops=(\d+)", line)
            if loops_match:
                final_loops = loops_match.group(1)

            fault_timeline.append(final_fault)

            if final_fault == "TIP_OVER":
                stats.tip_over_events += 1
                tip_over_streak += 1
                stats.max_tip_over_streak = max(stats.max_tip_over_streak, tip_over_streak)
            else:
                tip_over_streak = 0

    stats.foot_velocity_samples = len(foot_velocity_samples)
    stats.joint_velocity_samples = len(joint_velocity_samples)
    stats.foot_high_frequency_energy_ratio = high_frequency_energy_ratio(foot_velocity_samples)
    stats.joint_high_frequency_energy_ratio = high_frequency_energy_ratio(joint_velocity_samples)
    timeline_text = ",".join(fault_timeline)
    stats.fault_timeline_hash = hashlib.sha256(timeline_text.encode("utf-8")).hexdigest()[:12]
    stats.peak_command_tuple = (
        round(stats.peak_foot_vel_mps, 3),
        round(stats.peak_velocity_radps, 3),
    )
    stats.final_status_summary = f"mode={final_mode};fault={final_fault};loops={final_loops}"
    return stats


def build_fingerprint(stats: LogStats) -> ScenarioFingerprint:
    return ScenarioFingerprint(
        fault_timeline_hash=stats.fault_timeline_hash,
        peak_command_tuple=stats.peak_command_tuple,
        final_status_summary=stats.final_status_summary,
    )


def load_fingerprint_store(path: Path) -> dict[str, ScenarioFingerprint]:
    if not path.exists():
        return {}

    payload = json.loads(path.read_text(encoding="utf-8"))
    fingerprints: dict[str, ScenarioFingerprint] = {}
    for scenario, value in payload.items():
        tuple_values = value.get("peak_command_tuple", [0.0, 0.0])
        fingerprints[scenario] = ScenarioFingerprint(
            fault_timeline_hash=str(value.get("fault_timeline_hash", "")),
            peak_command_tuple=(float(tuple_values[0]), float(tuple_values[1])),
            final_status_summary=str(value.get("final_status_summary", "unknown")),
        )
    return fingerprints


def save_fingerprint_store(path: Path, store: dict[str, ScenarioFingerprint]) -> None:
    serializable = {
        scenario: {
            "fault_timeline_hash": fingerprint.fault_timeline_hash,
            "peak_command_tuple": [
                fingerprint.peak_command_tuple[0],
                fingerprint.peak_command_tuple[1],
            ],
            "final_status_summary": fingerprint.final_status_summary,
        }
        for scenario, fingerprint in sorted(store.items())
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(serializable, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_cmd(command: list[str], cwd: Path) -> int:
    print("+", " ".join(command))
    proc = subprocess.run(command, cwd=cwd)
    return proc.returncode


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scenario regression harness for observability and safety metrics.",
    )
    parser.add_argument(
        "--server-dir",
        default="hexapod-server",
        help="Path to hexapod-server directory (default: hexapod-server)",
    )
    parser.add_argument(
        "--scenario",
        default="scenarios/05_long_walk_observability.toml",
        help="Scenario path relative to server dir.",
    )
    parser.add_argument(
        "--server-bin",
        default="build/hexapod-server",
        help="Server binary path relative to server dir.",
    )
    parser.add_argument(
        "--configure",
        action="store_true",
        help="Run cmake --preset default before build.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip build step and run existing binary.",
    )
    parser.add_argument(
        "--threshold-profile",
        choices=sorted(THRESHOLD_PRESETS.keys()),
        default="current-baseline",
        help=(
            "Threshold preset profile. "
            "Use 'current-baseline' for current envelope or 'strict' for tighter historical bounds."
        ),
    )
    parser.add_argument(
        "--max-peak-foot-vel-mps",
        type=float,
        default=15.0,
        help="Fail if runtime.metrics peak_foot_vel_mps exceeds this threshold.",
    )
    parser.add_argument(
        "--max-peak-velocity-radps",
        type=float,
        default=120.0,
        help="Fail if runtime.metrics peak_velocity_radps exceeds this threshold.",
    )
    parser.add_argument(
        "--max-foot-high-frequency-energy-ratio",
        type=float,
        default=None,
        help=(
            "Fail if high-frequency spectral energy ratio of runtime.metrics peak_foot_vel_mps "
            "sequence exceeds this threshold."
        ),
    )
    parser.add_argument(
        "--max-joint-high-frequency-energy-ratio",
        type=float,
        default=None,
        help=(
            "Fail if high-frequency spectral energy ratio of runtime.metrics peak_velocity_radps "
            "sequence exceeds this threshold."
        ),
    )
    parser.add_argument(
        "--tip-over-persist-streak",
        type=int,
        default=None,
        help="Override profile threshold for consecutive fault=TIP_OVER status updates.",
    )
    parser.add_argument(
        "--output-log",
        default="build/scenario_05_long_walk_regression.log",
        help="Log output path relative to server dir.",
    )
    parser.add_argument(
        "--fingerprint-store",
        default="scripts/fixtures/scenario_regression_fingerprints.json",
        help="Fingerprint store path relative to repository root.",
    )
    parser.add_argument(
        "--refresh-fingerprint",
        action="store_true",
        help="Update stored fingerprint for the scenario instead of validating drift.",
    )
    args = parser.parse_args()
    thresholds = resolve_thresholds(args)
    validate_thresholds(thresholds)

    repo_root = Path(__file__).resolve().parents[1]
    server_dir = (repo_root / args.server_dir).resolve()
    scenario_path = server_dir / args.scenario
    server_bin = server_dir / args.server_bin
    sim_cfg = server_dir / "config.sim.txt"
    active_cfg = server_dir / "config.txt"
    log_path = server_dir / args.output_log
    fingerprint_store_path = repo_root / args.fingerprint_store

    if not server_dir.exists():
        raise SystemExit(f"ERROR: server dir does not exist: {server_dir}")
    if not scenario_path.exists():
        raise SystemExit(f"ERROR: scenario not found: {scenario_path}")
    if not sim_cfg.exists():
        raise SystemExit(f"ERROR: sim config not found: {sim_cfg}")

    if args.configure:
        code = run_cmd(["cmake", "--preset", "default"], server_dir)
        if code != 0:
            return code

    if not args.skip_build:
        code = run_cmd(["cmake", "--build", "--preset", "default", "-j"], server_dir)
        if code != 0:
            return code

    if not server_bin.exists():
        raise SystemExit(f"ERROR: server binary not found: {server_bin}")

    backup_data = active_cfg.read_bytes() if active_cfg.exists() else None

    try:
        active_cfg.write_bytes(sim_cfg.read_bytes())
        log_path.parent.mkdir(parents=True, exist_ok=True)
        scenario_rel = str(scenario_path.relative_to(server_dir))
        command = [str(server_bin), "--scenario", scenario_rel]
        print("+", " ".join(command), f"> {log_path}")

        with log_path.open("w", encoding="utf-8") as handle:
            proc = subprocess.run(command, cwd=server_dir, stdout=handle, stderr=subprocess.STDOUT)

        exit_ok = proc.returncode == 0
        stats = analyze_log(log_path)
        fingerprint = build_fingerprint(stats)
        store = load_fingerprint_store(fingerprint_store_path)

        failures: list[str] = []
        if not exit_ok:
            failures.append(f"scenario exited with code {proc.returncode}")
        if stats.runtime_metrics_lines == 0:
            failures.append("no runtime.metrics entries were found")
        if (
            not math.isfinite(stats.peak_foot_vel_mps)
            or stats.peak_foot_vel_mps > thresholds.max_peak_foot_vel_mps
        ):
            failures.append(
                f"peak_foot_vel_mps={stats.peak_foot_vel_mps:.6g} exceeds {thresholds.max_peak_foot_vel_mps:.6g}"
            )
        if (
            not math.isfinite(stats.peak_velocity_radps)
            or stats.peak_velocity_radps > thresholds.max_peak_velocity_radps
        ):
            failures.append(
                f"peak_velocity_radps={stats.peak_velocity_radps:.6g} exceeds {thresholds.max_peak_velocity_radps:.6g}"
            )
        if stats.foot_high_frequency_energy_ratio > thresholds.max_foot_high_frequency_energy_ratio:
            failures.append(
                "foot velocity high-frequency spectral energy ratio="
                f"{stats.foot_high_frequency_energy_ratio:.6g} exceeds "
                f"{thresholds.max_foot_high_frequency_energy_ratio:.6g}"
            )
        if stats.joint_high_frequency_energy_ratio > thresholds.max_joint_high_frequency_energy_ratio:
            failures.append(
                "joint velocity high-frequency spectral energy ratio="
                f"{stats.joint_high_frequency_energy_ratio:.6g} exceeds "
                f"{thresholds.max_joint_high_frequency_energy_ratio:.6g}"
            )
        if stats.max_tip_over_streak >= thresholds.tip_over_persist_streak:
            failures.append(
                "persistent fault=TIP_OVER observed "
                f"(max consecutive streak={stats.max_tip_over_streak}, threshold={thresholds.tip_over_persist_streak})"
            )

        expected_fingerprint = store.get(scenario_rel)
        if args.refresh_fingerprint or expected_fingerprint is None:
            store[scenario_rel] = fingerprint
            save_fingerprint_store(fingerprint_store_path, store)
        elif expected_fingerprint != fingerprint:
            failures.append(
                "scenario fingerprint drift detected "
                f"(expected={expected_fingerprint}, actual={fingerprint})"
            )

        print("Scenario regression summary")
        print(f"  scenario: {scenario_rel}")
        print(f"  threshold profile: {args.threshold_profile}")
        print(f"  log: {log_path}")
        print(f"  runtime.metrics lines: {stats.runtime_metrics_lines}")
        print(f"  peak_foot_vel_mps: {stats.peak_foot_vel_mps:.6g}")
        print(f"  peak_velocity_radps: {stats.peak_velocity_radps:.6g}")
        print(f"  foot velocity samples: {stats.foot_velocity_samples}")
        print(f"  joint velocity samples: {stats.joint_velocity_samples}")
        print(f"  foot high-frequency energy ratio: {stats.foot_high_frequency_energy_ratio:.6g}")
        print(f"  joint high-frequency energy ratio: {stats.joint_high_frequency_energy_ratio:.6g}")
        print(f"  fault=TIP_OVER events: {stats.tip_over_events}")
        print(f"  fault=TIP_OVER max consecutive streak: {stats.max_tip_over_streak}")
        print("  fingerprint:")
        print(f"    store: {fingerprint_store_path}")
        print(f"    fault_timeline_hash: {fingerprint.fault_timeline_hash}")
        print(
            "    peak_command_tuple: "
            f"({fingerprint.peak_command_tuple[0]:.3f}, {fingerprint.peak_command_tuple[1]:.3f})"
        )
        print(f"    final_status_summary: {fingerprint.final_status_summary}")
        print("  thresholds:")
        print(f"    max_peak_foot_vel_mps: {thresholds.max_peak_foot_vel_mps:.6g}")
        print(f"    max_peak_velocity_radps: {thresholds.max_peak_velocity_radps:.6g}")
        print(
            "    max_foot_high_frequency_energy_ratio: "
            f"{thresholds.max_foot_high_frequency_energy_ratio:.6g}"
        )
        print(
            "    max_joint_high_frequency_energy_ratio: "
            f"{thresholds.max_joint_high_frequency_energy_ratio:.6g}"
        )
        print(f"    tip_over_persist_streak: {thresholds.tip_over_persist_streak}")

        if failures:
            print("\nFAIL:")
            for failure in failures:
                print(" -", failure)
            return 1

        print("\nPASS: scenario regression checks satisfied.")
        return 0
    finally:
        if backup_data is None:
            if active_cfg.exists():
                active_cfg.unlink()
        else:
            active_cfg.write_bytes(backup_data)


if __name__ == "__main__":
    sys.exit(main())
