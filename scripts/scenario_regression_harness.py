#!/usr/bin/env python3
"""Run a scenario and enforce log-based regression checks."""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


FAULT_RE = re.compile(r"\bfault=([A-Z_]+)")
PEAK_FOOT_RE = re.compile(r"peak_foot_vel_mps:([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")
PEAK_JOINT_RE = re.compile(r"peak_velocity_radps:([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")


@dataclass
class LogStats:
    runtime_metrics_lines: int = 0
    peak_foot_vel_mps: float = 0.0
    peak_velocity_radps: float = 0.0
    tip_over_events: int = 0
    max_tip_over_streak: int = 0


def analyze_log(log_path: Path) -> LogStats:
    stats = LogStats()
    tip_over_streak = 0

    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "runtime.metrics" in line:
                stats.runtime_metrics_lines += 1

                match = PEAK_FOOT_RE.search(line)
                if match:
                    stats.peak_foot_vel_mps = max(stats.peak_foot_vel_mps, float(match.group(1)))

                match = PEAK_JOINT_RE.search(line)
                if match:
                    stats.peak_velocity_radps = max(stats.peak_velocity_radps, float(match.group(1)))

            fault_match = FAULT_RE.search(line)
            if not fault_match:
                continue

            if fault_match.group(1) == "TIP_OVER":
                stats.tip_over_events += 1
                tip_over_streak += 1
                stats.max_tip_over_streak = max(stats.max_tip_over_streak, tip_over_streak)
            else:
                tip_over_streak = 0

    return stats


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
        "--max-peak-foot-vel-mps",
        type=float,
        default=1.6,
        help="Fail if runtime.metrics peak_foot_vel_mps exceeds this threshold.",
    )
    parser.add_argument(
        "--max-peak-velocity-radps",
        type=float,
        default=25.0,
        help="Fail if runtime.metrics peak_velocity_radps exceeds this threshold.",
    )
    parser.add_argument(
        "--tip-over-persist-streak",
        type=int,
        default=3,
        help="Fail if fault=TIP_OVER appears in this many consecutive status updates.",
    )
    parser.add_argument(
        "--output-log",
        default="build/scenario_05_long_walk_regression.log",
        help="Log output path relative to server dir.",
    )
    args = parser.parse_args()

    if args.tip_over_persist_streak < 1:
        raise SystemExit("--tip-over-persist-streak must be >= 1")

    repo_root = Path(__file__).resolve().parents[1]
    server_dir = (repo_root / args.server_dir).resolve()
    scenario_path = server_dir / args.scenario
    server_bin = server_dir / args.server_bin
    sim_cfg = server_dir / "config.sim.txt"
    active_cfg = server_dir / "config.txt"
    log_path = server_dir / args.output_log

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

        failures: list[str] = []
        if not exit_ok:
            failures.append(f"scenario exited with code {proc.returncode}")
        if stats.runtime_metrics_lines == 0:
            failures.append("no runtime.metrics entries were found")
        if not math.isfinite(stats.peak_foot_vel_mps) or stats.peak_foot_vel_mps > args.max_peak_foot_vel_mps:
            failures.append(
                f"peak_foot_vel_mps={stats.peak_foot_vel_mps:.6g} exceeds {args.max_peak_foot_vel_mps:.6g}"
            )
        if not math.isfinite(stats.peak_velocity_radps) or stats.peak_velocity_radps > args.max_peak_velocity_radps:
            failures.append(
                f"peak_velocity_radps={stats.peak_velocity_radps:.6g} exceeds {args.max_peak_velocity_radps:.6g}"
            )
        if stats.max_tip_over_streak >= args.tip_over_persist_streak:
            failures.append(
                "persistent fault=TIP_OVER observed "
                f"(max consecutive streak={stats.max_tip_over_streak}, threshold={args.tip_over_persist_streak})"
            )

        print("Scenario regression summary")
        print(f"  scenario: {scenario_rel}")
        print(f"  log: {log_path}")
        print(f"  runtime.metrics lines: {stats.runtime_metrics_lines}")
        print(f"  peak_foot_vel_mps: {stats.peak_foot_vel_mps:.6g}")
        print(f"  peak_velocity_radps: {stats.peak_velocity_radps:.6g}")
        print(f"  fault=TIP_OVER events: {stats.tip_over_events}")
        print(f"  fault=TIP_OVER max consecutive streak: {stats.max_tip_over_streak}")

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
