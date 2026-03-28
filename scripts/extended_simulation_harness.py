#!/usr/bin/env python3
"""Run batched scenario simulation and enforce long-horizon drift budgets."""

from __future__ import annotations

import argparse
import math
import re
import statistics
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

FLOAT_RE = r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
INT_RE = r"(\d+)"

EST_ROLL_RE = re.compile(r"\best_roll_rad=" + FLOAT_RE)
EST_PITCH_RE = re.compile(r"\best_pitch_rad=" + FLOAT_RE)
FRESHNESS_REJECT_TOTAL_RE = re.compile(r"\bfreshness_rejects_total=" + INT_RE)
LOOPS_RE = re.compile(r"\bloops=" + INT_RE)

PEAK_JOINT_VEL_RE = re.compile(r"\bpeak_velocity_radps:" + FLOAT_RE)
PEAK_FOOT_VEL_RE = re.compile(r"\bpeak_foot_vel_mps:" + FLOAT_RE)


@dataclass(frozen=True)
class CycleSummary:
    cycle_index: int
    runtime_metrics_lines: int
    estimator_bias_start_rad: float
    estimator_bias_end_rad: float
    estimator_bias_drift_rad: float
    freshness_reject_growth_per_1k_loops: float
    cmd_peak_joint_vel_start_radps: float
    cmd_peak_joint_vel_end_radps: float
    cmd_peak_joint_vel_drift_radps: float
    cmd_peak_foot_vel_start_mps: float
    cmd_peak_foot_vel_end_mps: float
    cmd_peak_foot_vel_drift_mps: float


@dataclass(frozen=True)
class DriftBudgets:
    max_estimator_bias_drift_rad: float
    max_counter_growth_rate_drift_per_1k_loops: float
    max_cmd_peak_joint_vel_drift_radps: float
    max_cmd_peak_foot_vel_drift_mps: float


@dataclass(frozen=True)
class AggregateSummary:
    cycles: list[CycleSummary]
    estimator_bias_drift_rad: float
    counter_growth_rate_drift_per_1k_loops: float
    cmd_peak_joint_vel_drift_radps: float
    cmd_peak_foot_vel_drift_mps: float


def extract_float(pattern: re.Pattern[str], text: str) -> float | None:
    match = pattern.search(text)
    if not match:
        return None
    return float(match.group(1))


def extract_int(pattern: re.Pattern[str], text: str) -> int | None:
    match = pattern.search(text)
    if not match:
        return None
    return int(match.group(1))


def window_means(values: list[float], window_fraction: float = 0.2) -> tuple[float, float]:
    if not values:
        return (0.0, 0.0)
    window = max(1, int(math.ceil(len(values) * window_fraction)))
    start = statistics.fmean(values[:window])
    end = statistics.fmean(values[-window:])
    return (start, end)


def compute_cycle_summary(cycle_index: int, log_path: Path) -> CycleSummary:
    estimator_bias_values: list[float] = []
    counter_rates: list[float] = []
    cmd_peak_joint_vel_values: list[float] = []
    cmd_peak_foot_vel_values: list[float] = []

    runtime_metrics_lines = 0

    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "runtime.metrics" in line:
                runtime_metrics_lines += 1
                est_roll = extract_float(EST_ROLL_RE, line)
                est_pitch = extract_float(EST_PITCH_RE, line)
                loops = extract_int(LOOPS_RE, line)
                freshness_rejects = extract_int(FRESHNESS_REJECT_TOTAL_RE, line)

                if est_roll is not None and est_pitch is not None:
                    estimator_bias_values.append(max(abs(est_roll), abs(est_pitch)))

                if loops is not None and loops > 0 and freshness_rejects is not None:
                    counter_rates.append((freshness_rejects / loops) * 1000.0)

                peak_joint_vel = extract_float(PEAK_JOINT_VEL_RE, line)
                peak_foot_vel = extract_float(PEAK_FOOT_VEL_RE, line)
                if peak_joint_vel is not None:
                    cmd_peak_joint_vel_values.append(peak_joint_vel)
                if peak_foot_vel is not None:
                    cmd_peak_foot_vel_values.append(peak_foot_vel)

    est_start, est_end = window_means(estimator_bias_values)
    ctr_start, ctr_end = window_means(counter_rates)
    cmd_joint_start, cmd_joint_end = window_means(cmd_peak_joint_vel_values)
    cmd_foot_start, cmd_foot_end = window_means(cmd_peak_foot_vel_values)

    return CycleSummary(
        cycle_index=cycle_index,
        runtime_metrics_lines=runtime_metrics_lines,
        estimator_bias_start_rad=est_start,
        estimator_bias_end_rad=est_end,
        estimator_bias_drift_rad=est_end - est_start,
        freshness_reject_growth_per_1k_loops=ctr_end,
        cmd_peak_joint_vel_start_radps=cmd_joint_start,
        cmd_peak_joint_vel_end_radps=cmd_joint_end,
        cmd_peak_joint_vel_drift_radps=cmd_joint_end - cmd_joint_start,
        cmd_peak_foot_vel_start_mps=cmd_foot_start,
        cmd_peak_foot_vel_end_mps=cmd_foot_end,
        cmd_peak_foot_vel_drift_mps=cmd_foot_end - cmd_foot_start,
    )


def aggregate(cycles: list[CycleSummary]) -> AggregateSummary:
    if not cycles:
        return AggregateSummary([], 0.0, 0.0, 0.0, 0.0)

    first = cycles[0]
    last = cycles[-1]
    return AggregateSummary(
        cycles=cycles,
        estimator_bias_drift_rad=last.estimator_bias_end_rad - first.estimator_bias_start_rad,
        counter_growth_rate_drift_per_1k_loops=(
            last.freshness_reject_growth_per_1k_loops - first.freshness_reject_growth_per_1k_loops
        ),
        cmd_peak_joint_vel_drift_radps=(last.cmd_peak_joint_vel_end_radps - first.cmd_peak_joint_vel_start_radps),
        cmd_peak_foot_vel_drift_mps=(last.cmd_peak_foot_vel_end_mps - first.cmd_peak_foot_vel_start_mps),
    )


def validate_budgets(summary: AggregateSummary, budgets: DriftBudgets) -> list[str]:
    failures: list[str] = []
    if abs(summary.estimator_bias_drift_rad) > budgets.max_estimator_bias_drift_rad:
        failures.append(
            "estimator bias drift exceeded budget: "
            f"|{summary.estimator_bias_drift_rad:.6g}| > {budgets.max_estimator_bias_drift_rad:.6g}"
        )
    if abs(summary.counter_growth_rate_drift_per_1k_loops) > budgets.max_counter_growth_rate_drift_per_1k_loops:
        failures.append(
            "counter growth-rate drift exceeded budget: "
            f"|{summary.counter_growth_rate_drift_per_1k_loops:.6g}| > "
            f"{budgets.max_counter_growth_rate_drift_per_1k_loops:.6g}"
        )
    if abs(summary.cmd_peak_joint_vel_drift_radps) > budgets.max_cmd_peak_joint_vel_drift_radps:
        failures.append(
            "command peak joint velocity drift exceeded budget: "
            f"|{summary.cmd_peak_joint_vel_drift_radps:.6g}| > {budgets.max_cmd_peak_joint_vel_drift_radps:.6g}"
        )
    if abs(summary.cmd_peak_foot_vel_drift_mps) > budgets.max_cmd_peak_foot_vel_drift_mps:
        failures.append(
            "command peak foot velocity drift exceeded budget: "
            f"|{summary.cmd_peak_foot_vel_drift_mps:.6g}| > {budgets.max_cmd_peak_foot_vel_drift_mps:.6g}"
        )
    return failures


def run_cmd(command: list[str], cwd: Path) -> int:
    print("+", " ".join(command))
    proc = subprocess.run(command, cwd=cwd)
    return proc.returncode


def print_summary(summary: AggregateSummary, scenario_rel: str, simulated_hours: float) -> None:
    print("Extended simulation summary")
    print(f"  scenario: {scenario_rel}")
    print(f"  cycles: {len(summary.cycles)}")
    print(f"  simulated_horizon_hours: {simulated_hours:.3f}")
    print(f"  estimator_bias_drift_rad: {summary.estimator_bias_drift_rad:.6g}")
    print(
        "  counter_growth_rate_drift_per_1k_loops: "
        f"{summary.counter_growth_rate_drift_per_1k_loops:.6g}"
    )
    print(f"  cmd_peak_joint_vel_drift_radps: {summary.cmd_peak_joint_vel_drift_radps:.6g}")
    print(f"  cmd_peak_foot_vel_drift_mps: {summary.cmd_peak_foot_vel_drift_mps:.6g}")


def default_output_dir(server_dir: Path) -> Path:
    return server_dir / "build" / "extended-sim"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run batched scenario cycles and enforce long-horizon drift budgets for estimator bias, "
            "counter growth rate, and command envelopes."
        )
    )
    parser.add_argument("--server-dir", default="hexapod-server")
    parser.add_argument("--scenario", default="scenarios/11_endurance_speed_mix.toml")
    parser.add_argument("--server-bin", default="build/hexapod-server")
    parser.add_argument("--cycles", type=int, default=40, help="Number of scenario cycles to execute.")
    parser.add_argument(
        "--sim-hours-scale",
        type=float,
        default=1.0,
        help="Optional acceleration factor used when reporting simulated horizon.",
    )
    parser.add_argument("--configure", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--output-dir", default=None)

    parser.add_argument("--max-estimator-bias-drift-rad", type=float, default=0.12)
    parser.add_argument("--max-counter-growth-rate-drift-per-1k-loops", type=float, default=1.5)
    parser.add_argument("--max-cmd-peak-joint-vel-drift-radps", type=float, default=8.0)
    parser.add_argument("--max-cmd-peak-foot-vel-drift-mps", type=float, default=2.5)

    args = parser.parse_args()

    if args.cycles < 1:
        raise SystemExit("--cycles must be >= 1")
    if args.sim_hours_scale <= 0 or not math.isfinite(args.sim_hours_scale):
        raise SystemExit("--sim-hours-scale must be finite and > 0")

    budgets = DriftBudgets(
        max_estimator_bias_drift_rad=args.max_estimator_bias_drift_rad,
        max_counter_growth_rate_drift_per_1k_loops=args.max_counter_growth_rate_drift_per_1k_loops,
        max_cmd_peak_joint_vel_drift_radps=args.max_cmd_peak_joint_vel_drift_radps,
        max_cmd_peak_foot_vel_drift_mps=args.max_cmd_peak_foot_vel_drift_mps,
    )

    repo_root = Path(__file__).resolve().parents[1]
    server_dir = (repo_root / args.server_dir).resolve()
    scenario_path = server_dir / args.scenario
    server_bin = server_dir / args.server_bin
    sim_cfg = server_dir / "config.sim.txt"
    active_cfg = server_dir / "config.txt"
    output_dir = Path(args.output_dir).resolve() if args.output_dir else default_output_dir(server_dir)

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
    cycles: list[CycleSummary] = []

    try:
        active_cfg.write_bytes(sim_cfg.read_bytes())
        output_dir.mkdir(parents=True, exist_ok=True)
        scenario_rel = str(scenario_path.relative_to(server_dir))

        for cycle in range(1, args.cycles + 1):
            cycle_log = output_dir / f"cycle_{cycle:03d}.log"
            cmd = [str(server_bin), "--scenario", scenario_rel]
            print("+", " ".join(cmd), f"> {cycle_log}")
            with cycle_log.open("w", encoding="utf-8") as handle:
                proc = subprocess.run(cmd, cwd=server_dir, stdout=handle, stderr=subprocess.STDOUT)

            if proc.returncode != 0:
                print(f"FAIL: cycle {cycle} exited with code {proc.returncode}")
                return proc.returncode

            cycle_summary = compute_cycle_summary(cycle, cycle_log)
            if cycle_summary.runtime_metrics_lines == 0:
                print(f"FAIL: cycle {cycle} produced no runtime.metrics lines")
                return 1
            cycles.append(cycle_summary)

        summary = aggregate(cycles)
        scenario_duration_h = (cycles and (scenario_duration_ms(scenario_path) / 3_600_000.0) or 0.0)
        simulated_hours = len(cycles) * scenario_duration_h * args.sim_hours_scale
        print_summary(summary, scenario_rel, simulated_hours)

        failures = validate_budgets(summary, budgets)
        if failures:
            print("\nFAIL: long-horizon drift budget exceeded")
            for failure in failures:
                print(" -", failure)
            return 1

        print("\nPASS: long-horizon drift budgets satisfied.")
        return 0
    finally:
        if backup_data is None:
            if active_cfg.exists():
                active_cfg.unlink()
        else:
            active_cfg.write_bytes(backup_data)


def scenario_duration_ms(path: Path) -> int:
    # Keep parser lightweight to avoid adding TOML dependency to scripts.
    duration_re = re.compile(r"^\s*duration_ms\s*=\s*(\d+)\s*$")
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = duration_re.match(line)
            if match:
                return int(match.group(1))
    raise SystemExit(f"ERROR: could not parse duration_ms from scenario file: {path}")


if __name__ == "__main__":
    sys.exit(main())
