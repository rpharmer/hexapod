#!/usr/bin/env python3
"""Bounded rigid-mode A/B screen; never changes gates, fixtures, or defaults."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--screen", choices=(
        "aggressive", "straight", "turn", "reverse", "slow", "sequential",
        "turn-after-reverse", "turn-after-reverse-straight"), required=True)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--budget", choices=(8, 9, 10), type=int, default=10)
    parser.add_argument("--baseline", action="store_true")
    parser.add_argument("--capture-speed-limit", action="store_true",
                        help="capture one full-PD initiating speed trip per run, without changing guards")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be positive")
    root = Path(__file__).resolve().parents[1]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    for key in (
        "HEXAPOD_WALK_TEST_CASE", "HEXAPOD_WALK_TEST_STEP_LIMIT",
        "HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT",
        "HEXAPOD_SWING_LINK_RATE_EXPERIMENT", "HEXAPOD_SWING_LINK_RATE_TRACE",
        "HEXAPOD_SWING_LINK_RATE_BUDGET_RADPS",
        "HEXAPOD_PINOCCHIO_DISABLE_CONTACT_INERTIA",
        "HEXAPOD_PINOCCHIO_SPEED_LIMIT_SNAPSHOT_PATH",
        "HEXAPOD_PHYSICS_TRACE_PUBLISHED_LINK_SPEED",
    ):
        environment.pop(key, None)
    environment.update({
        "HEXAPOD_PHYSICS_SIM_EXE": str(root / "hexapod-physics-sim/build/hexapod-physics-sim"),
        "HEXAPOD_WALK_TEST_SOLVER_MODE": "pinocchio-proximal",
        "HEXAPOD_WALK_TEST_SOLVER_ITERATIONS": "24",
        "HEXAPOD_WALK_TEST_BODY_HEIGHT_M": "0.14",
        "HEXAPOD_LOCOMOTION_CHILD_STDIO": "1",
        "HEXAPOD_WALK_TEST_CHILD_STDIO": "1",
        "HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT": "1",
        "HEXAPOD_PROXIMAL_TRACE_FAILURES": "1",
    })
    if not args.baseline:
        environment.update({
            "HEXAPOD_SWING_LINK_RATE_EXPERIMENT": "1",
            "HEXAPOD_SWING_LINK_RATE_TRACE": "1",
            "HEXAPOD_SWING_LINK_RATE_BUDGET_RADPS": str(args.budget),
        })
    if args.screen == "aggressive":
        binary = root / "hexapod-server/build-tests/test_locomotion_regression_suite"
        command = [str(binary), "--profile", "canonical", "--case", "aggressive_governor",
                   "--solver-mode", "pinocchio-proximal", "--emit-metrics-json"]
    else:
        binary = root / "hexapod-server/build-tests/test_physics_sim_walk_distance"
        command = [str(binary), "--emit-metrics-json"]
        if args.screen not in ("sequential", "turn-after-reverse", "turn-after-reverse-straight"):
            environment["HEXAPOD_WALK_TEST_CASE"] = {
                "straight": "straight_walk",
                "turn": "turn_in_place",
                "reverse": "reverse_walk",
                "slow": "slow_forward_walk",
            }[args.screen]
        elif args.screen == "turn-after-reverse":
            environment["HEXAPOD_WALK_TEST_CASE"] = "turn_after_reverse"
        elif args.screen == "turn-after-reverse-straight":
            environment["HEXAPOD_WALK_TEST_CASE"] = "turn_after_reverse_straight"
    git = lambda *arguments: subprocess.check_output(["git", *arguments], cwd=root, text=True).strip()
    fixture = root / "docs/contact-snapshots/speed-limit-rigid-v2.json"
    report = {
        "schema_version": 1, "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "head": git("rev-parse", "HEAD"), "dirty_status": git("status", "--short"),
        "tracked_diff_sha256": hashlib.sha256(subprocess.check_output(
            ["git", "diff", "--binary", "HEAD"], cwd=root)).hexdigest(),
        "governor_source_sha256": {
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (
                root / "hexapod-server/include/control/leg_link_angular_velocity.hpp",
                root / "hexapod-server/include/control/swing_link_rate_governor.hpp",
                root / "hexapod-server/src/control/robot_runtime.cpp",
            )
        },
        "fixture_sha256": hashlib.sha256(fixture.read_bytes()).hexdigest(),
        "server_binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "sim_binary_sha256": hashlib.sha256(Path(environment["HEXAPOD_PHYSICS_SIM_EXE"]).read_bytes()).hexdigest(),
        "screen": args.screen, "baseline": args.baseline, "solver_mode": 1,
        "solver_iterations": 24, "body_height_m": 0.14, "budget_radps": args.budget,
        "command": command, "runs": [], "decision": "follow-up",
        "hexapod_environment": {key: value for key, value in sorted(environment.items()) if key.startswith("HEXAPOD_")},
    }
    prefix = f"{args.screen}-{'baseline' if args.baseline else 'governed-' + str(args.budget)}"
    for run in range(1, args.runs + 1):
        snapshot = args.output_dir / f"{prefix}-{run}-speed-limit.json"
        if args.capture_speed_limit:
            if snapshot.exists():
                parser.error(f"refusing to overwrite existing snapshot: {snapshot}")
            environment["HEXAPOD_PINOCCHIO_SPEED_LIMIT_SNAPSHOT_PATH"] = str(snapshot.resolve())
            environment["HEXAPOD_PHYSICS_TRACE_PUBLISHED_LINK_SPEED"] = "1"
        start = time.monotonic()
        process = subprocess.Popen(command, cwd=root, env=environment, text=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        timed_out = False
        try:
            stdout, stderr = process.communicate(timeout=180)
        except subprocess.TimeoutExpired:
            timed_out = True
            # This newly-created process group includes only this test and its
            # simulator child, not an unrelated user physics stack.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                stdout, stderr = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                stdout, stderr = process.communicate()
        completed = subprocess.CompletedProcess(command, 124 if timed_out else process.returncode, stdout, stderr)
        log = args.output_dir / f"{prefix}-{run}.log"
        log.write_text(completed.stdout + "\n--- STDERR (separate capture) ---\n" + completed.stderr)
        metrics = []
        counts = {"inactive": 0, "unavailable": 0, "infeasible": 0, "unchanged": 0, "limited": 0}
        labels = tuple(counts)
        minimum_scale = 1.0
        worst_predicted = 0.0
        parse_errors = []
        published_violations = []
        for line in completed.stdout.splitlines():
            if line.startswith("{"):
                try:
                    metrics.append(json.loads(line))
                except json.JSONDecodeError as error:
                    parse_errors.append(str(error))
        for line in completed.stderr.splitlines():
            if line.startswith("[published-link-speed] "):
                published_violations.append(json.loads(line.removeprefix("[published-link-speed] ")))
            if "[swing-link-rate]" in line:
                fields = dict(re.findall(r"(\w+)=([^ ]+)", line))
                counts[labels[int(fields["status"])]] += 1
                minimum_scale = min(minimum_scale, float(fields["scale"]))
                if int(fields["status"]) in (3, 4):
                    worst_predicted = max(worst_predicted, *(float(x) for x in fields["predicted_after"].split(",")))
        entry = {"run": run, "exit_code": completed.returncode,
                 "timed_out": timed_out,
                 "speed_limit_snapshot": str(snapshot) if snapshot.exists() else None,
                 "published_link_speed_violations": published_violations,
                 "wall_seconds": time.monotonic() - start, "log": str(log),
                 "governor_counts": counts, "minimum_scale": minimum_scale,
                 "maximum_feasible_predicted_rate": worst_predicted, "metrics": metrics,
                 "metric_parse_errors": parse_errors,
                 "expected_metrics_on_pass": {
                     "sequential": 5,
                     "turn-after-reverse": 2,
                     "turn-after-reverse-straight": 3,
                 }.get(args.screen, 1),
                 "speed_limit_trace_lines": completed.stderr.count("[proximal-speed-limit]"),
                 "integrated_speed_limit_trace_lines": completed.stderr.count("[proximal-integrated-speed-limit]"),
                 "failure_expectations": [line for line in (completed.stdout + completed.stderr).splitlines()
                                          if "FAIL:" in line or "expectation failed" in line]}
        entry["activation_evidence_valid"] = args.baseline or (counts["unchanged"] + counts["limited"] > 0)
        entry["screen_passed"] = (completed.returncode == 0 and not parse_errors
                                  and len(metrics) == entry["expected_metrics_on_pass"]
                                  and all(metric.get("passed") is True for metric in metrics)
                                  and entry["activation_evidence_valid"])
        report["runs"].append(entry)
        (args.output_dir / f"{prefix}.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"{prefix} run={run} exit={completed.returncode} screen_passed={entry['screen_passed']} "
              f"governor={counts} minimum_scale={minimum_scale:.6g}", flush=True)
    return 0 if all(run["screen_passed"] for run in report["runs"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
