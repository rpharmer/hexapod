#!/usr/bin/env python3
"""Save a compact, non-overwriting gait experiment process scorecard."""
import argparse
import hashlib
import json
import subprocess
import re
from datetime import datetime, timezone
from pathlib import Path
from summarize_gait_feasibility import load
from report_swing_clearance import summarize


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run_dir", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--experiment-note", default="")
    p.add_argument("--progress-log", type=Path, help="exit statuses for older runs without .exit sidecars")
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    processes = []
    exits = {}
    if args.progress_log:
        for line in args.progress_log.read_text().splitlines():
            match = re.fullmatch(r"done (\S+) (\S+)(?: (\d+))? exit=(\d+)", line)
            if match:
                screen, arm, run, code = match.groups()
                exits[(screen, arm, int(run or 1))] = int(code)
    for path in sorted(args.run_dir.glob("*.stdout")):
        screen, arm, run = path.stem.rsplit("-", 2)
        record = dict(screen=screen, arm=arm, run=int(run), log=str(path), sha256=sha(path))
        sidecar = path.with_suffix(".exit")
        record["exit_code"] = int(sidecar.read_text().strip()) if sidecar.exists() else exits.get((screen, arm, int(run)))
        if screen == "aggressive_governor":
            lines = [line for line in path.read_text().splitlines() if "stride_count=" in line]
            record["result_lines"] = lines
            record["passed"] = bool(lines) and all("passed=1" in line for line in lines)
        else:
            cases = load(path)
            expected = 5 if screen == "sequential" else 1
            record["complete"] = len(cases) == expected
            record["passed"] = record["complete"] and all(c["passed"] for c in cases)
            record["complete_held_zero"] = record["complete"] and all(c["metrics"]["solver_held_steps"] == 0 for c in cases)
            record["cases"] = []
            for case in cases:
                metrics = {key: value for key, value in case["metrics"].items()
                           if not isinstance(value, (list, dict))}
                metrics["swing_event_summary"] = summarize(case["metrics"].get("swing_events", []))
                record["cases"].append(dict(name=case["name"], passed=case["passed"], metrics=metrics))
        record["metrics_passed"] = record["passed"]
        record["passed"] = record["metrics_passed"] and record["exit_code"] == 0
        record["process_status_known"] = record["exit_code"] is not None
        processes.append(record)
    binaries = ["hexapod-server/build-tests/test_physics_sim_walk_distance",
                "hexapod-server/build-tests/test_locomotion_regression_suite",
                "hexapod-physics-sim/build/hexapod-physics-sim"]
    report = {
        "schema_version": 1, "reported_utc": datetime.now(timezone.utc).isoformat(),
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "provenance": "Existing dirty tree preserved. Identical binaries across arms; reporting-time hashes, no immutable capture-time tree manifest.",
        "binary_sha256": {path: sha(root / path) for path in binaries},
        "settings": {"solver_mode": 1, "iterations": 24, "body_height_m": .14,
                     "implicit_damping": False, "height_hold_scale": 1,
                     "selfweight_arm": "Bounded, self-weight on, foot-reaction off, scales 0/1/1, stiffness 1, LPF 0.08 s; existing limits retained" if any(r["arm"] == "selfweight" for r in processes) else None,
                     "experiment_note": args.experiment_note},
        "processes": processes,
        "scoring": "Incomplete processes never count as pass or complete/held-zero. Event clearance is measured FK; biased command error is not a gait-success criterion. Five repeats are a screen, not reliability qualification.",
    }
    serialized = json.dumps(report, indent=2, allow_nan=False)
    with args.output.open("x") as f:
        f.write(serialized + "\n")


if __name__ == "__main__":
    main()
