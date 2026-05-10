#!/usr/bin/env python3
"""Build baseline-*.json metadata + aggregates from locomotion --emit-metrics-json JSONL.

See docs/testing-baselines/README.md and docs/TESTING_FUNCTIONALITY.md (A/B Benchmark Protocol).
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any, DefaultDict, Dict, List, Tuple

# Subset aligned with existing docs/testing-baselines/baseline-*-*.json files.
METRIC_KEYS = [
    "path_length_m",
    "net_displacement_m",
    "mean_horizontal_speed_mps",
    "peak_horizontal_speed_mps",
    "yaw_delta_rad",
    "mean_yaw_rate_radps",
    "peak_yaw_rate_radps",
    "stride_count",
    "max_abs_roll_rad",
    "max_abs_pitch_rad",
    "max_body_rate_radps",
]


def _median(vals: List[float]) -> float:
    if not vals:
        return float("nan")
    return float(statistics.median(vals))


def _minmax(vals: List[float]) -> Tuple[float, float]:
    return float(min(vals)), float(max(vals))


def _iter_records(path: Path) -> List[dict]:
    out: List[dict] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict) and "name" in obj and "metrics" in obj:
            out.append(obj)
    return out


def _aggregate_case(runs: List[dict]) -> Dict[str, Any]:
    passed = sum(1 for r in runs if r.get("passed") is True)
    pass_rate = passed / len(runs) if runs else 0.0
    agg: Dict[str, Any] = {"pass_rate": pass_rate}
    metrics = [r["metrics"] for r in runs if isinstance(r.get("metrics"), dict)]
    if not metrics:
        return agg
    for key in METRIC_KEYS:
        vals: List[float] = []
        for m in metrics:
            v = m.get(key)
            if isinstance(v, bool):
                continue
            if isinstance(v, (int, float)) and math.isfinite(float(v)):
                vals.append(float(v))
        if not vals:
            continue
        lo, hi = _minmax(vals)
        agg[key] = {"median": _median(vals), "min": lo, "max": hi}
    return agg


def build_from_jsonl(
    jsonl: Path,
    *,
    baseline_name: str,
    server_commit: str,
    sim_commit: str,
    config_path: str,
    build_type: str,
    suite_binary: str,
    suite_profile: str,
    requested_repetitions: int,
    suite_exit_codes: List[int],
) -> dict:
    records = _iter_records(jsonl)
    by_name: DefaultDict[str, List[dict]] = defaultdict(list)
    for r in records:
        name = str(r.get("name", ""))
        if name:
            by_name[name].append(r)

    runs_per_case = {name: len(rs) for name, rs in sorted(by_name.items())}
    aggregates = {name: _aggregate_case(rs) for name, rs in sorted(by_name.items())}

    return {
        "baseline_name": baseline_name,
        "server_commit": server_commit,
        "sim_commit": sim_commit,
        "config_path": config_path,
        "config_digest": "not-recorded",
        "build_type": build_type,
        "suite_binary": suite_binary,
        "suite_profile": suite_profile,
        "requested_repetitions": requested_repetitions,
        "suite_exit_codes": suite_exit_codes,
        "runs_per_case": runs_per_case,
        "aggregates": aggregates,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("jsonl", type=Path, help="Captured metrics JSONL")
    ap.add_argument("-o", "--output", type=Path, required=True, help="Write baseline JSON here")
    ap.add_argument("--baseline-name", required=True)
    ap.add_argument("--server-commit", required=True)
    ap.add_argument("--sim-commit", required=True)
    ap.add_argument("--config-path", default="hexapod-server/config.physics-sim-wsl.txt")
    ap.add_argument("--build-type", default="build-tests preset")
    ap.add_argument("--suite-binary", default="hexapod-server/build-tests/test_locomotion_regression_suite")
    ap.add_argument("--suite-profile", required=True, choices=("canonical", "stress"))
    ap.add_argument("--repetitions", type=int, required=True)
    ap.add_argument(
        "--exit-codes",
        required=True,
        help="Comma-separated suite exit codes (one per repetition), e.g. 0,0,0,0,0",
    )
    args = ap.parse_args()
    codes = [int(x.strip()) for x in args.exit_codes.split(",") if x.strip() != ""]
    doc = build_from_jsonl(
        args.jsonl,
        baseline_name=args.baseline_name,
        server_commit=args.server_commit,
        sim_commit=args.sim_commit,
        config_path=args.config_path,
        build_type=args.build_type,
        suite_binary=args.suite_binary,
        suite_profile=args.suite_profile,
        requested_repetitions=args.repetitions,
        suite_exit_codes=codes,
    )
    args.output.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
