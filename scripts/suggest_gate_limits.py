#!/usr/bin/env python3
"""
Ingest one or more metrics JSONL files (from --emit-metrics-json), group by (suite, name),
and print per-metric p50 / p95 / max plus a suggested ceiling = p95 * slack.

Usage:
  python3 scripts/suggest_gate_limits.py [--slack 1.05] [--suite REGEX] [--name REGEX] run1.jsonl run2.jsonl
  cat *.jsonl | python3 scripts/suggest_gate_limits.py --slack 1.02
  python3 scripts/suggest_gate_limits.py --emit-manifest-fragment --fragment-suite motion_performance \\
      --fragment-name compass_forward --slack 1.05 capture.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import defaultdict
from typing import Any, Dict, Iterable, List, Mapping, Optional, Pattern, Tuple


def _iter_json_lines(stream: Iterable[str]) -> Iterable[dict]:
    for line in stream:
        line = line.strip()
        if not line or not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict) and "metrics" in obj:
            yield obj


def _flatten_metrics(obj: Any, prefix: str = "") -> List[Tuple[str, float]]:
    out: List[Tuple[str, float]] = []
    if isinstance(obj, bool):
        return out
    if isinstance(obj, (int, float)):
        if isinstance(obj, float) and not math.isfinite(obj):
            return out
        out.append((prefix.rstrip("."), float(obj)))
        return out
    if isinstance(obj, Mapping):
        for k, v in obj.items():
            key = f"{prefix}{k}" if not prefix else f"{prefix}.{k}"
            out.extend(_flatten_metrics(v, key))
        return out
    return out


def _percentile(sorted_vals: List[float], q: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = (len(sorted_vals) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_vals[lo]
    w = idx - lo
    return sorted_vals[lo] * (1.0 - w) + sorted_vals[hi] * w


def main() -> int:
    ap = argparse.ArgumentParser(description="Suggest numeric gate ceilings from metrics JSONL.")
    ap.add_argument(
        "jsonl",
        nargs="*",
        help="JSONL files (default: read stdin)",
    )
    ap.add_argument("--slack", type=float, default=1.05, help="Multiply p95 by this factor for suggested ceiling.")
    ap.add_argument("--suite", type=str, default=None, help="Regex filter on suite (default: all).")
    ap.add_argument("--name", type=str, default=None, help="Regex filter on case name (default: all).")
    ap.add_argument("--min-runs", type=int, default=1, help="Minimum samples per (suite,name) to print.")
    ap.add_argument(
        "--only-passed",
        action="store_true",
        help="Use only lines where passed is true (default: include failed runs too).",
    )
    ap.add_argument(
        "--emit-manifest-fragment",
        action="store_true",
        help="Print a schema_version=1 JSON document with suites[SUITE][NAME] flat numeric limits from p95*slack.",
    )
    ap.add_argument(
        "--fragment-suite",
        type=str,
        default=None,
        help="Exact suite id for --emit-manifest-fragment (e.g. motion_performance).",
    )
    ap.add_argument(
        "--fragment-name",
        type=str,
        default=None,
        help="Exact case name for --emit-manifest-fragment (e.g. compass_forward).",
    )
    args = ap.parse_args()

    suite_re = re.compile(args.suite) if args.suite else None
    name_re = re.compile(args.name) if args.name else None

    # (suite, name) -> metric_key -> list of values
    buckets: Dict[Tuple[str, str], Dict[str, List[float]]] = defaultdict(lambda: defaultdict(list))

    if args.jsonl:
        streams: List[Iterable[str]] = []
        for path in args.jsonl:
            streams.append(open(path, encoding="utf-8"))
        try:
            for f in streams:
                for rec in _iter_json_lines(f):
                    _ingest_rec(rec, buckets, suite_re, name_re, args.only_passed)
        finally:
            for f in streams:
                f.close()
    else:
        for rec in _iter_json_lines(sys.stdin):
            _ingest_rec(rec, buckets, suite_re, name_re, args.only_passed)

    if args.emit_manifest_fragment:
        if not args.fragment_suite or not args.fragment_name:
            print(
                "error: --emit-manifest-fragment requires --fragment-suite and --fragment-name",
                file=sys.stderr,
            )
            return 2
        key = (args.fragment_suite, args.fragment_name)
        if key not in buckets:
            print(f"error: no samples for suite={args.fragment_suite!r} name={args.fragment_name!r}", file=sys.stderr)
            return 1
        metrics_map = buckets[key]
        case_limits: Dict[str, float] = {}
        for mkey in sorted(metrics_map.keys()):
            vals = sorted(metrics_map[mkey])
            n = len(vals)
            if n < args.min_runs:
                continue
            p95 = _percentile(vals, 0.95)
            suggested = p95 * args.slack
            if not math.isfinite(suggested):
                continue
            leaf = mkey.split(".")[-1] if "." in mkey else mkey
            case_limits[leaf] = float(suggested)
        fragment: Dict[str, Any] = {
            "schema_version": 1,
            "comment": "fragment from suggest_gate_limits.py --emit-manifest-fragment (merge under docs/testing-baselines/gates/)",
            "suites": {args.fragment_suite: {args.fragment_name: case_limits}},
        }
        json.dump(fragment, sys.stdout, indent=2)
        sys.stdout.write("\n")
        return 0

    for (suite, name) in sorted(buckets.keys()):
        metrics_map = buckets[(suite, name)]
        n = max(len(v) for v in metrics_map.values()) if metrics_map else 0
        if n < args.min_runs:
            continue
        print(f"\n## {suite} / {name}  (samples ~ {n})")
        for mkey in sorted(metrics_map.keys()):
            vals = sorted(metrics_map[mkey])
            p50 = _percentile(vals, 0.50)
            p95 = _percentile(vals, 0.95)
            vmax = vals[-1]
            suggested = p95 * args.slack
            print(
                f"  {mkey}: p50={p50:.6g} p95={p95:.6g} max={vmax:.6g}  "
                f"suggested_ceiling@slack={args.slack:g} -> {suggested:.6g}"
            )

    return 0


def _ingest_rec(
    rec: dict,
    buckets: Dict[Tuple[str, str], Dict[str, List[float]]],
    suite_re: Optional[Pattern[str]],
    name_re: Optional[Pattern[str]],
    only_passed: bool,
) -> None:
    if only_passed and not rec.get("passed", False):
        return
    suite = rec.get("suite") or "(unknown_suite)"
    name = rec.get("name") or "(unknown_name)"
    if suite_re and not suite_re.search(str(suite)):
        return
    if name_re and not name_re.search(str(name)):
        return
    metrics = rec.get("metrics")
    if not isinstance(metrics, dict):
        return
    flat = _flatten_metrics(metrics)
    key = (str(suite), str(name))
    for mk, mv in flat:
        buckets[key][mk].append(mv)


if __name__ == "__main__":
    raise SystemExit(main())
