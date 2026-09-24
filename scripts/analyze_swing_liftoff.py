#!/usr/bin/env python3
"""Summarize the first quarter of a walk-capacity swing trace from stdin."""

import json
import statistics
import sys
from collections import defaultdict


def mean(values):
    return statistics.fmean(values) if values else None


def main():
    groups = defaultdict(list)
    count = 0
    for line in sys.stdin:
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get("kind") != "walk_capacity_trace":
            continue
        if record.get("schema_version") != 1:
            sys.exit("unsupported walk_capacity_trace schema")
        count += 1
        if record["planned_stance"]:
            continue
        tau = (record["phase"] - record["duty"]) / max(1.0 - record["duty"], 1e-9)
        if tau < 0.0 or tau >= 0.25:
            continue
        groups[record["leg"]].append(record)
    if count == 0:
        sys.exit("no walk_capacity_trace records on stdin")
    out = {"schema_version": 1, "trace_samples": count, "early_swing": {}}
    for leg, rows in sorted(groups.items()):
        ground = [
            r["measured_world_z_m"] - r["last_stance_z_m"]
            for r in rows if r["last_stance_z_m"] is not None
        ]
        out["early_swing"][str(leg)] = {
            "samples": len(rows),
            "raw_contact_fraction": mean([float(r["raw_contact"]) for r in rows]),
            "fused_load_fraction": mean([float(r["fused_load"]) for r in rows]),
            "commanded_body_z_m": mean([r["commanded_body_m"][2] for r in rows]),
            "measured_body_z_m": mean([r["measured_body_m"][2] for r in rows]),
            "commanded_minus_measured_z_m": mean([
                r["commanded_body_m"][2] - r["measured_body_m"][2] for r in rows
            ]),
            "measured_clearance_from_last_stance_m": mean(ground),
            "max_joint_error_rad": max(
                abs(a - b)
                for r in rows
                for a, b in zip(r["joint_target_rad"], r["joint_measured_rad"])
            ),
        }
    print(json.dumps(out, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
