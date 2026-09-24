#!/usr/bin/env python3
"""Summarize opt-in foothold reach traces from a walk-distance run on stdin."""

import json
import math
import statistics
import sys
from collections import defaultdict


def mean(values):
    return statistics.fmean(values) if values else None


def summarize(rows):
    if not rows:
        return {"samples": 0}
    p3 = [r["swing_p3_xy_m"] for r in rows if r["swing_p3_xy_m"] is not None]
    clamp = [
        math.hypot(r["pre_reach_body_m"][0] - r["emitted_body_m"][0],
                   r["pre_reach_body_m"][1] - r["emitted_body_m"][1])
        for r in rows
    ]
    early_contact = [
        r for r in rows
        if not r["planned_stance"] and r["fused_contact"] and 0.0 <= r.get("swing_tau_raw", -1.0) < 0.45
    ]
    early_forced_touchdown = [
        r for r in early_contact if r.get("swing_tau_used", -1.0) >= 0.999
    ]
    world_clearance = [
        r["emitted_world_z_m"] - r["support_world_z_m"]
        for r in rows
        if r.get("emitted_world_z_m") is not None
        and r.get("support_world_z_m") is not None
    ]
    return {
        "samples": len(rows),
        "p3_xy_m": [mean([p[axis] for p in p3]) for axis in range(2)],
        "pre_reach_xyz_m": [mean([r["pre_reach_body_m"][axis] for r in rows]) for axis in range(3)],
        "emitted_xyz_m": [mean([r["emitted_body_m"][axis] for r in rows]) for axis in range(3)],
        "nominal_xyz_m": [mean([r["nominal_body_m"][axis] for r in rows]) for axis in range(3)],
        "xy_clamp_mean_m": mean(clamp),
        "xy_clamp_max_m": max(clamp),
        "workspace_xy_hit_fraction": mean([float(r["workspace_xy_hit"]) for r in rows]),
        "fused_contact_fraction": mean([float(r["fused_contact"]) for r in rows]),
        "early_contact_samples": len(early_contact),
        "early_contact_forced_touchdown_samples": len(early_forced_touchdown),
        "commanded_clearance_from_last_support_mean_m": mean(world_clearance),
        "commanded_clearance_from_last_support_min_m": min(world_clearance) if world_clearance else None,
        "effective_height_mean_m": mean([r["effective_height_m"] for r in rows]),
        "height_hold_mean_m": mean([r["height_hold_m"] for r in rows]),
        "pose_pitch_mean_rad": mean([r["pose_pitch_rad"] for r in rows]),
        "measured_pitch_mean_rad": mean([r["measured_pitch_rad"] for r in rows]),
    }


def main():
    groups = defaultdict(lambda: defaultdict(list))
    previous_stance = {}
    previous_record = {}
    transitions = defaultdict(lambda: defaultdict(list))
    counts = {"trace_lines": 0, "other_lines": 0}
    for line in sys.stdin:
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            counts["other_lines"] += 1
            continue
        if record.get("kind") != "foothold_reach_trace":
            counts["other_lines"] += 1
            continue
        if record.get("schema_version") != 1 or record.get("leg") not in range(6):
            sys.exit("unsupported foothold trace schema or leg index")
        counts["trace_lines"] += 1
        leg = record["leg"]
        stance = record["planned_stance"]
        groups[leg]["all"].append(record)
        if stance:
            groups[leg]["stance"].append(record)
            if previous_stance.get(leg) is False:
                groups[leg]["stance_entry"].append(record)
                transitions[leg]["swing_to_stance"].append((previous_record[leg], record))
        else:
            groups[leg]["swing"].append(record)
            if previous_stance.get(leg) is True:
                transitions[leg]["stance_to_swing"].append((previous_record[leg], record))
            tau = (record["phase"] - record["duty"]) / (1.0 - record["duty"])
            if tau >= 0.9:
                groups[leg]["late_swing"].append(record)
        previous_stance[leg] = stance
        previous_record[leg] = record
    if counts["trace_lines"] == 0:
        sys.exit("no foothold_reach_trace records on stdin")
    transition_summary = {}
    for leg, by_name in sorted(transitions.items()):
        transition_summary[str(leg)] = {}
        for name, pairs in sorted(by_name.items()):
            transition_summary[str(leg)][name] = {
                "count": len(pairs),
                "pre_reach_delta_xyz_m": [
                    mean([after["pre_reach_body_m"][axis] - before["pre_reach_body_m"][axis]
                          for before, after in pairs]) for axis in range(3)
                ],
                "emitted_delta_xyz_m": [
                    mean([after["emitted_body_m"][axis] - before["emitted_body_m"][axis]
                          for before, after in pairs]) for axis in range(3)
                ],
                "emitted_jump_mean_m": mean([
                    math.dist(before["emitted_body_m"], after["emitted_body_m"])
                    for before, after in pairs
                ]),
                "emitted_jump_max_m": max(
                    math.dist(before["emitted_body_m"], after["emitted_body_m"])
                    for before, after in pairs
                ),
                "contact_before_fraction": mean([float(before["fused_contact"]) for before, _ in pairs]),
                "contact_after_fraction": mean([float(after["fused_contact"]) for _, after in pairs]),
            }
    result = {
        "schema_version": 1,
        "kind": "foothold_reach_summary",
        **counts,
        "legs": {str(leg): {name: summarize(rows) for name, rows in sorted(group.items())}
                 for leg, group in sorted(groups.items())},
        "transitions": transition_summary,
    }
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
