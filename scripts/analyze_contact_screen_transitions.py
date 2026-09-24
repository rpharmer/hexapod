#!/usr/bin/env python3
"""Measure emitted-foot target jumps when the opt-in clearance screen toggles."""

import json
import math
import statistics
import sys


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def main():
    previous = {}
    regular_jumps = []
    yaw_boundary = []
    corrections = []
    records = 0
    yaw_min = math.inf
    yaw_max = -math.inf
    yaw_nonzero_records = 0
    for line in sys.stdin:
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("kind") != "foothold_reach_trace":
            continue
        if row.get("schema_version") != 1:
            sys.exit("unsupported foothold trace")
        records += 1
        leg = row["leg"]
        yaw = row.get("command_yaw_radps", 0.0)
        yaw_min = min(yaw_min, yaw)
        yaw_max = max(yaw_max, yaw)
        yaw_nonzero_records += abs(yaw) >= 1.0e-9
        correction = row.get("contact_screen_delta_z_m", 0.0)
        if correction > 0.0:
            corrections.append(correction)
        before = previous.get(leg)
        if before is not None:
            time_gap_us = row["time_us"] - before["time_us"]
            if 0 < time_gap_us <= 20000:
                old_pos = before["emitted_body_m"]
                new_pos = row["emitted_body_m"]
                jump = math.dist(old_pos, new_pos)
                old_pure = abs(before.get("command_yaw_radps", 0.0)) < 1.0e-9
                new_pure = abs(row.get("command_yaw_radps", 0.0)) < 1.0e-9
                if old_pure != new_pure:
                    yaw_boundary.append({
                        "leg": leg,
                        "time_us": row["time_us"],
                        "planned_stance": row["planned_stance"],
                        "yaw_before_radps": before.get("command_yaw_radps"),
                        "yaw_after_radps": row.get("command_yaw_radps"),
                        "swing_tau_raw": row["swing_tau_raw"],
                        "target_jump_m": jump,
                        "target_z_jump_m": new_pos[2] - old_pos[2],
                        "screen_delta_before_m": before.get("contact_screen_delta_z_m", 0.0),
                        "screen_delta_after_m": correction,
                    })
                else:
                    regular_jumps.append(jump)
        previous[leg] = row
    if not records:
        sys.exit("no foothold_reach_trace records")
    print(json.dumps({
        "schema_version": 1,
        "kind": "contact_screen_transition_summary",
        "trace_records": records,
        "yaw_min_radps": yaw_min,
        "yaw_max_radps": yaw_max,
        "yaw_nonzero_records": yaw_nonzero_records,
        "screen_correction_samples": len(corrections),
        "screen_correction_p95_m": percentile(corrections, 0.95),
        "regular_target_jump_p95_m": percentile(regular_jumps, 0.95),
        "regular_target_jump_max_m": max(regular_jumps) if regular_jumps else None,
        "yaw_boundary": yaw_boundary,
    }, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
