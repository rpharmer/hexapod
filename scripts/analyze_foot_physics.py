#!/usr/bin/env python3
"""Summarize opt-in collision-sphere clearance and contact load during a walk."""

import json
import math
import statistics
import sys
from collections import defaultdict


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    below = int(position)
    above = min(below + 1, len(ordered) - 1)
    return ordered[below] + (ordered[above] - ordered[below]) * (position - below)


def mean(values):
    return statistics.fmean(values) if values else None


def summarize(rows):
    clearance = [row["clearance"] for row in rows]
    impulse = [row["impulse"] for row in rows]
    commanded = [row["commanded"] for row in rows if row["commanded"] is not None]
    return {
        "samples": len(rows),
        "manifold_contact_fraction": mean([float(row["contact"]) for row in rows]),
        "loaded_fraction": mean([float(row["impulse"] > 1.0e-5) for row in rows]),
        "normal_impulse_mean_ns": mean(impulse),
        "normal_impulse_p95_ns": percentile(impulse, 0.95),
        "sphere_plane_clearance_mean_m": mean(clearance),
        "sphere_plane_clearance_p20_m": percentile(clearance, 0.20),
        "sphere_plane_clearance_min_m": min(clearance) if clearance else None,
        "sphere_clearance_above_1mm_fraction": mean(
            [float(value > 0.001) for value in clearance]),
        "commanded_center_lift_from_support_mean_m": mean(commanded),
    }


def main():
    active = False
    pending = {}
    by_leg = defaultdict(lambda: defaultdict(list))
    counts = defaultdict(int)
    case = None
    for line in sys.stdin:
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = record.get("kind")
        if kind == "walk_capacity_window":
            if record.get("phase") == "begin":
                active = True
                case = record.get("case")
                pending.clear()
            elif record.get("phase") == "end":
                active = False
                pending.clear()
            continue
        if not active:
            continue
        if kind == "foothold_reach_trace":
            if record.get("schema_version") != 1 or record.get("leg") not in range(6):
                sys.exit("invalid foothold reach trace")
            pending[record["leg"]] = record
            continue
        if kind != "foot_physics_trace":
            continue
        if record.get("schema_version") != 1 or len(record.get("legs", [])) != 6:
            sys.exit("invalid foot physics trace")
        counts["physics_frames"] += 1
        for foot in record["legs"]:
            leg = foot["leg"]
            if leg not in range(6):
                sys.exit("invalid leg index")
            controller = pending.get(leg)
            if controller is None:
                counts["unpaired_legs"] += 1
                continue
            clearance = foot["plane_clearance_m"]
            impulse = foot["normal_impulse_ns"]
            if clearance is None or not all(math.isfinite(v) for v in (clearance, impulse)):
                counts["invalid_legs"] += 1
                continue
            planned_stance = controller["planned_stance"]
            tau = controller["swing_tau_raw"]
            if planned_stance:
                phase = "stance"
            elif 0.0 <= tau < 0.25:
                phase = "swing_early"
            elif 0.25 <= tau < 0.75:
                phase = "swing_middle"
            elif 0.75 <= tau < 1.0:
                phase = "swing_late"
            else:
                phase = "swing_unclassified"
            support_z = controller.get("support_world_z_m")
            emitted_z = controller.get("emitted_world_z_m")
            commanded = (
                emitted_z - support_z
                if support_z is not None and emitted_z is not None
                else None
            )
            sample = {
                "clearance": clearance,
                "impulse": impulse,
                "contact": foot["manifold_contact"],
                "commanded": commanded,
            }
            by_leg[leg][phase].append(sample)
            by_leg[leg]["all"].append(sample)
            counts["paired_legs"] += 1
        pending.clear()
    if not counts["physics_frames"]:
        sys.exit("no foot_physics_trace inside a walk window")
    if counts["paired_legs"] < 0.95 * counts["physics_frames"] * 6:
        sys.exit("too few controller/physics pairs for a phase census")
    print(json.dumps({
        "schema_version": 1,
        "kind": "foot_physics_summary",
        "case": case,
        "counts": dict(counts),
        "legs": {
            str(leg): {phase: summarize(rows) for phase, rows in sorted(phases.items())}
            for leg, phases in sorted(by_leg.items())
        },
    }, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
