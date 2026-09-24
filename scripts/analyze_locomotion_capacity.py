#!/usr/bin/env python3
"""Summarize opt-in Pinocchio servo-capacity JSON lines from stdin.

Run with HEXAPOD_PROXIMAL_TRACE_SERVO_CAPACITY=1 and
HEXAPOD_WALK_TEST_CHILD_STDIO=1. Other simulator/test output is ignored.
"""

import json
import math
import sys
from collections import Counter, defaultdict


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def summarize(rows):
    return {
        "samples": len(rows),
        "saturation_fraction": sum(row["saturated"] for row in rows) / len(rows),
        "zero_available_samples": sum(row["available_torque_nm"] <= 1e-9 for row in rows),
        "requested_over_available_p50": percentile(
            [abs(row["requested_torque_nm"]) / row["available_torque_nm"]
             for row in rows if row["available_torque_nm"] > 1e-9], 0.50),
        "requested_over_available_p95": percentile(
            [abs(row["requested_torque_nm"]) / row["available_torque_nm"]
             for row in rows if row["available_torque_nm"] > 1e-9], 0.95),
        "requested_over_available_p99": percentile(
            [abs(row["requested_torque_nm"]) / row["available_torque_nm"]
             for row in rows if row["available_torque_nm"] > 1e-9], 0.99),
        "applied_over_stall_p95": percentile(
            [abs(row["applied_torque_nm"]) / row["stall_torque_nm"] for row in rows], 0.95),
        "absolute_rate_radps_p95": percentile(
            [abs(row["rate_radps"]) for row in rows], 0.95),
        "actuator_work_j": sum(row["applied_torque_nm"] * row["rate_radps"] * row["dt_s"]
                               for row in rows),
    }


def main():
    groups = defaultdict(list)
    skipped = defaultdict(Counter)
    healthy_substeps = Counter()
    active_case = None
    max_envelope_error_nm = 0.0
    max_clamp_error_nm = 0.0
    max_impulse_error_nms = 0.0
    for line in sys.stdin:
        if line.startswith('{"kind":"walk_capacity_window"'):
            marker = json.loads(line)
            if marker["phase"] == "begin":
                active_case = marker["case"]
            elif marker["phase"] == "end":
                active_case = None
            else:
                raise SystemExit("invalid walk-capacity window marker")
            continue
        if not line.startswith('{"kind":"proximal_servo_capacity'):
            continue
        if active_case is None:
            continue  # Ignore stand warmup and other unscored physics steps.
        try:
            item = json.loads(line)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"invalid servo-capacity trace: {exc}") from exc
        if item["kind"] == "proximal_servo_capacity_skip":
            skipped[active_case][str(item["status"])] += 1
            continue
        joints = item["joints"]
        if len(joints) != 18 or sorted(row["wire"] for row in joints) != list(range(18)):
            raise SystemExit("servo-capacity trace must contain each wire index exactly once")
        healthy_substeps[active_case] += 1
        stall = item["stall_torque_nm"]
        no_load = item["no_load_speed_radps"]
        dt = item["dt_s"]
        if not all(math.isfinite(value) and value > 0 for value in (stall, no_load, dt)):
            raise SystemExit("invalid stall torque, no-load speed, or timestep")
        expected_peak_impulse = max(abs(joint["applied_torque_nm"]) * dt for joint in joints)
        max_impulse_error_nms = max(max_impulse_error_nms,
                                    abs(item["peak_actuator_impulse_nms"] - expected_peak_impulse))
        for joint in joints:
            requested = joint["requested_torque_nm"]
            rate = joint["rate_radps"]
            available = joint["available_torque_nm"]
            applied = joint["applied_torque_nm"]
            if not all(math.isfinite(value) for value in (requested, rate, available, applied)):
                raise SystemExit("non-finite servo-capacity value")
            expected_available = (stall * max(0.0, 1.0 - abs(rate) / no_load)
                                  if requested * rate > 0 else stall)
            expected_applied = max(-available, min(requested, available))
            max_envelope_error_nm = max(max_envelope_error_nm, abs(available - expected_available))
            max_clamp_error_nm = max(max_clamp_error_nm, abs(applied - expected_applied))
            row = dict(joint, stall_torque_nm=stall, dt_s=dt)
            groups[(active_case, joint["wire"], bool(joint["load_bearing"]))].append(row)
    if not healthy_substeps:
        raise SystemExit("no healthy explicit-motor samples inside a walk-capacity window")
    if max(max_envelope_error_nm, max_clamp_error_nm) > 1e-7 or max_impulse_error_nms > 1e-8:
        raise SystemExit("motor-envelope or torque-times-dt impulse accounting differs from trace")
    result = {
        "schema_version": 1,
        "max_envelope_error_nm": max_envelope_error_nm,
        "max_clamp_error_nm": max_clamp_error_nm,
        "max_impulse_error_nms": max_impulse_error_nms,
        "cases": [
            {"case": case, "healthy_substeps": healthy_substeps[case],
             "skipped_substeps_by_status": dict(sorted(skipped[case].items())),
             "by_wire_and_load": [
                 {"wire": wire, "leg": wire // 3, "joint": wire % 3,
                  "load_bearing": loaded, **summarize(rows)}
                 for (row_case, wire, loaded), rows in sorted(groups.items())
                 if row_case == case
             ]}
            for case in sorted(healthy_substeps)
        ],
    }
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
