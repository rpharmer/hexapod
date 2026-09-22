#!/usr/bin/env python3
"""Summarize a recorded tilt-safety case without conflating post-fault drift with walking."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def audit(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    rows = [r for r in rows if r.get("type") == "replay"]
    first_walk = next((i for i, r in enumerate(rows) if r["status"]["active_mode"] == 3), None)
    first_fault = next((i for i, r in enumerate(rows) if r["status"]["active_fault"] != 0), len(rows))
    positions = [r["estimated_state"]["body_twist_state"]["body_trans_m"] for r in rows]
    lengths = [0.] + [math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(positions, positions[1:])]
    metrics = json.loads(path.with_name("metrics.json").read_text())
    period = metrics["sample_period_s"]
    # LocomotionMetrics scores sum(speed*dt), not position differences.
    scored_lengths = [period * math.hypot(*r["estimated_state"]["body_twist_state"]["body_trans_mps"][:2]) for r in rows]
    window = rows[first_walk:first_fault] if first_walk is not None else []
    def frame(i):
        r = rows[i]
        e = r["estimated_state"]
        return {"step": i, "status": r["status"], "pose": e["body_twist_state"],
                "gyro_radps": e["imu"]["gyro_radps"], "contacts": e["foot_contacts"],
                "planned_stance": r["gait_state"]["in_stance"]}
    return {
        "schema_version": 1, "source": str(path),
        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "sample_count": len(rows), "first_walk_step": first_walk,
        "first_fault_step": first_fault if first_fault < len(rows) else None,
        "sample_period_s": period,
        "whole_run_planar_path_m": sum(scored_lengths),
        "strictly_prefault_planar_path_m": sum(scored_lengths[:first_fault]),
        "at_or_after_fault_planar_path_m": sum(scored_lengths[first_fault:]),
        "whole_run_pose_difference_path_m": sum(lengths),
        "strictly_prefault_pose_difference_path_m": sum(lengths[:first_fault]),
        "prefault_walk_peak_roll_rad": max((abs(r["estimated_state"]["body_twist_state"]["twist_pos_rad"][0]) for r in window), default=0),
        "prefault_walk_peak_planar_body_rate_radps": max((math.hypot(*r["estimated_state"]["imu"]["gyro_radps"][:2]) for r in window), default=0),
        "fault_window": [frame(i) for i in range(max(0, first_fault-3), min(len(rows), first_fault+2))],
        "note": "Legacy path_length_m integrates speed over the whole case, including after the fault. The corrected tilt gate uses strictly_prefault_planar_path_m. Pose-difference path is reported separately."
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.replay), indent=2, allow_nan=False))
