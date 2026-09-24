#!/usr/bin/env python3
"""Summarize the load-transfer history around a live locomotion replay failure.

This is diagnostic only: its thresholds identify trace windows, not safety or
locomotion acceptance limits. The input is the suite's saved replay.ndjson.
"""

import argparse
import json
from pathlib import Path


def snapshot(record, leg):
    debug = record["locomotion_debug"]
    gait = record["gait_state"]
    body = record["estimated_state"]["body_twist_state"]
    return {
        "step": record["sample_id"],
        "mode": record["status"]["active_mode"],
        "bus_ok": record["status"]["bus_ok"],
        "roll_rad": body["twist_pos_rad"][0],
        "body_height_m": body["body_trans_m"][2],
        "contact_count": sum(debug["raw_contact"]),
        "phase": gait["phase"][leg],
        "planned_stance": debug["planned_stance"][leg],
        "gait_hold": gait["hold_stance"][leg],
        "raw_contact": debug["raw_contact"][leg],
        "measured_foot_z_m": debug["measured_foot_world_m"][leg][2],
        "tracking_error_m": debug["commanded_tracking_error_m"][leg],
        "target_body_vy_mps": record["leg_targets"]["feet"][leg]["vel_body_mps"][1],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--leg", type=int, default=2)
    parser.add_argument("--from-step", type=int, default=2400)
    parser.add_argument("--mismatch-min-steps", type=int, default=20)
    args = parser.parse_args()
    if args.leg < 0 or args.leg >= 6:
        parser.error("--leg must be 0..5")

    first = {}
    maxima = {"abs_roll_rad": 0.0, "tracking_error_m": 0.0,
              "measured_foot_z_m": 0.0, "abs_target_body_vy_mps": 0.0}
    mismatch_start = None
    longest_mismatch = None
    last_step = None
    eligible_samples = 0

    with args.replay.open(encoding="utf-8") as replay:
        for line in replay:
            record = json.loads(line)
            if record.get("type") != "replay" or record["sample_id"] < args.from_step:
                continue
            point = snapshot(record, args.leg)
            step = point["step"]
            eligible_samples += 1
            maxima["abs_roll_rad"] = max(maxima["abs_roll_rad"], abs(point["roll_rad"]))
            maxima["tracking_error_m"] = max(maxima["tracking_error_m"], point["tracking_error_m"])
            maxima["measured_foot_z_m"] = max(maxima["measured_foot_z_m"], point["measured_foot_z_m"])
            maxima["abs_target_body_vy_mps"] = max(
                maxima["abs_target_body_vy_mps"], abs(point["target_body_vy_mps"]))
            for name, condition in (
                ("roll_over_0p3", abs(point["roll_rad"]) > 0.3),
                ("roll_over_0p5", abs(point["roll_rad"]) > 0.5),
                ("tracking_over_0p1", point["tracking_error_m"] > 0.1),
                ("stance_foot_world_z_over_0p1", point["planned_stance"]
                 and point["measured_foot_z_m"] > 0.1),
                ("failed_read", not point["bus_ok"]),
            ):
                if condition and name not in first:
                    first[name] = point

            mismatch = point["mode"] == 3 and point["planned_stance"] and not point["raw_contact"]
            if mismatch:
                if mismatch_start is None:
                    mismatch_start = point
            elif mismatch_start is not None:
                length = step - mismatch_start["step"]
                if longest_mismatch is None or length > longest_mismatch["steps"]:
                    longest_mismatch = {"steps": length, "start": mismatch_start,
                                        "end_step": step - 1}
                mismatch_start = None
            last_step = step

    if mismatch_start is not None:
        length = last_step - mismatch_start["step"] + 1
        if longest_mismatch is None or length > longest_mismatch["steps"]:
            longest_mismatch = {"steps": length, "start": mismatch_start,
                                "end_step": last_step}
    if eligible_samples == 0:
        parser.error("no replay samples in selected window")
    if longest_mismatch and longest_mismatch["steps"] < args.mismatch_min_steps:
        longest_mismatch = None
    print(json.dumps({"schema_version": 1, "replay": str(args.replay),
                      "leg": args.leg, "from_step": args.from_step,
                      "samples": eligible_samples, "first": first,
                      "longest_stance_contact_mismatch": longest_mismatch,
                      "maxima": maxima}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
