#!/usr/bin/env python3
"""P5 pose patches. Never edits frozen files.

modes:
  joints              sequential free-flyer + isolated joints; keep sequential
                      servo targets (P5-1; SpeedLimit held).
  isolated-at-seq-ff  sequential free-flyer + isolated joints/targets/mask;
                      clear warm starts (also SpeedLimit held).
  xy-only             isolated plant translated to sequential world XZ.
  yaw-only            isolated plant at sequential XZ with heading-only yaw.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

ISO_FIELDS = (
    "last_servo_targets",
    "effective_inertias",
    "target_inertias",
    "nominal_inertias",
    "last_leg_normal_impulse",
    "load_bearing_mask",
    "load_bearing_count",
    "reduced_support_dwell_s",
    "reduced_support_blend",
)


def heading_yaw_y_up(q: list[float]) -> float:
    qx, qy, qz, qw = q[3], q[4], q[5], q[6]
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r20 = 2.0 * (qx * qz - qy * qw)
    return math.atan2(r20, r00)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("sequential")
    parser.add_argument("isolated")
    parser.add_argument("output")
    parser.add_argument(
        "--mode",
        choices=("joints", "isolated-at-seq-ff", "xy-only", "yaw-only"),
        default="joints",
    )
    args = parser.parse_args()
    sequential = json.loads(Path(args.sequential).read_text())
    isolated = json.loads(Path(args.isolated).read_text())
    q_s = sequential["q"]
    v_s = sequential["v"]
    q_i = isolated["q"]
    v_i = isolated["v"]
    if len(q_s) != len(q_i) or len(v_s) != len(v_i):
        raise SystemExit(f"q/v size mismatch seq={len(q_s)},{len(v_s)} iso={len(q_i)},{len(v_i)}")

    if args.mode in ("xy-only", "yaw-only"):
        out = dict(isolated)
        q = list(q_i)
        q[0] = q_s[0]
        q[2] = q_s[2]
        if args.mode == "yaw-only":
            half = 0.5 * heading_yaw_y_up(q_s)
            q[3] = 0.0
            q[4] = math.sin(half)
            q[5] = 0.0
            q[6] = math.cos(half)
            norm = math.sqrt(q[3] * q[3] + q[4] * q[4] + q[5] * q[5] + q[6] * q[6])
            q[3] /= norm
            q[4] /= norm
            q[5] /= norm
            q[6] /= norm
        out["q"] = q
        out["v"] = list(v_i)
        out["warm_starts"] = []
        out["warm_start_count"] = 0
        out["p5_pose_transplant"] = args.mode
        Path(args.output).write_text(json.dumps(out) + "\n")
        return

    out = dict(sequential)
    out["q"] = list(q_s[:7]) + list(q_i[7:])
    out["v"] = list(v_s[:6]) + list(v_i[6:])
    out["warm_starts"] = []
    out["warm_start_count"] = 0
    if args.mode == "isolated-at-seq-ff":
        out["v"] = list(v_i)
        for key in ISO_FIELDS:
            out[key] = isolated[key]
        out["p5_pose_transplant"] = "isolated-at-seq-ff"
    else:
        out["p5_pose_transplant"] = True
    Path(args.output).write_text(json.dumps(out) + "\n")


if __name__ == "__main__":
    main()
