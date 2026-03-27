#!/usr/bin/env python3
"""Emit fake hexapod joint telemetry over UDP for local testing."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from typing import Callable, cast
LEG_ORDER = ["LF", "LM", "LR", "RF", "RM", "RR"]
SCHEMA_VERSION = 1


def _pose_patrol(t: float) -> dict[str, float]:
    return {
        "x_m": 0.55 + 0.45 * math.cos(t * 0.3),
        "y_m": 0.05 + 0.35 * math.sin(t * 0.4),
        "yaw_rad": math.sin(t * 0.25),
    }


def _pose_zigzag(t: float) -> dict[str, float]:
    y = 0.22 * math.sin(t * 0.8)
    return {
        "x_m": 0.10 + 0.90 * ((math.sin(t * 0.22) + 1.0) * 0.5),
        "y_m": y,
        "yaw_rad": math.atan2(0.22 * 0.8 * math.cos(t * 0.8), 0.90 * 0.22 * math.cos(t * 0.22) + 1e-6),
    }


def _pose_dock_and_return(t: float) -> dict[str, float]:
    phase = (t * 0.07) % 1.0
    if phase < 0.5:
        alpha = phase / 0.5
        x = 0.05 + alpha * 1.10
    else:
        alpha = (phase - 0.5) / 0.5
        x = 1.15 - alpha * 1.10
    y = 0.16 * math.sin(t * 0.5)
    yaw = 0.0 if phase < 0.5 else math.pi
    return {"x_m": x, "y_m": y, "yaw_rad": yaw}


AUTONOMY_SCENARIOS: dict[str, dict[str, object]] = {
    "patrol": {
        "waypoints": [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.35, "y_m": 0.20, "yaw_rad": 0.4},
            {"x_m": 0.80, "y_m": 0.10, "yaw_rad": 0.0},
            {"x_m": 1.05, "y_m": -0.30, "yaw_rad": -0.5},
        ],
        "active_rate_hz": 0.25,
        "pose_fn": _pose_patrol,
    },
    "zigzag_dense": {
        "waypoints": [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.18, "y_m": 0.22, "yaw_rad": 0.6},
            {"x_m": 0.36, "y_m": -0.18, "yaw_rad": -0.5},
            {"x_m": 0.54, "y_m": 0.25, "yaw_rad": 0.55},
            {"x_m": 0.72, "y_m": -0.23, "yaw_rad": -0.45},
            {"x_m": 0.90, "y_m": 0.20, "yaw_rad": 0.35},
            {"x_m": 1.08, "y_m": -0.14, "yaw_rad": -0.25},
        ],
        "active_rate_hz": 0.45,
        "pose_fn": _pose_zigzag,
    },
    "dock_and_return": {
        "waypoints": [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.55, "y_m": 0.15, "yaw_rad": 0.1},
            {"x_m": 1.10, "y_m": 0.05, "yaw_rad": 0.0},
            {"x_m": 1.15, "y_m": -0.02, "yaw_rad": math.pi},
            {"x_m": 0.50, "y_m": -0.18, "yaw_rad": math.pi},
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        ],
        "active_rate_hz": 0.20,
        "pose_fn": _pose_dock_and_return,
    },
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9870)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument(
        "--autonomy-scenario",
        choices=sorted(AUTONOMY_SCENARIOS.keys()),
        default="patrol",
        help="Autonomy waypoint/pose scenario to emit for visualiser validation.",
    )
    args = parser.parse_args()

    period = 1.0 / args.hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    scenario = AUTONOMY_SCENARIOS[args.autonomy_scenario]
    waypoints = scenario["waypoints"]
    active_rate_hz = float(scenario["active_rate_hz"])
    pose_fn = scenario["pose_fn"]

    geometry_msg = {
        "schema_version": SCHEMA_VERSION,
        "geometry": {
            "coxa": 35.0,
            "femur": 70.0,
            "tibia": 110.0,
            "body_radius": 60.0,
        }
    }
    sock.sendto(json.dumps(geometry_msg).encode("utf-8"), (args.host, args.port))

    t = 0.0
    while True:
        angles = {}
        for idx, leg in enumerate(LEG_ORDER):
            phase = t + idx * 0.5
            angles[leg] = [
                18.0 * math.sin(phase),
                20.0 + 16.0 * math.sin(phase * 1.2),
                -35.0 - 20.0 * math.sin(phase * 1.2),
            ]

        payload = {
            "schema_version": SCHEMA_VERSION,
            "type": "joints",
            "timestamp_ms": int(time.time() * 1000),
            "angles_deg": angles,
            "autonomy_debug": {
                "waypoints": waypoints,
                "active_waypoint_index": int((t * active_rate_hz) % len(waypoints)),
                "current_pose": pose_from_fn(pose_fn, t),
            },
        }
        sock.sendto(json.dumps(payload).encode("utf-8"), (args.host, args.port))
        t += period * 2.0
        time.sleep(period)


def pose_from_fn(pose_fn: object, t: float) -> dict[str, float]:
    if callable(pose_fn):
        return cast(Callable[[float], dict[str, float]], pose_fn)(t)
    return _pose_patrol(t)


if __name__ == "__main__":
    main()
