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


def _build_path_pose_fn(
    waypoints: list[dict[str, float]],
    speed_mps: float,
) -> Callable[[float], dict[str, float]]:
    points = [(point["x_m"], point["y_m"]) for point in waypoints if "x_m" in point and "y_m" in point]
    segments: list[tuple[float, float, float, float, float]] = []
    total_length = 0.0

    for index in range(len(points) - 1):
        x0, y0 = points[index]
        x1, y1 = points[index + 1]
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        segments.append((x0, y0, x1, y1, length))
        total_length += length

    if total_length <= 1e-6:
        anchor = waypoints[0] if waypoints else {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}

        def _stationary(_: float) -> dict[str, float]:
            return {
                "x_m": float(anchor.get("x_m", 0.0)),
                "y_m": float(anchor.get("y_m", 0.0)),
                "yaw_rad": float(anchor.get("yaw_rad", 0.0)),
            }

        return _stationary

    def _pose(t: float) -> dict[str, float]:
        distance = (t * max(0.01, speed_mps)) % total_length
        for x0, y0, x1, y1, length in segments:
            if distance <= length:
                alpha = distance / length
                yaw = math.atan2(y1 - y0, x1 - x0)
                return {
                    "x_m": x0 + (x1 - x0) * alpha,
                    "y_m": y0 + (y1 - y0) * alpha,
                    "yaw_rad": yaw,
                }
            distance -= length

        x0, y0, x1, y1, _ = segments[-1]
        return {"x_m": x1, "y_m": y1, "yaw_rad": math.atan2(y1 - y0, x1 - x0)}

    return _pose


def _scenario(waypoints: list[dict[str, float]], active_rate_hz: float, speed_mps: float) -> dict[str, object]:
    return {
        "waypoints": waypoints,
        "active_rate_hz": active_rate_hz,
        "pose_fn": _build_path_pose_fn(waypoints, speed_mps=speed_mps),
    }


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
    # Mirrors scenario names under hexapod-server/scenarios/ so overlay paths can be previewed
    # directly from simulate_telemetry.py.
    "nominal_stand_walk": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.35, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.70, "y_m": 0.25, "yaw_rad": 0.7},
            {"x_m": 0.95, "y_m": 0.20, "yaw_rad": 0.0},
            {"x_m": 1.15, "y_m": -0.18, "yaw_rad": -0.8},
            {"x_m": 0.85, "y_m": -0.28, "yaw_rad": math.pi},
            {"x_m": 0.45, "y_m": -0.05, "yaw_rad": 2.6},
        ],
        active_rate_hz=0.32,
        speed_mps=0.12,
    ),
    "command_timeout_fallback": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.22, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.40, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.45, "y_m": 0.0, "yaw_rad": 0.0},
        ],
        active_rate_hz=0.75,
        speed_mps=0.08,
    ),
    "power_fault_triggers": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.55, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.75, "y_m": 0.20, "yaw_rad": 1.0},
            {"x_m": 0.45, "y_m": 0.35, "yaw_rad": 2.4},
            {"x_m": 0.05, "y_m": 0.25, "yaw_rad": -2.7},
        ],
        active_rate_hz=0.45,
        speed_mps=0.09,
    ),
    "contact_loss_edge_cases": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.20, "y_m": 0.18, "yaw_rad": 0.8},
            {"x_m": 0.42, "y_m": -0.15, "yaw_rad": -0.8},
            {"x_m": 0.63, "y_m": 0.20, "yaw_rad": 0.8},
            {"x_m": 0.84, "y_m": -0.18, "yaw_rad": -0.7},
        ],
        active_rate_hz=0.62,
        speed_mps=0.11,
    ),
    "long_walk_observability": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.45, "y_m": 0.05, "yaw_rad": 0.1},
            {"x_m": 0.95, "y_m": 0.28, "yaw_rad": 0.8},
            {"x_m": 1.35, "y_m": 0.06, "yaw_rad": -0.4},
            {"x_m": 1.62, "y_m": -0.35, "yaw_rad": -1.1},
            {"x_m": 1.15, "y_m": -0.60, "yaw_rad": math.pi},
            {"x_m": 0.55, "y_m": -0.42, "yaw_rad": 2.7},
            {"x_m": 0.12, "y_m": -0.08, "yaw_rad": 2.2},
        ],
        active_rate_hz=0.20,
        speed_mps=0.14,
    ),
    "dynamic_turn_priority_safety": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.42, "y_m": 0.02, "yaw_rad": 0.0},
            {"x_m": 0.50, "y_m": 0.30, "yaw_rad": 1.4},
            {"x_m": 0.56, "y_m": -0.34, "yaw_rad": -1.4},
            {"x_m": 0.78, "y_m": -0.10, "yaw_rad": 0.5},
        ],
        active_rate_hz=0.58,
        speed_mps=0.10,
    ),
    "blocked_navigation_pause_resume": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.18, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.18, "y_m": 0.15, "yaw_rad": 1.57},
            {"x_m": 0.36, "y_m": 0.15, "yaw_rad": 0.0},
        ],
        active_rate_hz=0.95,
        speed_mps=0.09,
    ),
    "retry_replan_escalation": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.20, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.20, "y_m": 0.18, "yaw_rad": 1.57},
            {"x_m": 0.46, "y_m": 0.18, "yaw_rad": 0.0},
            {"x_m": 0.46, "y_m": -0.10, "yaw_rad": -1.2},
        ],
        active_rate_hz=1.05,
        speed_mps=0.09,
    ),
    "abort_on_budget_exhaustion": _scenario(
        [
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            {"x_m": 0.10, "y_m": 0.02, "yaw_rad": 0.2},
            {"x_m": 0.16, "y_m": 0.03, "yaw_rad": 0.1},
        ],
        active_rate_hz=1.15,
        speed_mps=0.05,
    ),
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
