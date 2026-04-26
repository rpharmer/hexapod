#!/usr/bin/env python3
"""Compare server-vs-renderer leg FK reconstruction from telemetry packets."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

INTERNAL_LEG_KEYS = ["RR", "LR", "RM", "LM", "RF", "LF"]


def rot_z(angle_rad: float, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    x, y, z = v
    return (c * x - s * y, s * x + c * y, z)


def fk_server(leg: dict, servo_deg: Tuple[float, float, float]) -> Tuple[float, float, float]:
    coxa_offset = math.radians(float(leg["coxa_attach_deg"]))
    femur_offset = math.radians(float(leg["femur_attach_deg"]))
    tibia_offset = math.radians(float(leg["tibia_attach_deg"]))
    coxa_sign = float(leg["coxa_sign"])
    femur_sign = float(leg["femur_sign"])
    tibia_sign = float(leg["tibia_sign"])

    s1 = math.radians(servo_deg[0])
    s2 = math.radians(servo_deg[1])
    s3 = math.radians(servo_deg[2])

    q1 = coxa_sign * (s1 - coxa_offset)
    q2 = femur_sign * (s2 - femur_offset)
    q3 = tibia_sign * (s3 - tibia_offset)

    coxa = float(leg["coxa_mm"]) * 1e-3
    femur = float(leg["femur_mm"]) * 1e-3
    tibia = float(leg["tibia_mm"]) * 1e-3

    rho = femur * math.cos(q2) + tibia * math.cos(q2 + q3)
    z_plane = femur * math.sin(q2) + tibia * math.sin(q2 + q3)
    x_leg = coxa + rho
    y_leg = -math.sin(q1) * z_plane
    z_leg = math.cos(q1) * z_plane

    mount = math.radians(float(leg["mount_angle_deg"]))
    x_rel, y_rel, z_rel = rot_z(mount, (x_leg, y_leg, z_leg))
    ox, oy, oz = [float(v) for v in leg["body_coxa_offset"]]
    return (ox + x_rel, oy + y_rel, oz + z_rel)


def fk_renderer(leg: dict, servo_deg: Tuple[float, float, float]) -> Tuple[float, float, float]:
    # Keep an explicit second implementation mirroring visualiser logic.
    s1 = math.radians(servo_deg[0])
    s2 = math.radians(servo_deg[1])
    s3 = math.radians(servo_deg[2])
    q1 = float(leg["coxa_sign"]) * (s1 - math.radians(float(leg["coxa_attach_deg"])))
    q2 = float(leg["femur_sign"]) * (s2 - math.radians(float(leg["femur_attach_deg"])))
    q3 = float(leg["tibia_sign"]) * (s3 - math.radians(float(leg["tibia_attach_deg"])))
    mount = math.radians(float(leg["mount_angle_deg"]))

    coxa = float(leg["coxa_mm"]) * 1e-3
    femur = float(leg["femur_mm"]) * 1e-3
    tibia = float(leg["tibia_mm"]) * 1e-3
    ox, oy, oz = [float(v) for v in leg["body_coxa_offset"]]

    c_mount = math.cos(mount)
    s_mount = math.sin(mount)

    def leg_to_body(x_leg: float, y_leg: float, z_leg: float) -> Tuple[float, float, float]:
        return (
            ox + c_mount * x_leg - s_mount * y_leg,
            oy + s_mount * x_leg + c_mount * y_leg,
            oz + z_leg,
        )

    shoulder = leg_to_body(coxa, 0.0, 0.0)
    femur_proj = femur * math.cos(q2)
    femur_vert = femur * math.sin(q2)
    knee = leg_to_body(
        coxa + femur_proj,
        -math.sin(q1) * femur_vert,
        math.cos(q1) * femur_vert,
    )
    tibia_total = q2 + q3
    tibia_proj = tibia * math.cos(tibia_total)
    z_plane = femur_vert + tibia * math.sin(tibia_total)
    foot = leg_to_body(
        coxa + femur_proj + tibia_proj,
        -math.sin(q1) * z_plane,
        math.cos(q1) * z_plane,
    )
    return foot


def dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def load_packets(path: Path) -> Tuple[dict, dict]:
    latest_geometry: dict = {}
    latest_joints: dict = {}
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            d = row.get("datagram")
            if not isinstance(d, dict):
                continue
            if d.get("type") == "geometry":
                latest_geometry = d
            elif d.get("type") == "joints" and isinstance(d.get("angles_deg"), dict):
                latest_joints = d
    return latest_geometry, latest_joints


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="NDJSON capture from capture_udp_telemetry.py")
    args = ap.parse_args()

    geometry_packet, joints_packet = load_packets(Path(args.input))
    if not joints_packet:
        print("No joints packets found.")
        return 2
    if not geometry_packet:
        print("No geometry packets found.")
        return 2

    geom = geometry_packet.get("geometry", {})
    legs = geom.get("legs", [])
    angles = joints_packet.get("angles_deg", {})
    if not isinstance(legs, list) or not isinstance(angles, dict):
        print("Packet missing geometry.legs or angles_deg.")
        return 2

    leg_by_key: Dict[str, dict] = {}
    for leg in legs:
        key = leg.get("key")
        if isinstance(key, str):
            leg_by_key[key] = leg

    angle_order = list(angles.keys())
    expected_present = [k for k in INTERNAL_LEG_KEYS if k in angle_order]
    print(f"angles_deg key order: {angle_order}")
    print(f"expected internal order prefix: {expected_present}")

    worst = 0.0
    for key in INTERNAL_LEG_KEYS:
        if key not in leg_by_key or key not in angles:
            continue
        vals = angles[key]
        if not (isinstance(vals, list) and len(vals) >= 3):
            continue
        servo = (float(vals[0]), float(vals[1]), float(vals[2]))
        server_foot = fk_server(leg_by_key[key], servo)
        renderer_foot = fk_renderer(leg_by_key[key], servo)
        err = dist(server_foot, renderer_foot)
        worst = max(worst, err)
        print(
            f"{key}: server={server_foot!r} renderer={renderer_foot!r} err_m={err:.9f}"
        )

    print(f"worst_err_m={worst:.9f}")
    if worst > 1e-9:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

