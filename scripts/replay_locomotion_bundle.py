#!/usr/bin/env python3
"""Replay a locomotion regression bundle into the existing visualiser UDP interface.

The locomotion regression suite writes:
  - geometry.json
  - replay.ndjson
  - summary.json / metrics.json

This helper streams geometry once, then replays each NDJSON control record as a
visualiser-friendly joints packet. The existing OpenGL visualiser can then be
pointed at the UDP port to inspect the run visually.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path
from typing import Any, Iterable


VISUALISER_LEG_KEYS = ["LF", "LM", "LR", "RF", "RM", "RR"]
INTERNAL_TO_VISUALISER_ORDER = [5, 3, 1, 4, 2, 0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-dir", required=True, help="Locomotion artifact directory")
    parser.add_argument("--case", help="Optional case subdirectory inside the artifact root")
    parser.add_argument("--host", default="127.0.0.1", help="Destination UDP host")
    parser.add_argument("--port", type=int, default=9870, help="Destination UDP port")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (>0)")
    parser.add_argument("--loop", action="store_true", help="Loop playback continuously")
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Skip frames before this offset")
    parser.add_argument("--frame-delay-ms", type=float, default=0.0, help="Extra delay between frames")
    return parser.parse_args()


def resolve_case_dir(artifact_dir: Path, case: str | None) -> Path:
    case_dir = artifact_dir / case if case else artifact_dir
    if not case_dir.exists():
        raise FileNotFoundError(f"artifact directory not found: {case_dir}")
    return case_dir


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object in {path}")
    return data


def load_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                continue
            if row.get("type") != "replay":
                continue
            ts_us = row.get("timestamp_us")
            if not isinstance(ts_us, int):
                continue
            row["_line_no"] = line_no
            records.append(row)
    if not records:
        raise ValueError(f"no replay records found in {path}")
    records.sort(key=lambda item: int(item["timestamp_us"]))
    return records


def filter_start_offset(records: list[dict[str, Any]], start_offset_s: float) -> list[dict[str, Any]]:
    if start_offset_s <= 0.0:
        return records
    first_ts = int(records[0]["timestamp_us"])
    threshold = first_ts + int(start_offset_s * 1_000_000.0)
    trimmed = [record for record in records if int(record["timestamp_us"]) >= threshold]
    return trimmed or [records[-1]]


def fmt_angle_deg(value_rad: float) -> float:
    return value_rad * 180.0 / 3.141592653589793


def joint_angles_deg(record: dict[str, Any]) -> dict[str, list[float]]:
    joint_targets = record.get("joint_targets")
    if not isinstance(joint_targets, list) or len(joint_targets) < 6:
        raise ValueError("replay record missing joint_targets")

    result: dict[str, list[float]] = {}
    for vis_index, key in enumerate(VISUALISER_LEG_KEYS):
        internal_index = INTERNAL_TO_VISUALISER_ORDER[vis_index]
        leg = joint_targets[internal_index]
        if not isinstance(leg, list) or len(leg) < 3:
            raise ValueError(f"joint_targets[{internal_index}] is malformed")
        result[key] = [fmt_angle_deg(float(leg[0])), fmt_angle_deg(float(leg[1])), fmt_angle_deg(float(leg[2]))]
    return result


def body_pose(record: dict[str, Any]) -> tuple[list[float] | None, float | None]:
    est = record.get("estimated_state")
    if not isinstance(est, dict):
        return None, None
    twist = est.get("body_twist_state")
    if not isinstance(twist, dict):
        return None, None
    body_trans = twist.get("body_trans_m")
    twist_pos = twist.get("twist_pos_rad")
    if not (isinstance(body_trans, list) and len(body_trans) >= 3 and isinstance(twist_pos, list) and len(twist_pos) >= 3):
        return None, None
    return [float(body_trans[0]), float(body_trans[1]), float(body_trans[2])], float(twist_pos[2])


def status_strings(record: dict[str, Any]) -> tuple[str, str]:
    status = record.get("status")
    if not isinstance(status, dict):
        return "SAFE_IDLE", "NONE"
    mode_map = {
        0: "SAFE_IDLE",
        1: "HOMING",
        2: "STAND",
        3: "WALK",
        4: "FAULT",
    }
    fault_map = {
        0: "NONE",
        1: "BUS_TIMEOUT",
        2: "ESTOP",
        3: "TIP_OVER",
        4: "ESTIMATOR_INVALID",
        5: "MOTOR_FAULT",
        6: "JOINT_LIMIT",
        7: "COMMAND_TIMEOUT",
    }
    mode = mode_map.get(int(status.get("active_mode", 0)), "SAFE_IDLE")
    fault = fault_map.get(int(status.get("active_fault", 0)), "NONE")
    return mode, fault


def synthesize_joints_packet(record: dict[str, Any], geometry: dict[str, Any]) -> dict[str, Any]:
    status = record.get("status")
    if not isinstance(status, dict):
        status = {}
    mode, fault = status_strings(record)
    body_position, body_yaw = body_pose(record)
    packet: dict[str, Any] = {
        "type": "joints",
        "schema_version": 4,
        "timestamp_ms": int(int(record["timestamp_us"]) / 1000),
        "loop_counter": int(status.get("loop_counter", 0)),
        "mode": int(status.get("active_mode", 0)),
        "active_mode": mode,
        "active_fault": fault,
        "bus_ok": bool(status.get("bus_ok", True)),
        "estimator_valid": bool(status.get("estimator_valid", True)),
        "angles_deg": joint_angles_deg(record),
        "geometry": geometry,
    }
    if body_position is not None:
        packet["body_position"] = body_position
    if body_yaw is not None:
        packet["body_yaw_rad"] = body_yaw

    # The visualiser treats these fields as optional; include zeros if the replay
    # artifact does not carry live power telemetry.
    est = record.get("estimated_state")
    if isinstance(est, dict):
        if "voltage" in est:
            packet["voltage"] = est["voltage"]
        if "current" in est:
            packet["current"] = est["current"]
    return packet


def replay_once(
    sock: socket.socket,
    host: str,
    port: int,
    geometry_packet: dict[str, Any],
    records: list[dict[str, Any]],
    speed: float,
    frame_delay_ms: float,
) -> int:
    sent = 0
    payload = json.dumps(geometry_packet, separators=(",", ":")).encode("utf-8")
    sock.sendto(payload, (host, port))
    sent += 1

    stream_start = time.monotonic()
    first_ts_us = int(records[0]["timestamp_us"])
    for record in records:
        delta_s = (int(record["timestamp_us"]) - first_ts_us) / 1_000_000.0
        delay_s = delta_s / speed
        while True:
            wait_s = stream_start + delay_s - time.monotonic()
            if wait_s <= 0:
                break
            time.sleep(min(wait_s, 0.01))
        payload = json.dumps(synthesize_joints_packet(record, geometry_packet["geometry"]), separators=(",", ":")).encode(
            "utf-8"
        )
        sock.sendto(payload, (host, port))
        sent += 1
        if frame_delay_ms > 0.0:
            time.sleep(frame_delay_ms / 1000.0)
    return sent


def main() -> int:
    args = parse_args()
    if args.speed <= 0.0:
        raise SystemExit("--speed must be > 0")

    case_dir = resolve_case_dir(Path(args.artifact_dir), args.case)
    geometry_path = case_dir / "geometry.json"
    replay_path = case_dir / "replay.ndjson"
    if not geometry_path.exists():
        raise FileNotFoundError(f"geometry packet not found: {geometry_path}")
    if not replay_path.exists():
        raise FileNotFoundError(f"replay file not found: {replay_path}")

    geometry_packet = load_json(geometry_path)
    records = filter_start_offset(load_records(replay_path), args.start_offset_s)
    print(
        f"Loaded {len(records)} replay record(s) from {replay_path}; "
        f"streaming geometry + joints to {args.host}:{args.port} at {args.speed:.3f}x"
    )

    total_sent = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while True:
            total_sent += replay_once(
                sock,
                args.host,
                args.port,
                geometry_packet,
                records,
                args.speed,
                args.frame_delay_ms,
            )
            if not args.loop:
                break

    print(f"Sent {total_sent} datagram(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
