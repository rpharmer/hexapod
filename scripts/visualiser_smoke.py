#!/usr/bin/env python3
"""Repository smoke test for visualiser UDP->WebSocket telemetry flow."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

from aiohttp import ClientSession, WSMsgType


def wait_for_port(host: str, port: int, timeout_s: float = 8.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.2)
            if sock.connect_ex((host, port)) == 0:
                return
        time.sleep(0.1)
    raise TimeoutError(f"Timed out waiting for {host}:{port}")


async def receive_state(host: str, http_port: int, timeout_s: float = 5.0) -> dict:
    ws_url = f"ws://{host}:{http_port}/ws"
    async with ClientSession() as session:
        async with session.ws_connect(ws_url, heartbeat=10) as ws:
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                msg = await ws.receive(timeout=timeout_s)
                if msg.type == WSMsgType.TEXT:
                    payload = json.loads(msg.data)
                    if isinstance(payload, dict) and payload.get("type") == "state":
                        return payload
                elif msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.ERROR):
                    break
    raise RuntimeError("Did not receive state payload from websocket")


def validate_payload(payload: dict) -> None:
    required = {"type", "geometry", "angles_deg", "timestamp_ms"}
    missing = required - set(payload.keys())
    if missing:
        raise AssertionError(f"missing top-level keys: {sorted(missing)}")

    geometry = payload["geometry"]
    for key in ("coxa", "femur", "tibia", "body_radius"):
        if key not in geometry:
            raise AssertionError(f"missing geometry key {key}")

    angles = payload["angles_deg"]
    for leg in ("LF", "LM", "LR", "RF", "RM", "RR"):
        if leg not in angles:
            raise AssertionError(f"missing leg key {leg}")
        values = angles[leg]
        if not (isinstance(values, list) and len(values) == 3):
            raise AssertionError(f"leg {leg} angle vector malformed: {values}")


def send_canonical_udp_packet(host: str, udp_port: int, timestamp_ms: int) -> None:
    payload = {
        "type": "joints",
        "schema_version": 1,
        "timestamp_ms": timestamp_ms,
        "geometry": {
            "coxa": 35.0,
            "femur": 70.0,
            "tibia": 110.0,
            "body_radius": 60.0,
        },
        "angles_deg": {
            "LF": [0.0, 20.0, -40.0],
            "LM": [1.0, 21.0, -41.0],
            "LR": [2.0, 22.0, -42.0],
            "RF": [3.0, 23.0, -43.0],
            "RM": [4.0, 24.0, -44.0],
            "RR": [5.0, 25.0, -45.0],
        },
    }
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(json.dumps(payload).encode("utf-8"), (host, udp_port))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=18080)
    parser.add_argument("--udp-port", type=int, default=19870)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    server_py = repo_root / "hexapod-visualiser" / "server.py"

    env = dict(os.environ)
    env.setdefault("PYTHONUNBUFFERED", "1")

    proc = subprocess.Popen(
        [sys.executable, str(server_py), "--http-port", str(args.http_port), "--udp-port", str(args.udp_port)],
        cwd=str(repo_root / "hexapod-visualiser"),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        wait_for_port(args.host, args.http_port)
        ts = int(time.time() * 1000)
        send_canonical_udp_packet(args.host, args.udp_port, ts)
        state = asyncio.run(receive_state(args.host, args.http_port, timeout_s=6.0))
        validate_payload(state)

        if state.get("timestamp_ms") != ts:
            raise AssertionError(
                f"timestamp mismatch: expected {ts}, got {state.get('timestamp_ms')}"
            )
        print("PASS: visualiser smoke test verified websocket state fields")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
