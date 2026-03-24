#!/usr/bin/env python3
"""Replay captured telemetry NDJSON to a UDP endpoint."""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Input NDJSON capture file")
    parser.add_argument("--host", default="127.0.0.1", help="Destination UDP host")
    parser.add_argument("--port", type=int, default=9870, help="Destination UDP port")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (>0)")
    parser.add_argument("--loop", action="store_true", help="Loop playback continuously")
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Skip frames before this capture offset")
    return parser.parse_args()


def load_frames(path: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                continue
            datagram = row.get("datagram")
            ts_ms = row.get("captured_at_unix_ms")
            if isinstance(datagram, dict) and isinstance(ts_ms, int):
                frames.append({"captured_at_unix_ms": ts_ms, "datagram": datagram, "line_no": line_no})

    if not frames:
        raise ValueError(f"No valid frames found in {path}")
    frames.sort(key=lambda item: item["captured_at_unix_ms"])
    return frames


def filter_start_offset(frames: list[dict[str, Any]], start_offset_s: float) -> list[dict[str, Any]]:
    if start_offset_s <= 0.0:
        return frames
    first_ts = frames[0]["captured_at_unix_ms"]
    threshold = first_ts + int(start_offset_s * 1000.0)
    trimmed = [frame for frame in frames if frame["captured_at_unix_ms"] >= threshold]
    return trimmed or [frames[-1]]


def replay_once(sock: socket.socket, host: str, port: int, frames: list[dict[str, Any]], speed: float) -> int:
    sent = 0
    stream_start = time.monotonic()
    first_ts = frames[0]["captured_at_unix_ms"]

    for frame in frames:
        delta_ms = frame["captured_at_unix_ms"] - first_ts
        delay_s = (delta_ms / 1000.0) / speed
        while True:
            wait_s = stream_start + delay_s - time.monotonic()
            if wait_s <= 0:
                break
            time.sleep(min(wait_s, 0.01))

        payload = json.dumps(frame["datagram"], separators=(",", ":")).encode("utf-8")
        sock.sendto(payload, (host, port))
        sent += 1
    return sent


def main() -> int:
    args = parse_args()
    if args.speed <= 0:
        raise SystemExit("--speed must be > 0")

    frames = load_frames(Path(args.input))
    frames = filter_start_offset(frames, args.start_offset_s)
    print(
        f"Loaded {len(frames)} frame(s) from {args.input}; "
        f"replaying to {args.host}:{args.port} at {args.speed:.3f}x"
    )

    total_sent = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while True:
            sent = replay_once(sock, args.host, args.port, frames, args.speed)
            total_sent += sent
            if not args.loop:
                break

    print(f"Sent {total_sent} frame(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
