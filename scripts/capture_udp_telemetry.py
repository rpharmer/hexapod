#!/usr/bin/env python3
"""Capture UDP JSON datagrams into timestamped NDJSON frames."""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Local host/interface to bind")
    parser.add_argument("--port", type=int, default=9870, help="Local UDP port to capture")
    parser.add_argument("--output", required=True, help="Output NDJSON path")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames (0 = unlimited)")
    return parser.parse_args()


def decode_datagram(payload: bytes) -> tuple[Any | None, str]:
    raw_text = payload.decode("utf-8", errors="replace")
    try:
        return json.loads(raw_text), raw_text
    except json.JSONDecodeError:
        return None, raw_text


def main() -> int:
    args = parse_args()
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    frame_count = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, out_path.open("w", encoding="utf-8") as out:
        sock.bind((args.host, args.port))
        print(f"Capturing UDP datagrams on {args.host}:{args.port} -> {out_path}")
        while True:
            data, addr = sock.recvfrom(65535)
            captured_ns = time.time_ns()
            parsed, raw_text = decode_datagram(data)
            frame = {
                "captured_at_unix_ms": captured_ns // 1_000_000,
                "captured_at_unix_ns": captured_ns,
                "source": {"host": addr[0], "port": addr[1]},
                "raw": raw_text,
                "datagram": parsed,
            }
            out.write(json.dumps(frame, separators=(",", ":")) + "\n")
            out.flush()

            frame_count += 1
            if args.max_frames > 0 and frame_count >= args.max_frames:
                break

    print(f"Captured {frame_count} frame(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
