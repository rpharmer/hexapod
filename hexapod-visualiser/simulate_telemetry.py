#!/usr/bin/env python3
"""Emit fake hexapod joint telemetry over UDP for local testing."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

LEG_ORDER = ["LF", "LM", "LR", "RF", "RM", "RR"]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9870)
    parser.add_argument("--hz", type=float, default=30.0)
    args = parser.parse_args()

    period = 1.0 / args.hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    geometry_msg = {
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
            "type": "joints",
            "timestamp_ms": int(time.time() * 1000),
            "angles_deg": angles,
        }
        sock.sendto(json.dumps(payload).encode("utf-8"), (args.host, args.port))
        t += period * 2.0
        time.sleep(period)


if __name__ == "__main__":
    main()
