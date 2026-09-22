#!/usr/bin/env python3
"""Label sequential turn pass vs fail support/load asymmetry. No 0.21 retune."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def classify(path: Path) -> dict:
    data = json.loads(path.read_text())
    samples = data.get("samples", [])
    net = float(data.get("net_horizontal_distance_m", 0.0))
    yaw = float(data.get("yaw_delta_rad", 0.0))
    if not samples:
        return {"path": str(path), "label": "unknown", "net": net, "yaw": yaw, "samples": 0}
    left = [s.get("left_fused", 0) for s in samples]
    right = [s.get("right_fused", 0) for s in samples]
    support = [s.get("support", 0) for s in samples]
    height = [s.get("z", 0.0) for s in samples]
    wz = [s.get("wz", 0.0) for s in samples]
    mean_left = sum(left) / len(left)
    mean_right = sum(right) / len(right)
    mean_support = sum(support) / len(support)
    mean_height = sum(height) / len(height)
    mean_wz = sum(wz) / len(wz)
    asym = abs(mean_left - mean_right)
    xs = [s.get("x", 0.0) for s in samples]
    ys = [s.get("y", 0.0) for s in samples]
    # Chord radius vs orbit: large net with yaw is translation; similar radius is orbit.
    label = "unknown"
    if asym >= 0.75:
        label = "support_asymmetric"
    elif net >= 0.21 and abs(yaw) >= 1.5:
        label = "orbit"
    return {
        "path": str(path),
        "label": label,
        "net": net,
        "yaw": yaw,
        "samples": len(samples),
        "mean_left_fused": mean_left,
        "mean_right_fused": mean_right,
        "asym": asym,
        "mean_support": mean_support,
        "mean_height": mean_height,
        "mean_wz": mean_wz,
        "start": [xs[0], ys[0]],
        "end": [xs[-1], ys[-1]],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dumps", nargs="+", type=Path)
    args = parser.parse_args()
    for path in args.dumps:
        result = classify(path)
        print(
            f"TURN_SUPPORT path={path.name} label={result['label']} "
            f"net={result['net']:.4f} yaw={result['yaw']:.3f} "
            f"mean_support={result['mean_support']:.2f} "
            f"left={result['mean_left_fused']:.2f} right={result['mean_right_fused']:.2f} "
            f"asym={result['asym']:.2f} height={result['mean_height']:.4f} "
            f"wz={result['mean_wz']:.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
