#!/usr/bin/env python3
"""Classify planned-stance vs physical-support divergence on a never-overwrite dump."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path


PHASE_CONFIRMED_STANCE = 3
PHASE_LOST_CANDIDATE = 4


def classify_leg(ticks: list[dict], leg: int) -> str:
    stance_ticks = []
    for tick in ticks:
        sample = tick["legs"][leg]
        if not sample["in_stance"]:
            continue
        stance_ticks.append(sample)
    if not stance_ticks:
        return "planned_swing"
    had_manifold = any(s["raw_contact"] or s["leg_contact_count"] > 0 for s in stance_ticks)
    last_manifold = stance_ticks[-1]["raw_contact"] or stance_ticks[-1]["leg_contact_count"] > 0
    any_fused = any(s["fused_load_bearing"] for s in stance_ticks)
    any_manifold = had_manifold
    if not any_manifold:
        return "never_acquired"
    if any_manifold and not last_manifold:
        return "acquired_then_lifted"
    if any_manifold and not any_fused:
        return "touching_undetected"
    return "supported"


def classify_dump(path: Path, winner_leg: int | None) -> dict:
    data = json.loads(path.read_text())
    ticks = data.get("samples", [])
    per_leg = {leg: classify_leg(ticks, leg) for leg in range(6)}
    counts = Counter(per_leg.values())
    named = "unknown"
    if winner_leg is not None and 0 <= winner_leg < 6:
        named = per_leg[winner_leg]
    elif counts["never_acquired"]:
        named = "never_acquired"
    elif counts["acquired_then_lifted"]:
        named = "acquired_then_lifted"
    elif counts["touching_undetected"]:
        named = "touching_undetected"
    return {
        "case": data.get("case"),
        "held": data.get("held"),
        "samples": len(ticks),
        "winner_leg": winner_leg,
        "named_class": named,
        "per_leg": per_leg,
        "counts": dict(counts),
        "note": "missing_mask_bit_is_not_planned_swing",
    }


def winner_from_history(path: Path | None) -> int | None:
    if path is None or not path.exists():
        return None
    data = json.loads(path.read_text())
    samples = data.get("accepted_history", [])
    if not samples:
        return None
    last = samples[-1]
    mask = int(last.get("load_bearing_mask", 0))
    for bit in range(6):
        if (mask & (1 << bit)) == 0:
            # Unloaded bit is a candidate; prefer last sample's unloaded tibia
            # only as a hint. Caller may override.
            pass
    winner = data.get("winner", {})
    frame = str(winner.get("frame", ""))
    for leg in range(6):
        if f"leg_{leg}_" in frame or frame.endswith(f"leg{leg}"):
            return leg
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dump", type=Path)
    parser.add_argument("--history", type=Path, default=None)
    parser.add_argument("--winner-leg", type=int, default=None)
    args = parser.parse_args()
    winner = args.winner_leg
    if winner is None:
        winner = winner_from_history(args.history)
    result = classify_dump(args.dump, winner)
    print(json.dumps(result, indent=2, sort_keys=True))
    print(
        f"SUPPORT_DIVERGENCE case={result['case']} named={result['named_class']} "
        f"winner={result['winner_leg']} held={result['held']} samples={result['samples']} "
        f"per_leg={result['per_leg']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
