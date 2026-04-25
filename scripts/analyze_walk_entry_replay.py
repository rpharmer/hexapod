#!/usr/bin/env python3
import json
import math
import statistics
import sys
from pathlib import Path


def usage() -> int:
    print("Usage: analyze_walk_entry_replay.py <replay.ndjson> [window_records]")
    return 1


def load_records(path: Path):
    records = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def fmt(value: float) -> str:
    if value is None or not math.isfinite(value):
        return "nan"
    return f"{value:.6f}"


def main() -> int:
    if len(sys.argv) < 2:
        return usage()

    path = Path(sys.argv[1])
    window = int(sys.argv[2]) if len(sys.argv) >= 3 else 120
    if not path.exists():
        print(f"Replay file not found: {path}", file=sys.stderr)
        return 2

    records = load_records(path)
    if not records:
        print("Replay file is empty", file=sys.stderr)
        return 3

    walk_index = None
    for idx, record in enumerate(records):
        if record.get("status", {}).get("active_mode") == 3:
            walk_index = idx
            break
    if walk_index is None:
        print("No WALK records found", file=sys.stderr)
        return 4

    end = min(len(records), walk_index + window)
    sample = records[walk_index:end]
    heights = [r.get("transition_diagnostics", {}).get("body_height_m", math.nan) for r in sample]
    margins = [r.get("gait_state", {}).get("static_stability_margin_m", math.nan) for r in sample]
    mismatches = [r.get("transition_diagnostics", {}).get("stance_contact_mismatch_count", 0) for r in sample]
    clearances = [
        r.get("gait_state", {}).get("support_liftoff_clearance_m", [math.nan] * 6)
        for r in sample
    ]
    rms_errors = [
        r.get("transition_diagnostics", {}).get("joint_tracking_rms_error_rad", [math.nan] * 6)
        for r in sample
    ]
    max_errors = [
        r.get("transition_diagnostics", {}).get("joint_tracking_max_abs_error_rad", [math.nan] * 6)
        for r in sample
    ]

    print(f"walk_entry_index={walk_index} window_records={len(sample)}")
    print(
        "body_height_m"
        f" min={fmt(min(heights))}"
        f" mean={fmt(statistics.fmean(heights))}"
        f" max={fmt(max(heights))}"
    )
    print(
        "static_stability_margin_m"
        f" min={fmt(min(margins))}"
        f" mean={fmt(statistics.fmean(margins))}"
        f" max={fmt(max(margins))}"
    )
    print(f"stance_contact_mismatch_count max={max(mismatches)}")

    spike_rows = []
    transition_summary = {}
    prev = None
    for idx, record in enumerate(sample):
        stance = record.get("gait_state", {}).get("in_stance", [False] * 6)
        phase = record.get("gait_state", {}).get("phase", [math.nan] * 6)
        peak = record.get("transition_diagnostics", {}).get("joint_tracking_max_abs_error_rad", [math.nan] * 6)
        rms = record.get("transition_diagnostics", {}).get("joint_tracking_rms_error_rad", [math.nan] * 6)
        clr = record.get("gait_state", {}).get("support_liftoff_clearance_m", [math.nan] * 6)
        for leg in range(6):
            flipped = None
            if prev is not None and prev.get("gait_state", {}).get("in_stance", [False] * 6)[leg] != stance[leg]:
                from_stance = prev.get("gait_state", {}).get("in_stance", [False] * 6)[leg]
                flipped = f"{int(from_stance)}->{int(stance[leg])}"
                key = (leg, flipped)
                transition_summary.setdefault(key, []).append(peak[leg])
            spike_rows.append((peak[leg], idx, leg, rms[leg], stance[leg], phase[leg], clr[leg], flipped))
        prev = record

    spike_rows.sort(reverse=True, key=lambda row: row[0])
    print("top_tracking_spikes")
    for peak, idx, leg, rms, stance, phase, clearance, flipped in spike_rows[:10]:
        flipped_text = flipped if flipped is not None else "-"
        print(
            f"  leg={leg} idx={idx} peak_rad={fmt(peak)} rms_rad={fmt(rms)}"
            f" stance={int(bool(stance))} phase={fmt(phase)} clearance_m={fmt(clearance)}"
            f" handoff={flipped_text}"
        )

    print("phase_handoff_summary")
    if transition_summary:
        for (leg, handoff), values in sorted(transition_summary.items()):
            print(
                f"  leg={leg} handoff={handoff} count={len(values)}"
                f" mean_peak_rad={fmt(statistics.fmean(values))}"
                f" max_peak_rad={fmt(max(values))}"
            )
    else:
        print("  none")

    for leg in range(6):
        leg_clearance = [row[leg] for row in clearances]
        leg_rms = [row[leg] for row in rms_errors]
        leg_max = [row[leg] for row in max_errors]
        print(
            f"leg_{leg}"
            f" clearance_min_m={fmt(min(leg_clearance))}"
            f" clearance_max_m={fmt(max(leg_clearance))}"
            f" joint_rms_mean_rad={fmt(statistics.fmean(leg_rms))}"
            f" joint_max_abs_peak_rad={fmt(max(leg_max))}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
