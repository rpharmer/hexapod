#!/usr/bin/env python3
import argparse
import json
import math
import statistics
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect locomotion replay NDJSON records.")
    parser.add_argument("replay_path", type=Path, help="Replay NDJSON file")
    parser.add_argument(
        "window_records",
        nargs="?",
        type=int,
        default=120,
        help="Generic walk-entry window size in records (default: 120)",
    )
    parser.add_argument(
        "--scenario05",
        action="store_true",
        help="Emit Scenario 05 long-walk window diagnostics",
    )
    parser.add_argument(
        "--scenario-file",
        type=Path,
        default=None,
        help="Optional scenario TOML path for --scenario05",
    )
    return parser.parse_args()


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


def default_scenario05_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "hexapod-server"
        / "scenarios"
        / "05_long_walk_observability.toml"
    )


def load_scenario_definition(path: Path) -> dict:
    scenario = {"events": []}
    current_event = None
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            if line == "[[events]]":
                current_event = {}
                scenario["events"].append(current_event)
                continue
            if "=" not in line:
                continue
            key, value = [part.strip() for part in line.split("=", 1)]
            target = current_event if current_event is not None else scenario
            if value.startswith('"') and value.endswith('"'):
                parsed = value[1:-1]
            elif value in ("true", "false"):
                parsed = value == "true"
            else:
                try:
                    parsed = int(value)
                except ValueError:
                    parsed = float(value)
            target[key] = parsed
    return scenario


def extract_leg_metric(records, key: str, leg: int):
    values = []
    for record in records:
        debug = record.get("locomotion_debug", {})
        row = debug.get(key, [])
        if leg < len(row):
            values.append(row[leg])
    return values


def extract_window(records, sample_period_s: float, start_s: float, end_s: float):
    selected = []
    for index, record in enumerate(records):
        t_s = index * sample_period_s
        if start_s <= t_s < end_s:
            selected.append((index, t_s, record))
    return selected


def summarize_window(label: str, phase_label: str, rows):
    if not rows:
        print(f"{label} phase={phase_label} unavailable")
        return

    heights = [
        row.get("transition_diagnostics", {}).get("body_height_m", math.nan)
        for _, _, row in rows
    ]
    contact_mismatch = [
        row.get("transition_diagnostics", {}).get("stance_contact_mismatch_count", 0)
        for _, _, row in rows
    ]
    min_measured_foot_world_z = [
        row.get("locomotion_debug", {}).get("min_measured_foot_world_z_m", math.nan)
        for _, _, row in rows
    ]
    max_tracking_error = [
        row.get("locomotion_debug", {}).get("max_commanded_tracking_error_m", math.nan)
        for _, _, row in rows
    ]
    walk_samples = sum(
        1 for _, _, row in rows if row.get("status", {}).get("active_mode") == 3
    )
    fault_samples = sum(
        1 for _, _, row in rows if row.get("status", {}).get("active_fault", 0) != 0
    )
    drift_by_leg = []
    held_drift_by_leg = []
    tracking_by_leg = []
    for leg in range(6):
        drift_by_leg.append(max(extract_leg_metric([row for _, _, row in rows], "contact_anchor_drift_m", leg), default=math.nan))
        held_drift_by_leg.append(max(extract_leg_metric([row for _, _, row in rows], "contact_anchor_max_drift_m", leg), default=math.nan))
        tracking_by_leg.append(max(extract_leg_metric([row for _, _, row in rows], "commanded_tracking_error_m", leg), default=math.nan))

    first_t = rows[0][1]
    last_t = rows[-1][1]
    print(
        f"{label} phase={phase_label}"
        f" start_s={fmt(first_t)} end_s={fmt(last_t)}"
        f" walk_samples={walk_samples}"
        f" fault_samples={fault_samples}"
        f" body_height_min_m={fmt(min(heights))}"
        f" body_height_mean_m={fmt(statistics.fmean(heights))}"
        f" measured_foot_z_min_m={fmt(min(min_measured_foot_world_z))}"
        f" contact_mismatch_max={max(contact_mismatch)}"
        f" tracking_error_max_m={fmt(max(max_tracking_error))}"
    )
    print(
        "  per_leg"
        f" drift_peak_m={[fmt(v) for v in drift_by_leg]}"
        f" held_drift_peak_m={[fmt(v) for v in held_drift_by_leg]}"
        f" tracking_peak_m={[fmt(v) for v in tracking_by_leg]}"
    )


def scenario05_phase_labels(scenario: dict):
    events = sorted(scenario.get("events", []), key=lambda row: row.get("at_ms", 0))
    labels = []
    duration_ms = scenario["duration_ms"]
    for idx, event in enumerate(events):
        start_s = event["at_ms"] / 1000.0
        end_s = (
            events[idx + 1]["at_ms"] / 1000.0
            if idx + 1 < len(events)
            else duration_ms / 1000.0
        )
        labels.append(
            (
                start_s,
                end_s,
                f"{event['mode'].lower()}_{event['gait'].lower()}_speed_{fmt(event.get('speed_mps', 0.0))}"
                f"_heading_{fmt(event.get('heading_rad', 0.0))}_yaw_{fmt(event.get('yaw_rad', 0.0))}",
            )
        )
    return labels


def phase_label_for_time(phase_labels, center_s: float) -> str:
    for start_s, end_s, label in phase_labels:
        if start_s <= center_s < end_s:
            return label
    return "unknown"


def analyze_scenario05(records, scenario_path: Path):
    if not scenario_path.exists():
        print(f"Scenario file not found: {scenario_path}", file=sys.stderr)
        return 5

    scenario = load_scenario_definition(scenario_path)
    duration_s = scenario["duration_ms"] / 1000.0
    sample_period_s = duration_s / max(1, len(records))
    phases = scenario05_phase_labels(scenario)

    print(
        f"scenario05 sample_count={len(records)} duration_s={fmt(duration_s)}"
        f" sample_period_s={fmt(sample_period_s)}"
    )

    windows = [
        ("slow_tripod_tail", 11.0, 12.0),
        ("fast_tripod_early", 12.0, 13.2),
        ("ripple_yaw_pose_window", 20.25, 21.75),
        ("wave_yaw_pose_window", 29.25, 30.75),
        ("tripod_yaw_pose_window", 38.25, 39.75),
    ]
    for label, start_s, end_s in windows:
        center_s = 0.5 * (start_s + end_s)
        summarize_window(
            label,
            phase_label_for_time(phases, center_s),
            extract_window(records, sample_period_s, start_s, end_s),
        )

    per_leg = []
    for leg in range(6):
        measured_z = extract_leg_metric(records, "measured_foot_world_m", leg)
        min_leg_measured_z = min((row[2] for row in measured_z if isinstance(row, list) and len(row) >= 3), default=math.nan)
        drift_peak = max(extract_leg_metric(records, "contact_anchor_drift_m", leg), default=math.nan)
        held_drift_peak = max(extract_leg_metric(records, "contact_anchor_max_drift_m", leg), default=math.nan)
        tracking_peak = max(extract_leg_metric(records, "commanded_tracking_error_m", leg), default=math.nan)
        per_leg.append((leg, drift_peak, held_drift_peak, tracking_peak, min_leg_measured_z))

    worst_drift_leg = max(per_leg, key=lambda row: row[2] if math.isfinite(row[2]) else -1.0)[0]
    worst_tracking_leg = max(per_leg, key=lambda row: row[3] if math.isfinite(row[3]) else -1.0)[0]
    worst_penetration_leg = min(per_leg, key=lambda row: row[4] if math.isfinite(row[4]) else math.inf)[0]

    print(
        "scenario05_leg_summary"
        f" worst_drift_leg={worst_drift_leg}"
        f" worst_tracking_leg={worst_tracking_leg}"
        f" worst_penetration_leg={worst_penetration_leg}"
    )
    for leg, drift_peak, held_drift_peak, tracking_peak, min_leg_measured_z in per_leg:
        print(
            f"  leg_{leg}"
            f" contact_drift_peak_m={fmt(drift_peak)}"
            f" held_drift_peak_m={fmt(held_drift_peak)}"
            f" commanded_tracking_peak_m={fmt(tracking_peak)}"
            f" measured_foot_world_z_min_m={fmt(min_leg_measured_z)}"
        )
    return 0


def main() -> int:
    args = parse_args()
    path = args.replay_path
    window = args.window_records
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
    locomotion_debug = [r.get("locomotion_debug", {}) for r in sample]
    min_measured_foot_world_z = [
        dbg.get("min_measured_foot_world_z_m", math.nan) for dbg in locomotion_debug
    ]
    max_commanded_tracking_error = [
        dbg.get("max_commanded_tracking_error_m", math.nan) for dbg in locomotion_debug
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
    print(
        "measured_foot_world_z_m"
        f" min={fmt(min(min_measured_foot_world_z))}"
        f" max_commanded_tracking_error_m={fmt(max(max_commanded_tracking_error))}"
    )

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
        leg_contact_drift = [
            dbg.get("contact_anchor_drift_m", [math.nan] * 6)[leg] for dbg in locomotion_debug
        ]
        leg_contact_max_drift = [
            dbg.get("contact_anchor_max_drift_m", [math.nan] * 6)[leg] for dbg in locomotion_debug
        ]
        leg_tracking = [
            dbg.get("commanded_tracking_error_m", [math.nan] * 6)[leg] for dbg in locomotion_debug
        ]
        print(
            f"leg_{leg}"
            f" clearance_min_m={fmt(min(leg_clearance))}"
            f" clearance_max_m={fmt(max(leg_clearance))}"
            f" joint_rms_mean_rad={fmt(statistics.fmean(leg_rms))}"
            f" joint_max_abs_peak_rad={fmt(max(leg_max))}"
            f" contact_drift_peak_m={fmt(max(leg_contact_drift))}"
            f" contact_max_drift_peak_m={fmt(max(leg_contact_max_drift))}"
            f" commanded_tracking_peak_m={fmt(max(leg_tracking))}"
        )

    if args.scenario05:
        scenario_path = args.scenario_file or default_scenario05_path()
        status = analyze_scenario05(records, scenario_path)
        if status != 0:
            return status

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
