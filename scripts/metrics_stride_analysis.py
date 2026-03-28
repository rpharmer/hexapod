#!/usr/bin/env python3
"""Analyze replayed telemetry for gait/limiter/freshness acceptance metrics."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    tomllib = None

LEG_ORDER = ["LF", "LM", "LR", "RF", "RM", "RR"]
JOINT_ORDER = ["coxa", "femur", "tibia"]
RIGHT_SIDE_LEGS = {"RF", "RM", "RR"}
MOUNT_ANGLES_RAD = {
    "LF": math.radians(40.0),
    "LM": math.radians(90.0),
    "LR": math.radians(140.0),
    "RF": math.radians(-40.0),
    "RM": math.radians(-90.0),
    "RR": math.radians(-140.0),
}


@dataclass
class LegFrame:
    t_s: float
    foot_body: tuple[float, float, float]
    foot_world: tuple[float, float, float]
    in_stance: bool | None
    duty_cycle: float | None


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * p
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def to_float(v: Any) -> float | None:
    if isinstance(v, (int, float)):
        fv = float(v)
        if math.isfinite(fv):
            return fv
    return None


def parse_bool(v: Any) -> bool | None:
    if isinstance(v, bool):
        return v
    return None


def fk_foot_body_mm(leg: str, angles_deg: list[float], geometry: dict[str, float]) -> tuple[float, float, float]:
    coxa_deg, femur_deg, tibia_deg = angles_deg
    mount_yaw = MOUNT_ANGLES_RAD[leg]
    yaw = mount_yaw + math.radians(coxa_deg)

    coxa = geometry["coxa"]
    femur = geometry["femur"]
    tibia = geometry["tibia"]
    body_r = geometry["body_radius"]

    anchor_x = body_r * math.cos(mount_yaw)
    anchor_y = body_r * math.sin(mount_yaw)

    shoulder_x = anchor_x + coxa * math.cos(yaw)
    shoulder_y = anchor_y + coxa * math.sin(yaw)

    side_sign = -1.0 if leg in RIGHT_SIDE_LEGS else 1.0
    femur_pitch = math.radians(femur_deg * side_sign)
    tibia_pitch = math.radians(tibia_deg * side_sign)

    femur_r = femur * math.cos(femur_pitch)
    knee_x = shoulder_x + femur_r * math.cos(yaw)
    knee_y = shoulder_y + femur_r * math.sin(yaw)
    knee_z = femur * math.sin(femur_pitch)

    tibia_total = femur_pitch + tibia_pitch
    tibia_r = tibia * math.cos(tibia_total)
    foot_x = knee_x + tibia_r * math.cos(yaw)
    foot_y = knee_y + tibia_r * math.sin(yaw)
    foot_z = knee_z + tibia * math.sin(tibia_total)
    return foot_x, foot_y, foot_z


def body_to_world(foot_body_mm: tuple[float, float, float], pose: tuple[float, float, float] | None) -> tuple[float, float, float]:
    x_m = foot_body_mm[0] / 1000.0
    y_m = foot_body_mm[1] / 1000.0
    z_m = foot_body_mm[2] / 1000.0
    if pose is None:
        return x_m, y_m, z_m
    px, py, yaw = pose
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    wx = px + cy * x_m - sy * y_m
    wy = py + sy * x_m + cy * y_m
    return wx, wy, z_m


def parse_pose(datagram: dict[str, Any]) -> tuple[float, float, float] | None:
    pose = datagram.get("autonomy_debug", {}).get("localization", {}).get("current_pose")
    if not isinstance(pose, dict):
        pose = datagram.get("autonomy_debug", {}).get("current_pose")
    if not isinstance(pose, dict):
        return None
    x = to_float(pose.get("x_m"))
    y = to_float(pose.get("y_m"))
    yaw = to_float(pose.get("yaw_rad"))
    if x is None or y is None or yaw is None:
        return None
    return (x, y, yaw)


def extract_thresholds(scenario_path: Path) -> dict[str, float | int]:
    if tomllib is None:
        raise RuntimeError("Python tomllib is unavailable; cannot parse scenario thresholds")
    data = tomllib.loads(scenario_path.read_text(encoding="utf-8"))
    current = data
    for key in ["acceptance", "telemetry_metrics"]:
        if not isinstance(current, dict) or key not in current:
            return {}
        current = current[key]
    if not isinstance(current, dict):
        return {}
    out: dict[str, float | int] = {}
    for key, value in current.items():
        if isinstance(value, (int, float)):
            out[key] = float(value)
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="NDJSON capture file")
    parser.add_argument("--scenario", help="Scenario TOML path for thresholds")
    parser.add_argument("--json-output", help="Optional path to write JSON result")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON")
    args = parser.parse_args()

    capture_path = Path(args.input)
    if not capture_path.exists() or capture_path.stat().st_size == 0:
        raise SystemExit(f"ERROR: missing/empty capture: {capture_path}")

    thresholds = extract_thresholds(Path(args.scenario)) if args.scenario else {}

    leg_frames: dict[str, list[LegFrame]] = {leg: [] for leg in LEG_ORDER}
    gait_cadence: list[float] = []
    duty_series: dict[str, list[float]] = {leg: [] for leg in LEG_ORDER}
    limiter_hard_counts = {"linear": 0, "yaw": 0, "reach": 0, "cadence": 0, "saturated": 0}
    limiter_adapt_lt1_counts = {"linear": 0, "yaw": 0, "cadence": 0, "step": 0}
    joint_samples_deg: dict[str, dict[str, list[float]]] = {
        leg: {joint: [] for joint in JOINT_ORDER} for leg in LEG_ORDER
    }

    total_frames = 0
    gait_frames = 0
    freshness_present_frames = 0
    freshness_total_increments = {
        "total_rejects": 0,
        "stale_age_rejects": 0,
        "invalid_sample_id_rejects": 0,
        "non_monotonic_id_rejects": 0,
    }
    freshness_max_burst = 0
    freshness_bursts: list[int] = []
    prev_freshness: dict[str, int] | None = None

    with capture_path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                row = json.loads(text)
            except json.JSONDecodeError as exc:
                raise SystemExit(f"ERROR: invalid JSON at line {line_no}: {exc}")
            datagram = row.get("datagram")
            if not isinstance(datagram, dict):
                continue

            total_frames += 1
            t_ms = to_float(datagram.get("timestamp_ms"))
            t_s = (t_ms / 1000.0) if t_ms is not None else float(total_frames) * 0.02

            geometry_raw = datagram.get("geometry")
            angles = datagram.get("angles_deg")
            dynamic = datagram.get("dynamic_gait")
            pose = parse_pose(datagram)

            if isinstance(dynamic, dict):
                gait_frames += 1
                cadence = to_float(dynamic.get("cadence_hz"))
                if cadence is not None:
                    gait_cadence.append(cadence)

                duty_arr = dynamic.get("leg_duty_cycle")
                if isinstance(duty_arr, list) and len(duty_arr) >= len(LEG_ORDER):
                    for idx, leg in enumerate(LEG_ORDER):
                        duty = to_float(duty_arr[idx])
                        if duty is not None:
                            duty_series[leg].append(clamp01(duty))

                limiter = dynamic.get("limiter")
                if isinstance(limiter, dict):
                    hard = limiter.get("hard_clamp")
                    if isinstance(hard, dict):
                        for key in limiter_hard_counts:
                            if parse_bool(hard.get(key)):
                                limiter_hard_counts[key] += 1
                    scales = limiter.get("adaptation_scales")
                    if isinstance(scales, dict):
                        for key in limiter_adapt_lt1_counts:
                            value = to_float(scales.get(key))
                            if value is not None and value < 0.999:
                                limiter_adapt_lt1_counts[key] += 1

            freshness = datagram.get("runtime_freshness")
            if isinstance(freshness, dict):
                freshness_present_frames += 1
                curr = {
                    "consecutive_rejects": int(max(0, to_float(freshness.get("consecutive_rejects")) or 0.0)),
                    "total_rejects": int(max(0, to_float(freshness.get("total_rejects")) or 0.0)),
                    "stale_age_rejects": int(max(0, to_float(freshness.get("stale_age_rejects")) or 0.0)),
                    "invalid_sample_id_rejects": int(max(0, to_float(freshness.get("invalid_sample_id_rejects")) or 0.0)),
                    "non_monotonic_id_rejects": int(max(0, to_float(freshness.get("non_monotonic_id_rejects")) or 0.0)),
                }
                if curr["consecutive_rejects"] > 0:
                    freshness_max_burst = max(freshness_max_burst, curr["consecutive_rejects"])
                if prev_freshness is not None:
                    for key in freshness_total_increments:
                        delta = curr[key] - prev_freshness[key]
                        if delta > 0:
                            freshness_total_increments[key] += delta
                    prev_consecutive = prev_freshness["consecutive_rejects"]
                    if prev_consecutive > 0 and curr["consecutive_rejects"] == 0:
                        freshness_bursts.append(prev_consecutive)
                prev_freshness = curr

            parsed_angles_by_leg: dict[str, list[float]] = {}
            if isinstance(angles, dict):
                for leg in LEG_ORDER:
                    ang = angles.get(leg)
                    if not (isinstance(ang, list) and len(ang) >= 3):
                        continue
                    values = [to_float(ang[0]), to_float(ang[1]), to_float(ang[2])]
                    if any(v is None for v in values):
                        continue
                    parsed_values = [float(values[0]), float(values[1]), float(values[2])]
                    parsed_angles_by_leg[leg] = parsed_values
                    for joint_idx, joint_name in enumerate(JOINT_ORDER):
                        joint_samples_deg[leg][joint_name].append(parsed_values[joint_idx])

            if not isinstance(geometry_raw, dict):
                continue
            geometry: dict[str, float] = {}
            for key in ("coxa", "femur", "tibia", "body_radius"):
                value = to_float(geometry_raw.get(key))
                if value is None:
                    geometry = {}
                    break
                geometry[key] = value
            if not geometry:
                continue

            stance_arr = dynamic.get("leg_in_stance") if isinstance(dynamic, dict) else None
            duty_arr = dynamic.get("leg_duty_cycle") if isinstance(dynamic, dict) else None
            for idx, leg in enumerate(LEG_ORDER):
                values = parsed_angles_by_leg.get(leg)
                if values is None:
                    continue
                foot_body = fk_foot_body_mm(leg, [values[0], values[1], values[2]], geometry)
                foot_world = body_to_world(foot_body, pose)

                in_stance = None
                if isinstance(stance_arr, list) and len(stance_arr) >= len(LEG_ORDER):
                    in_stance = parse_bool(stance_arr[idx])
                duty = None
                if isinstance(duty_arr, list) and len(duty_arr) >= len(LEG_ORDER):
                    duty = to_float(duty_arr[idx])
                leg_frames[leg].append(LegFrame(t_s=t_s, foot_body=foot_body, foot_world=foot_world, in_stance=in_stance, duty_cycle=duty))

    if prev_freshness and prev_freshness["consecutive_rejects"] > 0:
        freshness_bursts.append(prev_freshness["consecutive_rejects"])

    per_leg: dict[str, dict[str, Any]] = {}
    for leg in LEG_ORDER:
        frames = leg_frames[leg]
        if len(frames) < 3:
            per_leg[leg] = {"sample_count": len(frames)}
            continue

        speeds: list[float] = []
        for a, b in zip(frames, frames[1:]):
            dt = b.t_s - a.t_s
            if dt <= 1e-6:
                continue
            dx = b.foot_world[0] - a.foot_world[0]
            dy = b.foot_world[1] - a.foot_world[1]
            dz = b.foot_world[2] - a.foot_world[2]
            speeds.append(math.sqrt(dx * dx + dy * dy + dz * dz) / dt)

        cycle_starts = [0]
        for i in range(1, len(frames)):
            prev = frames[i - 1].in_stance
            curr = frames[i].in_stance
            if prev is False and curr is True:
                cycle_starts.append(i)

        stride_lengths: list[float] = []
        stride_heights: list[float] = []
        observed_duty: list[float] = []
        cycle_durations: list[float] = []

        for s_idx, e_idx in zip(cycle_starts, cycle_starts[1:]):
            cycle = frames[s_idx:e_idx + 1]
            if len(cycle) < 4:
                continue
            xs = [f.foot_body[0] / 1000.0 for f in cycle]
            stride_lengths.append(max(xs) - min(xs))

            stance_z = [f.foot_body[2] / 1000.0 for f in cycle if f.in_stance is True]
            swing_z = [f.foot_body[2] / 1000.0 for f in cycle if f.in_stance is False]
            if stance_z and swing_z:
                baseline = statistics.median(stance_z)
                stride_heights.append(max(swing_z) - baseline)

            known_stance = [f for f in cycle if f.in_stance is not None]
            if known_stance:
                stance_count = sum(1 for f in known_stance if f.in_stance)
                observed_duty.append(stance_count / len(known_stance))

            cycle_dt = cycle[-1].t_s - cycle[0].t_s
            if cycle_dt > 1e-6:
                cycle_durations.append(cycle_dt)

        reported_duty = [clamp01(v) for v in duty_series[leg]]
        duty_std = statistics.pstdev(reported_duty) if len(reported_duty) > 1 else 0.0
        observed_duty_std = statistics.pstdev(observed_duty) if len(observed_duty) > 1 else 0.0
        cadence_observed = [1.0 / d for d in cycle_durations if d > 1e-6]

        per_leg[leg] = {
            "sample_count": len(frames),
            "cycle_count": len(cycle_durations),
            "stride_length_m": {
                "median": percentile(stride_lengths, 0.5),
                "p10": percentile(stride_lengths, 0.1),
                "p90": percentile(stride_lengths, 0.9),
            },
            "stride_height_m": {
                "median": percentile(stride_heights, 0.5),
                "p10": percentile(stride_heights, 0.1),
                "p90": percentile(stride_heights, 0.9),
            },
            "foot_speed_mps": {
                "mean": statistics.fmean(speeds) if speeds else None,
                "p50": percentile(speeds, 0.5),
                "p90": percentile(speeds, 0.9),
                "p95": percentile(speeds, 0.95),
                "p99": percentile(speeds, 0.99),
            },
            "cadence_hz": {
                "observed_median": percentile(cadence_observed, 0.5),
                "observed_p90": percentile(cadence_observed, 0.9),
            },
            "duty_cycle": {
                "reported_median": percentile(reported_duty, 0.5),
                "reported_stddev": duty_std,
                "observed_median": percentile(observed_duty, 0.5),
                "observed_stddev": observed_duty_std,
            },
        }

    per_joint: dict[str, dict[str, Any]] = {}
    all_joint_ranges: list[float] = []
    for leg in LEG_ORDER:
        per_joint[leg] = {}
        for joint in JOINT_ORDER:
            series = joint_samples_deg[leg][joint]
            if not series:
                per_joint[leg][joint] = {"sample_count": 0, "range_deg": None}
                continue
            min_deg = min(series)
            max_deg = max(series)
            range_deg = max_deg - min_deg
            per_joint[leg][joint] = {
                "sample_count": len(series),
                "min_deg": min_deg,
                "avg_deg": statistics.fmean(series),
                "max_deg": max_deg,
                "range_deg": range_deg,
            }
            all_joint_ranges.append(range_deg)

    cadence_median = percentile(gait_cadence, 0.5)
    cadence_p95 = percentile(gait_cadence, 0.95)
    cadence_dev_p95 = percentile([abs(v - cadence_median) for v in gait_cadence], 0.95) if cadence_median is not None else None

    leg_duty_stddevs = [statistics.pstdev(vals) for vals in duty_series.values() if len(vals) > 1]
    duty_frame_spread: list[float] = []
    min_len = min((len(vals) for vals in duty_series.values() if vals), default=0)
    for i in range(min_len):
        row = [duty_series[leg][i] for leg in LEG_ORDER]
        duty_frame_spread.append(max(row) - min(row))

    limiter = {
        "hard_clamp_counts": limiter_hard_counts,
        "adaptation_scale_lt_0p999_counts": limiter_adapt_lt1_counts,
        "hard_clamp_ratio": {k: (v / gait_frames if gait_frames else None) for k, v in limiter_hard_counts.items()},
        "adaptation_scale_lt_0p999_ratio": {
            k: (v / gait_frames if gait_frames else None) for k, v in limiter_adapt_lt1_counts.items()
        },
    }

    freshness = {
        "telemetry_present_frames": freshness_present_frames,
        "total_incremental_rejects": freshness_total_increments,
        "max_consecutive_rejects": freshness_max_burst,
        "burst_lengths": freshness_bursts,
        "prolonged_bursts": [b for b in freshness_bursts if b >= 5],
    }

    global_metrics = {
        "frame_counts": {
            "total": total_frames,
            "dynamic_gait": gait_frames,
        },
        "dynamic_gait": {
            "cadence_hz": {
                "median": cadence_median,
                "p95": cadence_p95,
                "deviation_p95": cadence_dev_p95,
            },
            "duty_cycle_consistency": {
                "per_leg_reported_stddev_median": percentile(leg_duty_stddevs, 0.5),
                "cross_leg_spread_p95": percentile(duty_frame_spread, 0.95),
            },
        },
        "limiter": limiter,
        "freshness": freshness,
        "joint_range_deg": {
            "joint_count": len(all_joint_ranges),
            "min": min(all_joint_ranges) if all_joint_ranges else None,
            "average": statistics.fmean(all_joint_ranges) if all_joint_ranges else None,
            "max": max(all_joint_ranges) if all_joint_ranges else None,
        },
    }

    checks: list[dict[str, Any]] = []

    min_stride = thresholds.get("min_median_stride_length_m")
    if min_stride is not None:
        for leg in LEG_ORDER:
            observed = per_leg.get(leg, {}).get("stride_length_m", {}).get("median")
            passed = observed is not None and observed >= min_stride
            checks.append({
                "name": f"min_median_stride_length_m[{leg}]",
                "passed": passed,
                "observed": observed,
                "threshold": float(min_stride),
                "op": ">=",
            })

    min_height = thresholds.get("min_median_stride_height_m")
    if min_height is not None:
        for leg in LEG_ORDER:
            observed = per_leg.get(leg, {}).get("stride_height_m", {}).get("median")
            passed = observed is not None and observed >= min_height
            checks.append({
                "name": f"min_median_stride_height_m[{leg}]",
                "passed": passed,
                "observed": observed,
                "threshold": float(min_height),
                "op": ">=",
            })

    max_clamp = thresholds.get("max_hard_clamp_saturated_ratio")
    if max_clamp is not None:
        observed = limiter["hard_clamp_ratio"]["saturated"]
        passed = observed is not None and observed <= max_clamp
        checks.append({
            "name": "max_hard_clamp_saturated_ratio",
            "passed": passed,
            "observed": observed,
            "threshold": float(max_clamp),
            "op": "<=",
        })

    max_prolonged = thresholds.get("max_prolonged_freshness_reject_bursts")
    if max_prolonged is not None:
        observed = len(freshness["prolonged_bursts"])
        passed = observed <= int(max_prolonged)
        checks.append({
            "name": "max_prolonged_freshness_reject_bursts",
            "passed": passed,
            "observed": observed,
            "threshold": int(max_prolonged),
            "op": "<=",
        })

    min_joint_range = thresholds.get("min_joint_range_deg")
    if min_joint_range is not None:
        observed = global_metrics["joint_range_deg"]["min"]
        passed = observed is not None and observed >= float(min_joint_range)
        checks.append({
            "name": "min_joint_range_deg",
            "passed": passed,
            "observed": observed,
            "threshold": float(min_joint_range),
            "op": ">=",
        })

    result = {
        "input": str(capture_path),
        "scenario": str(args.scenario) if args.scenario else None,
        "thresholds": thresholds,
        "global": global_metrics,
        "per_leg": per_leg,
        "per_joint": per_joint,
        "checks": checks,
        "pass": all(item["passed"] for item in checks) if checks else True,
    }

    if args.json_output:
        out_path = Path(args.json_output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True), encoding="utf-8")

    print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
