#!/usr/bin/env python3
"""Fit simple locomotion threshold envelopes from recorded regression manifests.

The locomotion regression suite writes a manifest.json that contains the full
metrics payload for each case. This helper groups successful cases into a few
coarse regimes and reports conservative candidate bands that can be fed back
into tests or governor tuning.

This is intentionally lightweight and human-auditable. It is not an online
learner; it is just a reproducible offline summary of successful runs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterable


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "paths",
        nargs="+",
        help="Artifact root directories or manifest.json files to analyze",
    )
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON")
    return parser.parse_args()


def resolve_manifest(path: Path) -> Path:
    if path.is_file():
        return path
    candidate = path / "manifest.json"
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"manifest.json not found under {path}")


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"expected object at {path}")
    return data


def percentile(values: list[float], p: float) -> float:
    if not values:
        return math.nan
    if p <= 0.0:
        return min(values)
    if p >= 1.0:
        return max(values)
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = p * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    frac = position - lower
    return ordered[lower] * (1.0 - frac) + ordered[upper] * frac


def fmt(value: float) -> str:
    if value is None or not math.isfinite(value):
        return "nan"
    text = f"{value:.6f}"
    text = text.rstrip("0").rstrip(".")
    return text if text else "0"


def category_for_case(name: str) -> str:
    lowered = name.lower()
    if "turn" in lowered:
        return "turn"
    if "transition" in lowered:
        return "transition"
    if "tilt" in lowered or "fault" in lowered:
        return "safety"
    if "timeout" in lowered or "fallback" in lowered or "governor" in lowered or "support" in lowered:
        return "fallback"
    return "walk"


def collect_cases(manifest_paths: Iterable[Path]) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    for path in manifest_paths:
        manifest = load_manifest(resolve_manifest(path))
        for case in manifest.get("cases", []):
            if isinstance(case, dict):
                cases.append(case)
    return cases


def numeric_metric(metrics: dict[str, Any], key: str) -> float | None:
    value = metrics.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def fit_group(cases: list[dict[str, Any]]) -> dict[str, Any]:
    metrics_rows = [case.get("metrics", {}) for case in cases if case.get("passed")]
    if not metrics_rows:
        return {"case_count": 0}

    def gather(key: str) -> list[float]:
        values: list[float] = []
        for metrics in metrics_rows:
            if not isinstance(metrics, dict):
                continue
            value = numeric_metric(metrics, key)
            if value is not None and math.isfinite(value):
                values.append(value)
        return values

    return {
        "case_count": len(metrics_rows),
        "stride_count_min": min(gather("stride_count") or [math.nan]),
        "walk_sample_count_min": min(gather("walk_sample_count") or [math.nan]),
        "path_length_m_min": min(gather("path_length_m") or [math.nan]),
        "net_displacement_m_min": min(gather("net_displacement_m") or [math.nan]),
        "lateral_deviation_m_p95": percentile(gather("lateral_deviation_m"), 0.95),
        "max_abs_roll_rad_p95": percentile(gather("max_abs_roll_rad"), 0.95),
        "max_abs_pitch_rad_p95": percentile(gather("max_abs_pitch_rad"), 0.95),
        "max_body_rate_radps_p95": percentile(gather("max_body_rate_radps"), 0.95),
        "min_support_margin_m_p05": percentile(gather("min_support_margin_m"), 0.05),
        "min_model_trust_p05": percentile(gather("min_model_trust"), 0.05),
        "max_contact_mismatch_ratio_p95": percentile(gather("max_contact_mismatch_ratio"), 0.95),
        "min_command_scale_p05": percentile(gather("min_command_scale"), 0.05),
        "min_cadence_scale_p05": percentile(gather("min_cadence_scale"), 0.05),
        "max_governor_severity_p95": percentile(gather("max_governor_severity"), 0.95),
        "max_governed_speed_mps_p95": percentile(gather("max_governed_speed_mps"), 0.95),
        "max_governed_yaw_rate_radps_p95": percentile(gather("max_governed_yaw_rate_radps"), 0.95),
        "min_governed_speed_mps_p05": percentile(gather("min_governed_speed_mps"), 0.05),
        "min_governed_yaw_rate_radps_p05": percentile(gather("min_governed_yaw_rate_radps"), 0.05),
        "max_step_length_m_p95": percentile(gather("max_step_length_m"), 0.95),
        "max_swing_height_m_p95": percentile(gather("max_swing_height_m"), 0.95),
    }


def print_human(groups: dict[str, list[dict[str, Any]]]) -> None:
    for group_name, cases in sorted(groups.items()):
        fitted = fit_group(cases)
        print(f"[{group_name}] passed_cases={fitted.get('case_count', 0)}")
        if fitted.get("case_count", 0) == 0:
            continue
        print(
            "  "
            f"stride_count_min={fmt(fitted['stride_count_min'])} "
            f"walk_sample_count_min={fmt(fitted['walk_sample_count_min'])} "
            f"path_length_m_min={fmt(fitted['path_length_m_min'])} "
            f"net_displacement_m_min={fmt(fitted['net_displacement_m_min'])}"
        )
        print(
            "  "
            f"roll_p95={fmt(fitted['max_abs_roll_rad_p95'])} "
            f"pitch_p95={fmt(fitted['max_abs_pitch_rad_p95'])} "
            f"body_rate_p95={fmt(fitted['max_body_rate_radps_p95'])} "
            f"lateral_dev_p95={fmt(fitted['lateral_deviation_m_p95'])}"
        )
        print(
            "  "
            f"support_margin_p05={fmt(fitted['min_support_margin_m_p05'])} "
            f"model_trust_p05={fmt(fitted['min_model_trust_p05'])} "
            f"contact_mismatch_p95={fmt(fitted['max_contact_mismatch_ratio_p95'])}"
        )
        print(
            "  "
            f"command_scale_p05={fmt(fitted['min_command_scale_p05'])} "
            f"cadence_scale_p05={fmt(fitted['min_cadence_scale_p05'])} "
            f"governor_severity_p95={fmt(fitted['max_governor_severity_p95'])}"
        )
        print(
            "  "
            f"governed_speed_p95={fmt(fitted['max_governed_speed_mps_p95'])} "
            f"governed_yaw_rate_p95={fmt(fitted['max_governed_yaw_rate_radps_p95'])} "
            f"step_length_p95={fmt(fitted['max_step_length_m_p95'])} "
            f"swing_height_p95={fmt(fitted['max_swing_height_m_p95'])}"
        )


def main() -> int:
    args = parse_args()
    manifest_paths = [Path(path) for path in args.paths]
    cases = collect_cases(manifest_paths)
    if not cases:
        raise SystemExit("no cases found in the supplied manifests")

    groups: dict[str, list[dict[str, Any]]] = {}
    for case in cases:
        name = str(case.get("name", "unknown"))
        groups.setdefault(category_for_case(name), []).append(case)
    groups.setdefault("all", []).extend(cases)

    result = {
        "source_paths": [str(path) for path in manifest_paths],
        "groups": {name: fit_group(group_cases) for name, group_cases in sorted(groups.items())},
    }

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_human(groups)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
