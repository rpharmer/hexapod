#!/usr/bin/env python3
"""Retain the three liftoff balance probe arms and their provenance.

Run all arms with the same binary into RUN_DIR/{baseline,gravity-bias,
gravity-onset}.jsonl, then pass RUN_DIR and a new output JSON path.
"""
import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run_dir", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--moving", action="store_true", help="250 ms moving-reference audit arms: baseline, gravity, lead")
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    arms, logs = {}, {}
    for arm in (("baseline", "gravity", "lead") if args.moving else ("baseline", "gravity-bias", "gravity-onset")):
        path = args.run_dir / (arm + ".jsonl")
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        if {(r["leg"], r["pedestal"]) for r in rows} != {(2, False), (2, True), (5, False), (5, True)} or len(rows) != 4:
            raise ValueError(f"incomplete or duplicate probe results: {arm}")
        for row in rows:
            if row["gravity_bias"] != (arm != "baseline"):
                raise ValueError(f"incorrect arm: {arm}")
            if row["gravity_bias_start_s"] != (0 if arm == "gravity-onset" or (args.moving and arm != "baseline") else 1.5):
                raise ValueError(f"incorrect activation time: {arm}")
            if args.moving and (row["ramp_s"] != .25 or row["velocity_lead"] != (arm == "lead")):
                raise ValueError(f"incorrect moving experiment: {arm}")
        arms[arm] = rows
        logs[str(path)] = sha(path)
    source_paths = ["hexapod-physics-sim/tests/test_pinocchio_liftoff_probe.cpp",
                    "hexapod-physics-sim/src/demo/pinocchio_hexapod.cpp",
                    "hexapod-physics-sim/include/minphys3d/demo/pinocchio_hexapod.hpp"]
    report = {
        "schema_version": 1, "date_utc": datetime.now(timezone.utc).isoformat(),
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "tree_note": "Existing dirty campaign preserved. Source/binary hashes identify this probe, not a clean commit.",
        "source_sha256": {p: sha(root / p) for p in source_paths},
        "binary_sha256": sha(root / "hexapod-physics-sim/build/test_pinocchio_liftoff_probe"),
        "input_sha256": logs,
        "settings": {"solver_mode": 1, "iterations": 24, "command_dt_s": .005,
                     "substep_dt_s": 1/600, "gravity_mps2": 9.80665,
                     "nominal_body_height_m": .14, "implicit_damping": False},
        "method": "Constructed pose, 3 s settle, 1 s 30 mm lift, 2 s hold. Gravity bias g/Kp on one selected leg only, ramped over 250 ms. Bias g from centered finite differences of physical-body potential. Kp from actual accepted servo sample. Audit final 500 ms at command cadence, excluding retries. Contact torque inferred from M dv/dt + bias - actual torque; not an independent contact-Jacobian measurement.",
        "arms": arms,
        "decision": "Finite-PD self-weight sag explains supported equilibrium. Targeted compensation removes it in this probe. No production gain/contact law or controller default change; no live-gait acceptance claim.",
    }
    if args.moving:
        report["method"] = "Constructed pose; 3 s settle, 250 ms 30 mm lift, 2 s hold. Gravity g/Kp ramped over 250 ms from lift onset; lead adds 0.08 s times finite-difference IK reference rate. Gravity from independent potential derivative; reported nominal Kp. Moving audit first 400 ms, excluding retries. Contact torque inferred from M dv/dt + bias - actual torque; closure is algebraic, not independent contact validation. Absolute term magnitudes do not add because signed terms cancel. Test-only oracle bias is NOT bounded by server gravity angle cap. All physical torque/speed guards retained."
        report["decision"] = "Moving damping lag demonstrated on isolated leg. Gravity plus velocity lead improves apex tracking, but live velocity-only lead fails walking/turn/aggressive screens: rejected for production. No plant or controller default change."
    with args.output.open("x") as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write("\n")


if __name__ == "__main__":
    main()
