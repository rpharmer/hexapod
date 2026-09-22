#!/usr/bin/env python3
"""Report event-resolved walk census; never infer liftoff from case maxima.

Usage: python3 tools/report_swing_clearance.py RUN_DIR --output NEW_REPORT.json
Budgets use completed, non-left-censored swings only; first-contact-loss timing
also includes right-censored swings whose first contact loss was observed.
Raw contact loss/recontact is not geometric clearance or sustained flight.
"""
import argparse
import hashlib
import json
import statistics
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from summarize_gait_feasibility import load


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(events):
    known = [e for e in events if not e["left_censored"]]
    complete = [e for e in known if e["complete"]]
    lifted = [e["liftoff_ms"] for e in known if e["liftoff_ms"] >= 0]
    budgets = [e["at_command_peak"] for e in complete]
    return {
        "observed_events": len(events), "complete_known_entry": len(complete),
        "left_censored": len(events) - len(known),
        "first_contact_loss_observed": len(lifted),
        "first_contact_loss_ms_median_observed_only": statistics.median(lifted) if lifted else None,
        "completed_without_contact_loss": sum(e["liftoff_ms"] < 0 for e in complete),
        "censored_without_contact_loss": sum(not e["complete"] and e["liftoff_ms"] < 0 for e in known),
        "complete_with_recontact": sum(e["recontacts"] > 0 for e in complete),
        "complete_measured_peak_m_median": statistics.median(e["measured_peak_m"] for e in complete) if complete else None,
        "mean_budget_at_command_peak_m": {
            key: statistics.mean(b[key] for b in budgets) for key in budgets[0]
        } if budgets else {},
        "max_budget_closure_m": max((e["peak_closure_m"] for e in events), default=0),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    records, inputs = [], []
    for path in sorted(args.run_dir.glob("*.stdout")):
        cases = load(path)
        inputs.append({"path": str(path), "sha256": digest(path)})
        for case in cases:
            m = case["metrics"]
            if m.get("swing_event_schema") != 1:
                raise ValueError(f"Missing/unsupported event schema: {path}")
            events = m["swing_events"]
            records.append({
                "process": path.stem, "case": case["name"], "passed": case["passed"],
                "held": m["solver_held_steps"], "net_m": m["net_horizontal_distance_m"],
                "events": summarize(events),
                "legs": {str(leg): summarize([e for e in events if e["leg"] == leg]) for leg in range(6)},
            })
    probes = [json.loads(line) for line in (args.run_dir / "liftoff-probe-v2.stdout").read_text().splitlines()]
    binaries = ["hexapod-server/build-tests/test_physics_sim_walk_distance",
                "hexapod-server/build-tests/test_locomotion_regression_suite",
                "hexapod-physics-sim/build/hexapod-physics-sim",
                "hexapod-physics-sim/build/test_pinocchio_liftoff_probe"]
    git = lambda *cmd: subprocess.check_output(["git", *cmd], cwd=repo)
    report = {
        "schema_version": 1, "reported_at_utc": datetime.now(timezone.utc).isoformat(),
        "tree_revision_at_reporting": git("rev-parse", "HEAD").decode().strip(),
        "tracked_diff_sha256_at_reporting": hashlib.sha256(git("diff", "HEAD")).hexdigest(),
        "provenance_note": "Dirty campaign; reporting-time tree is not an immutable source snapshot. Binary hashes and raw log hashes identify tested artifacts. No original capture-time tree manifest exists.",
        "binary_sha256": {b: digest(repo / b) for b in binaries},
        "inputs": inputs,
        "settings": {"solver_mode": 1, "max_iterations": 24, "body_height_m": .14,
                     "command_dt_s": .005, "implicit_damping": False,
                     "arms": {"baseline": {"height_hold_scale": 1}, "hh0": {"height_hold_scale": 0}}},
        "aggregation": "Each case and leg reported separately. Geometry budgets use completed non-left-censored swings. Terms evaluated at one instant, the command peak; rotation/joint interaction assigned to joint term. FK point, not sphere bottom. Recontact includes normal late-swing touchdown.",
        "cases": records, "slow_liftoff_probe": probes,
        "decision": "Follow-up only; no production promotion. hh0 sequential still fails turn net; small samples do not establish abort elimination. Probe is a constructed pose, not the production gait.",
    }
    with args.output.open("x") as out:
        json.dump(report, out, indent=2, allow_nan=False)
        out.write("\n")


if __name__ == "__main__":
    main()
