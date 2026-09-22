#!/usr/bin/env python3
"""Summarize the contact-aware liftoff A/B.

Scores the mechanism (swing drag, liftoff delay, stored loaded-swing error,
realised support) alongside the case verdicts, so a candidate is not judged on
held counts alone.
"""
import json
import statistics as st
import sys
from pathlib import Path

CASES = ("forward_walk", "slow_forward_walk", "reverse_walk", "straight_walk",
         "turn_in_place", "aggressive_governor")


def load(path):
    text = path.read_text(errors="replace")
    dec = json.JSONDecoder()
    out, i = [], 0
    while True:
        j = text.find('{"suite"', i)
        if j < 0:
            return out
        try:
            # raw_decode returns an absolute end index, not a length.
            obj, i = dec.raw_decode(text, j)
            if isinstance(obj, dict) and obj.get("suite") == "physics_sim_walk_distance":
                out.append(obj)
        except ValueError:
            i = j + 1


def frac(part, whole):
    return [100.0 * p / w if w else 0.0 for p, w in zip(part, whole)]


def main(root):
    root = Path(root)
    for screen in ("reverse_walk", "turn_in_place", "sequential", "aggressive_governor"):
        for arm in ("baseline", "candidate"):
            files = sorted(root.glob(f"{screen}-{arm}-*.stdout"))
            if not files:
                continue
            if screen == "aggressive_governor":
                # Locomotion-regression case; it does not emit walk-distance JSON.
                for f in files:
                    text = f.read_text(errors="replace")
                    stride = [line for line in text.splitlines() if "stride_count=" in line]
                    print(f"{screen:20s} {arm:9s} "
                          f"{stride[0].strip() if stride else 'no stride_count line'}")
                continue
            expected = 5 if screen == "sequential" else 1
            passed, aborts, drag, delay, stored, support, nets, holds = 0, [], [], [], [], [], [], []
            for f in files:
                cases = load(f)
                # A run that stops emitting cases aborted; it is not a pass.
                run_ok = len(cases) == expected and all(c["passed"] for c in cases)
                passed += 1 if run_ok else 0
                if len(cases) < expected:
                    aborts.append(f"aborted after {len(cases)}/{expected} cases")
                if not cases:
                    continue
                for c in cases:
                    m = c["metrics"]
                    if not c["passed"]:
                        aborts.append(f"{c['name']}(held={m['solver_held_steps']},"
                                      f"net={m['net_horizontal_distance_m']:.3f})")
                    if "planned_swing_samples" not in m:
                        continue
                    drag += frac(m["planned_swing_contact_samples"], m["planned_swing_samples"])
                    delay.append(max(m["max_liftoff_delay_samples"]))
                    stored.append(m["max_loaded_swing_joint_error_rad"])
                    if m["support_census_samples"]:
                        support.append(m["raw_contact_count_sum"] / m["support_census_samples"])
                    holds.append(m["solver_held_steps"])
                    if c["name"] == "turn_in_place":
                        nets.append(m["net_horizontal_distance_m"])
            line = f"{screen:20s} {arm:9s} pass {passed}/{len(files)}"
            if drag:
                line += (f"  drag {st.mean(drag):5.1f}%"
                         f"  liftoff_delay p50 {st.median(delay):5.1f} max {max(delay)}"
                         f"  stored_err {st.mean(stored):.3f}/{max(stored):.3f}"
                         f"  raw_support {st.mean(support):.2f}"
                         f"  held {sum(holds)}")
            if nets:
                line += "  turn_net " + "/".join(f"{n:.3f}" for n in nets)
            print(line)
            if aborts:
                print(f"{'':30s} fails: {', '.join(aborts)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/hexapod-liftoff/ab")
