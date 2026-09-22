#!/usr/bin/env python3
"""Summarize the gait-feasibility screens (leftover §3.15 command mechanisms).

Scores whether the commanded gait is actually executed (drag, liftoff delay,
no-load-slew fraction, realised support, stance error) alongside case verdicts,
so an arm is not judged on held counts alone.
"""
import json
import statistics as st
import sys
from pathlib import Path

NO_LOAD_RADPS = 7.479983


def load(path):
    text = path.read_text(errors="replace")
    dec = json.JSONDecoder()
    out, i = [], 0
    while True:
        j = text.find('{"suite"', i)
        if j < 0:
            return out
        try:
            obj, i = dec.raw_decode(text, j)
            if isinstance(obj, dict) and obj.get("suite") == "physics_sim_walk_distance":
                out.append(obj)
        except ValueError:
            i = j + 1


def frac(part, whole):
    return [100.0 * p / w if w else 0.0 for p, w in zip(part, whole)]


def summarize(root, screen, arms):
    root = Path(root)
    for arm in arms:
        files = sorted(root.glob(f"{screen}-{arm}-*.stdout"))
        if not files:
            continue
        if screen == "aggressive_governor":
            for f in files:
                text = f.read_text(errors="replace")
                stride = [ln for ln in text.splitlines() if "stride_count=" in ln]
                print(f"{screen:16s} {arm:14s} "
                      f"{stride[0].strip() if stride else 'no stride_count line'}")
            continue
        expected = 5 if screen == "sequential" else 1
        passed, aborts, clean_complete, incomplete = 0, [], 0, 0
        drag, delay, holds, nets = [], [], [], []
        events = []
        femur, heave, rate, stroke = [], [], [], []
        for f in files:
            cases = load(f)
            exit_path = f.with_suffix(".exit")
            exit_ok = not exit_path.exists() or int(exit_path.read_text().strip()) == 0
            if len(cases) == expected and all(c["passed"] for c in cases) and exit_ok:
                passed += 1
            if not exit_ok:
                aborts.append(f"process_exit={exit_path.read_text().strip()}")
            if len(cases) < expected:
                incomplete += 1
                aborts.append(f"aborted {len(cases)}/{expected}")
            if len(cases) == expected and all(c["metrics"]["solver_held_steps"] == 0 for c in cases):
                clean_complete += 1
            for c in cases:
                m = c["metrics"]
                events.extend(m.get("swing_events", []))
                if not c["passed"]:
                    aborts.append(f"{c['name']}(held={m['solver_held_steps']},"
                                  f"net={m['net_horizontal_distance_m']:.3f})")
                if "planned_swing_samples" in m:
                    drag += frac(m["planned_swing_contact_samples"],
                                 m["planned_swing_samples"])
                    delay.append(max(m["max_liftoff_delay_samples"]))
                holds.append(m["solver_held_steps"])
                if c["name"] == "turn_in_place":
                    nets.append(m["net_horizontal_distance_m"])
                else:
                    stroke.append(m["net_horizontal_distance_m"])
                err = m.get("mean_planned_stance_joint_error_rad")
                if isinstance(err, list) and len(err) == 3:
                    femur.append(err[1])
                heave.append(m["maximum_body_height_m"] - m["minimum_body_height_m"])
                rate.append(m["maximum_servo_target_rate_radps"] / NO_LOAD_RADPS)
        line = f"{screen:16s} {arm:14s} pass {passed}/{len(files)}"
        if drag:
            line += (f"  drag {st.mean(drag):5.1f}%"
                     f"  median_case_max_swing_contact_run_samples {st.median(delay):5.1f}")
        if femur:
            line += f"  femur_err {st.mean(femur):.3f}"
        if heave:
            line += f"  heave {st.mean(heave)*1000:4.1f} mm"
        if rate:
            line += f"  peak_rate {st.mean(rate):.2f}x_noload"
        if stroke:
            line += f"  walk_net {st.mean(stroke):.3f}"
        line += f"  complete_held_zero {clean_complete}/{len(files)} incomplete {incomplete}"
        if nets:
            line += "  turn_net " + "/".join(f"{n:.3f}" for n in nets)
        print(line)
        if aborts:
            print(f"{'':32s} fails: {', '.join(aborts)}")
        if events:
            eligible = [e for e in events if not e["left_censored"]]
            lifted = [e["liftoff_ms"] for e in eligible if e["liftoff_ms"] >= 0]
            never = sum(e["complete"] and e["liftoff_ms"] < 0 for e in eligible)
            censored = sum(not e["complete"] and e["liftoff_ms"] < 0 for e in eligible)
            print(f"{'':32s} first_liftoff_ms_p50_lifted_only={st.median(lifted) if lifted else None} "
                  f"lifted={len(lifted)} never_lifted_complete={never} "
                  f"right_censored_before_lift={censored} "
                  f"recontact_events={sum(e['recontacts'] > 0 for e in eligible)} "
                  f"max_budget_closure_m={max(e['peak_closure_m'] for e in events):.3g}")


def main(argv):
    root = argv[1] if len(argv) > 1 else "/tmp/hexapod-gait-feas/ab"
    arms = argv[2].split(",") if len(argv) > 2 else None
    if arms is None:
        found = sorted({p.name.split("-")[-2] for p in Path(root).glob("*-*-*.stdout")})
        arms = found
    for screen in ("reverse_walk", "forward_walk", "turn_in_place", "sequential",
                   "aggressive_governor"):
        summarize(root, screen, arms)


if __name__ == "__main__":
    main(sys.argv)
