#!/usr/bin/env python3
"""Parse MINPHYS_DIAG_LOG / HEXAPOD_DIAG_LOG output and print hard numbers.

Usage:
    # capture both sim and server stderr into one file, then:
    python3 scripts/analyze_diag_log.py /tmp/diag.log

    # or pipe directly:
    MINPHYS_DIAG_LOG=1 HEXAPOD_DIAG_LOG=1 scripts/run_physics_stack.sh ... 2>/tmp/diag.log
    python3 scripts/analyze_diag_log.py /tmp/diag.log
"""

import re
import sys
from collections import defaultdict

SIM_RE = re.compile(
    r'\[DIAG_SIM\] seq=(\d+) body_y=(-?[\d.]+) body_vy=(-?[\d.]+)'
    r'(?: f\d=(-?[\d.]+))*'
    r'.*contacts=0x([0-9a-f]+)'
)
SIM_FOOT_RE = re.compile(r' f(\d)=(-?[\d.]+)')
SVR_RE = re.compile(
    r'\[DIAG_SVR\] leg=(\d+) ph=([\d.]+) tau_sw=([\d.]+) '
    r'grace=(\d) use_stance=(\d) fusion=(\d)'
)
BIAS_RE = re.compile(r'\[DIAG_BIAS\] anchor_err=([\d.]+) capped_to=([\d.]+)')


def parse(path):
    sim_rows = []
    svr_rows = []
    bias_rows = []

    with open(path) as f:
        for line in f:
            line = line.rstrip()
            if m := BIAS_RE.search(line):
                bias_rows.append((float(m.group(1)), float(m.group(2))))
                continue
            if m := SVR_RE.search(line):
                svr_rows.append({
                    'leg': int(m.group(1)),
                    'ph': float(m.group(2)),
                    'tau_sw': float(m.group(3)),
                    'grace': int(m.group(4)),
                    'use_stance': int(m.group(5)),
                    'fusion': int(m.group(6)),
                })
                continue
            if '[DIAG_SIM]' in line and 'seq=' in line:
                seq_m = re.search(r'seq=(\d+)', line)
                by_m = re.search(r'body_y=(-?[\d.]+)', line)
                bvy_m = re.search(r'body_vy=(-?[\d.]+)', line)
                contacts_m = re.search(r'contacts=0x([0-9a-f]+)', line)
                if not (seq_m and by_m and bvy_m and contacts_m):
                    continue
                feet = {int(fm.group(1)): float(fm.group(2))
                        for fm in SIM_FOOT_RE.finditer(line)}
                sim_rows.append({
                    'seq': int(seq_m.group(1)),
                    'body_y': float(by_m.group(1)),
                    'body_vy': float(bvy_m.group(1)),
                    'contacts': int(contacts_m.group(1), 16),
                    'feet': feet,
                })

    return sim_rows, svr_rows, bias_rows


def foot_height_stats(sim_rows):
    """Per-leg min/max/peak foot height across all samples."""
    stats = defaultdict(lambda: {'min': 1e9, 'max': -1e9, 'count': 0})
    for row in sim_rows:
        for leg, fy in row['feet'].items():
            s = stats[leg]
            s['min'] = min(s['min'], fy)
            s['max'] = max(s['max'], fy)
            s['count'] += 1
    return stats


def swing_cycles(sim_rows, leg, threshold=0.003):
    """Detect swing cycles (foot off ground) for a leg and return peak heights."""
    peaks = []
    in_swing = False
    current_max = 0.0
    for row in sim_rows:
        fy = row['feet'].get(leg)
        if fy is None:
            continue
        contact = bool(row['contacts'] & (1 << leg))
        airborne = (not contact) and fy > threshold
        if airborne:
            in_swing = True
            current_max = max(current_max, fy)
        elif in_swing:
            peaks.append(current_max)
            in_swing = False
            current_max = 0.0
    if in_swing and current_max > 0:
        peaks.append(current_max)
    return peaks


def launch_events(sim_rows, body_vy_threshold=1.0):
    """Return rows where body upward velocity exceeds threshold."""
    return [r for r in sim_rows if r['body_vy'] > body_vy_threshold]


def grace_stats(svr_rows):
    grace_by_leg = defaultdict(list)
    no_grace_by_leg = defaultdict(list)
    for r in svr_rows:
        if r['grace']:
            grace_by_leg[r['leg']].append(r['tau_sw'])
        else:
            no_grace_by_leg[r['leg']].append(r['tau_sw'])
    return grace_by_leg, no_grace_by_leg


def main():
    if len(sys.argv) < 2:
        print("Usage: analyze_diag_log.py <log_file>", file=sys.stderr)
        sys.exit(1)

    path = sys.argv[1]
    sim_rows, svr_rows, bias_rows = parse(path)

    print(f"=== Parsed {len(sim_rows)} sim samples, {len(svr_rows)} server events, "
          f"{len(bias_rows)} bias cap events ===\n")

    # ---- Body vertical velocity ----
    if sim_rows:
        body_vys = [r['body_vy'] for r in sim_rows]
        body_ys  = [r['body_y']  for r in sim_rows]
        print(f"Body Y position (sim, y-up):  min={min(body_ys):.4f}  max={max(body_ys):.4f} m")
        print(f"Body Y velocity (sim, y-up):  min={min(body_vys):.4f}  max={max(body_vys):.4f} m/s")
        launches = launch_events(sim_rows, body_vy_threshold=0.8)
        print(f"Launch events (body_vy > 0.8 m/s): {len(launches)}")
        if launches:
            worst = max(launches, key=lambda r: r['body_vy'])
            print(f"  Worst: seq={worst['seq']} body_vy={worst['body_vy']:.4f} body_y={worst['body_y']:.4f}")
        print()

    # ---- Foot height stats ----
    if sim_rows:
        fstats = foot_height_stats(sim_rows)
        print("Foot world height (sim Y, ground=0):")
        print(f"  {'leg':>3}  {'min_m':>8}  {'max_m':>8}  {'samples':>8}")
        for leg in sorted(fstats):
            s = fstats[leg]
            print(f"  {leg:>3}  {s['min']:>8.4f}  {s['max']:>8.4f}  {s['count']:>8}")
        print()

        print("Per-leg swing cycle peak heights (expect ~0.03..0.05 m if liftoff working):")
        any_swings = False
        for leg in range(6):
            peaks = swing_cycles(sim_rows, leg)
            if peaks:
                any_swings = True
                print(f"  leg {leg}: {len(peaks)} swings  peak_max={max(peaks):.4f}  "
                      f"peak_min={min(peaks):.4f}  peak_mean={sum(peaks)/len(peaks):.4f} m")
            else:
                print(f"  leg {leg}: 0 swings detected")
        if not any_swings:
            print("  *** No swing cycles detected — feet may never be leaving the ground ***")
        print()

    # ---- Server liftoff_grace stats ----
    if svr_rows:
        grace_by_leg, no_grace_by_leg = grace_stats(svr_rows)
        print("Server early-swing contact events (contact=true while swing phase):")
        print(f"  {'leg':>3}  {'grace_count':>12}  {'no_grace_count':>14}  {'tau_sw range (grace)':>22}")
        for leg in range(6):
            gc = grace_by_leg.get(leg, [])
            ngc = no_grace_by_leg.get(leg, [])
            tau_range = f"{min(gc):.3f}..{max(gc):.3f}" if gc else "n/a"
            print(f"  {leg:>3}  {len(gc):>12}  {len(ngc):>14}  {tau_range:>22}")
        total_grace = sum(len(v) for v in grace_by_leg.values())
        total_no_grace = sum(len(v) for v in no_grace_by_leg.values())
        if total_grace + total_no_grace > 0:
            pct = 100.0 * total_grace / (total_grace + total_no_grace)
            print(f"  grace fired on {pct:.1f}% of early-swing contact events")
        print()

    # ---- Anchor bias cap ----
    if bias_rows:
        errs = [r[0] for r in bias_rows]
        print(f"Anchor bias cap fired {len(bias_rows)} times")
        print(f"  error magnitude: min={min(errs):.4f}  max={max(errs):.4f}  "
              f"mean={sum(errs)/len(errs):.4f} m")
        over_10cm = [e for e in errs if e > 0.10]
        print(f"  events with error > 10 cm: {len(over_10cm)}")
    else:
        print("Anchor bias cap: 0 events (cap never fired or MINPHYS_DIAG_LOG not set)")

    print()
    print("Done.")


if __name__ == '__main__':
    main()
