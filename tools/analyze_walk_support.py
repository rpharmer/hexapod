#!/usr/bin/env python3
"""Analyze complete planned swings and support-loss windows in a motion trace."""
import argparse
import hashlib
import json
from pathlib import Path


def analyze(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not any(r['status']['active_mode'] == 3 for r in rows):
        raise ValueError('trace contains no WALK samples')
    events = []
    for leg in range(6):
        start = None
        for i in range(1, len(rows)):
            if any(r['status']['active_mode'] != 3 for r in rows[i-1:i+1]):
                start = None
                continue
            previous = rows[i-1]['locomotion_debug']['planned_stance'][leg]
            stance = rows[i]['locomotion_debug']['planned_stance'][leg]
            if previous and not stance:
                start = i - 1
            if not previous and stance and start is not None:
                window = rows[start:i+1]
                def foot(r, key):
                    return r['locomotion_debug'][key][leg][2]
                command_peak = max(window, key=lambda r: foot(r, 'commanded_foot_body_m'))
                measured_peak = max(window, key=lambda r: foot(r, 'measured_foot_world_m'))
                origin = window[0]
                body = lambda r: r['estimated_state']['body_twist_state']['body_trans_m'][2]
                events.append(dict(
                    leg=leg, start=origin['sample_id'], end=window[-1]['sample_id'],
                    measured_lift_m=foot(measured_peak, 'measured_foot_world_m')-foot(origin, 'measured_foot_world_m'),
                    command_body_lift_m=foot(command_peak, 'commanded_foot_body_m')-foot(origin, 'commanded_foot_body_m'),
                    command_apex=command_peak['sample_id'],
                    body_dz_at_command_apex_m=body(command_peak)-body(origin),
                    measured_body_dz_at_command_apex_m=foot(command_peak, 'measured_foot_body_m')-foot(origin, 'measured_foot_body_m'),
                    tracking_z_at_command_apex_m=foot(command_peak, 'measured_foot_body_m')-foot(command_peak, 'commanded_foot_body_m'),
                    raw_contact_fraction=sum(r['locomotion_debug']['raw_contact'][leg] for r in window)/len(window),
                    held_fraction=sum(r['locomotion_debug']['hold_stance'][leg] for r in window)/len(window),
                ))
                start = None
    worst = min((r for r in rows if r['status']['active_mode'] == 3),
                key=lambda r: r['gait_state']['static_stability_margin_m'])
    index = rows.index(worst)
    keys = ('planned_stance', 'hold_stance', 'raw_contact', 'fused_support', 'fused_contact_phase',
            'fused_contact_confidence', 'measured_foot_world_m', 'commanded_foot_world_m')
    return dict(schema_version=1, source=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                complete_swings=events, worst_margin_window=[
                    dict(sample_id=r['sample_id'], gait=r['gait_state'],
                         contacts={k:r['locomotion_debug'][k] for k in keys})
                    for r in rows[max(0,index-3):index+4]])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--window', nargs=2, type=int)
    parser.add_argument('--leg', type=int, choices=range(6), default=4)
    args = parser.parse_args()
    if args.window:
        for line in args.trace.read_text().splitlines():
            r = json.loads(line)
            i = r['sample_id']
            if args.window[0] <= i <= args.window[1] and i % 10 == 0:
                d = r['locomotion_debug']
                detail = r.get('planning_detail', {})
                print(json.dumps(dict(step=i, phase=r['gait_state']['phase'][args.leg],
                    stance=d['planned_stance'][args.leg], hold=d['hold_stance'][args.leg],
                    raw=d['raw_contact'][args.leg], measured=d['measured_foot_body_m'][args.leg],
                    command=d['commanded_foot_body_m'][args.leg],
                    world_z=d['measured_foot_world_m'][args.leg][2],
                    command_world_z=d['commanded_foot_world_m'][args.leg][2],
                    detail={k:v[args.leg] for k,v in detail.items()})))
        raise SystemExit(0)
    print(json.dumps(analyze(args.trace), indent=2, allow_nan=False))
