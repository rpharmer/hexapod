#!/usr/bin/env python3
"""Quasi-static stance torque vs MG996R stall, from the production geometry.

The controller plans tibia length 0.104 m (assembly, including the 18 mm foot
sphere). Femur+tibia reach is therefore 0.164 m; coxa mount z is -0.007 m, so
a 0.14 m body height asks for 0.133 m of vertical reach — 81% of the chain,
not a singularity. Gravity joint torque is the pitch moment of an equal-share
upward foot reaction. It is *not* the PD torque that chases tracking error.

Usage:
  python3 tools/stance_torque_authority.py
  python3 tools/stance_torque_authority.py --snapshot docs/contact-snapshots/sl-abort-default-straight-v1.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

G = 9.80665
BODY_M = 0.40
LEG_M = 0.055 + 0.070 + 0.055 + 0.008
TOTAL_M = BODY_M + 6 * LEG_M
STALL = 1.471
L_COXA = 0.043
L_FEMUR = 0.060
L_TIBIA = 0.104
COXA_Z = -0.007
REACH_MARGIN = 0.005
NOMINAL_REACH_FRAC = 0.55
WN = 25.0


def stance_rho(body_height_m: float) -> tuple[float, float, float]:
    femur_tibia = L_FEMUR + L_TIBIA - REACH_MARGIN
    foot_z = -body_height_m - COXA_Z
    if abs(foot_z) > femur_tibia:
        foot_z = math.copysign(femur_tibia, foot_z)
    max_rho = math.sqrt(max(0.0, femur_tibia * femur_tibia - foot_z * foot_z))
    desired = NOMINAL_REACH_FRAC * (L_FEMUR + L_TIBIA)
    return min(desired, max_rho), foot_z, femur_tibia


def femur_gravity_nm(body_height_m: float, support_count: int) -> float:
    rho, _, _ = stance_rho(body_height_m)
    share = TOTAL_M * G / max(support_count, 1)
    return rho * share


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", type=Path)
    args = parser.parse_args()

    print(f"mass {TOTAL_M:.3f} kg  weight {TOTAL_M*G:.2f} N  stall {STALL} N·m")
    print(f"tibia_plan {L_TIBIA} m (assembly)  femur+tibia {L_FEMUR+L_TIBIA:.3f} m")
    print()
    print("height  foot_z  rho    ext%   τ_femur 3-leg  2-leg  1-leg   vs stall (3/2/1)")
    for h in (0.16, 0.14, 0.12, 0.11, 0.10, 0.08):
        rho, foot_z, reach = stance_rho(h)
        ext = 100.0 * abs(foot_z) / reach
        t3, t2, t1 = (femur_gravity_nm(h, n) for n in (3, 2, 1))
        print(
            f" {h:4.2f}  {foot_z:+6.3f} {rho:5.3f}  {ext:4.1f}  "
            f"{t3:6.3f}     {t2:6.3f} {t1:6.3f}   "
            f"{t3/STALL:4.0%}/{t2/STALL:4.0%}/{t1/STALL:4.0%}"
        )

    if args.snapshot is None:
        return
    d = json.loads(args.snapshot.read_text())
    k = d["kinematics"]
    g = k["gravity_force"][6:]
    tau = k["tau"][6:]
    print()
    print(f"snapshot {args.snapshot.name}  implicit={d.get('implicit_damping')}  peak_pd={d.get('peak_pd_abs_error'):.3f}")
    print(f"  max |gravity_force| joint {max(abs(x) for x in g):.4f} N·m  ({max(abs(x) for x in g)/STALL:.1%} stall)")
    print(f"  max |tau| joint           {max(abs(x) for x in tau):.4f} N·m  ({max(abs(x) for x in tau)/STALL:.1%} stall)")
    print("  PD, not gravity, is using the envelope.")


if __name__ == "__main__":
    main()
