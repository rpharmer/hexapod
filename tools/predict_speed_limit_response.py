#!/usr/bin/env python3
"""Test-local coupled position-PD predictor; no simulator activation or tuning."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from audit_speed_limit_kinematics import audit


def motor_torque(requested, velocity, stall, no_load):
    available = stall * np.where(requested * velocity > 0.0,
        np.maximum(0.0, 1.0 - np.abs(velocity) / no_load), 1.0)
    return np.clip(requested, -available, available), available


def predict(path):
    start = time.perf_counter()
    reference = audit(path)
    data = json.loads(path.read_text())
    state = data["kinematics"]
    wires = sorted(state["wires"], key=lambda w: w["wire"])
    indices = np.array([w["v_index"] for w in wires])
    velocity = np.array(state["v_in"])
    incoming = velocity[indices]
    inertia = np.array([w["effective_inertia"] for w in wires])
    error = np.array([w["error"] for w in wires])
    rates = np.array([w["target_rate"] for w in wires])
    requested_reference = np.array([w["requested_tau"] for w in wires])
    available_reference = np.array([w["available_tau"] for w in wires])
    omega, zeta = state["omega_n"], state["zeta"]
    gain = state["servo_gain_scale"] * data["pd_gain"]
    no_load = data["no_load_speed"]
    if not all(np.isfinite(x) and x > 0.0 for x in (omega, zeta, gain, no_load)):
        raise ValueError("invalid actuator settings")
    reconstructed_requested = gain * inertia * (omega * omega * error - 2.0 * zeta * omega * incoming)
    requested_error = float(np.max(np.abs(reconstructed_requested - requested_reference)))
    if requested_error > 1e-10:
        raise ValueError("unrecorded load gain or mismatched actuator law; cannot predict fixture")
    # Infer, then verify, stall torque from the captured envelope. Do not assume
    # nominal torque or silently ignore a scale applied to the captured plant.
    factors = np.where(requested_reference * incoming > 0.0,
        np.maximum(0.0, 1.0 - np.abs(incoming) / no_load), 1.0)
    identifiable = factors > 1e-12
    if not identifiable.any():
        raise ValueError("stall torque is unidentifiable from this snapshot")
    stall = float(np.median(available_reference[identifiable] / factors[identifiable]))
    if stall <= 0.0 or not np.allclose(stall * factors, available_reference, atol=1e-10, rtol=1e-10):
        raise ValueError("inconsistent captured torque-speed envelope")
    mass = np.array(state["mass"])
    bias = np.array(state["nonlinear_force"])
    angular_jacobians = np.array([link["angular_jacobian"] for link in state["links"]])
    contact_dv = np.array(state["v_after"]) - np.array(state["v_free"])
    leg = data["winner_leg"]
    if not isinstance(leg, int) or not 0 <= leg < 6:
        raise ValueError("requires a leg winner for this bounded probe")
    command_dt = data["command_interval_s"]
    if not np.isfinite(command_dt) or command_dt <= 0.0:
        raise ValueError("invalid command interval")
    candidates = []
    for scale in (1.0, 0.75, 0.5, 0.25, 0.0):
        shift = np.zeros(18)
        # Counterfactual only: assumes previous-target anchoring over the last
        # command interval. This is not a measured alternate command stream.
        shift[3 * leg:3 * leg + 3] = (scale - 1.0) * rates[3 * leg:3 * leg + 3] * command_dt
        errors = np.remainder(error + shift + np.pi, 2 * np.pi) - np.pi
        requested = gain * inertia * (omega * omega * errors - 2.0 * zeta * omega * incoming)
        torque, available = motor_torque(requested, incoming, stall, no_load)
        generalized_torque = np.zeros(24)
        generalized_torque[indices] = torque
        free = velocity + data["dt"] * np.linalg.solve(mass, generalized_torque - bias)
        # A scalar estimate explicitly omits gravity, base and inter-joint
        # coupling, making its limitation measurable rather than implicit.
        diagonal = velocity.copy()
        diagonal[indices] += data["dt"] * torque / inertia
        speeds = np.linalg.norm(angular_jacobians @ free, axis=1)
        fixed_contact_speeds = np.linalg.norm(angular_jacobians @ (free + contact_dv), axis=1)
        winner = int(np.argmax(speeds))
        candidates.append({"common_scale": scale, "target_shift_rad": shift.tolist(),
            "torques_nm": torque.tolist(), "available_torques_nm": available.tolist(),
            "free_velocity": free.tolist(), "free_max_angular_speed": float(speeds[winner]),
            "free_worst_link": state["links"][winner]["name"],
            "diagonal_max_angular_speed": float(np.max(np.linalg.norm(angular_jacobians @ diagonal, axis=1))),
            "fixed_contact_probe_max_angular_speed": float(np.max(fixed_contact_speeds)),
            "actuator_work_j": float(data["dt"] * torque @ incoming),
            "free_kinetic_energy_delta_j": float(0.5 * (free @ mass @ free - velocity @ mass @ velocity)),
            "free_old_pose_within_angular_cap": bool(np.max(speeds) <= data["max_angular_speed"]),
            "physical_gates_passed": False})
    baseline = candidates[0]
    retry_candidates = []
    for p_scale, d_scale in ((1.0, 1.0), (.5, .5), (.5, 1.0)):
        requested = gain * inertia * (p_scale*omega*omega*error - d_scale*2*zeta*omega*incoming)
        torque, available = motor_torque(requested, incoming, stall, no_load)
        generalized = np.zeros(24)
        generalized[indices] = torque
        free = velocity + data["dt"] * np.linalg.solve(mass, generalized-bias)
        retry_candidates.append({"p_scale": p_scale, "d_scale": d_scale,
            "free_max_angular_speed": float(np.max(np.linalg.norm(angular_jacobians @ free, axis=1))),
            "fixed_contact_probe_max_angular_speed": float(np.max(np.linalg.norm(angular_jacobians @ (free+contact_dv), axis=1))),
            "actuator_work_j": float(data["dt"]*torque@incoming),
            "torques_nm": torque.tolist(), "physical_gates_passed": False})
    free_error = float(np.max(np.abs(np.array(baseline["free_velocity"]) - state["v_free"])))
    torque_error = float(np.max(np.abs(np.array(baseline["torques_nm"]) - np.array(state["tau"])[indices])))
    if free_error > 1e-10 or torque_error > 1e-10:
        raise ValueError("predictor does not reproduce captured torque/ABA response")
    return {"schema_version": 1, "fixture_sha256": reference["fixture_sha256"],
        "predictor": "clipped_position_PD_full_inverse_mass_one_step",
        "runtime_activation": False, "stall_torque_nm": stall,
        "settings": {"dt_s": data["dt"], "command_interval_s": command_dt,
            "omega_n": omega, "damping_ratio": zeta, "gain": gain, "no_load_radps": no_load,
            "angular_cap_radps": data["max_angular_speed"], "linear_cap_mps": data["max_linear_speed"]},
        "requested_torque_error": requested_error, "baseline_torque_error": torque_error,
        "baseline_free_velocity_error": free_error, "winner_leg": leg,
        "limitations": ["Frozen q, mass, nonlinear forces and angular Jacobian for one substep only",
            "Fixed-contact probe reuses captured delta, not a candidate contact re-solve",
            "Counterfactual reference shift assumes previous-target anchoring",
            "No integrated-pose, penetration, contact feasibility or locomotion guarantee"],
        "candidates": candidates, "retry_candidates": retry_candidates, "runtime_seconds": time.perf_counter() - start,
        "decision": "follow-up; not a production command governor"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = predict(args.fixture)
    root = Path(__file__).resolve().parents[1]
    result["date_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    result["head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    result["dirty_status"] = subprocess.check_output(["git", "status", "--short"], cwd=root, text=True).strip()
    result["predictor_source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if args.report:
        args.report.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: result[key] for key in ("baseline_free_velocity_error", "candidates", "decision")}, indent=2))


if __name__ == "__main__":
    main()
