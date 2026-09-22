#!/usr/bin/env python3
"""Replay frozen speed-trip algebra; diagnose actual versus requested link rates."""

import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np


def critical_rate(error, incoming_rate, target_rate, omega, seconds):
    """Exact isolated critically damped position-PD response to a fixed ramp.

    Diagnostic only: omits coupled inertia, gravity, contact and motor clipping.
    """
    coefficient = omega * error - target_rate - incoming_rate
    return target_rate - ((target_rate - incoming_rate) - omega * coefficient * seconds) * np.exp(-omega * seconds)


def audit(path):
    data = json.loads(path.read_text())
    if data.get("schema_version") != 2 or data.get("kind") != "speed_limit":
        raise ValueError("requires speed_limit schema 2 with complete kinematics")
    state = data["kinematics"]
    if state["tangent_convention"] != "free_flyer_local_linear_then_angular" or state["angular_frame"] != "LOCAL_WORLD_ALIGNED":
        raise ValueError("unknown tangent/frame convention")

    def array(key, shape):
        value = np.asarray(state[key], dtype=float)
        if value.shape != shape or not np.isfinite(value).all():
            raise ValueError(f"{key}: invalid dimensions or non-finite value")
        return value

    q = array("q", (25,))
    vin, free, after, tau = (array(key, (24,)) for key in ("v_in", "v_free", "v_after", "tau"))
    mass = array("mass", (24, 24))
    bias, gravity = (array(key, (24,)) for key in ("nonlinear_force", "gravity_force"))
    dt = data["dt"]
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("invalid timestep")
    if abs(np.linalg.norm(q[3:7]) - 1.0) > 1e-9:
        raise ValueError("invalid free-flyer quaternion")
    symmetry = float(np.max(np.abs(mass - mass.T)))
    if symmetry > 1e-10 or np.linalg.eigvalsh(mass).min() <= 0:
        raise ValueError("mass matrix is not symmetric positive definite")
    inverse = np.linalg.solve(mass, np.eye(24))
    motor_acceleration = inverse @ tau
    gravity_acceleration = -(inverse @ gravity)
    coriolis_acceleration = -(inverse @ (bias - gravity))
    dense_acceleration = motor_acceleration + gravity_acceleration + coriolis_acceleration
    captured_acceleration = (free - vin) / dt
    acceleration_error = float(np.linalg.norm(dense_acceleration - captured_acceleration, np.inf))
    if acceleration_error > 1e-8 + 1e-9 * np.linalg.norm(captured_acceleration, np.inf):
        raise ValueError(f"dense/production acceleration mismatch: {acceleration_error}")
    wires = state["wires"]
    if len(wires) != 18 or sorted(w["wire"] for w in wires) != list(range(18)):
        raise ValueError("wire IDs missing or duplicated")
    if sorted(w["q_index"] for w in wires) != list(range(7, 25)) or sorted(w["v_index"] for w in wires) != list(range(6, 24)):
        raise ValueError("inconsistent joint/tangent mapping")
    wires = sorted(wires, key=lambda w: w["wire"])
    requested_velocity = vin.copy()
    for wire in wires:
        for key in ("zero_angle", "error", "target_rate", "effective_inertia", "requested_tau", "available_tau"):
            if not np.isfinite(wire[key]):
                raise ValueError(f"wire {wire['wire']}: non-finite {key}")
        if wire["effective_inertia"] <= 0 or wire["available_tau"] < 0:
            raise ValueError("invalid actuator inertia/torque envelope")
        requested_velocity[wire["v_index"]] = wire["target_rate"]
    links = []
    names = set()
    jacobian_error = 0.0
    for link in state["links"]:
        name = link["name"]
        if name in names:
            raise ValueError("duplicate link ID/name")
        names.add(name)
        jacobian = np.asarray(link["angular_jacobian"], dtype=float)
        if jacobian.shape != (3, 24) or not np.isfinite(jacobian).all():
            raise ValueError("invalid angular Jacobian")
        angular = {}
        for key, velocity in (("in", vin), ("free", free), ("after", after)):
            expected = np.asarray(link["angular_" + key], dtype=float)
            if expected.shape != (3,) or not np.isfinite(expected).all():
                raise ValueError("invalid captured angular velocity")
            error = np.linalg.norm(jacobian @ velocity - expected, np.inf)
            jacobian_error = max(jacobian_error, float(error))
            if expected.shape != (3,) or not np.isfinite(expected).all() or error > 1e-9:
                raise ValueError("Jacobian/velocity mismatch")
            angular[key] = jacobian @ velocity
        entry = {"name": name,
                 "angular_in": angular["in"].tolist(), "angular_free": angular["free"].tolist(), "angular_after": angular["after"].tolist(),
                 "speed_in": float(np.linalg.norm(angular["in"])), "speed_free": float(np.linalg.norm(angular["free"])),
                 "speed_after": float(np.linalg.norm(angular["after"])),
                 "speed_requested_at_current_configuration": float(np.linalg.norm(jacobian @ requested_velocity)),
                 "angular_free_delta": (angular["free"] - angular["in"]).tolist(),
                 "angular_contact_delta": (angular["after"] - angular["free"]).tolist(),
                 "angular_motor_acceleration": (jacobian @ motor_acceleration).tolist(),
                 "angular_gravity_acceleration": (jacobian @ gravity_acceleration).tolist(),
                 "angular_coriolis_acceleration": (jacobian @ coriolis_acceleration).tolist()}
        match = re.fullmatch(r"leg_(\d+)_(?:coxa|femur|tibia)_body", name)
        if match:
            leg_wires = wires[3 * int(match[1]):3 * int(match[1]) + 3]
            entry["joints"] = []
            for wire in leg_wires:
                vi = wire["v_index"]
                inertia = wire["effective_inertia"]
                if not np.isfinite(inertia) or inertia <= 0:
                    raise ValueError("invalid effective inertia")
                entry["joints"].append({
                    "wire": wire["wire"], "target_rate": wire["target_rate"], "error": wire["error"],
                    "vin": float(vin[vi]), "free": float(free[vi]), "after": float(after[vi]),
                    "tau": float(tau[vi]), "available_tau": wire["available_tau"],
                    "braking": bool(tau[vi] * vin[vi] < 0),
                    "diagonal_motor_acceleration": float(tau[vi] / inertia),
                    "own_motor_coupled_acceleration": float(inverse[vi, vi] * tau[vi]),
                    "other_motors_acceleration": float(motor_acceleration[vi] - inverse[vi, vi] * tau[vi]),
                    "gravity_acceleration": float(gravity_acceleration[vi]), "coriolis_acceleration": float(coriolis_acceleration[vi]),
                    "free_acceleration": float(captured_acceleration[vi]), "contact_dv": float(after[vi] - free[vi])})
        links.append(entry)
    if len(links) != 19:
        raise ValueError("expected chassis and 18 links")
    winner = next(link for link in links if link["name"] == data["winner"]["frame"])
    if abs(winner["speed_after"] - data["winner"]["winner_w"]) > 1e-9:
        raise ValueError("captured guard winner does not match replay")
    cap = float(data["max_angular_speed"])
    incoming = [link["name"] for link in links if link["speed_in"] > cap]
    aba_over = [link["name"] for link in links if link["speed_in"] <= cap < link["speed_free"]]
    contact_amplified = [link["name"] for link in links if link["speed_free"] <= cap < link["speed_after"]]
    classes = []
    if incoming:
        classes.append("v1_incoming_over_cap")
    if aba_over:
        classes.append("aba_over_cap")
    if contact_amplified:
        classes.append("v2_contact_amplified")
    return {"schema_version": 1, "fixture_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "mass_symmetry_error": symmetry, "dense_acceleration_error": acceleration_error,
            "jacobian_velocity_error": jacobian_error, "winner": winner,
            "incoming_over_cap_links": incoming,
            "aba_over_cap_links": aba_over,
            "contact_amplified_links": contact_amplified,
            "classification": classes,
            "links": links}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = audit(args.fixture)
    serialized = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.report:
        args.report.write_text(serialized)
    print(json.dumps({key: report[key] for key in (
        "dense_acceleration_error", "incoming_over_cap_links", "classification", "winner")}, indent=2))


if __name__ == "__main__":
    main()
