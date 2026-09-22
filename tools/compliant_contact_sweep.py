#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from contact_basis_census import dot, mat_vec, norm, project_cone, solve_linear, sub


def extract_matrix(a, indices):
    return [[a[i][j] for j in indices] for i in indices]


def solve_compliant(g, bias, contacts, compliance, damping, tangent_regularisation, tolerance, max_iterations):
    count = len(contacts)
    dim = 3 * count
    system = [row[:] for row in g]
    for i in range(dim):
        system[i][i] += compliance + tangent_regularisation
    adjusted = bias[:]
    for i, contact in enumerate(contacts):
        adjusted[3 * i + 2] += damping * float(contact.get("penetration", 0.0))
    lam = project_cone(solve_linear(system, [-x for x in adjusted]), float(contacts[0].get("friction", 0.5)), count)
    previous = lam[:]
    step = 1.0 / max(max(sum(abs(value) for value in row) for row in system), 1.0e-9)
    for iteration in range(max_iterations):
        residual = sub(mat_vec(system, lam), [-x for x in adjusted])
        trial = [value - step * gradient for value, gradient in zip(lam, residual)]
        lam = project_cone(trial, float(contacts[0].get("friction", 0.5)), count)
        if norm(sub(lam, previous)) <= tolerance:
            previous = lam[:]
            break
        previous = lam[:]
    residual = sub(mat_vec(system, lam), [-x for x in adjusted])
    spectral_step = 1.0 / max(max(sum(abs(value) for value in row) for row in system), 1.0e-9)
    projected = project_cone([value - spectral_step * gradient for value, gradient in zip(lam, residual)], float(contacts[0].get("friction", 0.5)), count)
    projected_gradient_residual = norm(sub(lam, projected)) / spectral_step
    cone = 0.0
    complementarity = 0.0
    penetration = 0.0
    for i, contact in enumerate(contacts):
        base = 3 * i
        normal = lam[base + 2]
        tangent = math.sqrt(lam[base] * lam[base] + lam[base + 1] * lam[base + 1])
        cone = max(cone, max(0.0, -normal), max(0.0, tangent - float(contact.get("friction", 0.5)) * max(normal, 0.0)))
        complementarity += abs(normal * residual[base + 2])
        penetration = max(penetration, float(contact.get("penetration", 0.0)))
    velocity_norm = math.sqrt(max(0.0, dot(lam, mat_vec(g, lam))))
    return {
        "impulse": lam,
        "iterations": iteration + 1,
        "primal_residual": norm(residual),
        "dual_residual": norm(residual),
        "projected_gradient_residual": projected_gradient_residual,
        "complementarity_residual": complementarity,
        "cone_residual": cone,
        "max_penetration_m": penetration,
        "contact_work": dot(lam, bias),
        "velocity_delta_norm": velocity_norm,
        "mechanical_energy_delta_proxy": 0.5 * velocity_norm * velocity_norm + dot(lam, bias),
    }


def run_fixture(path, sweep):
    with open(path, encoding="utf-8") as stream:
        fixture = json.load(stream)
    contacts = sorted(fixture["contacts"], key=lambda item: int(item["id"]))
    count = len(contacts)
    all_rows = [3 * i + axis for i in range(count) for axis in range(3)]
    g = extract_matrix(fixture["delassus_dense"], all_rows)
    bias = [value for contact in contacts for value in contact["biased_velocity"]]
    records = []
    started = time.perf_counter()
    for compliance in sweep["normal_compliance"]:
        for damping in sweep["damping_ratio"]:
            for tangent in sweep["tangential_regularisation"]:
                result = solve_compliant(g, bias, contacts, compliance, damping, tangent, 1.0e-8, 800)
                result.update({
                    "normal_compliance": compliance,
                    "damping_ratio": damping,
                    "tangential_regularisation": tangent,
                    "contact_ids": [item["id"] for item in contacts],
                    "solver": "test-local-sap-like-projected-convex",
                })
                result["passes_frozen_physical_gate"] = result["projected_gradient_residual"] <= 1.0e-5 and result["cone_residual"] <= 1.0e-8 and not result["max_penetration_m"] >= 0.003 and result["mechanical_energy_delta_proxy"] <= 1.0e-8
                records.append(result)
    passing = [item for item in records if item["passes_frozen_physical_gate"]]
    fastest = min(passing, key=lambda item: (item["iterations"], item["normal_compliance"], item["damping_ratio"], item["tangential_regularisation"])) if passing else None
    return {
        "fixture": path,
        "contact_count": count,
        "contact_ids": [item["id"] for item in contacts],
        "parameter_count": len(records),
        "results": records,
        "selected": fastest,
        "decision": "candidate-frozen-physical-pass" if fastest is not None else "rejected-frozen-physical-gates",
        "runtime_ms": (time.perf_counter() - started) * 1000.0,
    }


def main():
    parser = argparse.ArgumentParser(description="Fixture-only compliant contact parameter sweep")
    parser.add_argument("fixtures", nargs="+")
    parser.add_argument("--output")
    args = parser.parse_args()
    sweep = {
        "normal_compliance": [1.0e-6, 1.0e-5, 1.0e-4],
        "damping_ratio": [0.7, 1.0, 1.5],
        "tangential_regularisation": [1.0e-7, 1.0e-6, 1.0e-5],
    }
    report = {
        "schema_version": 1,
        "solver_mode": "test-local-sap-like",
        "runtime_activation": "disabled",
        "sweep": sweep,
        "fixtures": [run_fixture(path, sweep) for path in args.fixtures],
    }
    report["passed"] = all(item["selected"] is not None for item in report["fixtures"])
    encoded = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
    print(encoded)
    return 0


if __name__ == "__main__":
    sys.exit(main())
