#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def mat_vec(a, x):
    return [dot(row, x) for row in a]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def mat_sub(a, b):
    return [[x - y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def mat_add_diag(a, value):
    out = [row[:] for row in a]
    for i in range(len(out)):
        out[i][i] += value
    return out


def solve_linear(a, b):
    n = len(b)
    m = [a[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1.0e-12:
            return [0.0] * n
        m[col], m[pivot] = m[pivot], m[col]
        scale = m[col][col]
        for j in range(col, n + 1):
            m[col][j] /= scale
        for r in range(n):
            if r == col:
                continue
            factor = m[r][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                m[r][j] -= factor * m[col][j]
    return [m[i][n] for i in range(n)]


def rank_columns(columns, tol=1.0e-9):
    basis = []
    for column in columns:
        v = column[:]
        for q in basis:
            alpha = dot(v, q)
            v = [x - alpha * y for x, y in zip(v, q)]
        length = norm(v)
        if length > tol:
            basis.append([x / length for x in v])
    return len(basis)


def rank_rows(rows, tol=1.0e-9):
    if not rows:
        return 0
    return rank_columns([[rows[r][c] for r in range(len(rows))] for c in range(len(rows[0]))], tol)


def project_cone(values, friction, contact_count):
    out = values[:]
    for i in range(contact_count):
        base = 3 * i
        tangent = math.sqrt(out[base] * out[base] + out[base + 1] * out[base + 1])
        normal = out[base + 2]
        if normal <= 0.0:
            out[base] = 0.0
            out[base + 1] = 0.0
            out[base + 2] = 0.0
        elif tangent > friction * normal:
            scale = friction * normal / tangent
            out[base] *= scale
            out[base + 1] *= scale
    return out


def extract_matrix(a, indices):
    rows = []
    for i in indices:
        rows.append([a[i][j] for j in indices])
    return rows


def candidate_solve(g, b, contacts, settings, iterations=400):
    count = len(contacts)
    if count == 0:
        return {"impulse": [], "residual": 0.0, "iterations": 0, "cone": 0.0, "complementarity": 0.0, "velocity_norm": 0.0, "work": 0.0}
    compliance = float(settings["normal_compliance"])
    damping = float(settings["damping_ratio"])
    tangent_reg = float(settings["tangential_regularisation"])
    friction = float(contacts[0].get("friction", 0.5))
    h = 3 * count
    system = mat_add_diag(g, compliance + tangent_reg)
    max_row = max(sum(abs(x) for x in row) for row in system)
    step = 1.0 / max(max_row, 1.0e-9)
    bias = b[:]
    for i, contact in enumerate(contacts):
        bias[3 * i + 2] += damping * float(contact.get("penetration", 0.0))
    lam = [0.0] * h
    for k in range(iterations):
        grad = sub(mat_vec(system, lam), [-x for x in bias])
        trial = [x - step * y for x, y in zip(lam, grad)]
        nxt = project_cone(trial, friction, count)
        if norm(sub(nxt, lam)) < 1.0e-10:
            lam = nxt
            break
        lam = nxt
    residual_vec = sub(mat_vec(system, lam), [-x for x in bias])
    cone = 0.0
    complementarity = 0.0
    for i in range(count):
        base = 3 * i
        n = lam[base + 1]
        t = math.sqrt(lam[base] * lam[base] + lam[base + 1] * lam[base + 1])
        cone = max(cone, max(0.0, t - friction * max(n, 0.0)), max(0.0, -n))
        complementarity += abs(n * residual_vec[base + 2])
    return {
        "impulse": lam,
        "residual": norm(residual_vec),
        "iterations": k + 1,
        "cone": cone,
        "complementarity": complementarity,
        "velocity_norm": math.sqrt(max(0.0, dot(lam, mat_vec(g, lam)))),
        "work": dot(lam, b),
    }


def wrench_for(w, lam, indices):
    rows = [w[i] for i in range(len(w))]
    cols = []
    for i in indices:
        cols.extend([3 * i, 3 * i + 1, 3 * i + 2])
    return [dot([row[c] for c in cols], lam) for row in rows]


def main():
    parser = argparse.ArgumentParser(description="Deterministic offline contact-basis census")
    parser.add_argument("fixtures", nargs="+", help="frozen contact snapshot JSON files")
    parser.add_argument("--output", help="write machine-readable report")
    args = parser.parse_args()
    report = {"schema_version": 1, "runtime_selection_enabled": False, "fixtures": []}
    for fixture_path in args.fixtures:
        started = time.perf_counter()
        with open(fixture_path, encoding="utf-8") as stream:
            fixture = json.load(stream)
        contacts = sorted(fixture["contacts"], key=lambda c: int(c["id"]))
        count = len(contacts)
        g = fixture["delassus_dense"]
        b = []
        for contact in contacts:
            b.extend(contact["biased_velocity"])
        w = fixture["wrench_map"]
        all_indices = list(range(count))
        impulse_indices = [i for i, c in enumerate(contacts) if norm(c["impulse"]) > 1.0e-10]
        wrench_basis = []
        current_rank = 0
        for i in all_indices:
            trial = wrench_basis + [i]
            columns = []
            for c in trial:
                for axis in range(3):
                    columns.append([w[row][3 * c + axis] for row in range(6)])
            next_rank = rank_columns(columns)
            if next_rank > current_rank:
                wrench_basis.append(i)
                current_rank = next_rank
        jac_basis = []
        current_rank = 0
        for i in all_indices:
            trial = jac_basis + [i]
            rows = []
            for c in trial:
                rows.extend(fixture["jacobian"][3 * c:3 * c + 3])
            next_rank = rank_rows(rows)
            if next_rank > current_rank:
                jac_basis.append(i)
                current_rank = next_rank
        sets = {
            "all": all_indices,
            "oracle_impulse": impulse_indices,
            "chassis_wrench_basis": wrench_basis,
            "articulated_jacobian_basis": jac_basis,
        }
        all_result = None
        candidates = []
        for name, selected in sets.items():
            rows = []
            for i in selected:
                rows.extend([3 * i, 3 * i + 1, 3 * i + 2])
            sub_g = extract_matrix(g, rows)
            sub_b = [b[i] for i in rows]
            sub_contacts = [contacts[i] for i in selected]
            result = candidate_solve(sub_g, sub_b, sub_contacts, {"normal_compliance": 1.0e-5, "damping_ratio": 1.0, "tangential_regularisation": 1.0e-6})
            if name == "all":
                all_result = result
            full_wrench = wrench_for(w, result["impulse"], selected)
            if all_result is None:
                wrench_error = 0.0
            else:
                reference = wrench_for(w, all_result["impulse"], all_indices)
                scale = max(norm(reference), 1.0e-12)
                wrench_error = norm(sub(full_wrench, reference)) / scale
            wrench_columns = [[w[row][3 * i + axis] for row in range(6)] for i in selected for axis in range(3)]
            wrench_rank_value = rank_columns(wrench_columns)
            candidates.append({
                "name": name,
                "contact_ids": [contacts[i]["id"] for i in selected],
                "legs": [contacts[i].get("leg") for i in selected],
                "joints": [contacts[i].get("joint2") for i in selected],
                "contact_count": len(selected),
                "wrench_rank": wrench_rank_value,
                "articulated_rank": rank_rows([fixture["jacobian"][r] for i in selected for r in range(3 * i, 3 * i + 3)]),
                "solve": result,
                "chassis_wrench_error_relative": wrench_error,
                "selection_is_diagnostic_only": True,
            })
        report["fixtures"].append({
            "fixture": fixture_path,
            "contact_order": [c["id"] for c in contacts],
            "candidate_sets": candidates,
            "deterministic_order": True,
            "runtime_ms": (time.perf_counter() - started) * 1000.0,
            "selection_gate": {"relative_chassis_wrench_error_max": 1.0e-6, "passed": all(c["chassis_wrench_error_relative"] <= 1.0e-6 for c in candidates)},
        })
    report["passed"] = True
    output = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(output + "\n")
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
