#!/usr/bin/env python3
"""Strict validator for frozen Pinocchio contact snapshots."""
import argparse, hashlib, json, math, sys

def fail(msg):
    raise ValueError(msg)

def finite(value, path="root"):
    if isinstance(value, bool):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            fail(path + " is not finite")
        return
    if isinstance(value, list):
        for i, item in enumerate(value):
            finite(item, path + "[" + str(i) + "]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            finite(item, path + "." + key)

def matrix(value, name):
    if not isinstance(value, list) or not value or not all(isinstance(row, list) for row in value):
        fail(name + " must be a non-empty matrix")
    width = len(value[0])
    if width == 0 or any(len(row) != width for row in value):
        fail(name + " has inconsistent row sizes")
    return value

def max_abs_difference(a, b):
    if len(a) != len(b) or len(a[0]) != len(b[0]):
        return math.inf
    return max(abs(float(a[i][j]) - float(b[i][j])) for i in range(len(a)) for j in range(len(a[0])))

def symmetry_error(a):
    if len(a) != len(a[0]):
        return math.inf
    return max(abs(float(a[i][j]) - float(a[j][i])) for i in range(len(a)) for j in range(len(a)))

def validate(path):
    with open(path, encoding="utf-8") as stream:
        data = json.load(stream)
    finite(data)
    required = ["schema_version", "tree_revision", "protocol_revision", "fixture_checksum", "model", "step", "settings", "contacts", "jacobian", "delassus_articulated", "delassus_dense", "delassus_cholesky", "wrench_map", "solver_result", "audit"]
    for key in required:
        if key not in data:
            fail("missing " + key)
    if data["schema_version"] != 1:
        fail("unsupported schema version")
    if not isinstance(data["tree_revision"], str) or not data["tree_revision"]:
        fail("tree_revision must be a non-empty string")
    if not isinstance(data["protocol_revision"], str) or not data["protocol_revision"]:
        fail("protocol_revision must be a non-empty string")
    checksum = data["fixture_checksum"]
    if not isinstance(checksum, str) or len(checksum) != 64:
        fail("fixture_checksum must be a SHA-256 hex string")
    canonical = dict(data)
    canonical.pop("fixture_checksum", None)
    expected_checksum = hashlib.sha256(json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    if checksum != expected_checksum:
        fail("fixture checksum mismatch")
    nq = int(data["model"]["nq"]); nv = int(data["model"]["nv"]); count = int(data["model"]["contact_count"])
    if nq <= 0 or nv <= 0 or count <= 0:
        fail("invalid model dimensions")
    contacts = data["contacts"]
    if len(contacts) != count:
        fail("contact count mismatch")
    ids = [int(item["id"]) for item in contacts]
    if len(set(ids)) != len(ids):
        fail("duplicate contact IDs")
    for index, item in enumerate(contacts):
        for key in ["id", "leg", "joint1", "joint2", "friction", "restitution", "penetration", "point", "normal", "free_velocity", "biased_velocity", "impulse"]:
            if key not in item:
                fail("contact " + str(index) + " missing " + key)
        for key in ["point", "normal", "free_velocity", "biased_velocity", "impulse"]:
            if len(item[key]) != 3:
                fail("contact " + str(index) + " " + key + " dimension")
    jac = matrix(data["jacobian"], "jacobian")
    articulated = matrix(data["delassus_articulated"], "delassus_articulated")
    dense = matrix(data["delassus_dense"], "delassus_dense")
    chol = matrix(data["delassus_cholesky"], "delassus_cholesky")
    wrench = matrix(data["wrench_map"], "wrench_map")
    rows = 3 * count
    if len(jac) != rows or len(jac[0]) != nv:
        fail("jacobian dimension")
    for name, value in [("delassus_articulated", articulated), ("delassus_dense", dense), ("delassus_cholesky", chol)]:
        if len(value) != rows or len(value[0]) != rows:
            fail(name + " dimension")
    if len(wrench) != 6 or len(wrench[0]) != rows:
        fail("wrench map dimension")
    report = {
        "fixture": path,
        "schema_version": data["schema_version"],
        "contact_count": count,
        "ids": ids,
        "symmetry_error": {
            "articulated": symmetry_error(articulated),
            "dense": symmetry_error(dense),
            "cholesky": symmetry_error(chol),
        },
        "max_abs_difference": {
            "dense_articulated": max_abs_difference(dense, articulated),
            "dense_cholesky": max_abs_difference(dense, chol),
            "articulated_cholesky": max_abs_difference(articulated, chol),
        },
        "solver_result": data["solver_result"],
        "audit": data["audit"],
    }
    if report["symmetry_error"]["articulated"] > 1e-8 or report["symmetry_error"]["dense"] > 1e-8:
        fail("Delassus matrix is not symmetric")
    if report["max_abs_difference"]["dense_articulated"] > 1e-7:
        fail("dense/articulated mismatch")
    if report["max_abs_difference"]["dense_cholesky"] > 1e-6:
        fail("dense/Cholesky mismatch")
    return report

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fixtures", nargs="+")
    parser.add_argument("--output")
    args = parser.parse_args()
    reports = []
    try:
        for path in args.fixtures:
            reports.append(validate(path))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print("contact snapshot validation failed: " + str(error), file=sys.stderr)
        return 1
    result = {"schema_version": 1, "passed": True, "fixtures": reports}
    encoded = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
    print(encoded)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
