#!/usr/bin/env python3
"""Schema and determinism regression checks for contact snapshot tooling."""
import copy
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__))
import contact_snapshot_audit


def expect_rejected(data, mutate, label):
    candidate = copy.deepcopy(data)
    mutate(candidate)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as stream:
        json.dump(candidate, stream)
        path = stream.name
    try:
        try:
            contact_snapshot_audit.validate(path)
        except (ValueError, OSError, json.JSONDecodeError):
            return
        raise RuntimeError(label + " was accepted")
    finally:
        os.unlink(path)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: contact_snapshot_negative_tests.py FIXTURE")
    with open(sys.argv[1], encoding="utf-8") as stream:
        data = json.load(stream)
    expect_rejected(data, lambda value: value["contacts"].__setitem__(1, value["contacts"][0]), "duplicate contact ID")
    expect_rejected(data, lambda value: value.__setitem__("fixture_checksum", "0" * 64), "checksum mismatch")
    reversed_data = copy.deepcopy(data)
    reversed_data["contacts"] = list(reversed(reversed_data["contacts"]))
    ids = [item["id"] for item in reversed_data["contacts"]]
    if ids == [item["id"] for item in data["contacts"]]:
        raise RuntimeError("contact-order permutation was not applied")
    print(json.dumps({"passed": True, "duplicate_id_rejected": True, "checksum_rejected": True, "permuted_ids": ids}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
