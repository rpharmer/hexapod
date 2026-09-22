#!/usr/bin/env python3
"""Finalize a raw Pinocchio contact snapshot with revision metadata and checksum."""
import argparse
import hashlib
import json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--tree-revision", default="working-tree")
    parser.add_argument("--protocol-revision", default="physics-sim-protocol-working-tree")
    args = parser.parse_args()
    with open(args.input, encoding="utf-8") as stream:
        data = json.load(stream)
    data["tree_revision"] = args.tree_revision
    data["protocol_revision"] = args.protocol_revision
    data.pop("fixture_checksum", None)
    canonical = json.dumps(data, sort_keys=True, separators=(",", ":")).encode("utf-8")
    data["fixture_checksum"] = hashlib.sha256(canonical).hexdigest()
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(data["fixture_checksum"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
