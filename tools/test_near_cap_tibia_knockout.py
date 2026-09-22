#!/usr/bin/env python3
"""Dense knockouts on frozen sl-abort-near-cap-v1. Does not rewrite the dump."""

import hashlib
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "docs/contact-snapshots/sl-abort-near-cap-v1.json"
EXPECTED_SHA256 = "11eeaef27a3284674a8e969baf68ab4f6aff1a42a40b6347b61512d65e48e3dd"
CAP = 10.0
WINNER = "leg_5_tibia_body"
WIRE_TIBIA = 17
WIRE_FEMUR = 16


def dense_speed(data, tau):
    state = data["kinematics"]
    vin = np.asarray(state["v_in"], dtype=float)
    mass = np.asarray(state["mass"], dtype=float)
    gravity = np.asarray(state["gravity_force"], dtype=float)
    bias = np.asarray(state["nonlinear_force"], dtype=float)
    dt = float(data["dt"])
    inverse = np.linalg.solve(mass, np.eye(24))
    acceleration = inverse @ tau - inverse @ gravity - inverse @ (bias - gravity)
    free = vin + dt * acceleration
    winner = next(link for link in state["links"] if link["name"] == WINNER)
    jacobian = np.asarray(winner["angular_jacobian"], dtype=float)
    return float(np.linalg.norm(jacobian @ free))


def classify(speed):
    return "under_cap" if speed < CAP else "still_over"


class NearCapTibiaKnockoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        digest = hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
        if digest != EXPECTED_SHA256:
            raise AssertionError(f"frozen dump hash {digest} != {EXPECTED_SHA256}")
        cls.data = json.loads(FIXTURE.read_text())
        cls.tau = np.asarray(cls.data["kinematics"]["tau"], dtype=float)
        cls.wires = {int(wire["wire"]): wire for wire in cls.data["kinematics"]["wires"]}
        cls.baseline = dense_speed(cls.data, cls.tau)
        after = np.asarray(cls.data["kinematics"]["v_after"], dtype=float)
        free = np.asarray(cls.data["kinematics"]["v_free"], dtype=float)
        winner = next(link for link in cls.data["kinematics"]["links"] if link["name"] == WINNER)
        jacobian = np.asarray(winner["angular_jacobian"], dtype=float)
        cls.contact_delta = float(np.linalg.norm(jacobian @ after) - np.linalg.norm(jacobian @ free))

    def knockout(self, *zero_wires, keep_only=None):
        tau = self.tau.copy()
        if keep_only is not None:
            for wire in range(18):
                if wire == keep_only:
                    continue
                tau[self.wires[wire]["v_index"]] = 0.0
        for wire in zero_wires:
            tau[self.wires[wire]["v_index"]] = 0.0
        speed = dense_speed(self.data, tau)
        return speed, classify(speed), speed - self.baseline

    def test_fixture_and_winner(self):
        self.assertEqual(self.data["kind"], "speed_limit")
        self.assertEqual(self.data["winner"]["frame"], WINNER)
        self.assertGreater(self.baseline, CAP)
        self.assertAlmostEqual(self.wires[WIRE_TIBIA]["error"], 0.630, places=2)

    def test_zero_winner_tibia_is_necessary(self):
        speed, label, delta = self.knockout(WIRE_TIBIA)
        print(f"zero_wire17 speed_free={speed:.9g} {label} delta={delta:.9g}")
        self.assertEqual(label, "under_cap")
        self.assertLess(delta, -0.05)

    def test_zero_femur_16_is_not_the_lever(self):
        speed, label, delta = self.knockout(WIRE_FEMUR)
        print(f"zero_wire16 speed_free={speed:.9g} {label} delta={delta:.9g}")
        self.assertEqual(label, "still_over")
        self.assertGreater(delta, -0.02)

    def test_other_fifteen_motors_are_not_the_lever(self):
        speed, label, delta = self.knockout(keep_only=WIRE_TIBIA)
        print(f"only_wire17 speed_free={speed:.9g} {label} delta={delta:.9g}")
        self.assertEqual(label, "still_over")

    def test_contact_is_not_v2(self):
        print(f"contact_delta={self.contact_delta:.9g}")
        self.assertLess(abs(self.contact_delta), 0.02)

    def test_wire_17_is_the_single_largest_under_cap_knockout(self):
        tibia = self.knockout(WIRE_TIBIA)
        femur = self.knockout(WIRE_FEMUR)
        only = self.knockout(keep_only=WIRE_TIBIA)
        self.assertEqual(tibia[1], "under_cap")
        self.assertLess(tibia[2], femur[2])
        self.assertLess(tibia[2], only[2])

    def test_remainder_0p25_lands_under_cap(self):
        error = abs(float(self.wires[WIRE_TIBIA]["error"]))
        tau = self.tau.copy()
        tau[self.wires[WIRE_TIBIA]["v_index"]] *= 0.25 / error
        speed = dense_speed(self.data, tau)
        print(f"remainder_0.25 speed_free={speed:.9g} {classify(speed)}")
        self.assertEqual(classify(speed), "under_cap")
        tau[self.wires[WIRE_TIBIA]["v_index"]] = self.tau[self.wires[WIRE_TIBIA]["v_index"]]
        dump_error = dense_speed(self.data, tau)
        self.assertGreater(dump_error, CAP)


if __name__ == "__main__":
    if hashlib.sha256(FIXTURE.read_bytes()).hexdigest() != EXPECTED_SHA256:
        print(f"FAIL: {FIXTURE} hash mismatch", file=sys.stderr)
        sys.exit(1)
    unittest.main()
