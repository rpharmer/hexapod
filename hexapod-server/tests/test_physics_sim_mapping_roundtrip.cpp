// Algebraic round-trip for physics-sim wire angles <-> servo LegState (default geometry).

#include "geometry_config.hpp"
#include "physics_sim_joint_wire_mapping.hpp"
#include "physics_sim_protocol.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool ok, const char* msg) {
    if (!ok) {
        std::cerr << "FAIL: " << msg << '\n';
    }
    return ok;
}

double wrap_pi(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

bool near_angle(float a, float b, double eps) {
    return std::abs(wrap_pi(static_cast<double>(a) - static_cast<double>(b))) <= eps;
}

bool near_servo_leg(const LegState& a, const LegState& b, double eps) {
    for (int j = 0; j < kJointsPerLeg; ++j) {
        if (std::abs(a.joint_state[j].pos_rad.value - b.joint_state[j].pos_rad.value) > eps) {
            return false;
        }
    }
    return true;
}

bool sim_round_trip_algebra(const HexapodGeometry& geo) {
    constexpr double kAngleEps = 2.0e-5;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geo.legGeometry[static_cast<std::size_t>(leg)].servo;
        for (int k = 0; k < 400; ++k) {
            const double t = static_cast<double>(k) * 0.03;
            const float sim_c = static_cast<float>(0.11 * std::sin(t + leg));
            const float sim_f = static_cast<float>(0.2 * std::sin(t * 1.1));
            const float sim_t = static_cast<float>(0.25 * std::cos(t * 0.9));

            const LegState servo =
                physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(cal, leg, sim_c, sim_f, sim_t);
            float rc = 0.0f;
            float rf = 0.0f;
            float rt = 0.0f;
            physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(cal, leg, servo, rc, rf, rt);

            if (!expect(near_angle(rc, sim_c, kAngleEps), "coxa sim round-trip") ||
                !expect(near_angle(rf, sim_f, kAngleEps), "femur sim round-trip") ||
                !expect(near_angle(rt, sim_t, kAngleEps), "tibia sim round-trip")) {
                std::cerr << " leg=" << leg << " k=" << k << " sim=(" << sim_c << "," << sim_f << "," << sim_t
                          << ") got=(" << rc << "," << rf << "," << rt << ")\n";
                return false;
            }
        }
    }
    return true;
}

bool servo_round_trip_through_wire(const HexapodGeometry& geo) {
    constexpr double kEps = 1.0e-6;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geo.legGeometry[static_cast<std::size_t>(leg)].servo;
        for (int k = 0; k < 200; ++k) {
            const double t = static_cast<double>(k) * 0.07;
            const float sim_c = static_cast<float>(0.09 * std::sin(t + leg));
            const float sim_f = static_cast<float>(0.18 * std::sin(t * 1.07));
            const float sim_t = static_cast<float>(0.21 * std::cos(t * 0.97));
            const LegState servo0 =
                physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(cal, leg, sim_c, sim_f, sim_t);
            float rc = 0.0f;
            float rf = 0.0f;
            float rt = 0.0f;
            physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(cal, leg, servo0, rc, rf, rt);
            const LegState servo1 =
                physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(cal, leg, rc, rf, rt);
            if (!expect(near_servo_leg(servo0, servo1, kEps), "servo invariance servo=G(F(servo))")) {
                std::cerr << " leg=" << leg << " k=" << k << '\n';
                return false;
            }
        }
    }
    return true;
}

bool servo_targets_match_mechanical_wire_formula(const HexapodGeometry& geo) {
    constexpr double kAngleEps = 2.0e-5;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geo.legGeometry[static_cast<std::size_t>(leg)].servo;
        for (int k = 0; k < 200; ++k) {
            const double t = static_cast<double>(k) * 0.07;
            LegState joint{};
            joint.joint_state[COXA].pos_rad = AngleRad{0.15 * std::sin(t + 0.1 * leg)};
            joint.joint_state[FEMUR].pos_rad = AngleRad{-0.3 + 0.25 * std::sin(t * 0.9 + leg)};
            joint.joint_state[TIBIA].pos_rad = AngleRad{-0.7 + 0.35 * std::cos(t * 1.1 + 0.3 * leg)};

            const LegState servo = cal.toServoAngles(joint);
            float sim_c = 0.0f;
            float sim_f = 0.0f;
            float sim_t = 0.0f;
            physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(cal, leg, servo, sim_c, sim_f, sim_t);

            const float expected_c = static_cast<float>(
                joint.joint_state[COXA].pos_rad.value - physics_sim::kWireZeroCoxaMechanicalRad);
            const float expected_f = static_cast<float>(
                joint.joint_state[FEMUR].pos_rad.value - physics_sim::kWireZeroFemurMechanicalRad);
            const float expected_t = static_cast<float>(
                joint.joint_state[TIBIA].pos_rad.value - physics_sim::kWireZeroTibiaMechanicalRad);

            if (!expect(near_angle(sim_c, expected_c, kAngleEps), "coxa wire target should match sim zero offset") ||
                !expect(near_angle(sim_f, expected_f, kAngleEps), "femur wire target should match sim zero offset") ||
                !expect(near_angle(sim_t, expected_t, kAngleEps), "tibia wire target should match sim zero offset")) {
                std::cerr << " leg=" << leg << " k=" << k << " expected=(" << expected_c << "," << expected_f << ","
                          << expected_t << ") got=(" << sim_c << "," << sim_f << "," << sim_t << ")\n";
                return false;
            }
        }
    }
    return true;
}

} // namespace

int main() {
    const HexapodGeometry geo = geometry_config::buildDefaultHexapodGeometry();
    if (!expect(sim_round_trip_algebra(geo), "f(g(sim)) == sim")) {
        return EXIT_FAILURE;
    }
    if (!expect(servo_round_trip_through_wire(geo), "servo -> sim -> servo")) {
        return EXIT_FAILURE;
    }
    if (!expect(servo_targets_match_mechanical_wire_formula(geo), "servo -> sim should preserve mechanical joints")) {
        return EXIT_FAILURE;
    }
    std::cout << "test_physics_sim_mapping_roundtrip ok\n";
    return EXIT_SUCCESS;
}
