// Control pipeline joint targets (servo space) should round-trip through the same sim wire mapping
// used by PhysicsSimBridge: servo -> sim angles -> servo preserves mechanical joint angles.

#include "control_pipeline.hpp"
#include "geometry_config.hpp"
#include "physics_sim_joint_wire_mapping.hpp"

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

bool near_joint_leg(const LegState& a, const LegState& b, double eps) {
    for (int j = 0; j < kJointsPerLeg; ++j) {
        if (std::abs(a.joint_state[j].pos_rad.value - b.joint_state[j].pos_rad.value) > eps) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    ControlPipeline pipeline;

    RobotState estimated{};
    estimated.timestamp_us = now_us();

    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.active_fault = FaultCode::NONE;

    const PipelineStepResult step = pipeline.runStep(estimated, intent, safety, true, 1);
    constexpr double kEps = 1.0e-6;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geo.legGeometry[static_cast<std::size_t>(leg)].servo;
        const LegState& servo0 = step.joint_targets.leg_states[static_cast<std::size_t>(leg)];
        float sc = 0.0f;
        float sf = 0.0f;
        float st = 0.0f;
        physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(cal, leg, servo0, sc, sf, st);

        const LegState j0_true = physics_sim_joint_wire_mapping::jointMechanicalFromServoLeg(cal, servo0);
        const float expected_sc =
            static_cast<float>(j0_true.joint_state[COXA].pos_rad.value - physics_sim::kWireZeroCoxaMechanicalRad);
        const float expected_sf =
            static_cast<float>(j0_true.joint_state[FEMUR].pos_rad.value - physics_sim::kWireZeroFemurMechanicalRad);
        const float expected_st =
            static_cast<float>(j0_true.joint_state[TIBIA].pos_rad.value - physics_sim::kWireZeroTibiaMechanicalRad);
        if (!expect(std::abs(sc - expected_sc) <= kEps, "IK coxa wire target should match sim zero offset") ||
            !expect(std::abs(sf - expected_sf) <= kEps, "IK femur wire target should match sim zero offset") ||
            !expect(std::abs(st - expected_st) <= kEps, "IK tibia wire target should match sim zero offset")) {
            std::cerr << " leg=" << leg << " expected=(" << expected_sc << "," << expected_sf << "," << expected_st
                      << ") got=(" << sc << "," << sf << "," << st << ")\n";
            return EXIT_FAILURE;
        }

        const LegState servo1 =
            physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(cal, leg, sc, sf, st);

        const LegState j0 = physics_sim_joint_wire_mapping::jointMechanicalFromServoLeg(cal, servo0);
        const LegState j1 = physics_sim_joint_wire_mapping::jointMechanicalFromServoLeg(cal, servo1);
        if (!expect(near_joint_leg(j0, j1, kEps), "IK servo targets round-trip through sim wire mapping")) {
            std::cerr << " leg=" << leg << " j0=(" << j0.joint_state[0].pos_rad.value << ","
                      << j0.joint_state[1].pos_rad.value << "," << j0.joint_state[2].pos_rad.value << ") j1=("
                      << j1.joint_state[0].pos_rad.value << "," << j1.joint_state[1].pos_rad.value << ","
                      << j1.joint_state[2].pos_rad.value << ")\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "test_physics_sim_ik_wire_roundtrip ok\n";
    return EXIT_SUCCESS;
}
