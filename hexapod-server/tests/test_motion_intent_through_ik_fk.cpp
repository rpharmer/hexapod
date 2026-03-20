#include "body_controller.hpp"
#include "control_pipeline.hpp"
#include "gait_scheduler.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "leg_ik.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool finiteJointTargets(const JointTargets& targets) {
    for (const auto& leg : targets.leg_raw_states) {
        for (const auto& joint : leg.joint_raw_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

bool gaitSchedulerRespondsToWalkIntent() {
    GaitScheduler gait;
    EstimatedState est{};
    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    gait.update(est, walk, safety);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const GaitState advanced = gait.update(est, walk, safety);

    const bool leg0_leg1_offset = std::fabs(advanced.phase[0] - advanced.phase[1]) > 0.25;
    const bool stride_active = advanced.stride_phase_rate_hz.value >= 0.5;
    const bool timestamp_set = !advanced.timestamp_us.isZero();

    return expect(stride_active, "walk intent should produce positive stride phase rate") &&
           expect(leg0_leg1_offset, "tripod gait should offset neighboring leg phases") &&
           expect(timestamp_set, "gait update should stamp output time");
}

bool bodyControllerUsesGaitState() {
    BodyController body;
    EstimatedState est{};
    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;

    SafetyState safety{};
    safety.inhibit_motion = false;

    GaitState gait{};
    gait.in_stance.fill(true);
    gait.phase.fill(0.0);
    gait.in_stance[1] = false;
    gait.phase[1] = 0.75;

    const LegTargets targets = body.update(est, walk, gait, safety);

    const double leg0_x = targets.feet[0].pos_body_m.x;
    const double leg1_x = targets.feet[1].pos_body_m.x;
    const double leg1_z = targets.feet[1].pos_body_m.z;

    return expect(leg0_x != leg1_x, "stance and swing legs should get different x placement") &&
           expect(leg1_z > targets.feet[0].pos_body_m.z, "swing leg should receive swing height lift");
}

bool ikFkChainTracksBodyTargets() {
    const HexapodGeometry geometry = defaultHexapodGeometry();

    LegIK ik(geometry);
    LegFK fk;

    EstimatedState est{};
    est.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.leg_enabled.fill(true);

    LegTargets body_targets{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegRawState known_joint{};
        known_joint.joint_raw_state[0].pos_rad = AngleRad{0.15};
        known_joint.joint_raw_state[1].pos_rad = AngleRad{-0.25};
        known_joint.joint_raw_state[2].pos_rad = AngleRad{-0.85};
        body_targets.feet[leg] = fk.footInBodyFrame(known_joint, geometry.legGeometry[leg]);
    }

    const JointTargets joints = ik.solve(est, body_targets, safety);

    constexpr int kReferenceLeg = 0;
    const LegRawState joint_frame =
        geometry.legGeometry[kReferenceLeg].servo.toJointAngles(joints.leg_raw_states[kReferenceLeg]);
    const FootTarget fk_body = fk.footInBodyFrame(joint_frame, geometry.legGeometry[kReferenceLeg]);
    const Vec3 diff = fk_body.pos_body_m - body_targets.feet[kReferenceLeg].pos_body_m;
    const double err = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    if (!expect(err < 0.04, "ik/fk round trip should stay close to body target")) {
        return false;
    }

    return expect(finiteJointTargets(joints), "ik output should stay finite for reachable body targets");
}

bool controlPipelineProducesStableOutputs() {
    ControlPipeline pipeline;

    EstimatedState estimated{};
    estimated.timestamp_us = now_us();

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.timestamp_us = now_us();

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.active_fault = FaultCode::NONE;

    const PipelineStepResult result = pipeline.runStep(estimated, walk_intent, safety, true, 99);

    return expect(result.status.active_mode == RobotMode::WALK, "pipeline should preserve walk mode") &&
           expect(result.status.loop_counter == 99, "pipeline should preserve loop counter") &&
           expect(finiteJointTargets(result.joint_targets), "pipeline joint targets should be finite");
}

} // namespace

int main() {
    if (!gaitSchedulerRespondsToWalkIntent()) {
        return EXIT_FAILURE;
    }
    if (!bodyControllerUsesGaitState()) {
        return EXIT_FAILURE;
    }
    if (!ikFkChainTracksBodyTargets()) {
        return EXIT_FAILURE;
    }
    if (!controlPipelineProducesStableOutputs()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
