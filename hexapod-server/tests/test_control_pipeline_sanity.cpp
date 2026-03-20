#include "control_pipeline.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool allFinite(const JointTargets& targets) {
    for (const auto& leg : targets.leg_raw_states) {
        for (const auto& joint : leg.joint_raw_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

int main() {
    ControlPipeline pipeline;

    EstimatedState estimated{};
    estimated.timestamp_us = now_us();

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.timestamp_us = now_us();

    SafetyState healthy_safety{};
    healthy_safety.inhibit_motion = false;
    healthy_safety.active_fault = FaultCode::NONE;

    const PipelineStepResult nominal = pipeline.runStep(
        estimated, walk_intent, healthy_safety, true, 42);

    if (!expect(nominal.status.active_mode == RobotMode::WALK, "pipeline should keep WALK mode when safety is healthy") ||
        !expect(nominal.status.estimator_valid, "estimator should be valid for non-zero timestamp") ||
        !expect(nominal.status.bus_ok, "status should propagate bus health") ||
        !expect(nominal.status.loop_counter == 42, "status should preserve loop counter") ||
        !expect(allFinite(nominal.joint_targets), "joint target outputs should remain finite")) {
        return EXIT_FAILURE;
    }

    SafetyState faulted_safety = healthy_safety;
    faulted_safety.active_fault = FaultCode::MOTOR_FAULT;

    const PipelineStepResult faulted = pipeline.runStep(
        estimated, walk_intent, faulted_safety, true, 43);

    if (!expect(faulted.status.active_mode == RobotMode::FAULT, "active safety fault should force FAULT mode") ||
        !expect(faulted.status.active_fault == FaultCode::MOTOR_FAULT, "status should expose active safety fault") ||
        !expect(allFinite(faulted.joint_targets), "faulted run should still produce finite joint targets")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
