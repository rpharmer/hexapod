#include "gait_policy_planner.hpp"

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

MotionIntent walkIntent(double speed_mps, double yaw_rate_radps, GaitType gait = GaitType::TRIPOD) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.gait = gait;
    intent.speed_mps = LinearRateMps{speed_mps};
    intent.twist.twist_vel_radps.z = yaw_rate_radps;
    return intent;
}

} // namespace

int main() {
    GaitPolicyPlanner planner{};
    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.stable = true;

    const MotionIntent pivot_tripod = walkIntent(0.01, 1.2, GaitType::TRIPOD);
    const RuntimeGaitPolicy pivot_policy = planner.plan(est, pivot_tripod, safety);
    if (!expect(pivot_policy.region != DynamicGaitRegion::ARC, "high-yaw low-speed command should leave arc envelope") ||
        !expect(!pivot_policy.envelope.allow_tripod, "pivot envelope should explicitly disallow tripod") ||
        !expect(pivot_policy.gait_family != GaitType::TRIPOD, "pivot envelope should force non-tripod family")) {
        return EXIT_FAILURE;
    }

    RobotState tipped_est = est;
    tipped_est.body_twist_state.twist_pos_rad.x = 0.30;
    const RuntimeGaitPolicy degraded_policy = planner.plan(tipped_est, pivot_tripod, safety);
    if (!expect(degraded_policy.fallback_stage == GaitFallbackStage::DEGRADED_LOCOMOTION,
                "high roll should escalate to degraded locomotion fallback") ||
        !expect(degraded_policy.gait_family == GaitType::WAVE,
                "degraded locomotion should force wave gait") ||
        !expect(degraded_policy.per_leg[0].duty_cycle >= 0.84,
                "degraded locomotion should raise duty cycle envelope")) {
        return EXIT_FAILURE;
    }

    SafetyState faulted = safety;
    faulted.active_fault = FaultCode::TIP_OVER;
    const RuntimeGaitPolicy fault_hold_policy = planner.plan(est, pivot_tripod, faulted);
    if (!expect(fault_hold_policy.fallback_stage == GaitFallbackStage::FAULT_HOLD,
                "active safety fault should enter fault-hold stage") ||
        !expect(fault_hold_policy.suppression.suppress_stride_progression,
                "fault-hold should suppress stride progression") ||
        !expect(fault_hold_policy.cadence_hz.value == 0.0,
                "fault-hold should force zero cadence")) {
        return EXIT_FAILURE;
    }

    SafetyState inhibited = safety;
    inhibited.inhibit_motion = true;
    const RuntimeGaitPolicy safe_stop_policy = planner.plan(est, walkIntent(0.15, 0.0, GaitType::RIPPLE), inhibited);
    if (!expect(safe_stop_policy.fallback_stage == GaitFallbackStage::SAFE_STOP,
                "inhibited motion should trigger safe-stop stage") ||
        !expect(safe_stop_policy.suppression.suppress_turning,
                "safe-stop should suppress turning") ||
        !expect(safe_stop_policy.per_leg[0].step_length_m.value == 0.0,
                "safe-stop should zero planned step length")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
