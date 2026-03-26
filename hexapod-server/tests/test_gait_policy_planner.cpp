#include "gait_policy_planner.hpp"

#include <iostream>
#include <string>

namespace {

int g_failures = 0;

bool expect(const bool cond, const std::string& message)
{
    if (!cond) {
        std::cerr << "[FAIL] " << message << '\n';
        ++g_failures;
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

MotionIntent walkingIntent(const double speed_norm, const double yaw_norm)
{
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.speed_mps = LinearRateMps{speed_norm * control_config::kDefaultGaitNominalMaxSpeedMps};
    intent.twist.twist_vel_radps.z = yaw_norm * control_config::kDefaultTurnYawRateEnterRadps;
    return intent;
}

SafetyState safeState()
{
    SafetyState safety{};
    safety.stable = true;
    return safety;
}

bool warmPlanner(GaitPolicyPlanner& planner, const MotionIntent& intent, const SafetyState& safety, const int steps)
{
    RobotState est{};
    for (int i = 0; i < steps; ++i) {
        (void)planner.plan(est, intent, safety);
    }
    return true;
}

bool testArcTripodSelection()
{
    GaitPolicyPlanner planner{};
    const SafetyState safety = safeState();
    warmPlanner(planner, walkingIntent(0.90, 0.10), safety, 16);

    RobotState est{};
    const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.90, 0.10), safety);
    return expect(policy.region == LocomotionRegion::ARC, "high speed + low yaw should classify ARC") &&
           expect(policy.gait_family == GaitType::TRIPOD, "high speed + low yaw in ARC should pick TRIPOD") &&
           expect(policy.turn_mode == TurnMode::CRAB, "ARC should map to CRAB turn mode");
}

bool testPivotSelection()
{
    GaitPolicyPlanner planner{};
    const SafetyState safety = safeState();
    warmPlanner(planner, walkingIntent(0.05, 0.95), safety, 16);

    RobotState est{};
    const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.05, 0.95), safety);
    return expect(policy.region == LocomotionRegion::PIVOT, "low speed + high yaw should classify PIVOT") &&
           expect(policy.gait_family != GaitType::TRIPOD, "PIVOT should never use TRIPOD") &&
           expect(policy.turn_mode == TurnMode::IN_PLACE, "PIVOT should map to IN_PLACE turn mode");
}

bool testAntiChatterHysteresis()
{
    GaitPolicyPlanner planner{};
    const SafetyState safety = safeState();
    warmPlanner(planner, walkingIntent(0.85, 0.10), safety, 18);

    RobotState est{};
    RuntimeGaitPolicy first = planner.plan(est, walkingIntent(0.85, 0.10), safety);
    RuntimeGaitPolicy near_boundary = planner.plan(est, walkingIntent(0.68, 0.12), safety);
    RuntimeGaitPolicy deeper_boundary{};
    for (int i = 0; i < 8; ++i) {
        deeper_boundary = planner.plan(est, walkingIntent(0.58, 0.12), safety);
    }

    return expect(first.gait_family == GaitType::TRIPOD, "initial command should settle to TRIPOD") &&
           expect(near_boundary.gait_family == GaitType::TRIPOD,
                  "hysteresis should keep TRIPOD near 0.70 transition") &&
           expect(deeper_boundary.gait_family == GaitType::RIPPLE,
                  "gait should downshift once command moves further below transition");
}

} // namespace

int main()
{
    testArcTripodSelection();
    testPivotSelection();
    testAntiChatterHysteresis();

    if (g_failures != 0) {
        std::cerr << g_failures << " test(s) failed\n";
        return 1;
    }
    return 0;
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
