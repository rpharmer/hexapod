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
}
