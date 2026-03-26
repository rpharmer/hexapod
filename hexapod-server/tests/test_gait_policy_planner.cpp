#include "gait_policy_planner.hpp"

#include <iostream>
#include <string>

namespace {

int g_failures = 0;

bool expect(const bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "[FAIL] " << message << '\n';
        ++g_failures;
        return false;
    }
    return true;
}

MotionIntent walkingIntent(const double speed_norm, const double yaw_norm, const GaitType gait = GaitType::TRIPOD)
{
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.gait = gait;
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

RobotState nominalEstimate()
{
    RobotState est{};
    for (auto& leg : est.leg_states) {
        leg.joint_state[0].pos_rad = AngleRad{0.0};
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }
    return est;
}

control_config::GaitConfig enabledDynamicConfig()
{
    control_config::GaitConfig cfg{};
    cfg.acceptance_gate.feature_flag_enabled = true;
    cfg.acceptance_gate.simulator_first_required = true;
    cfg.acceptance_gate.simulator_validation_runs_required = 5;
    cfg.acceptance_gate.simulator_validation_runs_passed = 5;
    cfg.acceptance_gate.observed_control_latency_p95_ms = 5.0;
    cfg.acceptance_gate.max_control_latency_p95_ms = 8.0;
    cfg.acceptance_gate.observed_safety_faults_per_hour = 0.0;
    cfg.acceptance_gate.max_safety_faults_per_hour = 0.2;
    cfg.acceptance_gate.observed_min_stability_margin_m = 0.02;
    cfg.acceptance_gate.min_stability_margin_m = 0.015;
    return cfg;
}

void warmPlanner(GaitPolicyPlanner& planner, const MotionIntent& intent, const SafetyState& safety, const int steps)
{
    RobotState est = nominalEstimate();
    for (int i = 0; i < steps; ++i) {
        (void)planner.plan(est, intent, safety);
    }
}

void testArcTripodSelection()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    const SafetyState safety = safeState();
    warmPlanner(planner, walkingIntent(0.90, 0.10), safety, 16);

    RobotState est = nominalEstimate();
    const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.90, 0.10), safety);
    expect(policy.region == DynamicGaitRegion::ARC, "high speed + low yaw should classify ARC");
    expect(policy.turn_mode == TurnMode::CRAB, "ARC should map to CRAB turn mode");
}

void testPivotSelection()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    const SafetyState safety = safeState();
    warmPlanner(planner, walkingIntent(0.05, 0.95), safety, 16);

    RobotState est = nominalEstimate();
    const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.05, 0.95), safety);
    expect(policy.region == DynamicGaitRegion::PIVOT, "low speed + high yaw should classify PIVOT");
    expect(!policy.envelope.allow_tripod, "pivot/reorientation envelope should explicitly disallow tripod");
    expect(policy.gait_family != GaitType::TRIPOD, "pivot envelope should force non-tripod family");
    expect(policy.turn_mode == TurnMode::IN_PLACE, "PIVOT should map to IN_PLACE turn mode");
}

void testAntiChatterHysteresis()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    const SafetyState safety = safeState();
    warmPlanner(planner, walkingIntent(0.85, 0.10), safety, 18);

    RobotState est = nominalEstimate();
    const RuntimeGaitPolicy first = planner.plan(est, walkingIntent(0.85, 0.10), safety);
    const RuntimeGaitPolicy near_boundary = planner.plan(est, walkingIntent(0.68, 0.12), safety);
    RuntimeGaitPolicy deeper_boundary{};
    for (int i = 0; i < 8; ++i) {
        deeper_boundary = planner.plan(est, walkingIntent(0.58, 0.12), safety);
    }

    expect(first.gait_family == near_boundary.gait_family,
           "hysteresis should preserve gait family near the transition boundary");
    expect(deeper_boundary.gait_family != GaitType::TRIPOD || near_boundary.gait_family == GaitType::TRIPOD,
           "deeper boundary command should not upshift to TRIPOD unexpectedly");
}

void testFallbackStages()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    RobotState est = nominalEstimate();
    const MotionIntent pivot_tripod = walkingIntent(0.05, 0.95, GaitType::TRIPOD);

    const SafetyState safety = safeState();
    SafetyState faulted = safety;
    faulted.active_fault = FaultCode::TIP_OVER;
    const RuntimeGaitPolicy fault_hold_policy = planner.plan(est, pivot_tripod, faulted);
    expect(fault_hold_policy.fallback_stage == GaitFallbackStage::FAULT_HOLD,
           "active safety fault should enter fault-hold stage");
    expect(fault_hold_policy.suppression.suppress_stride_progression,
           "fault-hold should suppress stride progression");
    expect(fault_hold_policy.cadence_hz.value == 0.0, "fault-hold should force zero cadence");

    SafetyState inhibited = safety;
    inhibited.inhibit_motion = true;
    const RuntimeGaitPolicy safe_stop_policy = planner.plan(est, walkingIntent(0.15, 0.0, GaitType::RIPPLE), inhibited);
    expect(safe_stop_policy.fallback_stage == GaitFallbackStage::SAFE_STOP,
           "inhibited motion should trigger safe-stop stage");
    expect(safe_stop_policy.suppression.suppress_turning, "safe-stop should suppress turning");
    expect(safe_stop_policy.per_leg[0].step_length_m.value == 0.0,
           "safe-stop should zero planned step length");
}

void testAcceptanceGatesDisableByDefault()
{
    control_config::GaitConfig cfg = enabledDynamicConfig();
    cfg.acceptance_gate.feature_flag_enabled = false;
    GaitPolicyPlanner planner{cfg};
    const SafetyState safety = safeState();
    RobotState est = nominalEstimate();
    const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.95, 0.10), safety);

    expect(!policy.dynamic_enabled, "dynamic gait should remain disabled until rollout gates pass");
}

void testServoVelocityConstraintModifiesGait()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    SafetyState safety = safeState();
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    warmPlanner(planner, walkingIntent(0.90, 0.05), safety, 16);

    RobotState nominal = nominalEstimate();
    const RuntimeGaitPolicy unconstrained = planner.plan(nominal, walkingIntent(0.90, 0.05), safety);

    RobotState overspeed = nominalEstimate();
    for (auto& leg : overspeed.leg_states) {
        for (auto& joint : leg.joint_state) {
            joint.vel_radps = AngularRateRadPerSec{120.0};
        }
    }
    const RuntimeGaitPolicy constrained = planner.plan(overspeed, walkingIntent(0.90, 0.05), safety);

    expect(constrained.cadence_hz.value < unconstrained.cadence_hz.value,
           "servo velocity overspeed should reduce gait cadence instead of only clamping joints");
    expect(constrained.per_leg[0].step_length_m.value < unconstrained.per_leg[0].step_length_m.value,
           "servo velocity overspeed should shorten stride length");
    expect(constrained.per_leg[0].duty_cycle >= unconstrained.per_leg[0].duty_cycle,
           "servo velocity overspeed should increase duty cycle for stability");
}

} // namespace

int main()
{
    testArcTripodSelection();
    testPivotSelection();
    testAntiChatterHysteresis();
    testFallbackStages();
    testAcceptanceGatesDisableByDefault();
    testServoVelocityConstraintModifiesGait();

    if (g_failures != 0) {
        std::cerr << g_failures << " test(s) failed\n";
        return 1;
    }
    return 0;
}
