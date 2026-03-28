#include "gait_policy_planner.hpp"
#include "gait_scheduler.hpp"
#include "motion_intent_utils.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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
    intent.body_pose_setpoint.angular_velocity_radps.z = yaw_norm * control_config::kDefaultTurnYawRateEnterRadps;
    return intent;
}

SafetyState safeState()
{
    SafetyState safety{};
    safety.stable = true;
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.active_fault = FaultCode::NONE;
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



double wrappedPositiveDelta(const double from, const double to)
{
    const double raw = std::fmod((to - from) + 1.0, 1.0);
    return raw < 0.0 ? raw + 1.0 : raw;
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
    expect(!policy.hard_clamp_cadence,
           "cadence hard-clamp telemetry should stay clear when limiter path is effectively disabled");
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
    expect(constrained.hard_clamp_cadence,
           "overspeed constraint should set cadence hard-clamp telemetry");
    expect(constrained.adaptation_scale_cadence < 1.0,
           "overspeed constraint should lower cadence adaptation scale");
}

void testMotionLimiterUsesJointMarginBeforeHardClamp()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    SafetyState safety = safeState();
    warmPlanner(planner, walkingIntent(0.95, 0.05), safety, 16);

    RobotState baseline = nominalEstimate();
    const RuntimeGaitPolicy nominal = planner.plan(baseline, walkingIntent(0.95, 0.05), safety);

    RobotState tight_margin = nominalEstimate();
    for (auto& leg : tight_margin.leg_states) {
        for (auto& joint : leg.joint_state) {
            joint.vel_radps = AngularRateRadPerSec{50.0};
        }
    }
    const RuntimeGaitPolicy adapted = planner.plan(tight_margin, walkingIntent(0.95, 0.05), safety);

    expect(adapted.cadence_hz.value < nominal.cadence_hz.value,
           "motion limiter should lower cadence when estimated joint velocity margin is small");
    expect(adapted.per_leg[0].step_length_m.value < nominal.per_leg[0].step_length_m.value,
           "motion limiter should shorten step length when velocity margin is small");
    expect(adapted.per_leg[0].duty_cycle > nominal.per_leg[0].duty_cycle,
           "motion limiter should increase duty cycle as margin shrinks");
}

void testRegionThresholdBoundaries()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    const SafetyState safety = safeState();
    RobotState est = nominalEstimate();

    warmPlanner(planner, walkingIntent(0.90, 0.10), safety, 20);
    RuntimeGaitPolicy at_arc_boundary{};
    for (int i = 0; i < 8; ++i) {
        at_arc_boundary = planner.plan(est, walkingIntent(0.16, 0.60), safety);
    }
    expect(at_arc_boundary.region == DynamicGaitRegion::ARC,
           "exact ARC exit thresholds should not force a region transition");

    RuntimeGaitPolicy beyond_arc_boundary = planner.plan(est, walkingIntent(0.05, 0.95), safety);
    for (int i = 0; i < 3; ++i) {
        beyond_arc_boundary = planner.plan(est, walkingIntent(0.05, 0.95), safety);
    }
    expect(beyond_arc_boundary.region != DynamicGaitRegion::ARC,
           "strong yaw command from ARC should leave ARC via hysteresis transition");
}

void testPivotHysteresisTransitionBoundary()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    const SafetyState safety = safeState();
    RobotState est = nominalEstimate();
    warmPlanner(planner, walkingIntent(0.05, 0.95), safety, 20);

    RuntimeGaitPolicy at_pivot_exit{};
    for (int i = 0; i < 8; ++i) {
        at_pivot_exit = planner.plan(est, walkingIntent(0.18, 0.52), safety);
    }
    expect(at_pivot_exit.region == DynamicGaitRegion::PIVOT,
           "exact PIVOT exit thresholds should hold pivot via hysteresis");

    RuntimeGaitPolicy beyond_pivot_exit{};
    for (int i = 0; i < 8; ++i) {
        beyond_pivot_exit = planner.plan(est, walkingIntent(0.20, 0.48), safety);
    }
    expect(beyond_pivot_exit.region == DynamicGaitRegion::REORIENTATION,
           "crossing PIVOT exit thresholds should move to reorientation");
}

void testYawRateThresholdNoiseSweepMaintainsHysteresisAndCoherence()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    const SafetyState safety = safeState();
    RobotState est = nominalEstimate();

    const std::array<double, 12> low_noise{
        -0.010, 0.006, -0.004, 0.009, -0.007, 0.003, -0.005, 0.008, -0.002, 0.005, -0.006, 0.004};
    const std::array<double, 16> high_noise{
        0.010, -0.006, 0.005, -0.007, 0.003, -0.004, 0.009, -0.005,
        0.006, -0.003, 0.004, -0.008, 0.007, -0.002, 0.005, -0.006};

    // ARC hold: hover around arc exit yaw (0.60) without crossing.
    warmPlanner(planner, walkingIntent(0.90, 0.10), safety, 24);
    std::vector<DynamicGaitRegion> arc_regions;
    for (const double noise : low_noise) {
        const double yaw_norm = 0.60 + noise;
        const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.20, yaw_norm), safety);
        arc_regions.push_back(policy.region);
        expect(policy.region == DynamicGaitRegion::ARC,
               "yaw sweep just below ARC exit threshold should remain ARC");
        expect(policy.turn_mode == TurnMode::CRAB, "ARC region should keep CRAB turn mode under noisy yaw input");
        expect(policy.envelope.allow_tripod, "ARC region should keep tripod gait allowance coherent");
        expect(policy.suppression.suppress_turning,
               "CRAB with low absolute yaw-rate should suppress turning near arc exit boundary");
    }

    // Cross ARC exit threshold with small noise and verify one-way transition into REORIENTATION.
    std::vector<DynamicGaitRegion> arc_exit_regions;
    for (const double noise : high_noise) {
        const double yaw_norm = 0.62 + noise;
        const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.30, yaw_norm), safety);
        arc_exit_regions.push_back(policy.region);
    }
    int arc_transitions = 0;
    bool arc_seen_reorientation = false;
    for (std::size_t i = 0; i < arc_exit_regions.size(); ++i) {
        if (arc_exit_regions[i] == DynamicGaitRegion::REORIENTATION) {
            arc_seen_reorientation = true;
        }
        if (i > 0 && arc_exit_regions[i] != arc_exit_regions[i - 1]) {
            ++arc_transitions;
        }
    }
    expect(arc_seen_reorientation, "yaw sweep above ARC exit threshold should leave ARC within hysteresis window");
    expect(arc_transitions <= 1, "ARC threshold sweep should not oscillate region every cycle");
    for (std::size_t i = arc_exit_regions.size() - 5; i < arc_exit_regions.size(); ++i) {
        expect(arc_exit_regions[i] == DynamicGaitRegion::REORIENTATION,
               "post-transition ARC sweep should settle in REORIENTATION");
    }

    // PIVOT hold: hover around pivot exit yaw (0.52) from above without crossing.
    warmPlanner(planner, walkingIntent(0.05, 0.95), safety, 24);
    std::vector<DynamicGaitRegion> pivot_regions;
    for (const double noise : low_noise) {
        const double yaw_norm = 0.52 + std::abs(noise);
        const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.20, yaw_norm), safety);
        pivot_regions.push_back(policy.region);
        expect(policy.region == DynamicGaitRegion::PIVOT,
               "yaw sweep just above PIVOT exit threshold should remain PIVOT");
        expect(policy.turn_mode == TurnMode::IN_PLACE, "PIVOT region should keep IN_PLACE turn mode under noise");
        expect(!policy.envelope.allow_tripod, "PIVOT region should keep tripod suppression coherent");
        expect(policy.gait_family != GaitType::TRIPOD,
               "PIVOT envelope should continue forcing non-tripod gait family");
    }

    // Cross pivot exit threshold with noise and verify no boundary ping-pong.
    std::vector<DynamicGaitRegion> pivot_exit_regions;
    for (const double noise : high_noise) {
        const double yaw_norm = 0.50 + noise;
        const RuntimeGaitPolicy policy = planner.plan(est, walkingIntent(0.20, yaw_norm), safety);
        pivot_exit_regions.push_back(policy.region);
    }

    int transitions = 0;
    for (std::size_t i = 1; i < pivot_exit_regions.size(); ++i) {
        if (pivot_exit_regions[i] != pivot_exit_regions[i - 1]) {
            ++transitions;
        }
    }
    bool pivot_seen_reorientation = false;
    for (const DynamicGaitRegion region : pivot_exit_regions) {
        if (region == DynamicGaitRegion::REORIENTATION) {
            pivot_seen_reorientation = true;
        }
    }
    expect(pivot_seen_reorientation, "yaw sweep below PIVOT exit threshold should leave PIVOT");
    expect(transitions <= 1, "region should not oscillate every cycle around PIVOT exit boundary");
    for (std::size_t i = pivot_exit_regions.size() - 5; i < pivot_exit_regions.size(); ++i) {
        expect(pivot_exit_regions[i] == DynamicGaitRegion::REORIENTATION,
               "after crossing PIVOT exit threshold, region should settle in REORIENTATION");
    }

    const RuntimeGaitPolicy settled = planner.plan(est, walkingIntent(0.20, 0.50), safety);
    expect(settled.region == DynamicGaitRegion::REORIENTATION,
           "post-sweep command should preserve REORIENTATION after hysteresis transition");
    expect(settled.turn_mode == TurnMode::CRAB, "REORIENTATION should map coherently to CRAB turn mode");
    expect(!settled.envelope.allow_tripod, "REORIENTATION envelope should keep tripod disallowed");
    expect(settled.gait_family != GaitType::TRIPOD,
           "tripod suppression should remain coherent after pivot-exit transition");
    expect(settled.suppression.suppress_turning,
           "CRAB mode with low yaw should coherently suppress turning in settled state");
}

void testFallbackEscalationPrecedence()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    RobotState est = nominalEstimate();
    const MotionIntent pivot_intent = walkingIntent(0.05, 0.95, GaitType::RIPPLE);
    const SafetyState baseline = safeState();
    warmPlanner(planner, pivot_intent, baseline, 20);

    SafetyState degraded_safety = baseline;
    degraded_safety.stable = false;
    const RuntimeGaitPolicy degraded = planner.plan(est, pivot_intent, degraded_safety);
    expect(degraded.fallback_stage == GaitFallbackStage::DEGRADED_LOCOMOTION,
           "instability without inhibit/fault should select degraded locomotion");

    SafetyState safe_stop_safety = degraded_safety;
    safe_stop_safety.inhibit_motion = true;
    const RuntimeGaitPolicy safe_stop = planner.plan(est, pivot_intent, safe_stop_safety);
    expect(safe_stop.fallback_stage == GaitFallbackStage::SAFE_STOP,
           "inhibit motion should take precedence over degraded locomotion");

    SafetyState fault_safety = safe_stop_safety;
    fault_safety.active_fault = FaultCode::TIP_OVER;
    const RuntimeGaitPolicy fault_hold = planner.plan(est, pivot_intent, fault_safety);
    expect(fault_hold.fallback_stage == GaitFallbackStage::FAULT_HOLD,
           "active fault should take highest fallback precedence over all lower stages");
}



void testRapidModeToggleMaintainsPhaseContinuityAndDutyExpectations()
{
    GaitScheduler scheduler{enabledDynamicConfig()};

    RobotState est = nominalEstimate();
    est.foot_contacts = {true, true, true, true, true, true};

    SafetyState safety = safeState();

    MotionIntent walk_intent = walkingIntent(0.70, 0.05, GaitType::TRIPOD);
    MotionIntent idle_intent = walk_intent;
    idle_intent.requested_mode = RobotMode::SAFE_IDLE;

    RuntimeGaitPolicy policy{};
    policy.dynamic_enabled = true;
    policy.gait_family = GaitType::TRIPOD;
    policy.cadence_hz = FrequencyHz{4.0};
    policy.suppression.suppress_stride_progression = false;
    policy.per_leg[0].phase_offset = 0.0;
    policy.per_leg[1].phase_offset = 0.5;
    policy.per_leg[2].phase_offset = 0.0;
    policy.per_leg[3].phase_offset = 0.5;
    policy.per_leg[4].phase_offset = 0.0;
    policy.per_leg[5].phase_offset = 0.5;
    for (auto& leg : policy.per_leg) {
        leg.duty_cycle = 0.64;
    }

    GaitState previous{};
    bool have_previous = false;
    double previous_walk_rate_hz = 0.0;

    constexpr int kSamples = 40;
    for (int sample = 0; sample < kSamples; ++sample) {
        const bool walking = (sample % 2) == 0;
        const MotionIntent& intent = walking ? walk_intent : idle_intent;

        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        const GaitState current = scheduler.update(est, intent, safety, policy);

        for (int leg = 0; leg < kNumLegs; ++leg) {
            expect(current.phase[leg] >= 0.0 && current.phase[leg] < 1.0,
                   "phase should remain in [0,1) under rapid mode toggles");

            const bool expected_stance =
                walking ? (current.phase[leg] < std::clamp(policy.per_leg[leg].duty_cycle, 0.05, 0.95)) : true;
            expect(current.in_stance[leg] == expected_stance,
                   "stance mask should match duty-cycle expectation for current phase/mode");
        }

        if (walking) {
            expect(current.stride_phase_rate_hz.value >= 0.0,
                   "walk samples should keep non-negative stride phase rate");
            expect(current.stride_phase_rate_hz.value <= policy.cadence_hz.value + 1e-9,
                   "walk samples should clamp stride phase rate to commanded cadence");
            if (have_previous) {
                expect(current.stride_phase_rate_hz.value <= previous_walk_rate_hz + 1.2,
                       "walk samples should slew cadence upward instead of instant jumps");
            }
            previous_walk_rate_hz = current.stride_phase_rate_hz.value;
        } else {
            expect(current.stride_phase_rate_hz.value == 0.0,
                   "non-walk samples should force zero stride phase rate");
            previous_walk_rate_hz = 0.0;
        }

        if (have_previous) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const double delta = wrappedPositiveDelta(previous.phase[leg], current.phase[leg]);
                expect(delta < 0.08,
                       "sequential gait samples should not exhibit large phase discontinuities");
            }
        }

        previous = current;
        have_previous = true;
    }
}
void testScenarioDerivedYawRateFeedsRegionTurnModeAndSuppression()
{
    GaitPolicyPlanner planner{enabledDynamicConfig()};
    const SafetyState safety = safeState();
    RobotState est = nominalEstimate();

    ScenarioMotionIntent scenario_motion{};
    scenario_motion.enabled = true;
    scenario_motion.mode = RobotMode::WALK;
    scenario_motion.gait = GaitType::TRIPOD;
    scenario_motion.body_height_m = 0.20;
    scenario_motion.speed_mps = 0.03;
    scenario_motion.heading_rad = 0.0;
    scenario_motion.yaw_rad = 0.52;
    const MotionIntent near_exit_intent = makeMotionIntent(scenario_motion);

    warmPlanner(planner, near_exit_intent, safety, 20);
    const DynamicGaitRegion near_exit_region = planner.selectRegion(near_exit_intent);
    expect(near_exit_region == DynamicGaitRegion::PIVOT,
           "scenario yaw-rate should drive selectRegion classification into PIVOT");
    expect(planner.selectTurnMode(near_exit_region) == TurnMode::IN_PLACE,
           "scenario-derived PIVOT region should map to IN_PLACE turn mode");

    const GaitSuppressionFlags pivot_flags =
        planner.computeSuppressionFlags(est, near_exit_intent, safety, planner.selectTurnMode(near_exit_region));
    expect(!pivot_flags.suppress_turning,
           "IN_PLACE turn mode from scenario-derived yaw-rate should not suppress turning");

    scenario_motion.speed_mps = 0.45;
    scenario_motion.yaw_rad = 0.0;
    const MotionIntent straight_intent = makeMotionIntent(scenario_motion);
    warmPlanner(planner, straight_intent, safety, 20);
    const DynamicGaitRegion straight_region = planner.selectRegion(straight_intent);
    expect(straight_region == DynamicGaitRegion::ARC,
           "scenario zero yaw-rate with forward speed should classify ARC");
    expect(planner.selectTurnMode(straight_region) == TurnMode::CRAB,
           "scenario-derived ARC region should map to CRAB turn mode");

    const GaitSuppressionFlags crab_flags =
        planner.computeSuppressionFlags(est, straight_intent, safety, planner.selectTurnMode(straight_region));
    expect(crab_flags.suppress_turning,
           "CRAB mode with near-zero scenario yaw-rate should suppress turning");
}

} // namespace

int main()
{
    testArcTripodSelection();
    testPivotSelection();
    testAntiChatterHysteresis();
    testRegionThresholdBoundaries();
    testPivotHysteresisTransitionBoundary();
    testYawRateThresholdNoiseSweepMaintainsHysteresisAndCoherence();
    testFallbackStages();
    testFallbackEscalationPrecedence();
    testRapidModeToggleMaintainsPhaseContinuityAndDutyExpectations();
    testScenarioDerivedYawRateFeedsRegionTurnModeAndSuppression();
    testAcceptanceGatesDisableByDefault();
    testServoVelocityConstraintModifiesGait();
    testMotionLimiterUsesJointMarginBeforeHardClamp();

    if (g_failures != 0) {
        std::cerr << g_failures << " test(s) failed\n";
        return 1;
    }
    return 0;
}
