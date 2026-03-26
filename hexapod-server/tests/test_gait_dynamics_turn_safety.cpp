#include "foothold_planner.hpp"
#include "gait_policy_planner.hpp"
#include "gait_scheduler.hpp"

#include <algorithm>
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

RobotState nominalEstimate() {
    RobotState est{};
    est.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : est.leg_states) {
        leg.joint_state[0].pos_rad = AngleRad{0.0};
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }
    return est;
}

MotionIntent walkingIntent(double speed_mps, double yaw_radps) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.gait = GaitType::TRIPOD;
    intent.speed_mps = LinearRateMps{speed_mps};
    intent.twist.twist_vel_radps.z = yaw_radps;
    intent.twist.body_trans_mps.x = speed_mps;
    intent.timestamp_us = now_us();
    return intent;
}

bool testStrideDutyPhaseDynamics() {
    control_config::GaitConfig cfg{};
    cfg.phase_offsets.tripod = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
    cfg.duty.tripod = 0.60;

    GaitPolicyPlanner planner{cfg};
    GaitScheduler scheduler{cfg};

    const RobotState est = nominalEstimate();
    const MotionIntent intent = walkingIntent(0.12, 0.0);

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.stable = true;

    const auto params = planner.computePerLegDynamicParameters(GaitType::TRIPOD);
    const RuntimeGaitPolicy runtime = planner.plan(est, intent, safety);
    const GaitState stepped = scheduler.update(est, intent, safety);

    const double wrapped_diff = std::fabs(std::remainder(params[1].phase_offset - params[0].phase_offset, 1.0));
    const bool duty_matches_stance =
        stepped.in_stance[0] == (stepped.phase[0] < std::clamp(params[0].duty_cycle, 0.05, 0.95));

    return expect(std::abs(params[0].duty_cycle - 0.60) < 1e-9,
                  "configured tripod duty should propagate into per-leg policy") &&
           expect(std::abs(params[1].phase_offset - 0.5) < 1e-9,
                  "configured phase offsets should propagate into per-leg policy") &&
           expect(runtime.cadence_hz.value > 0.0,
                  "walking gait policy should command positive cadence") &&
           expect(wrapped_diff > 0.25,
                  "tripod phase offsets should keep neighboring legs out of phase") &&
           expect(duty_matches_stance,
                  "duty cycle threshold should determine stance vs swing");
}


bool testTurnModesAndInnerOuterLegBehavior() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    FootholdPlanner planner{geometry};

    GaitState gait{};
    gait.in_stance.fill(true);
    gait.phase.fill(0.0);
    gait.stride_phase_rate_hz = FrequencyHz{1.0};

    RuntimeGaitPolicy policy{};
    policy.turn_mode = TurnMode::IN_PLACE;
    policy.suppression.suppress_turning = false;
    for (auto& leg : policy.per_leg) {
        leg.step_length_m = LengthM{0.08};
        leg.duty_cycle = 0.5;
    }

    MotionIntent turning = walkingIntent(0.0, 0.7);
    const Vec3 nominal{0.18, 0.0, -0.15};

    double inner_sum = 0.0;
    double outer_sum = 0.0;
    int inner_count = 0;
    int outer_count = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const PlannedFoothold foothold = planner.plan(leg, nominal, turning, gait, policy, true);
        const double delta_mag = std::hypot(foothold.pos_body_m.x - nominal.x, foothold.pos_body_m.y - nominal.y);
        const bool is_left_leg = geometry.legGeometry[leg].bodyCoxaOffset.y >= 0.0;
        if (is_left_leg) {
            inner_sum += delta_mag;
            ++inner_count;
        } else {
            outer_sum += delta_mag;
            ++outer_count;
        }
    }

    const double inner_avg = inner_sum / std::max(inner_count, 1);
    const double outer_avg = outer_sum / std::max(outer_count, 1);

    policy.suppression.suppress_turning = true;
    double suppressed_inner = 0.0;
    double suppressed_outer = 0.0;
    int suppressed_inner_count = 0;
    int suppressed_outer_count = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const PlannedFoothold foothold = planner.plan(leg, nominal, turning, gait, policy, true);
        const double delta_mag = std::hypot(foothold.pos_body_m.x - nominal.x, foothold.pos_body_m.y - nominal.y);
        const bool is_left_leg = geometry.legGeometry[leg].bodyCoxaOffset.y >= 0.0;
        if (is_left_leg) {
            suppressed_inner += delta_mag;
            ++suppressed_inner_count;
        } else {
            suppressed_outer += delta_mag;
            ++suppressed_outer_count;
        }
    }

    const double suppressed_inner_avg = suppressed_inner / std::max(suppressed_inner_count, 1);
    const double suppressed_outer_avg = suppressed_outer / std::max(suppressed_outer_count, 1);
    const double unsuppressed_ratio = outer_avg / std::max(inner_avg, 1e-6);
    const double suppressed_ratio = suppressed_outer_avg / std::max(suppressed_inner_avg, 1e-6);

    return expect(policy.turn_mode == TurnMode::IN_PLACE,
                  "turning test should execute in IN_PLACE turn mode") &&
           expect(outer_avg > inner_avg,
                  "in-place turns should scale outer legs larger than inner legs") &&
           expect(std::abs(1.0 - suppressed_ratio) < std::abs(1.0 - unsuppressed_ratio),
                  "turn suppression should reduce inner/outer scaling bias");
}

bool testSuppressionPrioritySafetyBehavior() {
    control_config::GaitConfig cfg{};
    cfg.priority_suppression.stability_priority = 0.2;

    GaitPolicyPlanner planner{cfg};
    RobotState est = nominalEstimate();

    SafetyState safe{};
    safe.stable = true;

    const RuntimeGaitPolicy nominal = planner.plan(est, walkingIntent(0.10, 0.0), safe);
    if (!expect(!nominal.suppression.prioritize_stability,
                "priority flag should follow configured stability priority")) {
        return false;
    }

    SafetyState inhibited = safe;
    inhibited.inhibit_motion = true;
    const RuntimeGaitPolicy safe_stop = planner.plan(est, walkingIntent(0.10, 0.2), inhibited);
    if (!expect(safe_stop.fallback_stage == GaitFallbackStage::SAFE_STOP,
                "inhibit motion should force safe-stop fallback") ||
        !expect(safe_stop.suppression.suppress_stride_progression,
                "safe-stop should suppress stride progression") ||
        !expect(safe_stop.suppression.suppress_turning,
                "safe-stop should suppress turning")) {
        return false;
    }

    SafetyState faulted = safe;
    faulted.active_fault = FaultCode::TIP_OVER;
    const RuntimeGaitPolicy fault_hold = planner.plan(est, walkingIntent(0.10, 0.2), faulted);
    return expect(fault_hold.fallback_stage == GaitFallbackStage::FAULT_HOLD,
                  "active fault should force fault-hold fallback") &&
           expect(fault_hold.suppression.prioritize_stability,
                  "fault-hold should always prioritize stability");
}

} // namespace

int main() {
    if (!testStrideDutyPhaseDynamics()) {
        return EXIT_FAILURE;
    }
    if (!testTurnModesAndInnerOuterLegBehavior()) {
        return EXIT_FAILURE;
    }
    if (!testSuppressionPrioritySafetyBehavior()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
