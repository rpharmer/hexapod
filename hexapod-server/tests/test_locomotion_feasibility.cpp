#include "geometry_config.hpp"
#include "locomotion_feasibility.hpp"
#include "motion_intent_utils.hpp"

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

RobotState nominalState() {
    RobotState est{};
    est.valid = true;
    est.timestamp_us = TimePointUs{1'000'000};
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.z = 0.13;
    est.body_twist_state.twist_pos_rad.x = 0.04;
    est.body_twist_state.twist_pos_rad.y = 0.03;
    est.has_imu = true;
    est.imu.valid = true;
    est.imu.gyro_radps.x = 0.12;
    est.imu.gyro_radps.y = 0.10;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        est.foot_contacts[leg] = true;
        est.foot_contact_fusion[leg].phase = ContactPhase::ConfirmedStance;
        est.foot_contact_fusion[leg].confidence = 0.95;
    }
    return est;
}

GaitState nominalGait() {
    GaitState gait{};
    gait.duty_factor = 0.60;
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    gait.in_stance.fill(true);
    gait.support_liftoff_safe_to_lift.fill(true);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        gait.phase[leg] = 0.20;
    }
    return gait;
}

MotionIntent walkIntent() {
    MotionIntent intent = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.14);
    intent.cmd_vx_mps = LinearRateMps{0.24};
    intent.cmd_yaw_radps = AngularRateRadPerSec{0.0};
    intent.speed_mps = LinearRateMps{0.24};
    intent.timestamp_us = TimePointUs{1'000'000};
    intent.sample_id = 1;
    return intent;
}

} // namespace

int main() {
    RobotState est = nominalState();
    GaitState gait = nominalGait();
    const GaitState previous = gait;
    SafetyState safety{};
    safety.inhibit_motion = false;
    const HexapodGeometry geometry = defaultHexapodGeometry();

    control_config::LocomotionRedesignConfig cfg{};
    cfg.control_margin_source = control_config::ControlMarginSource::Conservative;
    const LocomotionFeasibility f =
        computeLocomotionFeasibility(est, walkIntent(), gait, previous, safety, geometry, cfg);
    if (!expect(f.valid, "feasibility should be valid") ||
        !expect(f.enabled, "default redesign telemetry flag should mark feasibility enabled") ||
        !expect(f.control_margin_m == std::min(f.nominal_margin_m, f.actual_margin_m),
                "conservative control margin should use the smaller margin") ||
        !expect(f.support.support_count == kNumLegs, "all contacts in stance should count as support")) {
        return EXIT_FAILURE;
    }

    gait = nominalGait();
    gait.in_stance[0] = false;
    gait.phase[0] = 0.68; // early swing for duty 0.60 -> tau 0.20
    est.foot_contacts[0] = true;
    const auto contact_grace = computeLegContactDecisions(est, gait, safety);
    if (!expect(contact_grace[0].mode == LegContactMode::ContactGrace,
                "early swing raw contact should be contact grace") ||
        !expect(!contact_grace[0].use_stance_kinematics,
                "contact grace should allow swing kinematics")) {
        return EXIT_FAILURE;
    }

    gait.stability_hold_stance[1] = true;
    gait.in_stance[1] = false;
    gait.phase[1] = 0.80;
    est.foot_contacts[1] = true;
    const auto held = computeLegContactDecisions(est, gait, safety);
    if (!expect(held[1].mode == LegContactMode::HeldStance,
                "stability-held contact should be held stance") ||
        !expect(held[1].use_stance_kinematics,
                "held stance should use stance kinematics")) {
        return EXIT_FAILURE;
    }

    CommandGovernorState governor{};
    governor.requested_body_height_m = 0.14;
    governor.body_height_delta_m = -0.004;
    const HeightPolicySnapshot height =
        evaluateHeightPolicySnapshot(est, walkIntent(), nominalGait(), governor, cfg);
    if (!expect(height.valid, "height policy snapshot should be valid") ||
        !expect(height.compliance_sag_m > 0.0, "height policy should expose compliance sag") ||
        !expect(height.governor_delta_m < 0.0, "height policy should expose governor squat")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
