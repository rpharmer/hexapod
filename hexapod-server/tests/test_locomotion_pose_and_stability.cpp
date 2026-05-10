#include "body_pose_controller.hpp"
#include "geometry_config.hpp"
#include "locomotion_stability.hpp"
#include "support_assessment.hpp"

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

bool nearlyEqual(double lhs, double rhs, double eps = 1e-6) {
    return std::abs(lhs - rhs) <= eps;
}

MotionIntent walkIntent(double body_height_m, double body_x = 0.0, double body_y = 0.0) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.twist.body_trans_m = PositionM3{body_x, body_y, body_height_m};
    intent.twist.twist_pos_rad = EulerAnglesRad3{0.1, 0.2, 0.3};
    return intent;
}

GaitState allStanceWalkGait(double phase_each, FrequencyHz stride_hz) {
    GaitState gait{};
    gait.duty_factor = 0.5;
    gait.stride_phase_rate_hz = stride_hz;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        gait.phase[static_cast<std::size_t>(leg)] = phase_each;
        gait.in_stance[leg] = true;
    }
    return gait;
}

bool testBodyPoseZeroMarginNoLean() {
    MotionIntent intent = walkIntent(0.12);
    PlanarMotionCommand cmd{0.2, 0.0, 0.5};
    const BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, 0.0, 1.0);
    return expect(nearlyEqual(pose.roll_rad, 0.1), "roll should match intent when margin is zero") &&
           expect(nearlyEqual(pose.pitch_rad, 0.2), "pitch should match intent when margin is zero") &&
           expect(nearlyEqual(pose.yaw_rad, 0.3), "yaw should match intent when margin is zero");
}

bool testBodyPoseFullMarginLeanForward() {
    MotionIntent intent = walkIntent(0.12);
    PlanarMotionCommand cmd{0.2, 0.0, 0.0};
    constexpr double kSoftMargin = 0.022;
    constexpr double kHardMargin = 0.004;
    constexpr double kLeanPitchPerVx = 0.22;
    constexpr double kLeanVxRefMps = 0.20;
    const BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, kSoftMargin, 1.0);
    const double margin_scale =
        std::clamp((kSoftMargin - kHardMargin) / std::max(kSoftMargin - kHardMargin, 1e-6), 0.0, 1.0);
    const double expected_pitch =
        0.2 - kLeanPitchPerVx * std::clamp(cmd.vx_mps / kLeanVxRefMps, -1.2, 1.2) * margin_scale;
    return expect(nearlyEqual(pose.pitch_rad, expected_pitch, 1e-9),
                  "full margin should apply full forward lean into +vx");
}

bool testBodyPoseDefaultHeight() {
    MotionIntent intent = walkIntent(0.0);
    PlanarMotionCommand cmd{};
    const BodyPoseSetpoint pose = computeBodyPoseSetpoint(intent, cmd, 0.022, 1.0);
    return expect(nearlyEqual(pose.body_height_m, 0.14), "zero height setpoint should fall back to default");
}

bool testBodyPoseYawLeanRoll() {
    MotionIntent intent = walkIntent(0.12);
    PlanarMotionCommand cmd{0.0, 0.0, 0.45};
    constexpr double kSoftMargin = 0.022;
    constexpr double kHardMargin = 0.004;
    constexpr double kLeanRollPerYaw = 0.18;
    constexpr double kLeanYawRefRadps = 0.45;
    const BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, kSoftMargin, 1.0);
    const double margin_scale =
        std::clamp((kSoftMargin - kHardMargin) / std::max(kSoftMargin - kHardMargin, 1e-6), 0.0, 1.0);
    const double expected_roll =
        0.1 + kLeanRollPerYaw * std::clamp(cmd.yaw_rate_radps / kLeanYawRefRadps, -1.2, 1.2) * margin_scale;
    return expect(nearlyEqual(pose.roll_rad, expected_roll, 1e-9),
                  "full margin should apply roll lean into yaw rate command");
}

bool testBodyPoseSlowStrideBoostsLean() {
    MotionIntent intent = walkIntent(0.12);
    PlanarMotionCommand cmd{0.2, 0.0, 0.0};
    const double margin = 0.013;
    const BodyPoseSetpoint fast = computeBodyPoseSetpoint(intent, cmd, margin, 1.0);
    const BodyPoseSetpoint slow = computeBodyPoseSetpoint(intent, cmd, margin, 0.5);
    const double fast_lean_mag = std::abs(fast.pitch_rad - 0.2);
    const double slow_lean_mag = std::abs(slow.pitch_rad - 0.2);
    return expect(slow_lean_mag > fast_lean_mag * 1.01,
                  "sub-threshold stride rate should increase commanded lean magnitude");
}

bool testStabilityStandClears() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12);
    intent.requested_mode = RobotMode::STAND;
    GaitState gait = allStanceWalkGait(0.48, FrequencyHz{1.0});
    gait.static_stability_margin_m = 0.05;
    gait.stability_hold_stance.fill(true);

    stability.apply(RobotState{}, intent, gait);

    if (!expect(gait.static_stability_margin_m == 0.0, "STAND should zero reported static margin")) {
        return false;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(!gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                     "STAND should clear stability hold flags")) {
            return false;
        }
    }
    return true;
}

bool testStabilityWalkCenteredPositiveMargin() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    GaitState gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    stability.apply(RobotState{}, intent, gait);
    return expect(gait.static_stability_margin_m > 0.002,
                  "centered COM with all feet in stance should yield positive static margin");
}

bool testStabilitySlowGaitIsNotMoreConservativeByDefault() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);

    GaitState fast_gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    GaitState slow_gait = allStanceWalkGait(0.25, FrequencyHz{0.5});
    stability.apply(RobotState{}, intent, fast_gait);
    stability.apply(RobotState{}, intent, slow_gait);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double fast_clearance = fast_gait.support_liftoff_clearance_m[static_cast<std::size_t>(leg)];
        const double slow_clearance = slow_gait.support_liftoff_clearance_m[static_cast<std::size_t>(leg)];
        if (!expect(slow_clearance > fast_clearance,
                    "slow gait should be at least as permissive as nominal gait by default")) {
            return false;
        }
    }
    return true;
}

bool testStabilityComOutsideHullNegativeMargin() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 5.0, 0.0);
    GaitState gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    stability.apply(RobotState{}, intent, gait);
    return expect(gait.static_stability_margin_m < 0.0,
                  "large body XY offset should push projected COM outside nominal support");
}

bool testStabilityHighTiltForcesHold() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    RobotState est{};
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.twist_pos_rad = EulerAnglesRad3{0.4, 0.3, 0.0};

    GaitState gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    stability.apply(est, intent, gait);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                     "large measured tilt should force all legs to hold stance")) {
            return false;
        }
    }
    return true;
}

bool testStabilitySupportDiagnosticsExposeAsymmetry() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 0.05, 0.02);
    GaitState gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    stability.apply(RobotState{}, intent, gait);

    const auto [min_it, max_it] = std::minmax_element(gait.support_liftoff_clearance_m.begin(),
                                                      gait.support_liftoff_clearance_m.end());
    if (!expect(std::isfinite(*min_it) && std::isfinite(*max_it),
                "support diagnostics should remain finite")) {
        return false;
    }
    return expect((*max_it - *min_it) > 0.005,
                  "per-leg support diagnostics should show asymmetry when the body shifts");
}

bool testStabilityUsesConfirmedSupportWhenPhaseLags() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    GaitState empty_support_gait{};
    empty_support_gait.duty_factor = 0.5;
    empty_support_gait.stride_phase_rate_hz = FrequencyHz{1.0};
    empty_support_gait.phase = {0.75, 0.25, 0.75, 0.25, 0.75, 0.25};
    empty_support_gait.in_stance = {false, false, false, false, false, false};

    GaitState gait = empty_support_gait;
    RobotState est{};
    est.valid = true;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        est.foot_contacts[static_cast<std::size_t>(leg)] = (leg % 2) == 0;
        est.foot_contact_fusion[static_cast<std::size_t>(leg)].phase =
            est.foot_contacts[static_cast<std::size_t>(leg)] ? ContactPhase::ConfirmedStance : ContactPhase::Search;
    }

    stability.apply(RobotState{}, intent, empty_support_gait);
    stability.apply(est, intent, gait);

    if (!expect(gait.static_stability_margin_m > empty_support_gait.static_stability_margin_m + 0.05,
                "confirmed support should materially improve the support margin when phase lags")) {
        return false;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if ((leg % 2) != 0) {
            continue;
        }
        if (!expect(gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                    "supported swing legs should be held until support transfers cleanly")) {
            return false;
        }
    }
    return true;
}

bool testSupportAssessmentUsesConfirmedSupportWhenPhaseLags() {
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    GaitState gait{};
    gait.duty_factor = 0.5;
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    gait.phase = {0.75, 0.25, 0.75, 0.25, 0.75, 0.25};
    gait.in_stance = {false, false, false, false, false, false};

    RobotState est{};
    est.valid = true;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        est.foot_contacts[static_cast<std::size_t>(leg)] = (leg % 2) == 0;
        est.foot_contact_fusion[static_cast<std::size_t>(leg)].phase =
            est.foot_contacts[static_cast<std::size_t>(leg)] ? ContactPhase::ConfirmedStance : ContactPhase::Search;
    }

    const HexapodGeometry geometry = geometry_config::activeHexapodGeometry();
    const SupportAssessment no_observation = assessSupportState(RobotState{}, intent, gait, geometry);
    const SupportAssessment confirmed_support = assessSupportState(est, intent, gait, geometry);
    return expect(confirmed_support.support_count == 3,
                  "support assessment should count confirmed stance contacts") &&
           expect(confirmed_support.confirmed_support_count == 3,
                  "support assessment should mark confirmed stance support as confirmed") &&
           expect(confirmed_support.uncertain_support_count == 0,
                  "support assessment should not add uncertainty when all support is confirmed") &&
           expect(confirmed_support.search_leg_count == 3,
                  "support assessment should count the lagging search-phase legs") &&
           expect(confirmed_support.all_support_confirmed,
                  "support assessment should recognize when all effective support is confirmed") &&
           expect(confirmed_support.static_margin_m > no_observation.static_margin_m + 0.05,
                  "support assessment should improve support margin when confirmed support is present") &&
           expect(confirmed_support.degraded_support,
                  "alternating confirmed support with no planned stance should still be treated as degraded");
}

bool testSupportAssessmentTracksUncertainSupportPhases() {
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    GaitState gait = allStanceWalkGait(0.25, FrequencyHz{1.0});

    RobotState est{};
    est.valid = true;
    est.foot_contacts = {true, true, true, true, false, false};
    est.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
    est.foot_contact_fusion[1].phase = ContactPhase::ExpectedTouchdown;
    est.foot_contact_fusion[2].phase = ContactPhase::LostCandidate;
    est.foot_contact_fusion[3].phase = ContactPhase::Search;
    est.foot_contact_fusion[4].phase = ContactPhase::Swing;
    est.foot_contact_fusion[5].phase = ContactPhase::Swing;

    const HexapodGeometry geometry = geometry_config::activeHexapodGeometry();
    const SupportAssessment support = assessSupportState(est, intent, gait, geometry);
    return expect(support.support_count == 4,
                  "support assessment should preserve effective support from contact-backed stance legs") &&
           expect(support.confirmed_support_count == 1,
                  "only confirmed-stance support should count as confirmed under uncertain phases") &&
           expect(support.uncertain_support_count == 3,
                  "support assessment should report uncertainty for search, touchdown, and lost-candidate support") &&
           expect(support.search_leg_count == 1,
                  "support assessment should report search-phase legs") &&
           expect(support.lost_candidate_count == 1,
                  "support assessment should report lost-candidate legs") &&
           expect(support.expected_touchdown_count == 1,
                  "support assessment should report expected-touchdown legs") &&
           expect(!support.all_support_confirmed,
                  "support assessment should refuse all-support-confirmed when any support leg is uncertain");
}

bool testStabilityFastBodyRateThrottlesStride() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    intent.cmd_vx_mps = LinearRateMps{0.30};
    intent.speed_mps = LinearRateMps{0.30};

    RobotState calm{};
    calm.valid = true;
    calm.has_imu = true;
    calm.imu.valid = true;
    calm.imu.gyro_radps = AngularVelocityRadPerSec3{0.1, 0.1, 0.0};

    RobotState fast = calm;
    fast.imu.gyro_radps = AngularVelocityRadPerSec3{0.95, 0.95, 0.0};

    GaitState calm_gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    GaitState fast_gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    stability.apply(calm, intent, calm_gait);
    stability.apply(fast, intent, fast_gait);

    if (!expect(fast_gait.step_length_m < calm_gait.step_length_m,
                "high body rate should reduce commanded stride length")) {
        return false;
    }
    return expect(fast_gait.stride_phase_rate_hz.value <= calm_gait.stride_phase_rate_hz.value,
                  "high body rate should not increase stride cadence");
}

bool testStabilityRaisesSwingHeightOnBodySag() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.14, 0.0, 0.0);
    intent.cmd_yaw_radps = AngularRateRadPerSec{0.45};
    intent.speed_mps = LinearRateMps{0.0};

    RobotState sagging{};
    sagging.valid = true;
    sagging.has_body_twist_state = true;
    sagging.body_twist_state.body_trans_m.z = 0.096;

    GaitState gait = allStanceWalkGait(0.25, FrequencyHz{0.9});
    gait.swing_height_m = 0.03;

    stability.apply(sagging, intent, gait);

    if (!expect(gait.swing_height_m > 0.05,
                "sagging turn should raise swing height above the nominal floor")) {
        return false;
    }
    return expect(gait.swing_height_m <= 0.06 + 1e-9,
                  "governed swing height should stay within the safe ceiling");
}

bool testStabilitySlowCommandDoesNotThrottleStride() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    intent.cmd_vx_mps = LinearRateMps{0.06};
    intent.speed_mps = LinearRateMps{0.06};

    RobotState calm{};
    calm.valid = true;
    calm.has_imu = true;
    calm.imu.valid = true;
    calm.imu.gyro_radps = AngularVelocityRadPerSec3{0.1, 0.1, 0.0};

    RobotState fast = calm;
    fast.imu.gyro_radps = AngularVelocityRadPerSec3{0.95, 0.95, 0.0};

    GaitState calm_gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    GaitState fast_gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    stability.apply(calm, intent, calm_gait);
    stability.apply(fast, intent, fast_gait);

    if (!expect(nearlyEqual(fast_gait.step_length_m, calm_gait.step_length_m, 1e-9),
                "low-speed command should not get body-rate stride throttling")) {
        return false;
    }
    return expect(nearlyEqual(fast_gait.stride_phase_rate_hz.value, calm_gait.stride_phase_rate_hz.value, 1e-9),
                  "low-speed command should not change stride cadence under body-rate throttling");
}

bool testStabilityNegativeMarginRecoveryForcesAllStance() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 5.0, 0.0);
    intent.cmd_vx_mps = LinearRateMps{0.25};
    intent.speed_mps = LinearRateMps{0.25};

    GaitState gait{};
    gait.duty_factor = 0.5;
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    gait.phase = {0.25, 0.75, 0.25, 0.75, 0.25, 0.75};
    gait.in_stance = {true, false, true, false, true, false};

    stability.apply(RobotState{}, intent, gait);

    if (!expect(gait.static_stability_margin_m < 0.0,
                "recovery-stance test should produce a negative support margin")) {
        return false;
    }
    if (!expect(gait.step_length_m == 0.0,
                "negative-margin recovery should stop step progression")) {
        return false;
    }
    if (!expect(gait.stride_phase_rate_hz.value == 0.0,
                "negative-margin recovery should freeze gait cadence")) {
        return false;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(gait.in_stance[static_cast<std::size_t>(leg)],
                    "negative-margin recovery should force every leg into stance") ||
            !expect(gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                    "negative-margin recovery should hold every leg in stance")) {
            return false;
        }
    }
    return true;
}

bool testStabilityStrictConfigLatchesNearLiftoff() {
    LocomotionStabilityConfig cfg{};
    cfg.min_margin_required_m = 50.0;
    cfg.support_inset_m = 0.0;
    cfg.liftoff_gate_phase_span = 0.07;
    LocomotionStability stability{cfg};

    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    GaitState gait = allStanceWalkGait(0.48, FrequencyHz{1.0});
    stability.apply(RobotState{}, intent, gait);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                     "impossible support margin should latch hold in liftoff window")) {
            return false;
        }
        if (!expect(gait.support_liftoff_clearance_m[static_cast<std::size_t>(leg)] < 0.0,
                     "support clearance should report the blocked liftoff margin")) {
            return false;
        }
    }
    return true;
}

bool testStabilityResetClearsLatch() {
    LocomotionStabilityConfig cfg{};
    cfg.min_margin_required_m = 50.0;
    cfg.support_inset_m = 0.0;
    cfg.liftoff_gate_phase_span = 0.07;
    LocomotionStability stability{cfg};

    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    GaitState gait = allStanceWalkGait(0.48, FrequencyHz{1.0});
    stability.apply(RobotState{}, intent, gait);

    stability.reset();
    gait = allStanceWalkGait(0.20, FrequencyHz{1.0});
    stability.apply(RobotState{}, intent, gait);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                     "reset should not bypass fresh support checks when the margin is still impossible")) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    if (!testBodyPoseZeroMarginNoLean() || !testBodyPoseFullMarginLeanForward() ||
        !testBodyPoseDefaultHeight() || !testBodyPoseYawLeanRoll() ||
        !testBodyPoseSlowStrideBoostsLean() ||
        !testStabilityStandClears() || !testStabilityWalkCenteredPositiveMargin() ||
        !testStabilitySlowGaitIsNotMoreConservativeByDefault() ||
        !testStabilityComOutsideHullNegativeMargin() || !testStabilitySupportDiagnosticsExposeAsymmetry() ||
        !testStabilityUsesConfirmedSupportWhenPhaseLags() ||
        !testSupportAssessmentUsesConfirmedSupportWhenPhaseLags() ||
        !testSupportAssessmentTracksUncertainSupportPhases() ||
        !testStabilityFastBodyRateThrottlesStride() || !testStabilityRaisesSwingHeightOnBodySag() ||
        !testStabilitySlowCommandDoesNotThrottleStride() ||
        !testStabilityNegativeMarginRecoveryForcesAllStance() ||
        !testStabilityHighTiltForcesHold() ||
        !testStabilityStrictConfigLatchesNearLiftoff() ||
        !testStabilityResetClearsLatch()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
