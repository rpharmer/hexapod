#include "body_pose_controller.hpp"
#include "locomotion_stability.hpp"

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
    const BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, kSoftMargin, 1.0);
    constexpr double kLeanPitchPerVx = 0.22;
    const double expected_pitch = 0.2 - kLeanPitchPerVx * 1.0;
    return expect(nearlyEqual(pose.pitch_rad, expected_pitch, 1e-9),
                  "full margin should apply full forward lean into +vx");
}

bool testBodyPoseDefaultHeight() {
    MotionIntent intent = walkIntent(0.0);
    PlanarMotionCommand cmd{};
    const BodyPoseSetpoint pose = computeBodyPoseSetpoint(intent, cmd, 0.022, 1.0);
    return expect(nearlyEqual(pose.body_height_m, 0.05), "zero height setpoint should fall back to default");
}

bool testBodyPoseYawLeanRoll() {
    MotionIntent intent = walkIntent(0.12);
    PlanarMotionCommand cmd{0.0, 0.0, 0.45};
    constexpr double kSoftMargin = 0.022;
    constexpr double kLeanRollPerYaw = 0.18;
    const BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, cmd, kSoftMargin, 1.0);
    const double expected_roll = 0.1 + kLeanRollPerYaw * 1.0;
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

    stability.apply(intent, gait);

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
    stability.apply(intent, gait);
    return expect(gait.static_stability_margin_m > 0.002,
                  "centered COM with all feet in stance should yield positive static margin");
}

bool testStabilityComOutsideHullNegativeMargin() {
    LocomotionStability stability{};
    MotionIntent intent = walkIntent(0.12, 5.0, 0.0);
    GaitState gait = allStanceWalkGait(0.25, FrequencyHz{1.0});
    stability.apply(intent, gait);
    return expect(gait.static_stability_margin_m < 0.0,
                  "large body XY offset should push projected COM outside nominal support");
}

bool testStabilityStrictConfigLatchesNearLiftoff() {
    LocomotionStabilityConfig cfg{};
    cfg.min_margin_required_m = 50.0;
    cfg.support_inset_m = 0.0;
    cfg.liftoff_gate_phase_span = 0.07;
    LocomotionStability stability{cfg};

    MotionIntent intent = walkIntent(0.12, 0.0, 0.0);
    GaitState gait = allStanceWalkGait(0.48, FrequencyHz{1.0});
    stability.apply(intent, gait);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                     "impossible support margin should latch hold in liftoff window")) {
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
    stability.apply(intent, gait);

    stability.reset();
    gait = allStanceWalkGait(0.20, FrequencyHz{1.0});
    stability.apply(intent, gait);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(!gait.stability_hold_stance[static_cast<std::size_t>(leg)],
                     "reset + early stance phase should clear stability hold latches")) {
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
        !testStabilityComOutsideHullNegativeMargin() || !testStabilityStrictConfigLatchesNearLiftoff() ||
        !testStabilityResetClearsLatch()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
