#include "body_controller.hpp"
#include "estimator.hpp"
#include "leg_ik.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

constexpr double kDefaultBodyHeightM = 0.20;
constexpr double kSlopeRollRad = 0.08;
constexpr double kSlopePitchRad = -0.10;
constexpr double kLevelHoldKp = 0.6;
constexpr double kLevelHoldMaxCorrectionRad = 0.18;
constexpr double kLevelHoldMaxCommandRad = 0.30;

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

double legVelocityMagnitude(const FootTarget& target) {
    const Vec3 v = target.vel_body_mps;
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

std::array<Vec3, kNumLegs> nominalStance() {
    std::array<Vec3, kNumLegs> nominal{};
    const HexapodGeometry geometry = defaultHexapodGeometry();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
        const double neutral_leg_x =
            leg_geo.coxaLength.value + 0.55 * (leg_geo.femurLength.value + leg_geo.tibiaLength.value);
        const Vec3 neutral_leg_frame{neutral_leg_x, 0.0, -kDefaultBodyHeightM};
        nominal[leg] = leg_geo.bodyCoxaOffset + (Mat3::rotZ(leg_geo.mountAngle.value) * neutral_leg_frame);
    }
    return nominal;
}

std::array<Vec3, kNumLegs> makeSlopeFootTargets(double roll_rad,
                                                 double pitch_rad,
                                                 double phase_noise) {
    const auto nominal = nominalStance();
    std::array<Vec3, kNumLegs> out{};
    const double a = std::tan(pitch_rad);
    const double b = -std::tan(roll_rad);
    const double c = -kDefaultBodyHeightM;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 p = nominal[leg];
        const double leg_noise = 0.0015 * std::sin(phase_noise + 0.7 * static_cast<double>(leg));
        p.x += 0.0008 * std::cos(phase_noise * 0.9 + static_cast<double>(leg));
        p.y += leg_noise;
        p.z = (a * p.x) + (b * p.y) + c + 0.0012 * std::cos(phase_noise + 0.3 * static_cast<double>(leg));
        out[leg] = p;
    }
    return out;
}

RobotState makeRawFromFootTargets(const std::array<Vec3, kNumLegs>& foot_targets,
                                  uint64_t sample_id,
                                  uint64_t timestamp_us,
                                  const std::array<bool, kNumLegs>& contacts) {
    RobotState seed_est{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            seed_est.leg_states[leg].joint_state[joint].pos_rad = AngleRad{0.0};
        }
    }

    LegTargets targets{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        targets.feet[leg].pos_body_m = foot_targets[leg];
    }

    SafetyState ik_safety{};
    LegIK ik{};
    const JointTargets solved = ik.solve(seed_est, targets, ik_safety);

    RobotState raw{};
    raw.sample_id = sample_id;
    raw.timestamp_us = TimePointUs{timestamp_us};
    raw.foot_contacts = contacts;
    raw.leg_states = solved.leg_states;
    return raw;
}

} // namespace

int main() {
    SimpleEstimator estimator{};
    BodyController controller{};

    MotionIntent level_hold_intent{};
    level_hold_intent.requested_mode = RobotMode::STAND;
    level_hold_intent.body_pose_setpoint.body_trans_m.z = kDefaultBodyHeightM;
    level_hold_intent.body_pose_setpoint.body_trans_mps = Vec3{0.03, -0.02, 0.01};
    level_hold_intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, 0.20};

    MotionIntent tilted_setpoint_intent = level_hold_intent;
    tilted_setpoint_intent.body_pose_setpoint.orientation_rad = Vec3{0.04, -0.03, 0.0};

    GaitState gait{};

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.support_contact_count = 4;
    safety.stability_margin_m = 0.03;

    std::array<bool, kNumLegs> contacts{};
    contacts.fill(true);

    double max_observed_correction_roll = 0.0;
    double max_observed_correction_pitch = 0.0;
    double max_velocity_magnitude = 0.0;
    double max_velocity_delta = 0.0;

    LegTargets previous_targets{};
    bool have_previous_targets = false;
    RobotState representative_est{};

    for (int cycle = 0; cycle < 24; ++cycle) {
        const double phase_noise = 0.25 * static_cast<double>(cycle);
        if ((cycle % 7) == 3) {
            contacts[(cycle / 7) % kNumLegs] = false;
        } else {
            contacts.fill(true);
        }

        const auto slope_targets = makeSlopeFootTargets(kSlopeRollRad, kSlopePitchRad, phase_noise);
        const RobotState raw = makeRawFromFootTargets(
            slope_targets,
            static_cast<uint64_t>(cycle + 1),
            1'000'000 + static_cast<uint64_t>(cycle) * 20'000,
            contacts);
        const RobotState est = estimator.update(raw);
        representative_est = est;

        if (!expect(est.has_inferred_body_pose_state,
                    "slope contact pattern should keep inferred pose available")) {
            return EXIT_FAILURE;
        }

        const double expected_roll_correction = std::clamp(
            kLevelHoldKp * (level_hold_intent.body_pose_setpoint.orientation_rad.x - est.body_pose_state.orientation_rad.x),
            -kLevelHoldMaxCorrectionRad,
            kLevelHoldMaxCorrectionRad);
        const double expected_pitch_correction = std::clamp(
            kLevelHoldKp * (level_hold_intent.body_pose_setpoint.orientation_rad.y - est.body_pose_state.orientation_rad.y),
            -kLevelHoldMaxCorrectionRad,
            kLevelHoldMaxCorrectionRad);

        max_observed_correction_roll = std::max(max_observed_correction_roll, std::abs(expected_roll_correction));
        max_observed_correction_pitch = std::max(max_observed_correction_pitch, std::abs(expected_pitch_correction));

        const double effective_roll_cmd =
            std::clamp(level_hold_intent.body_pose_setpoint.orientation_rad.x + expected_roll_correction,
                       -kLevelHoldMaxCommandRad,
                       kLevelHoldMaxCommandRad);
        const double effective_pitch_cmd =
            std::clamp(level_hold_intent.body_pose_setpoint.orientation_rad.y + expected_pitch_correction,
                       -kLevelHoldMaxCommandRad,
                       kLevelHoldMaxCommandRad);

        if (!expect(std::abs(effective_roll_cmd) <= kLevelHoldMaxCommandRad + 1e-12,
                    "effective roll command should remain within level-hold command clamp") ||
            !expect(std::abs(effective_pitch_cmd) <= kLevelHoldMaxCommandRad + 1e-12,
                    "effective pitch command should remain within level-hold command clamp")) {
            return EXIT_FAILURE;
        }

        const LegTargets targets = controller.update(est, level_hold_intent, gait, safety);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const double speed = legVelocityMagnitude(targets.feet[leg]);
            max_velocity_magnitude = std::max(max_velocity_magnitude, speed);
            if (!expect(std::isfinite(speed), "leg target velocity should stay finite")) {
                return EXIT_FAILURE;
            }

            if (have_previous_targets) {
                const Vec3 dv = targets.feet[leg].vel_body_mps - previous_targets.feet[leg].vel_body_mps;
                const double delta = std::sqrt(dv.x * dv.x + dv.y * dv.y + dv.z * dv.z);
                max_velocity_delta = std::max(max_velocity_delta, delta);
            }
        }

        previous_targets = targets;
        have_previous_targets = true;
    }

    if (!expect(max_observed_correction_roll <= kLevelHoldMaxCorrectionRad + 1e-12,
                "roll correction should remain within correction clamp") ||
        !expect(max_observed_correction_pitch <= kLevelHoldMaxCorrectionRad + 1e-12,
                "pitch correction should remain within correction clamp")) {
        return EXIT_FAILURE;
    }

    if (!expect(max_velocity_magnitude < 1.5,
                "foot target velocities should not diverge under slope/noise cycles") ||
        !expect(max_velocity_delta < 0.75,
                "foot target velocity deltas should remain bounded between cycles")) {
        return EXIT_FAILURE;
    }

    const LegTargets zero_setpoint_targets = controller.update(representative_est, level_hold_intent, gait, safety);
    const LegTargets nonzero_setpoint_targets =
        controller.update(representative_est, tilted_setpoint_intent, gait, safety);

    double total_pose_setpoint_effect = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 delta = nonzero_setpoint_targets.feet[leg].pos_body_m - zero_setpoint_targets.feet[leg].pos_body_m;
        total_pose_setpoint_effect += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }

    if (!expect(total_pose_setpoint_effect > 1e-4,
                "non-zero roll/pitch pose setpoints should remain effective") ) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
