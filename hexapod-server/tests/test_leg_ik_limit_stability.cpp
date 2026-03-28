#include "leg_ik.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "reach_envelope.hpp"

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool finiteJointTargets(const JointTargets& joints) {
    for (const auto& leg : joints.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

bool nearlyEqual(double lhs, double rhs, double eps = 1e-6) {
    return std::abs(lhs - rhs) <= eps;
}

double wrappedAngleDistance(double a_rad, double b_rad) {
    return std::abs(std::remainder(a_rad - b_rad, 2.0 * kPi));
}

Vec3 legFrameToBodyFrame(const Vec3& foot_leg, const LegGeometry& leg) {
    return (Mat3::rotZ(leg.mountAngle.value) * foot_leg) + leg.bodyCoxaOffset;
}

} // namespace

int main() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegIK ik{geometry};

    RobotState est{};
    for (auto& leg : est.leg_states) {
        leg.joint_state[0].pos_rad = AngleRad{0.0};
        leg.joint_state[1].pos_rad = AngleRad{-0.7};
        leg.joint_state[2].pos_rad = AngleRad{-1.0};
    }

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    int clamp_trigger_count = 0;
    int fallback_trigger_count = 0;
    double max_joint_delta_rad = 0.0;

    constexpr int kSamples = 120;
    for (int sample = 0; sample < kSamples; ++sample) {
        const double phase = static_cast<double>(sample) / static_cast<double>(kSamples - 1);
        const bool extension_sweep = sample < (kSamples / 2);

        LegTargets targets{};
        for (int leg_id = 0; leg_id < kNumLegs; ++leg_id) {
            const LegGeometry& leg = geometry.legGeometry[leg_id];
            const kinematics::LegReachEnvelope envelope = kinematics::legReachEnvelope(leg);

            const double boundary = extension_sweep ? envelope.max_reach_m : envelope.min_reach_m;
            const double overshoot = 0.006 * std::sin(phase * 2.0 * kPi + static_cast<double>(leg_id));
            const double solved_reach = std::max(1e-4, boundary + overshoot);

            const double q1 = leg.mountAngle.value + 0.18 * std::sin(phase * 2.0 * kPi + 0.3 * leg_id);
            const double rho = solved_reach * std::cos(0.10 * std::sin(phase * 2.0 * kPi)) +
                               (extension_sweep ? 0.0 : 0.002);
            const double z = solved_reach * std::sin(0.10 * std::sin(phase * 2.0 * kPi));
            const double r = leg.coxaLength.value + rho;
            const Vec3 request_leg{r * std::cos(q1), r * std::sin(q1), z};

            const Vec3 clamped_leg = kinematics::clampFootToReachEnvelope(request_leg, leg);
            const Vec3 clamp_error = request_leg - clamped_leg;
            const double clamp_error_norm =
                std::sqrt(clamp_error.x * clamp_error.x + clamp_error.y * clamp_error.y + clamp_error.z * clamp_error.z);
            if (clamp_error_norm > 1e-6) {
                ++clamp_trigger_count;
            }

            targets.feet[leg_id].pos_body_m = legFrameToBodyFrame(request_leg, leg);
        }

        safety.leg_enabled.fill(true);
        if ((sample % 11) == 0) {
            // Force estimator fallback on one leg while the others keep solving.
            safety.leg_enabled[2] = false;
        }

        const JointTargets solved = ik.solve(est, targets, safety);
        if (!expect(finiteJointTargets(solved), "IK outputs must remain finite near reach limits")) {
            return EXIT_FAILURE;
        }

        if (!safety.leg_enabled[2]) {
            ++fallback_trigger_count;
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                if (!expect(nearlyEqual(solved.leg_states[2].joint_state[joint].pos_rad.value,
                                        est.leg_states[2].joint_state[joint].pos_rad.value,
                                        1e-9),
                            "disabled leg should use estimator fallback instead of unstable solve jumps")) {
                    return EXIT_FAILURE;
                }
            }
        }

        for (int leg_id = 0; leg_id < kNumLegs; ++leg_id) {
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double delta = wrappedAngleDistance(
                    solved.leg_states[leg_id].joint_state[joint].pos_rad.value,
                    est.leg_states[leg_id].joint_state[joint].pos_rad.value);
                max_joint_delta_rad = std::max(max_joint_delta_rad, delta);
            }
        }

        est.leg_states = solved.leg_states;
    }

    if (!expect(clamp_trigger_count > 0,
                "near-limit targets should trigger reach-envelope clamping")) {
        return EXIT_FAILURE;
    }
    if (!expect(fallback_trigger_count > 0,
                "test sweep should trigger estimator fallback on disabled-leg samples")) {
        return EXIT_FAILURE;
    }
    if (!expect(max_joint_delta_rad < 3.0,
                "joint deltas across near-limit IK samples should stay bounded")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
