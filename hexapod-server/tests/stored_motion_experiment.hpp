#pragma once

#include "control/servo_dynamics_clamp.hpp"
#include "control/swing_link_rate_governor.hpp"
#include "hexapod_dynamics_constants.hpp"
#include <cstdlib>
#include <string>

namespace physics_sim_test_utils {
// Diagnostic controller, not a plant modification. Position PD is equivalently
// Kd * (error/(Kd/Kp) - measured_rate). Bound that stored velocity request in
// the composed-link frame instead of bounding only successive target increments.
// This is NOT a guarantee on contact-accelerated or transient physical velocity.
class StoredMotionExperiment {
public:
    explicit StoredMotionExperiment(double dt) : dt_(dt) {}
    static bool enabledFromEnv() {
        const char* value = std::getenv("HEXAPOD_WALK_TEST_STORED_MOTION");
        return value && std::string(value) == "1";
    }
    JointTargets apply(const JointTargets& reference, const RobotState* measured,
                       const HexapodGeometry& geometry, bool enabled) {
        JointTargets sent = reference;
        if (enabled && measured && measured->bus_ok && measured->has_body_twist_state) {
            constexpr double dampingTime = 2 * hexapod_dynamics::kServoZeta / hexapod_dynamics::kServoOmegaN;
            for (int leg=0; leg<kNumLegs; ++leg) {
                const auto& quality = measured->joint_state_quality[leg];
                if (!quality.position_valid || quality.source != JointStateSource::Simulated) continue;
                std::array<double, kJointsPerLeg> rates{};
                for (int j=0; j<kJointsPerLeg; ++j)
                    rates[j] = std::remainder(reference.leg_states[leg].joint_state[j].pos_rad.value
                        - measured->leg_states[leg].joint_state[j].pos_rad.value,
                        6.28318530717958647692) / dampingTime;
                const auto projected = projectSwingLinkRates(geometry.legGeometry[leg],
                    measured->leg_states[leg], rates,
                    measured->body_twist_state.twist_vel_radps.raw(), kSpeedGuardLinkAngularRadps);
                if (projected.status != SwingLinkRateStatus::Limited) continue;
                for (int j=0; j<kJointsPerLeg; ++j)
                    sent.leg_states[leg].joint_state[j].pos_rad.value =
                        measured->leg_states[leg].joint_state[j].pos_rad.value
                        + dampingTime * rates[j] * projected.scale;
            }
            if (havePrevious_) (void)refreshJointTargetVelocities(previous_, sent, dt_);
        }
        previous_ = sent;
        havePrevious_ = true;
        return sent;
    }
private:
    double dt_;
    JointTargets previous_{};
    bool havePrevious_{};
};
}
