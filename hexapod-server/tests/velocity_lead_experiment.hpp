#pragma once

#include "types.hpp"
#include "hexapod_dynamics_constants.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace physics_sim_test_utils {
// Test-only actuator experiment, NOT a production controller or new plant mode.
// q_motor = q_reference + (Kd/Kp) qdot_reference makes the unchanged position
// motor equivalent to PD on tracking velocity. The physical motor still clamps
// torque against its speed envelope and all solver safety guards still apply.
// This deliberately acts after the server's reference limiter; it does not
// claim the anticipated motor target obeys that reference's slew/angle bounds.
class VelocityLeadExperiment {
public:
    explicit VelocityLeadExperiment(double dt) : dt_(dt) {}
    static bool enabledFromEnv() {
        const char* value = std::getenv("HEXAPOD_WALK_TEST_VELOCITY_LEAD");
        return value && (std::string(value) == "1" || std::string(value) == "filtered");
    }
    static bool filteredFromEnv() {
        const char* value = std::getenv("HEXAPOD_WALK_TEST_VELOCITY_LEAD");
        return value && std::string(value) == "filtered";
    }
    JointTargets apply(const JointTargets& reference, bool enabled, bool filtered = false) {
        if (!enabled) {
            bias_ = {};
            havePrevious_ = false;
            return reference;
        }
        if (!std::isfinite(dt_) || dt_ <= 0) throw std::runtime_error("invalid lead experiment cadence");
        JointTargets sent = reference;
        constexpr double leadS = 2 * hexapod_dynamics::kServoZeta / hexapod_dynamics::kServoOmegaN;
        for (int leg=0;leg<kNumLegs;++leg) for (int joint=0;joint<kJointsPerLeg;++joint) {
            const auto& request = reference.leg_states[leg].joint_state[joint];
            auto& motor = sent.leg_states[leg].joint_state[joint];
            if (!std::isfinite(request.pos_rad.value) || !std::isfinite(request.vel_radps.value))
                throw std::runtime_error("non-finite lead experiment reference");
            const double v = std::clamp(request.vel_radps.value,
                -hexapod_dynamics::kServoNoLoadSpeedRadPerSec,
                 hexapod_dynamics::kServoNoLoadSpeedRadPerSec);
            double& offset = bias_[leg*kJointsPerLeg+joint];
            // A separate counterfactual: smooth the lead using the existing
            // Kd/Kp timescale, not a tuned gain. Zero-state startup and exact
            // exponential response make elapsed-time behaviour cadence-neutral.
            offset = filtered ? offset + (-std::expm1(-dt_/leadS)) * (leadS*v-offset) : leadS*v;
            motor.pos_rad.value += offset;
            // Wire velocity describes the target actually sent (not the
            // reference velocity used to construct it); no hidden double lead.
            motor.vel_radps.value = havePrevious_
                ? std::remainder(motor.pos_rad.value-previous_.leg_states[leg].joint_state[joint].pos_rad.value,
                                 6.28318530717958647692)/dt_ : 0;
        }
        previous_ = sent;
        havePrevious_ = true;
        return sent;
    }
private:
    double dt_;
    JointTargets previous_{};
    std::array<double, kNumLegs*kJointsPerLeg> bias_{};
    bool havePrevious_{};
};
}
