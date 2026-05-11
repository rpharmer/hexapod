#include "servo_observer.hpp"

#include "math_types.hpp"

#include <algorithm>
#include <cmath>

ServoObserver::ServoObserver(ServoObserverConfig config)
    : config_(config) {}

void ServoObserver::reset() {
    state_ = RobotState{};
    initialized_ = false;
}

void ServoObserver::update(const JointTargets& command, const double dt_s) {
    const double dt = std::max(0.0, dt_s);
    if (!initialized_) {
        state_.leg_states = command.leg_states;
        initialized_ = true;
    }

    const double tau = std::max(0.0, config_.lag_tau_s);
    const double alpha = tau <= 0.0 ? 1.0 : std::clamp(1.0 - std::exp(-dt / tau), 0.0, 1.0);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            JointState& observed = state_.leg_states[leg].joint_state[joint];
            const double target = command.leg_states[leg].joint_state[joint].pos_rad.value;
            const double error = target - observed.pos_rad.value;
            if (std::abs(error) <= config_.deadband_rad) {
                observed.vel_radps = AngularRateRadPerSec{0.0};
                continue;
            }
            double delta = error * alpha;
            const double rate_limit = error >= 0.0 ? config_.positive_rate_limit_radps
                                                   : config_.negative_rate_limit_radps;
            const double max_delta = std::max(0.0, rate_limit) * dt;
            if (max_delta > 0.0) {
                delta = std::clamp(delta, -max_delta, max_delta);
            }
            observed.pos_rad.value += delta;
            observed.vel_radps = AngularRateRadPerSec{dt > 0.0 ? delta / dt : 0.0};
        }
        state_.joint_state_quality[leg].position_valid = true;
        state_.joint_state_quality[leg].velocity_valid = dt > 0.0;
        state_.joint_state_quality[leg].source = JointStateSource::ObserverEstimate;
        state_.joint_state_quality[leg].age_us = 0;
        state_.joint_state_quality[leg].confidence = config_.initial_confidence;
    }
    state_.valid = true;
    state_.has_valid_flag = true;
}
