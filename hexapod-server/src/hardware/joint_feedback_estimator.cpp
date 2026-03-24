#include "joint_feedback_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_config.hpp"

namespace {

constexpr DurationSec kSoftwareFeedbackDefaultDtSec{0.002};

}  // namespace

void JointFeedbackEstimator::set_enabled(bool enabled) {
    enabled_ = enabled;
}

bool JointFeedbackEstimator::enabled() const {
    return enabled_;
}

void JointFeedbackEstimator::reset() {
    estimated_state_ = RobotState{};
    last_written_ = JointTargets{};
    last_timestamp_ = TimePointUs{};
}

void JointFeedbackEstimator::on_write(const JointTargets& targets) {
    last_written_ = targets;
}

void JointFeedbackEstimator::on_hardware_read(const RobotState& state) {
    if (!enabled_) {
        estimated_state_ = state;
    }
}

void JointFeedbackEstimator::synthesize(RobotState& out) {
    if (!enabled_) {
        return;
    }

    const TimePointUs current_ts = out.timestamp_us;
    DurationSec dt_s = kSoftwareFeedbackDefaultDtSec;
    if (!last_timestamp_.isZero() && current_ts.value > last_timestamp_.value) {
        dt_s = DurationSec{static_cast<double>((current_ts - last_timestamp_).value) * 1e-6};
    }
    last_timestamp_ = current_ts;

    const double dt = std::max(dt_s.value, 0.0);
    const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            AngleRad& simulated = estimated_state_.leg_states[leg].joint_state[joint].pos_rad;
            const AngleRad target = last_written_.leg_states[leg].joint_state[joint].pos_rad;
            const double error = target.value - simulated.value;
            const ServoJointDynamics& dyn = geometry.legGeometry[leg].servoDynamics[joint];
            const ServoDirectionDynamics& direction =
                (error >= 0.0) ? dyn.positive_direction : dyn.negative_direction;

            const double tau = direction.tau_s;
            const double alpha = (tau <= 0.0) ? 1.0 : clamp01(1.0 - std::exp(-dt / tau));
            double delta = alpha * error;

            const double vmax = std::max(direction.vmax_radps, 0.0);
            const double max_delta = vmax * dt;
            if (max_delta > 0.0) {
                delta = std::clamp(delta, -max_delta, max_delta);
            }

            simulated.value += delta;
            out.leg_states[leg].joint_state[joint].pos_rad = simulated;
        }
    }
}
