#include "physics_sim_estimator.hpp"

void PhysicsSimEstimator::configure(const control_config::FusionConfig& config) {
    fusion_.configure(config);
}

void PhysicsSimEstimator::reset() {
    fusion_.reset();
}

RobotState PhysicsSimEstimator::update(const RobotState& raw) {
    RobotState out = fusion_.update(raw, state_fusion::FusionSourceMode::Measured);
    if (raw.has_body_twist_state) {
        out.has_body_twist_state = true;
    }
    return out;
}
