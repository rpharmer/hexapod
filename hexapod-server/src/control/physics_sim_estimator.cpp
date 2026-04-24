#include "physics_sim_estimator.hpp"

void PhysicsSimEstimator::configure(const control_config::FusionConfig& config) {
    fusion_.configure(config);
}

void PhysicsSimEstimator::reset() {
    fusion_.reset();
}

RobotState PhysicsSimEstimator::update(const RobotState& raw) {
    RobotState source = raw;
    if (!source.has_body_twist_state) {
        source.has_body_twist_state = true;
    }
    RobotState out = fusion_.update(source, state_fusion::FusionSourceMode::Measured);
    if (raw.has_body_twist_state) {
        out.has_body_twist_state = true;
    }
    return out;
}
