#include "physics_sim_estimator.hpp"

void PhysicsSimEstimator::configure(const control_config::FusionConfig& config) {
    fusion_.configure(config);
}

void PhysicsSimEstimator::reset() {
    fusion_.reset();
}

RobotState PhysicsSimEstimator::update(const RobotState& raw) {
    RobotState out = raw;
    out.has_body_twist_state = true;
    RobotState fused = fusion_.update(out, state_fusion::FusionSourceMode::Predictive);
    return fused;
}
