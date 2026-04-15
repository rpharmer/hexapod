#include "physics_sim_estimator.hpp"

RobotState PhysicsSimEstimator::update(const RobotState& raw) {
    RobotState out = raw;
    out.has_body_twist_state = true;
    out.has_valid_flag = true;
    return out;
}
