#pragma once

#include "estimator.hpp"

/// Uses rigid-body sim state already embedded in RobotState from PhysicsSimBridge.
class PhysicsSimEstimator final : public IEstimator {
public:
    RobotState update(const RobotState& raw) override;
};
