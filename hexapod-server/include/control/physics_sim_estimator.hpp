#pragma once

#include "estimator.hpp"

/// Uses rigid-body sim state already embedded in RobotState from PhysicsSimBridge, and
/// publishes fused contact diagnostics so the control stack can reason about the sim like
/// the hardware-backed estimators do.
class PhysicsSimEstimator final : public IEstimator {
public:
    void configure(const control_config::FusionConfig& config) override;
    void reset() override;
    RobotState update(const RobotState& raw) override;

private:
    state_fusion::StateFusion fusion_{};
};
