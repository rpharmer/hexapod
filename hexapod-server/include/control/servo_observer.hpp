#pragma once

#include "types.hpp"

#include <array>

struct ServoObserverConfig {
    double positive_rate_limit_radps{8.0};
    double negative_rate_limit_radps{8.0};
    double lag_tau_s{0.08};
    double deadband_rad{0.002};
    double initial_confidence{0.65};
};

class ServoObserver {
public:
    explicit ServoObserver(ServoObserverConfig config = {});

    void reset();
    void update(const JointTargets& command, double dt_s);
    [[nodiscard]] const RobotState& state() const { return state_; }

private:
    ServoObserverConfig config_{};
    RobotState state_{};
    bool initialized_{false};
};
