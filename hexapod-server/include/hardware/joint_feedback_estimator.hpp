#pragma once

#include "types.hpp"

class JointFeedbackEstimator {
public:
    void set_enabled(bool enabled);
    bool enabled() const;

    void reset();
    void on_write(const JointTargets& targets);
    void on_hardware_read(const RobotState& state);
    void synthesize(RobotState& out);

private:
    bool enabled_{false};
    RobotState estimated_state_{};
    JointTargets last_written_{};
    TimePointUs last_timestamp_{};
};
