#pragma once

#include "control_config.hpp"
#include "types.hpp"

namespace state_fusion {

enum class FusionSourceMode {
    Measured,
    Predictive,
};

class StateFusion {
public:
    explicit StateFusion(control_config::FusionConfig config = {});

    void configure(const control_config::FusionConfig& config);
    void reset();

    [[nodiscard]] RobotState update(const RobotState& source, FusionSourceMode mode);
    [[nodiscard]] const FusionDiagnostics& diagnostics() const;

private:
    struct FootTracker {
        FootContactFusion state{};
        uint32_t raw_contact_streak{0};
        uint32_t raw_gap_streak{0};
    };

    static double phaseConfidenceFloor(ContactPhase phase);

    control_config::FusionConfig config_{};
    std::array<FootTracker, kNumLegs> foot_trackers_{};
    RobotState last_output_{};
    FusionDiagnostics diagnostics_{};
    TimePointUs last_update_us_{};
    bool have_last_output_{false};
};

} // namespace state_fusion
