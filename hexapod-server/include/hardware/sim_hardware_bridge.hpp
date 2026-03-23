#pragma once

#include <array>
#include <mutex>
#include <optional>

#include "hardware_bridge.hpp"

struct SimHardwareFaultToggles {
    bool drop_bus{false};
    bool low_voltage{false};
    bool high_current{false};
    std::optional<std::array<bool, kNumLegs>> forced_contacts{};

    float nominal_voltage{12.0f};
    float low_voltage_value{6.0f};
    float nominal_current{1.0f};
    float high_current_value{25.0f};
};

class SimHardwareBridge final : public IHardwareBridge {
public:
    explicit SimHardwareBridge(SimHardwareFaultToggles fault_toggles = {},
                               DurationSec read_cycle_period = DurationSec{0.02},
                               DurationSec response_time_constant = DurationSec{0.08});
    ~SimHardwareBridge() override = default;

    bool init() override;
    bool read(RobotState& out) override;
    bool write(const JointTargets& in) override;
    void setFaultToggles(const SimHardwareFaultToggles& fault_toggles);

private:
    SimHardwareFaultToggles fault_toggles_{};
    DurationSec read_cycle_period_{};
    DurationSec response_time_constant_{};

    bool initialized_{false};
    mutable std::mutex mutex_{};
    RobotState state_{};
    JointTargets commanded_targets_{};
};
