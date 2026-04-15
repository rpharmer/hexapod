#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "hardware_bridge.hpp"

namespace logging {
class AsyncLogger;
}

/// UDP client to hexapod-physics-sim --serve; steps physics in read() after write().
class PhysicsSimBridge final : public IHardwareBridge {
public:
    PhysicsSimBridge(std::string host,
                     int port,
                     int bus_loop_period_us,
                     int physics_solver_iterations = 24,
                     std::shared_ptr<logging::AsyncLogger> logger = nullptr);
    ~PhysicsSimBridge() override;

    bool init() override;
    bool read(RobotState& out) override;
    bool write(const JointTargets& in) override;
    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override;

private:
    bool recvWithTimeout(void* buf, std::size_t len, int timeout_ms);
    void setLastError(BridgeError err, BridgeFailurePhase phase);

    std::string host_;
    int port_{9871};
    int bus_loop_period_us_{2000};
    int physics_solver_iterations_{24};
    std::shared_ptr<logging::AsyncLogger> logger_;

    int sock_{-1};
    bool initialized_{false};
    JointTargets pending_targets_{};
    BridgeCommandResultMetadata last_result_{};
};
