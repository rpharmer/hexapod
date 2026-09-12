#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "hardware_bridge.hpp"
#include "physics_sim_protocol.hpp"

namespace logging {
class AsyncLogger;
}

struct PhysicsSimObstacleFootprint {
    double center_x_m{0.0};
    double center_y_m{0.0};
    double half_extent_x_m{0.0};
    double half_extent_y_m{0.0};
    double yaw_rad{0.0};
};

class IPhysicsSimObstacleFootprintProvider {
public:
    virtual ~IPhysicsSimObstacleFootprintProvider() = default;

    [[nodiscard]] virtual std::vector<PhysicsSimObstacleFootprint> latestObstacleFootprints() const = 0;
};

struct PhysicsSimSolverSettings {
    physics_sim::PhysicsSolverMode mode{physics_sim::PhysicsSolverMode::LegacyPgs};
    int iterations{24};
    float proximal_mu{1.0e-6f};
    float absolute_tolerance{1.0e-8f};
    float relative_tolerance{1.0e-6f};
    float contact_regularization{1.0e-10f};
};

struct PhysicsSimSolverTelemetry {
    physics_sim::SolverStatus status{physics_sim::SolverStatus::Healthy};
    physics_sim::SolverFailureReason failure_reason{physics_sim::SolverFailureReason::None};
    std::uint16_t iterations{0};
    float primal_residual{0.0f};
    float dual_residual{0.0f};
    float complementarity_residual{0.0f};
    float ncp_dual_residual{0.0f};
    float ncp_complementarity_residual{0.0f};
    float cone_residual{0.0f};
    float peak_normal_impulse{0.0f};
    float peak_friction_impulse{0.0f};
    float peak_structural_impulse{0.0f};
    float peak_actuator_impulse{0.0f};
    float peak_servo_torque_utilization{0.0f};
    float preintegration_linear_speed{0.0f};
    float preintegration_angular_speed{0.0f};
    float max_contact_penetration{0.0f};
    float mechanical_energy_delta{0.0f};
    float actuator_work{0.0f};
    float dynamics_time_ms{0.0f};
    float contact_setup_time_ms{0.0f};
    float collision_time_ms{0.0f};
    float constraint_assembly_time_ms{0.0f};
    float delassus_time_ms{0.0f};
    float admm_time_ms{0.0f};
    float integration_time_ms{0.0f};
    float total_step_time_ms{0.0f};
    float admm_rho{0.0f};
    float delassus_condition_estimate{0.0f};
    std::uint32_t contact_manifold_count{0};
    std::uint32_t contact_constraint_count{0};
    std::uint64_t warm_start_reset_count{0};
    std::uint64_t retry_count{0};
    std::uint64_t rollback_count{0};
    std::uint64_t held_state_count{0};
    std::uint64_t unsupported_island_count{0};
    std::uint64_t worst_contact_id{0};
    std::uint64_t contact_set_signature{0};
};

/// Chassis height used by the bridge's initial assembled standing pose.
double physicsSimStandingBodyHeightM();

/// UDP client to hexapod-physics-sim --serve; steps physics in read() after write().
class PhysicsSimBridge final : public IHardwareBridge, public IPhysicsSimObstacleFootprintProvider {
public:
    PhysicsSimBridge(std::string host,
                     int port,
                     int bus_loop_period_us,
                     int physics_solver_iterations = 24,
                     std::shared_ptr<logging::AsyncLogger> logger = nullptr);
    PhysicsSimBridge(std::string host,
                     int port,
                     int bus_loop_period_us,
                     PhysicsSimSolverSettings solver_settings,
                     std::shared_ptr<logging::AsyncLogger> logger = nullptr);
    ~PhysicsSimBridge() override;

    bool init() override;
    bool read(RobotState& out) override;
    bool write(const JointTargets& in) override;
    bool sendStateCorrection(const physics_sim::StateCorrection& correction);
    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override;
    [[nodiscard]] bool supportsAutomaticBusTimeoutRecovery() const override { return true; }
    [[nodiscard]] bool latestSampleHealthyForAutomaticRecovery() const override;
    [[nodiscard]] std::optional<PhysicsSimSolverTelemetry> latestSolverTelemetry() const;
    [[nodiscard]] std::vector<PhysicsSimObstacleFootprint> latestObstacleFootprints() const override;

private:
    bool recvWithTimeout(void* buf, std::size_t len, int timeout_ms);
    void setLastError(BridgeError err, BridgeFailurePhase phase);

    std::string host_;
    int port_{9871};
    int bus_loop_period_us_{2000};
    int physics_solver_iterations_{24};
    PhysicsSimSolverSettings solver_settings_{};
    std::shared_ptr<logging::AsyncLogger> logger_;

    int sock_{-1};
    bool initialized_{false};
    JointTargets pending_targets_{};
    TimePointUs last_motion_trace_log_us_{};
    BridgeCommandResultMetadata last_result_{};
    mutable std::mutex solver_telemetry_mutex_{};
    std::optional<PhysicsSimSolverTelemetry> latest_solver_telemetry_{};
    mutable std::mutex obstacle_mutex_{};
    std::vector<PhysicsSimObstacleFootprint> latest_obstacle_footprints_{};
};
