#pragma once

#include "local_map.hpp"
#include "robot_control.hpp"

#include <array>
#include <string>
#include <vector>

struct ScenarioMotionIntent {
    bool enabled{false};
    RobotMode mode{RobotMode::SAFE_IDLE};
    GaitType gait{GaitType::TRIPOD};
    double body_height_m{0.05};
    double speed_mps{0.0};
    double heading_rad{0.0};
    double yaw_rad{0.0};
    /** Body-frame yaw rate (rad/s); optional alternative to deriving motion from speed/heading only. */
    double yaw_rate_radps{0.0};
    /** When true, `vx_mps` / `vy_mps` override polar `speed_mps` / `heading_rad` for the motion intent. */
    bool has_direct_velocity{false};
    double vx_mps{0.0};
    double vy_mps{0.0};
};

struct ScenarioSensorOverrides {
    bool clear_contacts{false};
    std::array<bool, kNumLegs> contacts{true, true, true, true, true, true};
};

struct ScenarioFaultOverrides {
    bool bus_down{false};
    bool low_voltage{false};
    float low_voltage_value_v{6.0f};
    bool high_current{false};
    float high_current_value_a{25.0f};
};

struct ScenarioNavigationCommand {
    enum class Action {
        None,
        NavigateToPose,
        Cancel,
    };

    bool enabled{false};
    Action action{Action::None};
    GaitType gait{GaitType::TRIPOD};
    double body_height_m{0.06};
    double goal_x_m{0.0};
    double goal_y_m{0.0};
    double goal_yaw_rad{0.0};
};

struct ScenarioMapObservationOverride {
    std::vector<LocalMapObservationSample> samples{};
};

struct ScenarioEvent {
    uint64_t at_ms{0};
    ScenarioMotionIntent motion{};
    bool has_sensor_overrides{false};
    ScenarioSensorOverrides sensors{};
    bool has_fault_overrides{false};
    ScenarioFaultOverrides faults{};
    bool has_navigation_command{false};
    ScenarioNavigationCommand navigation{};
    bool has_map_observation_override{false};
    ScenarioMapObservationOverride map_observation{};
};

struct ScenarioDefinition {
    std::string name{};
    uint64_t duration_ms{5000};
    uint64_t tick_ms{20};
    bool refresh_motion_intent{true};
    std::vector<LocalMapObservationSample> initial_map_obstacles{};
    std::vector<ScenarioEvent> events{};
};

class ScenarioDriver {
public:
    enum class ValidationMode {
        Permissive,
        Strict,
    };

    static bool loadFromToml(const std::string& path, ScenarioDefinition& out,
                             std::string& error);

    static bool loadFromToml(const std::string& path, ScenarioDefinition& out,
                             std::string& error, ValidationMode mode);

    static bool run(RobotControl& robot, const ScenarioDefinition& scenario,
                    std::shared_ptr<logging::AsyncLogger> logger);
};
