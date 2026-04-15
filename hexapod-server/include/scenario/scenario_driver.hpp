#pragma once

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

struct ScenarioEvent {
    uint64_t at_ms{0};
    ScenarioMotionIntent motion{};
    bool has_sensor_overrides{false};
    ScenarioSensorOverrides sensors{};
    bool has_fault_overrides{false};
    ScenarioFaultOverrides faults{};
};

struct ScenarioDefinition {
    std::string name{};
    uint64_t duration_ms{5000};
    uint64_t tick_ms{20};
    bool refresh_motion_intent{true};
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
