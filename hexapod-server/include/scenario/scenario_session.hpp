#pragma once

#include "local_map.hpp"
#include "logger.hpp"
#include "robot_control.hpp"
#include "scenario_driver.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

/** Tickable scenario host for interactive mode (scenario > nav > gamepad). */
class ScenarioSession {
public:
    [[nodiscard]] bool active() const { return active_; }
    [[nodiscard]] bool failed() const { return failed_; }
    [[nodiscard]] const std::string& name() const { return scenario_.name; }
    [[nodiscard]] const ScenarioDefinition& definition() const { return scenario_; }

    /** Load and arm a scenario. Cancels any active nav and clears prior session state. */
    bool start(RobotControl& robot,
               ScenarioDefinition scenario,
               std::shared_ptr<logging::AsyncLogger> logger,
               std::string& error);

    /** Advance timeline to wall-clock now. Returns false when the session ends or fails. */
    bool tick(RobotControl& robot, std::chrono::steady_clock::time_point now);

    /** Stop early: cancel nav, clear fault/safety overrides, idle intent. */
    void stop(RobotControl& robot);

private:
    bool applyEvent(RobotControl& robot, const ScenarioEvent& event);
    void finish(RobotControl& robot, bool clear_nav);

    ScenarioDefinition scenario_{};
    std::shared_ptr<logging::AsyncLogger> logger_;
    std::shared_ptr<SyntheticLocalMapObservationSource> map_source_{};
    SimHardwareFaultToggles toggles_{};
    MotionIntent current_intent_{};
    std::size_t event_idx_{0};
    std::chrono::steady_clock::time_point start_time_{};
    bool active_{false};
    bool failed_{false};
    bool uses_navigation_{false};
};
