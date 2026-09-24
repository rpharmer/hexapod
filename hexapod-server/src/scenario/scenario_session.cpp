#include "scenario_session.hpp"

#include "motion_intent_utils.hpp"
#include "navigation_manager.hpp"

#include <algorithm>

namespace {

MotionIntent makeWalkBase(GaitType gait, double body_height_m) {
    return makeMotionIntent(RobotMode::WALK, gait, body_height_m);
}

std::optional<std::array<bool, kNumLegs>> buildContacts(const ScenarioSensorOverrides& sensors) {
    if (sensors.clear_contacts) {
        return std::nullopt;
    }
    return sensors.contacts;
}

} // namespace

bool ScenarioSession::start(RobotControl& robot,
                            ScenarioDefinition scenario,
                            std::shared_ptr<logging::AsyncLogger> logger,
                            std::string& error) {
    stop(robot);

    scenario_ = std::move(scenario);
    logger_ = std::move(logger);
    event_idx_ = 0;
    toggles_ = {};
    current_intent_ = {};
    current_intent_.requested_mode = RobotMode::SAFE_IDLE;
    current_intent_.gait = GaitType::TRIPOD;
    map_source_.reset();
    failed_ = false;

    uses_navigation_ =
        !scenario_.initial_map_obstacles.empty() ||
        std::any_of(scenario_.events.begin(), scenario_.events.end(), [](const ScenarioEvent& event) {
            return event.has_navigation_command || event.has_map_observation_override;
        });

    if (uses_navigation_) {
        if (robot.navigationManager() == nullptr) {
            error = "Scenario navigation requested but NavigationManager is not installed";
            return false;
        }
        map_source_ = std::make_shared<SyntheticLocalMapObservationSource>();
        map_source_->setStaticSamples(scenario_.initial_map_obstacles);
        robot.navigationManager()->addObservationSource(map_source_);
    }

    start_time_ = std::chrono::steady_clock::now();
    active_ = true;
    if (logger_) {
        LOG_INFO(logger_, "Scenario session started: ", scenario_.name);
    }
    return true;
}

bool ScenarioSession::applyEvent(RobotControl& robot, const ScenarioEvent& event) {
    if (event.motion.enabled) {
        current_intent_ = makeMotionIntent(event.motion);
        robot.setMotionIntent(current_intent_);
        if (logger_) {
            LOG_DEBUG(logger_, "Scenario event @", event.at_ms, "ms mode update");
        }
    }

    if (event.has_sensor_overrides) {
        toggles_.forced_contacts = buildContacts(event.sensors);
    }

    if (event.has_fault_overrides) {
        toggles_.drop_bus = event.faults.bus_down;
        toggles_.low_voltage = event.faults.low_voltage;
        toggles_.low_voltage_value = event.faults.low_voltage_value_v;
        toggles_.high_current = event.faults.high_current;
        toggles_.high_current_value = event.faults.high_current_value_a;
    }

    if (event.has_safety_overrides && event.safety.has_legs_enabled) {
        robot.setSafetyLegEnabledTestMask(
            std::optional<std::array<bool, kNumLegs>>{event.safety.legs_enabled});
    }

    if (event.has_map_observation_override && map_source_ != nullptr) {
        map_source_->setStaticSamples(event.map_observation.samples);
    }

    if (event.has_navigation_command && robot.navigationManager() != nullptr) {
        if (event.navigation.action == ScenarioNavigationCommand::Action::NavigateToPose) {
            MotionIntent walk_base =
                makeWalkBase(event.navigation.gait, event.navigation.body_height_m);
            robot.navigationManager()->startNavigateToPose(
                walk_base,
                NavPose2d{
                    event.navigation.goal_x_m,
                    event.navigation.goal_y_m,
                    event.navigation.goal_yaw_rad,
                });
            if (logger_) {
                LOG_INFO(logger_,
                         "Scenario navigation start @",
                         event.at_ms,
                         "ms goal=(",
                         event.navigation.goal_x_m,
                         ",",
                         event.navigation.goal_y_m,
                         ",",
                         event.navigation.goal_yaw_rad,
                         ")");
            }
        } else if (event.navigation.action == ScenarioNavigationCommand::Action::Cancel) {
            robot.navigationManager()->cancel();
            if (logger_) {
                LOG_INFO(logger_, "Scenario navigation cancel @", event.at_ms, "ms");
            }
        }
    }

    return true;
}

bool ScenarioSession::tick(RobotControl& robot, std::chrono::steady_clock::time_point now) {
    if (!active_) {
        return false;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_);
    const uint64_t elapsed_ms = static_cast<uint64_t>(std::max<std::int64_t>(0, elapsed.count()));

    while (event_idx_ < scenario_.events.size() && scenario_.events[event_idx_].at_ms <= elapsed_ms) {
        applyEvent(robot, scenario_.events[event_idx_]);
        ++event_idx_;
    }

    if (!robot.setSimFaultToggles(toggles_)) {
        if (logger_) {
            LOG_ERROR(logger_, "Scenario session requires sim runtime (SimHardwareBridge)");
        }
        failed_ = true;
        finish(robot, true);
        return false;
    }

    if (scenario_.refresh_motion_intent) {
        stampIntentStreamMotionFields(current_intent_);
        current_intent_.timestamp_us = now_us();
        robot.setMotionIntent(current_intent_);
    }

    if (elapsed_ms > scenario_.duration_ms) {
        finish(robot, false);
        return false;
    }
    return true;
}

void ScenarioSession::finish(RobotControl& robot, bool clear_nav) {
    if (!active_ && map_source_ == nullptr) {
        return;
    }
    const bool was_active = active_;
    active_ = false;
    if (clear_nav && robot.navigationManager() != nullptr) {
        robot.navigationManager()->cancel();
    }
    if (map_source_ != nullptr && robot.navigationManager() != nullptr) {
        robot.navigationManager()->clearObservationSources();
        map_source_.reset();
    }
    robot.setSafetyLegEnabledTestMask(std::nullopt);
    toggles_ = {};
    (void)robot.setSimFaultToggles(toggles_);
    if (was_active && logger_) {
        LOG_INFO(logger_, "Scenario session finished: ", scenario_.name);
    }
}

void ScenarioSession::stop(RobotControl& robot) {
    if (!active_ && map_source_ == nullptr) {
        return;
    }
    if (robot.navigationManager() != nullptr) {
        robot.navigationManager()->cancel();
    }
    finish(robot, false);
    MotionIntent idle = makeMotionIntent(RobotMode::SAFE_IDLE, GaitType::TRIPOD, 0.14);
    stampIntentStreamMotionFields(idle);
    robot.setMotionIntent(idle);
}
