#include "scenario_driver.hpp"

#include "local_map.hpp"
#include "navigation_manager.hpp"
#include "logger.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <chrono>
#include <initializer_list>
#include <optional>
#include <set>
#include <thread>
#include <toml.hpp>

namespace {

std::optional<RobotMode> parseRobotMode(const std::string& mode) {
    if (mode == "SAFE_IDLE") {
        return {RobotMode::SAFE_IDLE};
    }
    if (mode == "STAND") {
        return {RobotMode::STAND};
    }
    if (mode == "WALK") {
        return {RobotMode::WALK};
    }
    return std::nullopt;
}

std::optional<GaitType> parseGait(const std::string& gait) {
    if (gait == "TRIPOD") {
        return {GaitType::TRIPOD};
    }
    if (gait == "RIPPLE") {
        return {GaitType::RIPPLE};
    }
    if (gait == "WAVE") {
        return {GaitType::WAVE};
    }
    if (gait == "CRAWL") {
        return {GaitType::CRAWL};
    }
    if (gait == "TURN_IN_PLACE") {
        return {GaitType::TURN_IN_PLACE};
    }
    return std::nullopt;
}

const char* modeName(RobotMode mode) {
    switch (mode) {
    case RobotMode::SAFE_IDLE:
        return "SAFE_IDLE";
    case RobotMode::HOMING:
        return "HOMING";
    case RobotMode::STAND:
        return "STAND";
    case RobotMode::WALK:
        return "WALK";
    case RobotMode::FAULT:
        return "FAULT";
    }
    return "UNKNOWN";
}

const char* gaitName(GaitType gait) {
    switch (gait) {
    case GaitType::TRIPOD:
        return "TRIPOD";
    case GaitType::RIPPLE:
        return "RIPPLE";
    case GaitType::WAVE:
        return "WAVE";
    case GaitType::CRAWL:
        return "CRAWL";
    case GaitType::TURN_IN_PLACE:
        return "TURN_IN_PLACE";
    }
    return "UNKNOWN";
}

std::optional<ScenarioNavigationCommand::Action> parseNavigationAction(const std::string& action) {
    if (action == "navigate_to_pose") {
        return ScenarioNavigationCommand::Action::NavigateToPose;
    }
    if (action == "cancel") {
        return ScenarioNavigationCommand::Action::Cancel;
    }
    return std::nullopt;
}

std::optional<LocalMapCellState> parseLocalMapCellState(const std::string& state) {
    if (state == "occupied") {
        return LocalMapCellState::Occupied;
    }
    if (state == "free") {
        return LocalMapCellState::Free;
    }
    if (state == "unknown") {
        return LocalMapCellState::Unknown;
    }
    return std::nullopt;
}

bool containsOnlyKeys(const toml::value& table, const std::set<std::string>& allowed,
                      const std::string& section_name, std::string& error);

bool parseMapObservationSamples(const toml::value& parent,
                                const std::string& key,
                                std::vector<LocalMapObservationSample>& out,
                                std::string& error,
                                ScenarioDriver::ValidationMode mode,
                                const std::string& section_name) {
    out.clear();
    if (!parent.contains(key)) {
        return true;
    }

    const auto entries = toml::find_or<std::vector<toml::value>>(parent, key, {});
    out.reserve(entries.size());
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const toml::value& entry = entries[i];
        if (mode == ScenarioDriver::ValidationMode::Strict &&
            !containsOnlyKeys(entry, {"x_m", "y_m", "state", "z_m"},
                              section_name + "." + key + "[" + std::to_string(i) + "]", error)) {
            return false;
        }
        if (!entry.contains("x_m") || !entry.contains("y_m")) {
            error = section_name + "." + key + "[" + std::to_string(i) + "] requires x_m and y_m";
            return false;
        }

        const std::string state_name = toml::find_or<std::string>(entry, "state", "occupied");
        const auto parsed_state = parseLocalMapCellState(state_name);
        if (!parsed_state.has_value()) {
            error = "invalid map obstacle state '" + state_name + "'";
            return false;
        }
        LocalMapObservationSample sample{
            toml::find<double>(entry, "x_m"),
            toml::find<double>(entry, "y_m"),
            *parsed_state,
        };
        if (entry.contains("z_m")) {
            sample.z_m = toml::find<double>(entry, "z_m");
        }
        out.push_back(std::move(sample));
    }
    return true;
}

std::optional<std::array<bool, kNumLegs>> buildContacts(const ScenarioSensorOverrides& sensors) {
    if (sensors.clear_contacts) {
        return std::nullopt;
    }
    return sensors.contacts;
}

bool containsOnlyKeys(const toml::value& table, const std::set<std::string>& allowed,
                      const std::string& section_name, std::string& error) {
    if (!table.is_table()) {
        return true;
    }

    for (const auto& [key, _] : table.as_table()) {
        if (!allowed.contains(key)) {
            error = "unknown key '" + key + "' in " + section_name;
            return false;
        }
    }
    return true;
}

} // namespace

bool ScenarioDriver::loadFromToml(const std::string& path, ScenarioDefinition& out,
                                  std::string& error) {
    return loadFromToml(path, out, error, ValidationMode::Permissive);
}

bool ScenarioDriver::loadFromToml(const std::string& path, ScenarioDefinition& out,
                                  std::string& error, ValidationMode mode) {
    try {
        const toml::value root = toml::parse(path, toml::spec::v(1, 1, 0));

        if (mode == ValidationMode::Strict &&
            !containsOnlyKeys(root, {"name", "duration_ms", "tick_ms", "refresh_motion_intent", "map_obstacles", "events"},
                              "scenario root", error)) {
            return false;
        }

        out.name = toml::find_or<std::string>(root, "name", "unnamed");
        out.duration_ms = toml::find_or<uint64_t>(root, "duration_ms", 5000);
        out.tick_ms = std::max<uint64_t>(1, toml::find_or<uint64_t>(root, "tick_ms", 20));
        out.refresh_motion_intent = toml::find_or<bool>(root, "refresh_motion_intent", true);
        if (!parseMapObservationSamples(root, "map_obstacles", out.initial_map_obstacles, error, mode, "scenario root")) {
            return false;
        }

        out.events.clear();
        const auto events = toml::find_or<std::vector<toml::value>>(root, "events", {});
        out.events.reserve(events.size());

        for (std::size_t event_index = 0; event_index < events.size(); ++event_index) {
            const auto& event_value = events[event_index];
            ScenarioEvent event{};

            if (mode == ValidationMode::Strict &&
                !containsOnlyKeys(event_value,
                                  {"at_ms", "mode", "gait", "body_height_m", "speed_mps", "heading_rad", "yaw_rad",
                                   "yaw_rate_radps", "vx_mps", "vy_mps", "faults", "sensors", "navigation",
                                   "map_obstacles"},
                                  "events[" + std::to_string(event_index) + "]", error)) {
                return false;
            }

            event.at_ms = toml::find_or<uint64_t>(event_value, "at_ms", 0);

            const std::string mode_name = toml::find_or<std::string>(event_value, "mode", "");
            if (!mode_name.empty()) {
                const auto parsed_mode = parseRobotMode(mode_name);
                if (!parsed_mode.has_value()) {
                    error = "invalid scenario mode '" + mode_name + "'";
                    return false;
                }

                const std::string gait = toml::find_or<std::string>(event_value, "gait", "TRIPOD");
                const auto parsed_gait = parseGait(gait);
                if (!parsed_gait.has_value()) {
                    error = "invalid scenario gait '" + gait + "'";
                    return false;
                }

                event.motion.enabled = true;
                event.motion.mode = *parsed_mode;
                event.motion.gait = *parsed_gait;
                event.motion.body_height_m = toml::find_or<double>(event_value, "body_height_m", 0.14);
                event.motion.speed_mps = toml::find_or<double>(event_value, "speed_mps", 0.0);
                event.motion.heading_rad = toml::find_or<double>(event_value, "heading_rad", 0.0);
                event.motion.yaw_rad = toml::find_or<double>(event_value, "yaw_rad", 0.0);
                event.motion.yaw_rate_radps = toml::find_or<double>(event_value, "yaw_rate_radps", 0.0);
                if (event_value.contains("vx_mps") || event_value.contains("vy_mps")) {
                    event.motion.has_direct_velocity = true;
                    event.motion.vx_mps = toml::find_or<double>(event_value, "vx_mps", 0.0);
                    event.motion.vy_mps = toml::find_or<double>(event_value, "vy_mps", 0.0);
                }
            } else if (mode == ValidationMode::Strict &&
                       (event_value.contains("gait") || event_value.contains("body_height_m") ||
                        event_value.contains("speed_mps") || event_value.contains("heading_rad") ||
                        event_value.contains("yaw_rad") || event_value.contains("yaw_rate_radps") ||
                        event_value.contains("vx_mps") || event_value.contains("vy_mps"))) {
                error = "scenario event specifies motion fields without mode";
                return false;
            }

            if (event_value.contains("faults")) {
                event.has_fault_overrides = true;
                const auto& faults = event_value.at("faults");
                if (mode == ValidationMode::Strict &&
                    !containsOnlyKeys(faults,
                                      {"bus_down", "low_voltage", "low_voltage_value_v", "high_current", "high_current_value_a"},
                                      "events[" + std::to_string(event_index) + "].faults", error)) {
                    return false;
                }
                event.faults.bus_down = toml::find_or<bool>(faults, "bus_down", false);
                event.faults.low_voltage = toml::find_or<bool>(faults, "low_voltage", false);
                event.faults.low_voltage_value_v =
                    toml::find_or<double>(faults, "low_voltage_value_v", 6.0);
                event.faults.high_current = toml::find_or<bool>(faults, "high_current", false);
                event.faults.high_current_value_a =
                    toml::find_or<double>(faults, "high_current_value_a", 25.0);

                if (mode == ValidationMode::Strict) {
                    if (event.faults.low_voltage_value_v <= 0.0) {
                        error = "scenario faults.low_voltage_value_v must be > 0";
                        return false;
                    }
                    if (event.faults.high_current_value_a <= 0.0) {
                        error = "scenario faults.high_current_value_a must be > 0";
                        return false;
                    }
                    if (event_value.at("faults").contains("low_voltage_value_v") && !event.faults.low_voltage) {
                        error = "scenario faults.low_voltage_value_v requires low_voltage=true";
                        return false;
                    }
                    if (event_value.at("faults").contains("high_current_value_a") && !event.faults.high_current) {
                        error = "scenario faults.high_current_value_a requires high_current=true";
                        return false;
                    }
                }
            }

            if (event_value.contains("sensors")) {
                event.has_sensor_overrides = true;
                const auto& sensors = event_value.at("sensors");
                if (mode == ValidationMode::Strict &&
                    !containsOnlyKeys(sensors, {"clear_contacts", "contacts"},
                                      "events[" + std::to_string(event_index) + "].sensors", error)) {
                    return false;
                }
                event.sensors.clear_contacts = toml::find_or<bool>(sensors, "clear_contacts", false);
                if (!event.sensors.clear_contacts) {
                    const auto contacts = toml::find_or<std::vector<bool>>(sensors, "contacts", {});
                    if (!contacts.empty() && contacts.size() != kNumLegs) {
                        error = "scenario sensors.contacts must contain exactly " + std::to_string(kNumLegs) + " values";
                        return false;
                    }
                    if (contacts.size() == kNumLegs) {
                        for (int i = 0; i < kNumLegs; ++i) {
                            event.sensors.contacts[static_cast<std::size_t>(i)] = contacts[static_cast<std::size_t>(i)];
                        }
                    }
                } else if (mode == ValidationMode::Strict && sensors.contains("contacts")) {
                    error = "scenario sensors.contacts cannot be provided when clear_contacts=true";
                    return false;
                }
            }

            if (event_value.contains("navigation")) {
                event.has_navigation_command = true;
                const auto& navigation = event_value.at("navigation");
                if (mode == ValidationMode::Strict &&
                    !containsOnlyKeys(navigation,
                                      {"action", "goal_x_m", "goal_y_m", "goal_yaw_rad", "gait", "body_height_m"},
                                      "events[" + std::to_string(event_index) + "].navigation", error)) {
                    return false;
                }

                const std::string action_name = toml::find_or<std::string>(navigation, "action", "");
                const auto action = parseNavigationAction(action_name);
                if (!action.has_value()) {
                    error = "invalid navigation action '" + action_name + "'";
                    return false;
                }
                const std::string navigation_gait_name = toml::find_or<std::string>(navigation, "gait", "TRIPOD");
                const auto navigation_gait = parseGait(navigation_gait_name);
                if (!navigation_gait.has_value()) {
                    error = "invalid scenario gait '" + navigation_gait_name + "'";
                    return false;
                }
                event.navigation.enabled = true;
                event.navigation.action = *action;
                event.navigation.gait = *navigation_gait;
                event.navigation.body_height_m = toml::find_or<double>(navigation, "body_height_m", 0.14);
                event.navigation.goal_x_m = toml::find_or<double>(navigation, "goal_x_m", 0.0);
                event.navigation.goal_y_m = toml::find_or<double>(navigation, "goal_y_m", 0.0);
                event.navigation.goal_yaw_rad = toml::find_or<double>(navigation, "goal_yaw_rad", 0.0);

                if (mode == ValidationMode::Strict &&
                    event.navigation.action == ScenarioNavigationCommand::Action::NavigateToPose &&
                    (!navigation.contains("goal_x_m") || !navigation.contains("goal_y_m"))) {
                    error = "navigate_to_pose requires goal_x_m and goal_y_m";
                    return false;
                }
            }

            if (event_value.contains("map_obstacles")) {
                event.has_map_observation_override = true;
                if (!parseMapObservationSamples(event_value,
                                                "map_obstacles",
                                                event.map_observation.samples,
                                                error,
                                                mode,
                                                "events[" + std::to_string(event_index) + "]")) {
                    return false;
                }
            }

            out.events.push_back(event);
        }

        std::sort(out.events.begin(), out.events.end(),
                  [](const ScenarioEvent& a, const ScenarioEvent& b) { return a.at_ms < b.at_ms; });

        return true;
    } catch (const std::exception& ex) {
        error = ex.what();
        return false;
    }
}

bool ScenarioDriver::run(RobotControl& robot, const ScenarioDefinition& scenario,
                         std::shared_ptr<logging::AsyncLogger> logger) {
    SimHardwareFaultToggles toggles{};
    MotionIntent current_intent{};
    current_intent.requested_mode = RobotMode::SAFE_IDLE;
    current_intent.gait = GaitType::TRIPOD;

    const bool scenario_uses_navigation =
        !scenario.initial_map_obstacles.empty() ||
        std::any_of(scenario.events.begin(), scenario.events.end(), [](const ScenarioEvent& event) {
            return event.has_navigation_command || event.has_map_observation_override;
        });

    std::shared_ptr<SyntheticLocalMapObservationSource> map_source{};
    if (scenario_uses_navigation) {
        if (robot.navigationManager() == nullptr) {
            if (logger) {
                LOG_ERROR(logger, "Scenario navigation requested but NavigationManager is not installed");
            }
            return false;
        }
        map_source = std::make_shared<SyntheticLocalMapObservationSource>();
        map_source->setStaticSamples(scenario.initial_map_obstacles);
        robot.navigationManager()->addObservationSource(map_source);
    }

    std::size_t event_idx = 0;
    const auto start = std::chrono::steady_clock::now();

    for (uint64_t elapsed_ms = 0; elapsed_ms <= scenario.duration_ms; elapsed_ms += scenario.tick_ms) {
        while (event_idx < scenario.events.size() && scenario.events[event_idx].at_ms <= elapsed_ms) {
            const ScenarioEvent& event = scenario.events[event_idx];

            if (event.motion.enabled) {
                current_intent = makeMotionIntent(event.motion);
                robot.setMotionIntent(current_intent);
                if (logger) {
                    LOG_DEBUG(logger,
                              "Scenario event @", event.at_ms, "ms mode update",
                              " mode=", modeName(event.motion.mode),
                              " gait=", gaitName(event.motion.gait),
                              " body_height_m=", event.motion.body_height_m);
                }
            }

            if (event.has_sensor_overrides) {
                toggles.forced_contacts = buildContacts(event.sensors);
            }

            if (event.has_fault_overrides) {
                toggles.drop_bus = event.faults.bus_down;
                toggles.low_voltage = event.faults.low_voltage;
                toggles.low_voltage_value = event.faults.low_voltage_value_v;
                toggles.high_current = event.faults.high_current;
                toggles.high_current_value = event.faults.high_current_value_a;
            }

            if (event.has_map_observation_override && map_source != nullptr) {
                map_source->setStaticSamples(event.map_observation.samples);
            }

            if (event.has_navigation_command && robot.navigationManager() != nullptr) {
                if (event.navigation.action == ScenarioNavigationCommand::Action::NavigateToPose) {
                    MotionIntent walk_base = makeMotionIntent(RobotMode::WALK,
                                                              event.navigation.gait,
                                                              event.navigation.body_height_m);
                    robot.navigationManager()->startNavigateToPose(
                        walk_base,
                        NavPose2d{
                            event.navigation.goal_x_m,
                            event.navigation.goal_y_m,
                            event.navigation.goal_yaw_rad,
                        });
                    if (logger) {
                        LOG_INFO(logger,
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
                    if (logger) {
                        LOG_INFO(logger, "Scenario navigation cancel @", event.at_ms, "ms");
                    }
                }
            }

            ++event_idx;
        }

        if (!robot.setSimFaultToggles(toggles)) {
            if (logger) {
                LOG_ERROR(logger, "Scenario driver requires sim runtime (SimHardwareBridge)");
            }
            return false;
        }

        if (scenario.refresh_motion_intent) {
            current_intent.timestamp_us = now_us();
            robot.setMotionIntent(current_intent);
        }

        const auto target_time = start + std::chrono::milliseconds(elapsed_ms + scenario.tick_ms);
        std::this_thread::sleep_until(target_time);
    }

    return true;
}
