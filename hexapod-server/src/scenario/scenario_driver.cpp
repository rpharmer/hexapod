#include "scenario_driver.hpp"

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
    return std::nullopt;
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

MotionIntent blendMotionIntent(const MotionIntent& from, const MotionIntent& to, double t) {
    MotionIntent blended = to;
    blended.speed_mps.value = lerp(from.speed_mps.value, to.speed_mps.value, t);
    blended.twist.body_trans_m.z = lerp(from.twist.body_trans_m.z, to.twist.body_trans_m.z, t);
    blended.heading_rad.value = lerpAngleShortest(from.heading_rad.value, to.heading_rad.value, t);
    blended.twist.twist_pos_rad.z = lerpAngleShortest(from.twist.twist_pos_rad.z, to.twist.twist_pos_rad.z, t);
    return blended;
}

bool isRampEligible(const MotionIntent& from, const MotionIntent& to) {
    return from.requested_mode == RobotMode::WALK &&
           to.requested_mode == RobotMode::WALK;
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
            !containsOnlyKeys(root, {"name", "duration_ms", "tick_ms", "refresh_motion_intent", "motion_ramp_ms", "events"},
                              "scenario root", error)) {
            return false;
        }

        out.name = toml::find_or<std::string>(root, "name", "unnamed");
        out.duration_ms = toml::find_or<uint64_t>(root, "duration_ms", 5000);
        out.tick_ms = std::max<uint64_t>(1, toml::find_or<uint64_t>(root, "tick_ms", 20));
        out.refresh_motion_intent = toml::find_or<bool>(root, "refresh_motion_intent", true);
        out.motion_ramp_ms = toml::find_or<uint64_t>(root, "motion_ramp_ms", 0);

        out.events.clear();
        const auto events = toml::find_or<std::vector<toml::value>>(root, "events", {});
        out.events.reserve(events.size());

        for (std::size_t event_index = 0; event_index < events.size(); ++event_index) {
            const auto& event_value = events[event_index];
            ScenarioEvent event{};

            if (mode == ValidationMode::Strict &&
                !containsOnlyKeys(event_value,
                                  {"at_ms", "mode", "gait", "body_height_m", "speed_mps", "heading_rad", "yaw_rad", "faults", "sensors"},
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
                event.motion.body_height_m = toml::find_or<double>(event_value, "body_height_m", 0.20);
                event.motion.speed_mps = toml::find_or<double>(event_value, "speed_mps", 0.0);
                event.motion.heading_rad = toml::find_or<double>(event_value, "heading_rad", 0.0);
                event.motion.yaw_rad = toml::find_or<double>(event_value, "yaw_rad", 0.0);
            } else if (mode == ValidationMode::Strict &&
                       (event_value.contains("gait") || event_value.contains("body_height_m") ||
                        event_value.contains("speed_mps") || event_value.contains("heading_rad") ||
                        event_value.contains("yaw_rad"))) {
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

    std::size_t event_idx = 0;
    const auto start = std::chrono::steady_clock::now();
    bool ramp_active = false;
    MotionIntent ramp_start_intent{};
    MotionIntent ramp_target_intent{};
    uint64_t ramp_start_ms = 0;
    uint64_t ramp_end_ms = 0;

    for (uint64_t elapsed_ms = 0; elapsed_ms <= scenario.duration_ms; elapsed_ms += scenario.tick_ms) {
        while (event_idx < scenario.events.size() && scenario.events[event_idx].at_ms <= elapsed_ms) {
            const ScenarioEvent& event = scenario.events[event_idx];

            if (event.motion.enabled) {
                MotionIntent target_intent = makeMotionIntent(event.motion);
                if (scenario.motion_ramp_ms > 0 && isRampEligible(current_intent, target_intent)) {
                    ramp_active = true;
                    ramp_start_intent = current_intent;
                    ramp_target_intent = target_intent;
                    ramp_start_ms = elapsed_ms;
                    ramp_end_ms = elapsed_ms + scenario.motion_ramp_ms;
                } else {
                    current_intent = target_intent;
                    ramp_active = false;
                    robot.setMotionIntent(current_intent);
                }
                if (logger) {
                    LOG_INFO(logger, "Scenario event @", event.at_ms, "ms mode update");
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

            ++event_idx;
        }

        if (ramp_active) {
            const uint64_t elapsed_in_ramp_ms = elapsed_ms > ramp_end_ms ? (ramp_end_ms - ramp_start_ms)
                                                                          : (elapsed_ms - ramp_start_ms);
            const double duration_ms = static_cast<double>(ramp_end_ms - ramp_start_ms);
            const double t = duration_ms > 0.0
                                 ? (static_cast<double>(elapsed_in_ramp_ms) / duration_ms)
                                 : 1.0;
            current_intent = blendMotionIntent(ramp_start_intent, ramp_target_intent, t);
            robot.setMotionIntent(current_intent);
            if (elapsed_ms >= ramp_end_ms) {
                current_intent = ramp_target_intent;
                ramp_active = false;
            }
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
