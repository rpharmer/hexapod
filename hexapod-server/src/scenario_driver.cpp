#include "scenario_driver.hpp"

#include "logger.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
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

} // namespace

bool ScenarioDriver::loadFromToml(const std::string& path, ScenarioDefinition& out,
                                  std::string& error) {
    try {
        const toml::value root = toml::parse(path, toml::spec::v(1, 1, 0));
        out.name = toml::find_or<std::string>(root, "name", "unnamed");
        out.duration_ms = toml::find_or<uint64_t>(root, "duration_ms", 5000);
        out.tick_ms = std::max<uint64_t>(1, toml::find_or<uint64_t>(root, "tick_ms", 20));
        out.refresh_motion_intent = toml::find_or<bool>(root, "refresh_motion_intent", true);

        out.events.clear();
        const auto events = toml::find_or<std::vector<toml::value>>(root, "events", {});
        out.events.reserve(events.size());

        for (const auto& event_value : events) {
            ScenarioEvent event{};
            event.at_ms = toml::find_or<uint64_t>(event_value, "at_ms", 0);

            const std::string mode = toml::find_or<std::string>(event_value, "mode", "");
            if (!mode.empty()) {
                const auto parsed_mode = parseRobotMode(mode);
                if (!parsed_mode.has_value()) {
                    error = "invalid scenario mode '" + mode + "'";
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
            }

            if (event_value.contains("faults")) {
                event.has_fault_overrides = true;
                const auto& faults = event_value.at("faults");
                event.faults.bus_down = toml::find_or<bool>(faults, "bus_down", false);
                event.faults.low_voltage = toml::find_or<bool>(faults, "low_voltage", false);
                event.faults.low_voltage_value_v =
                    toml::find_or<double>(faults, "low_voltage_value_v", 6.0);
                event.faults.high_current = toml::find_or<bool>(faults, "high_current", false);
                event.faults.high_current_value_a =
                    toml::find_or<double>(faults, "high_current_value_a", 25.0);
            }

            if (event_value.contains("sensors")) {
                event.has_sensor_overrides = true;
                const auto& sensors = event_value.at("sensors");
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

    for (uint64_t elapsed_ms = 0; elapsed_ms <= scenario.duration_ms; elapsed_ms += scenario.tick_ms) {
        while (event_idx < scenario.events.size() && scenario.events[event_idx].at_ms <= elapsed_ms) {
            const ScenarioEvent& event = scenario.events[event_idx];

            if (event.motion.enabled) {
                current_intent = makeMotionIntent(event.motion);
                robot.setMotionIntent(current_intent);
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
