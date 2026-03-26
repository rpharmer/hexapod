#include "scenario_driver.hpp"

#include "logger.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <chrono>
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

bool containsOnlyKeys(const std::set<std::string>& present,
                      const std::set<std::string>& allowed,
                      const std::string& section_name,
                      std::string& error) {
    for (const auto& key : present) {
        if (!allowed.contains(key)) {
            error = "unknown key '" + key + "' in " + section_name;
            return false;
        }
    }
    return true;
}

std::set<std::string> collectKeys(const toml::value& table) {
    std::set<std::string> keys;
    if (!table.is_table()) {
        return keys;
    }
    for (const auto& [key, _] : table.as_table()) {
        keys.insert(key);
    }
    return keys;
}

MotionIntent blendMotionIntent(const MotionIntent& from, const MotionIntent& to, double t) {
    MotionIntent blended = to;
    blended.speed_mps.value = lerp(from.speed_mps.value, to.speed_mps.value, t);
    blended.body_pose_setpoint.body_trans_m.z = lerp(from.body_pose_setpoint.body_trans_m.z, to.body_pose_setpoint.body_trans_m.z, t);
    blended.heading_rad.value = lerpAngleShortest(from.heading_rad.value, to.heading_rad.value, t);
    blended.body_pose_setpoint.orientation_rad.z = lerpAngleShortest(from.body_pose_setpoint.orientation_rad.z, to.body_pose_setpoint.orientation_rad.z, t);
    return blended;
}

bool isRampEligible(const MotionIntent& from, const MotionIntent& to) {
    return from.requested_mode == RobotMode::WALK &&
           to.requested_mode == RobotMode::WALK;
}

namespace parser {

struct DecodedFaults {
    ScenarioFaultOverrides faults{};
    std::set<std::string> keys{};
    bool has_low_voltage_value{false};
    bool has_high_current_value{false};
};

struct DecodedSensors {
    ScenarioSensorOverrides sensors{};
    std::set<std::string> keys{};
    bool has_contacts{false};
    std::size_t contacts_count{0};
};

struct DecodedEvent {
    uint64_t at_ms{0};
    std::set<std::string> keys{};
    std::optional<std::string> mode_name{};
    std::optional<std::string> gait_name{};
    std::optional<double> body_height_m{};
    std::optional<double> speed_mps{};
    std::optional<double> heading_rad{};
    std::optional<double> yaw_rad{};
    std::optional<DecodedFaults> faults{};
    std::optional<DecodedSensors> sensors{};
};

struct DecodedScenario {
    ScenarioDefinition scenario{};
    std::set<std::string> root_keys{};
    std::vector<DecodedEvent> events{};
};

bool decodeFaults(const toml::value& event_value, DecodedEvent& decoded_event) {
    if (!event_value.contains("faults")) {
        return true;
    }

    DecodedFaults decoded_faults{};
    const auto& faults = event_value.at("faults");
    decoded_faults.keys = collectKeys(faults);
    decoded_faults.faults.bus_down = toml::find_or<bool>(faults, "bus_down", false);
    decoded_faults.faults.low_voltage = toml::find_or<bool>(faults, "low_voltage", false);
    decoded_faults.faults.low_voltage_value_v = toml::find_or<double>(faults, "low_voltage_value_v", 6.0);
    decoded_faults.faults.high_current = toml::find_or<bool>(faults, "high_current", false);
    decoded_faults.faults.high_current_value_a = toml::find_or<double>(faults, "high_current_value_a", 25.0);
    decoded_faults.has_low_voltage_value = faults.contains("low_voltage_value_v");
    decoded_faults.has_high_current_value = faults.contains("high_current_value_a");
    decoded_event.faults = decoded_faults;
    return true;
}

bool decodeSensors(const toml::value& event_value, DecodedEvent& decoded_event) {
    if (!event_value.contains("sensors")) {
        return true;
    }

    DecodedSensors decoded_sensors{};
    const auto& sensors = event_value.at("sensors");
    decoded_sensors.keys = collectKeys(sensors);
    decoded_sensors.sensors.clear_contacts = toml::find_or<bool>(sensors, "clear_contacts", false);
    decoded_sensors.has_contacts = sensors.contains("contacts");

    if (!decoded_sensors.sensors.clear_contacts) {
        const auto contacts = toml::find_or<std::vector<bool>>(sensors, "contacts", {});
        decoded_sensors.contacts_count = contacts.size();
        if (contacts.size() == kNumLegs) {
            for (int i = 0; i < kNumLegs; ++i) {
                decoded_sensors.sensors.contacts[static_cast<std::size_t>(i)] =
                    contacts[static_cast<std::size_t>(i)];
            }
        }
    }

    decoded_event.sensors = decoded_sensors;
    return true;
}

bool decodeEvent(const toml::value& event_value, DecodedEvent& out) {
    out.keys = collectKeys(event_value);
    out.at_ms = toml::find_or<uint64_t>(event_value, "at_ms", 0);

    const std::string mode_name = toml::find_or<std::string>(event_value, "mode", "");
    if (!mode_name.empty()) {
        out.mode_name = mode_name;
    }

    if (event_value.contains("gait")) {
        out.gait_name = toml::find_or<std::string>(event_value, "gait", "TRIPOD");
    }
    if (event_value.contains("body_height_m")) {
        out.body_height_m = toml::find_or<double>(event_value, "body_height_m", 0.20);
    }
    if (event_value.contains("speed_mps")) {
        out.speed_mps = toml::find_or<double>(event_value, "speed_mps", 0.0);
    }
    if (event_value.contains("heading_rad")) {
        out.heading_rad = toml::find_or<double>(event_value, "heading_rad", 0.0);
    }
    if (event_value.contains("yaw_rad")) {
        out.yaw_rad = toml::find_or<double>(event_value, "yaw_rad", 0.0);
    }

    return decodeFaults(event_value, out) && decodeSensors(event_value, out);
}

bool decode(const std::string& path, DecodedScenario& out) {
    const toml::value root = toml::parse(path, toml::spec::v(1, 1, 0));
    out.root_keys = collectKeys(root);
    out.scenario.name = toml::find_or<std::string>(root, "name", "unnamed");
    out.scenario.duration_ms = toml::find_or<uint64_t>(root, "duration_ms", 5000);
    out.scenario.tick_ms = std::max<uint64_t>(1, toml::find_or<uint64_t>(root, "tick_ms", 20));
    out.scenario.refresh_motion_intent = toml::find_or<bool>(root, "refresh_motion_intent", true);
    out.scenario.motion_ramp_ms = toml::find_or<uint64_t>(root, "motion_ramp_ms", 0);

    out.events.clear();
    const auto event_values = toml::find_or<std::vector<toml::value>>(root, "events", {});
    out.events.reserve(event_values.size());
    for (const auto& event_value : event_values) {
        DecodedEvent decoded_event{};
        if (!decodeEvent(event_value, decoded_event)) {
            return false;
        }
        out.events.push_back(decoded_event);
    }

    return true;
}

} // namespace parser

namespace validator {

bool validateRootKeys(const parser::DecodedScenario& decoded,
                      ScenarioDriver::ValidationMode mode,
                      std::string& error) {
    if (mode != ScenarioDriver::ValidationMode::Strict) {
        return true;
    }

    return containsOnlyKeys(decoded.root_keys,
                            {"name", "duration_ms", "tick_ms", "refresh_motion_intent", "motion_ramp_ms", "events"},
                            "scenario root", error);
}

bool validateMotion(const parser::DecodedEvent& decoded_event,
                    ScenarioDriver::ValidationMode mode,
                    ScenarioEvent& out_event,
                    std::string& error) {
    const bool has_mode = decoded_event.mode_name.has_value();
    const bool has_motion_without_mode = decoded_event.gait_name.has_value() ||
                                         decoded_event.body_height_m.has_value() ||
                                         decoded_event.speed_mps.has_value() ||
                                         decoded_event.heading_rad.has_value() ||
                                         decoded_event.yaw_rad.has_value();

    if (!has_mode) {
        if (mode == ScenarioDriver::ValidationMode::Strict && has_motion_without_mode) {
            error = "scenario event specifies motion fields without mode";
            return false;
        }
        return true;
    }

    const auto parsed_mode = parseRobotMode(*decoded_event.mode_name);
    if (!parsed_mode.has_value()) {
        error = "invalid scenario mode '" + *decoded_event.mode_name + "'";
        return false;
    }

    const std::string gait_name = decoded_event.gait_name.value_or("TRIPOD");
    const auto parsed_gait = parseGait(gait_name);
    if (!parsed_gait.has_value()) {
        error = "invalid scenario gait '" + gait_name + "'";
        return false;
    }

    out_event.motion.enabled = true;
    out_event.motion.mode = *parsed_mode;
    out_event.motion.gait = *parsed_gait;
    out_event.motion.body_height_m = decoded_event.body_height_m.value_or(0.20);
    out_event.motion.speed_mps = decoded_event.speed_mps.value_or(0.0);
    out_event.motion.heading_rad = decoded_event.heading_rad.value_or(0.0);
    out_event.motion.yaw_rad = decoded_event.yaw_rad.value_or(0.0);
    return true;
}

bool validateFaults(const parser::DecodedEvent& decoded_event,
                    std::size_t event_index,
                    ScenarioDriver::ValidationMode mode,
                    ScenarioEvent& out_event,
                    std::string& error) {
    if (!decoded_event.faults.has_value()) {
        return true;
    }

    const auto& decoded_faults = *decoded_event.faults;
    if (mode == ScenarioDriver::ValidationMode::Strict &&
        !containsOnlyKeys(decoded_faults.keys,
                          {"bus_down", "low_voltage", "low_voltage_value_v", "high_current", "high_current_value_a"},
                          "events[" + std::to_string(event_index) + "].faults", error)) {
        return false;
    }

    out_event.has_fault_overrides = true;
    out_event.faults = decoded_faults.faults;

    if (mode == ScenarioDriver::ValidationMode::Strict) {
        if (out_event.faults.low_voltage_value_v <= 0.0) {
            error = "scenario faults.low_voltage_value_v must be > 0";
            return false;
        }
        if (out_event.faults.high_current_value_a <= 0.0) {
            error = "scenario faults.high_current_value_a must be > 0";
            return false;
        }
        if (decoded_faults.has_low_voltage_value && !out_event.faults.low_voltage) {
            error = "scenario faults.low_voltage_value_v requires low_voltage=true";
            return false;
        }
        if (decoded_faults.has_high_current_value && !out_event.faults.high_current) {
            error = "scenario faults.high_current_value_a requires high_current=true";
            return false;
        }
    }

    return true;
}

bool validateSensors(const parser::DecodedEvent& decoded_event,
                     std::size_t event_index,
                     ScenarioDriver::ValidationMode mode,
                     ScenarioEvent& out_event,
                     std::string& error) {
    if (!decoded_event.sensors.has_value()) {
        return true;
    }

    const auto& decoded_sensors = *decoded_event.sensors;
    if (mode == ScenarioDriver::ValidationMode::Strict &&
        !containsOnlyKeys(decoded_sensors.keys, {"clear_contacts", "contacts"},
                          "events[" + std::to_string(event_index) + "].sensors", error)) {
        return false;
    }

    out_event.has_sensor_overrides = true;
    out_event.sensors = decoded_sensors.sensors;

    if (decoded_sensors.has_contacts && !decoded_sensors.sensors.clear_contacts &&
        decoded_sensors.contacts_count != 0 && decoded_sensors.contacts_count != kNumLegs) {
        error = "scenario sensors.contacts must contain exactly " + std::to_string(kNumLegs) + " values";
        return false;
    }

    if (decoded_sensors.has_contacts && decoded_sensors.sensors.clear_contacts &&
        mode == ScenarioDriver::ValidationMode::Strict) {
        error = "scenario sensors.contacts cannot be provided when clear_contacts=true";
        return false;
    }

    return true;
}

bool validateEventKeys(const parser::DecodedEvent& decoded_event,
                       std::size_t event_index,
                       ScenarioDriver::ValidationMode mode,
                       std::string& error) {
    if (mode != ScenarioDriver::ValidationMode::Strict) {
        return true;
    }

    return containsOnlyKeys(decoded_event.keys,
                            {"at_ms", "mode", "gait", "body_height_m", "speed_mps", "heading_rad", "yaw_rad", "faults", "sensors"},
                            "events[" + std::to_string(event_index) + "]", error);
}

bool buildScenario(const parser::DecodedScenario& decoded,
                   ScenarioDriver::ValidationMode mode,
                   ScenarioDefinition& out,
                   std::string& error) {
    out = decoded.scenario;
    out.events.clear();
    out.events.reserve(decoded.events.size());

    if (!validateRootKeys(decoded, mode, error)) {
        return false;
    }

    for (std::size_t event_index = 0; event_index < decoded.events.size(); ++event_index) {
        const auto& decoded_event = decoded.events[event_index];
        if (!validateEventKeys(decoded_event, event_index, mode, error)) {
            return false;
        }

        ScenarioEvent event{};
        event.at_ms = decoded_event.at_ms;

        if (!validateMotion(decoded_event, mode, event, error)) {
            return false;
        }
        if (!validateFaults(decoded_event, event_index, mode, event, error)) {
            return false;
        }
        if (!validateSensors(decoded_event, event_index, mode, event, error)) {
            return false;
        }

        out.events.push_back(event);
    }

    std::sort(out.events.begin(), out.events.end(),
              [](const ScenarioEvent& a, const ScenarioEvent& b) { return a.at_ms < b.at_ms; });

    return true;
}

} // namespace validator

namespace executor {

class ScenarioExecutor {
public:
    explicit ScenarioExecutor(RobotControl& robot, std::shared_ptr<logging::AsyncLogger> logger)
        : robot_(robot), logger_(std::move(logger)) {
        current_intent_.requested_mode = RobotMode::SAFE_IDLE;
        current_intent_.gait = GaitType::TRIPOD;
    }

    bool run(const ScenarioDefinition& scenario) {
        const auto start = std::chrono::steady_clock::now();
        for (uint64_t elapsed_ms = 0; elapsed_ms <= scenario.duration_ms; elapsed_ms += scenario.tick_ms) {
            applyDueEvents(scenario, elapsed_ms);
            applyRamp(elapsed_ms);

            if (!robot_.setSimFaultToggles(toggles_)) {
                if (logger_) {
                    LOG_ERROR(logger_, "Scenario driver requires sim runtime (SimHardwareBridge)");
                }
                return false;
            }

            if (scenario.refresh_motion_intent) {
                current_intent_.timestamp_us = now_us();
                robot_.setMotionIntent(current_intent_);
            }

            const auto target_time = start + std::chrono::milliseconds(elapsed_ms + scenario.tick_ms);
            std::this_thread::sleep_until(target_time);
        }

        return true;
    }

private:
    void applyDueEvents(const ScenarioDefinition& scenario, uint64_t elapsed_ms) {
        while (event_idx_ < scenario.events.size() && scenario.events[event_idx_].at_ms <= elapsed_ms) {
            const ScenarioEvent& event = scenario.events[event_idx_];
            if (event.motion.enabled) {
                applyMotionEvent(scenario, elapsed_ms, event);
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
            ++event_idx_;
        }
    }

    void applyMotionEvent(const ScenarioDefinition& scenario, uint64_t elapsed_ms, const ScenarioEvent& event) {
        MotionIntent target_intent = makeMotionIntent(event.motion);
        if (scenario.motion_ramp_ms > 0 && isRampEligible(current_intent_, target_intent)) {
            ramp_active_ = true;
            ramp_start_intent_ = current_intent_;
            ramp_target_intent_ = target_intent;
            ramp_start_ms_ = elapsed_ms;
            ramp_end_ms_ = elapsed_ms + scenario.motion_ramp_ms;
        } else {
            current_intent_ = target_intent;
            ramp_active_ = false;
            robot_.setMotionIntent(current_intent_);
        }

        if (logger_) {
            LOG_INFO(logger_, "Scenario event @", event.at_ms, "ms mode update");
        }
    }

    void applyRamp(uint64_t elapsed_ms) {
        if (!ramp_active_) {
            return;
        }

        const uint64_t elapsed_in_ramp_ms = elapsed_ms > ramp_end_ms_
                                                ? (ramp_end_ms_ - ramp_start_ms_)
                                                : (elapsed_ms - ramp_start_ms_);
        const double duration_ms = static_cast<double>(ramp_end_ms_ - ramp_start_ms_);
        const double t = duration_ms > 0.0
                             ? (static_cast<double>(elapsed_in_ramp_ms) / duration_ms)
                             : 1.0;
        current_intent_ = blendMotionIntent(ramp_start_intent_, ramp_target_intent_, t);
        robot_.setMotionIntent(current_intent_);

        if (elapsed_ms >= ramp_end_ms_) {
            current_intent_ = ramp_target_intent_;
            ramp_active_ = false;
        }
    }

    RobotControl& robot_;
    std::shared_ptr<logging::AsyncLogger> logger_;
    SimHardwareFaultToggles toggles_{};
    MotionIntent current_intent_{};
    std::size_t event_idx_{0};
    bool ramp_active_{false};
    MotionIntent ramp_start_intent_{};
    MotionIntent ramp_target_intent_{};
    uint64_t ramp_start_ms_{0};
    uint64_t ramp_end_ms_{0};
};

} // namespace executor

} // namespace

bool ScenarioDriver::loadFromToml(const std::string& path, ScenarioDefinition& out,
                                  std::string& error) {
    return loadFromToml(path, out, error, ValidationMode::Permissive);
}

bool ScenarioDriver::loadFromToml(const std::string& path, ScenarioDefinition& out,
                                  std::string& error, ValidationMode mode) {
    try {
        parser::DecodedScenario decoded{};
        if (!parser::decode(path, decoded)) {
            return false;
        }
        return validator::buildScenario(decoded, mode, out, error);
    } catch (const std::exception& ex) {
        error = ex.what();
        return false;
    }
}

bool ScenarioDriver::run(RobotControl& robot, const ScenarioDefinition& scenario,
                         std::shared_ptr<logging::AsyncLogger> logger) {
    executor::ScenarioExecutor executor(robot, std::move(logger));
    return executor.run(scenario);
}
