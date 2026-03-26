#include "scenario_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool writeScenarioFile(const std::string& path, const std::string& content) {
    std::ofstream out(path);
    out << content;
    return out.good();
}

bool test_invalid_mode_rejected() {
    const std::string path = "/tmp/scenario_invalid_mode.toml";
    if (!writeScenarioFile(path, R"(
name = "invalid-mode"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, mode = "FLY", gait = "TRIPOD", body_height_m = 0.2 }
]
)")) {
        return false;
    }

    ScenarioDefinition scenario{};
    std::string error;
    const bool ok = ScenarioDriver::loadFromToml(path, scenario, error);
    std::remove(path.c_str());
    return expect(!ok && error.find("invalid scenario mode") != std::string::npos,
                  "invalid mode should fail with explicit error");
}

bool test_invalid_contacts_size_rejected() {
    const std::string path = "/tmp/scenario_invalid_contacts.toml";
    if (!writeScenarioFile(path, R"(
name = "invalid-contacts"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, mode = "WALK", gait = "TRIPOD", sensors = { clear_contacts = false, contacts = [true, false] } }
]
)")) {
        return false;
    }

    ScenarioDefinition scenario{};
    std::string error;
    const bool ok = ScenarioDriver::loadFromToml(path, scenario, error);
    std::remove(path.c_str());
    return expect(!ok && error.find("sensors.contacts") != std::string::npos,
                  "invalid contacts length should fail with explicit error");
}

bool test_unknown_key_permissive_vs_strict() {
    const std::string path = "/tmp/scenario_unknown_key.toml";
    if (!writeScenarioFile(path, R"(
name = "unknown-key"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, mode = "WALK", gait = "TRIPOD", typo_key = 1 }
]
)")) {
        return false;
    }

    ScenarioDefinition scenario{};
    std::string error;
    const bool permissive_ok =
        ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Permissive);
    const bool strict_ok =
        ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Strict);
    std::remove(path.c_str());

    return expect(permissive_ok, "permissive mode should accept unknown keys") &&
           expect(!strict_ok && error.find("unknown key") != std::string::npos,
                  "strict mode should reject unknown keys");
}

bool test_invalid_combination_permissive_vs_strict() {
    struct ValidationCase {
        std::string name;
        std::string toml;
        bool expect_permissive_ok;
        std::string expected_strict_error_substring;
    };

    const std::vector<ValidationCase> cases{
        {
            "motion-fields-without-mode",
            R"(
name = "invalid-combo-motion-fields-without-mode"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, gait = "TRIPOD", body_height_m = 0.25 }
]
)",
            true,
            "without mode",
        },
        {
            "clear-contacts-with-contacts",
            R"(
name = "invalid-combo-clear-contacts"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, sensors = { clear_contacts = true, contacts = [true, false, true, false, true, false] } }
]
)",
            true,
            "contacts cannot be provided when clear_contacts=true",
        },
        {
            "low-voltage-value-without-flag",
            R"(
name = "invalid-combo-low-voltage-flag"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, faults = { low_voltage = false, low_voltage_value_v = 5.5 } }
]
)",
            true,
            "requires low_voltage=true",
        },
        {
            "high-current-value-without-flag",
            R"(
name = "invalid-combo-high-current-flag"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, faults = { high_current = false, high_current_value_a = 12.0 } }
]
)",
            true,
            "requires high_current=true",
        },
    };

    for (const auto& test_case : cases) {
        const std::string path = "/tmp/scenario_invalid_combo_" + test_case.name + ".toml";
        if (!writeScenarioFile(path, test_case.toml)) {
            return false;
        }

        ScenarioDefinition scenario{};
        std::string error;
        const bool permissive_ok =
            ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Permissive);
        const bool strict_ok =
            ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Strict);
        std::remove(path.c_str());

        if (!expect(permissive_ok == test_case.expect_permissive_ok,
                    ("permissive result mismatch for " + test_case.name).c_str())) {
            return false;
        }
        if (!expect(!strict_ok && error.find(test_case.expected_strict_error_substring) != std::string::npos,
                    ("strict error mismatch for " + test_case.name).c_str())) {
            return false;
        }
    }

    return true;
}

bool test_strict_invalid_gait_rejected() {
    const std::string path = "/tmp/scenario_invalid_gait_strict.toml";
    if (!writeScenarioFile(path, R"(
name = "invalid-gait-strict"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, mode = "WALK", gait = "HOPPING" }
]
)")) {
        return false;
    }

    ScenarioDefinition scenario{};
    std::string error;
    const bool ok =
        ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Strict);
    std::remove(path.c_str());
    return expect(!ok && error.find("invalid scenario gait") != std::string::npos,
                  "strict mode should reject invalid gait");
}

bool test_strict_low_voltage_value_non_positive_rejected() {
    const std::string path = "/tmp/scenario_low_voltage_non_positive_strict.toml";
    if (!writeScenarioFile(path, R"(
name = "low-voltage-non-positive-strict"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, faults = { low_voltage = true, low_voltage_value_v = 0.0 } }
]
)")) {
        return false;
    }

    ScenarioDefinition scenario{};
    std::string error;
    const bool ok =
        ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Strict);
    std::remove(path.c_str());
    return expect(!ok && error.find("faults.low_voltage_value_v must be > 0") != std::string::npos,
                  "strict mode should reject non-positive low_voltage_value_v");
}

bool test_strict_high_current_value_non_positive_rejected() {
    const std::string path = "/tmp/scenario_high_current_non_positive_strict.toml";
    if (!writeScenarioFile(path, R"(
name = "high-current-non-positive-strict"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, faults = { high_current = true, high_current_value_a = -1.0 } }
]
)")) {
        return false;
    }

    ScenarioDefinition scenario{};
    std::string error;
    const bool ok =
        ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Strict);
    std::remove(path.c_str());
    return expect(!ok && error.find("faults.high_current_value_a must be > 0") != std::string::npos,
                  "strict mode should reject non-positive high_current_value_a");
}

bool test_strict_error_messages_table_driven() {
    struct StrictErrorCase {
        std::string name;
        std::string toml;
        std::string expected_error;
    };

    const std::vector<StrictErrorCase> cases{
        {
            "unknown-root-key",
            R"(
name = "unknown-root-key"
duration_ms = 100
tick_ms = 20
extra_root_key = true
events = []
)",
            "unknown key 'extra_root_key' in scenario root",
        },
        {
            "unknown-event-key",
            R"(
name = "unknown-event-key"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, mode = "WALK", event_typo = 1 }
]
)",
            "unknown key 'event_typo' in events[0]",
        },
        {
            "unknown-fault-key",
            R"(
name = "unknown-fault-key"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, faults = { low_voltage = true, typo_fault = 1 } }
]
)",
            "unknown key 'typo_fault' in events[0].faults",
        },
        {
            "unknown-sensor-key",
            R"(
name = "unknown-sensor-key"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, sensors = { clear_contacts = false, typo_sensor = true } }
]
)",
            "unknown key 'typo_sensor' in events[0].sensors",
        },
    };

    for (const auto& test_case : cases) {
        const std::string path = "/tmp/scenario_strict_error_" + test_case.name + ".toml";
        if (!writeScenarioFile(path, test_case.toml)) {
            return false;
        }

        ScenarioDefinition scenario{};
        std::string error;
        const bool ok =
            ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Strict);
        std::remove(path.c_str());

        if (!expect(!ok, ("strict mode should fail for " + test_case.name).c_str())) {
            return false;
        }
        if (!expect(error == test_case.expected_error,
                    ("strict error should match exactly for " + test_case.name).c_str())) {
            std::cerr << "expected: " << test_case.expected_error << "\nactual:   " << error << '\n';
            return false;
        }
    }

    return true;
}



bool test_dynamic_turn_priority_safety_scenario_loads_strict() {
    namespace fs = std::filesystem;
    const fs::path test_file = fs::path(__FILE__).lexically_normal();
    const fs::path scenario_path = test_file.parent_path().parent_path() / "scenarios" / "06_dynamic_turn_priority_safety.toml";

    ScenarioDefinition scenario{};
    std::string error;
    const bool ok = ScenarioDriver::loadFromToml(scenario_path.string(), scenario, error,
                                                 ScenarioDriver::ValidationMode::Strict);

    if (!expect(ok, "dynamic turn/priority/safety scenario should parse in strict mode")) {
        std::cerr << "strict parse error: " << error << '\n';
        return false;
    }

    bool saw_motion = false;
    bool saw_fault = false;
    bool saw_sensor_override = false;
    for (const auto& event : scenario.events) {
        saw_motion = saw_motion || event.motion.enabled;
        saw_fault = saw_fault || event.has_fault_overrides;
        saw_sensor_override = saw_sensor_override || event.has_sensor_overrides;
    }

    return expect(saw_motion, "scenario should include motion events for gait/turn transitions") &&
           expect(saw_fault, "scenario should include safety fault overrides") &&
           expect(saw_sensor_override, "scenario should include contact/sensor overrides");
}
} // namespace

int main() {
    if (!test_invalid_mode_rejected()) {
        return EXIT_FAILURE;
    }
    if (!test_invalid_contacts_size_rejected()) {
        return EXIT_FAILURE;
    }
    if (!test_unknown_key_permissive_vs_strict()) {
        return EXIT_FAILURE;
    }
    if (!test_invalid_combination_permissive_vs_strict()) {
        return EXIT_FAILURE;
    }
    if (!test_strict_invalid_gait_rejected()) {
        return EXIT_FAILURE;
    }
    if (!test_strict_low_voltage_value_non_positive_rejected()) {
        return EXIT_FAILURE;
    }
    if (!test_strict_high_current_value_non_positive_rejected()) {
        return EXIT_FAILURE;
    }
    if (!test_strict_error_messages_table_driven()) {
        return EXIT_FAILURE;
    }
    if (!test_dynamic_turn_priority_safety_scenario_loads_strict()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
