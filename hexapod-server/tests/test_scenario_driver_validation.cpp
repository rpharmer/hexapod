#include "scenario_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

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
    const std::string path = "/tmp/scenario_invalid_combo.toml";
    if (!writeScenarioFile(path, R"(
name = "invalid-combo"
duration_ms = 100
tick_ms = 20
events = [
  { at_ms = 0, gait = "TRIPOD", body_height_m = 0.25 }
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

    return expect(permissive_ok, "permissive mode should allow partial motion fields") &&
           expect(!strict_ok && error.find("without mode") != std::string::npos,
                  "strict mode should reject gait/body_height without mode");
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
    return EXIT_SUCCESS;
}
