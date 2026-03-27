#include "autonomy/modules/autonomy_stack.hpp"
#include "scenario_driver.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

std::string join(const std::vector<std::string>& values) {
    std::string out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            out += " -> ";
        }
        out += values[i];
    }
    return out;
}

autonomy::WaypointMission makeMission() {
    autonomy::WaypointMission mission{};
    mission.mission_id = "scenario-autonomy";
    mission.waypoints = {
        autonomy::Waypoint{.frame_id = "map", .x_m = 0.0, .y_m = 0.0, .yaw_rad = 0.0},
        autonomy::Waypoint{.frame_id = "map", .x_m = 1.0, .y_m = 0.0, .yaw_rad = 0.0},
    };
    return mission;
}

std::string missionStateToString(autonomy::MissionState state) {
    switch (state) {
    case autonomy::MissionState::Idle:
        return "IDLE";
    case autonomy::MissionState::Ready:
        return "READY";
    case autonomy::MissionState::Exec:
        return "EXEC";
    case autonomy::MissionState::Paused:
        return "PAUSED";
    case autonomy::MissionState::Aborted:
        return "ABORTED";
    case autonomy::MissionState::Complete:
        return "COMPLETE";
    }
    return "UNKNOWN";
}

std::string recoveryActionToString(autonomy::RecoveryAction action) {
    switch (action) {
    case autonomy::RecoveryAction::None:
        return "NONE";
    case autonomy::RecoveryAction::Hold:
        return "HOLD";
    case autonomy::RecoveryAction::Retry:
        return "RETRY";
    case autonomy::RecoveryAction::Replan:
        return "REPLAN";
    case autonomy::RecoveryAction::Abort:
        return "ABORT";
    }
    return "UNKNOWN";
}

bool isFullyBlockedContacts(const ScenarioSensorOverrides& sensors) {
    for (bool in_contact : sensors.contacts) {
        if (in_contact) {
            return false;
        }
    }
    return true;
}

bool runScenarioAndCheckOutcomes(const std::filesystem::path& scenario_path,
                                 uint64_t retry_budget) {
    ScenarioDefinition scenario{};
    std::string error;
    if (!expect(ScenarioDriver::loadFromToml(scenario_path.string(), scenario, error,
                                             ScenarioDriver::ValidationMode::Strict),
                "strict parse should succeed for " + scenario_path.filename().string() +
                    " error=" + error)) {
        return false;
    }
    if (!expect(scenario.expected.enabled,
                "scenario should declare [expected] signatures: " + scenario_path.filename().string())) {
        return false;
    }

    autonomy::AutonomyStack stack(autonomy::AutonomyStackConfig{
        .no_progress_timeout_ms = 1000,
        .recovery_retry_budget = retry_budget,
    });
    if (!expect(stack.init(), "stack init should succeed")) {
        return false;
    }
    if (!expect(stack.start(), "stack start should succeed")) {
        return false;
    }
    if (!expect(stack.loadMission(makeMission()).accepted, "load mission should succeed")) {
        return false;
    }
    if (!expect(stack.startMission().accepted, "start mission should succeed")) {
        return false;
    }

    std::size_t event_idx = 0;
    bool blocked = false;
    bool saw_locomotion_gate = false;
    std::vector<std::string> state_signature{"EXEC"};
    std::vector<std::string> recovery_signature{};

    for (uint64_t elapsed_ms = 0; elapsed_ms <= scenario.duration_ms; elapsed_ms += scenario.tick_ms) {
        while (event_idx < scenario.events.size() && scenario.events[event_idx].at_ms <= elapsed_ms) {
            const ScenarioEvent& event = scenario.events[event_idx];
            if (event.has_sensor_overrides) {
                blocked = !event.sensors.clear_contacts && isFullyBlockedContacts(event.sensors);
            }
            if (event.has_fault_overrides && event.faults.bus_down) {
                blocked = true;
            }
            ++event_idx;
        }

        autonomy::AutonomyStepOutput output{};
        if (!expect(stack.step(autonomy::AutonomyStepInput{
                                   .now_ms = elapsed_ms,
                                   .blocked = blocked,
                                   .map_slice_input = autonomy::MapSliceInput{
                                       .has_occupancy = true,
                                       .occupancy = blocked ? 1.0 : 0.1,
                                   },
                               },
                               &output),
                    "scenario step should succeed at " + std::to_string(elapsed_ms) + "ms")) {
            return false;
        }

        if (!output.motion_decision.allow_motion || !output.locomotion_command.sent) {
            saw_locomotion_gate = true;
        }

        const std::string state = missionStateToString(output.mission_event.state);
        if (state_signature.empty() || state_signature.back() != state) {
            state_signature.push_back(state);
        }

        const std::string action = recoveryActionToString(output.recovery_decision.action);
        if (recovery_signature.empty() || recovery_signature.back() != action) {
            recovery_signature.push_back(action);
        }

        if (output.mission_event.state == autonomy::MissionState::Aborted) {
            break;
        }
    }

    const std::string final_state = state_signature.empty() ? std::string{} : state_signature.back();
    const bool aborted = final_state == "ABORTED";

    stack.stop();

    return expect(state_signature == scenario.expected.mission_states,
                  "mission state signature mismatch in " + scenario.name +
                      " expected=[" + join(scenario.expected.mission_states) +
                      "] actual=[" + join(state_signature) + "]") &&
           expect(recovery_signature == scenario.expected.recovery_actions,
                  "recovery signature mismatch in " + scenario.name +
                      " expected=[" + join(scenario.expected.recovery_actions) +
                      "] actual=[" + join(recovery_signature) + "]") &&
           expect(final_state == scenario.expected.final_mission_state,
                  "final mission state mismatch in " + scenario.name) &&
           expect(aborted == scenario.expected.mission_should_abort,
                  "abort expectation mismatch in " + scenario.name) &&
           expect(saw_locomotion_gate == scenario.expected.locomotion_should_be_gated,
                  "locomotion gate expectation mismatch in " + scenario.name);
}

bool testScenarioTelemetrySignatures() {
    namespace fs = std::filesystem;
    const fs::path test_file = fs::path(__FILE__).lexically_normal();
    const fs::path scenarios_dir = test_file.parent_path().parent_path() / "scenarios";

    return runScenarioAndCheckOutcomes(scenarios_dir / "07_blocked_navigation_pause_resume.toml", 1) &&
           runScenarioAndCheckOutcomes(scenarios_dir / "08_retry_replan_escalation.toml", 2) &&
           runScenarioAndCheckOutcomes(scenarios_dir / "09_abort_on_budget_exhaustion.toml", 2);
}

} // namespace

int main() {
    if (!testScenarioTelemetrySignatures()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
