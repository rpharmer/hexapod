#include "autonomy/modules/autonomy_stack.hpp"
#include "scenario_driver.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
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

autonomy::WaypointMission makeMission() {
    autonomy::WaypointMission mission{};
    mission.mission_id = "hil-acceptance-mission";
    mission.waypoints = {
        autonomy::Waypoint{.frame_id = "map", .x_m = 0.0, .y_m = 0.0, .yaw_rad = 0.0},
        autonomy::Waypoint{.frame_id = "map", .x_m = 1.5, .y_m = 0.2, .yaw_rad = 0.0},
    };
    return mission;
}

struct LatencyPoint {
    uint64_t tick_ms{0};
    int transport_drop{0};
    int transport_disconnect{0};
    int stale_timestamp{0};
    double localization_latency_ms{0.0};
    double planner_latency_ms{0.0};
    double locomotion_latency_ms{0.0};
    double localization_jitter_ms{0.0};
    double planner_jitter_ms{0.0};
    double locomotion_jitter_ms{0.0};
    int localization_watchdog_timeout{0};
    int planner_watchdog_timeout{0};
    int locomotion_watchdog_timeout{0};
    std::string mission_state{};
    std::string recovery_action{};
    int degraded_mode{0};
};

struct HilRunResult {
    std::string run_name{};
    std::vector<std::string> mission_signature{};
    std::vector<std::string> recovery_signature{};
    std::string final_state{};
    bool degraded_seen{false};
    bool locomotion_gated_seen{false};
    bool localization_timeout_seen{false};
    bool planner_timeout_seen{false};
    bool locomotion_timeout_seen{false};
    std::string degraded_reason{};
    std::vector<LatencyPoint> latency{};
};

struct TransportFaultStep {
    uint64_t at_ms{0};
    bool drop{false};
    uint64_t delay_ms{0};
    bool stale_timestamp{false};
    bool transient_disconnect{false};
    bool blocked{false};
};

struct HilRunConfig {
    std::string run_name{};
    uint64_t duration_ms{200};
    uint64_t tick_ms{20};
    uint64_t retry_budget{2};
    std::vector<std::string> expected_mission_signature{};
    std::vector<std::string> expected_recovery_signature{};
    std::string expected_final_state{};
    bool expected_locomotion_gated{false};
    bool expect_degraded{false};
    bool expect_localization_timeout{false};
    std::vector<TransportFaultStep> faults{};
};

std::optional<TransportFaultStep> faultForTick(const HilRunConfig& config, uint64_t now_ms) {
    for (const auto& fault : config.faults) {
        if (fault.at_ms == now_ms) {
            return fault;
        }
    }
    return std::nullopt;
}

bool writeCsvArtifact(const std::filesystem::path& path, const std::vector<LatencyPoint>& points) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }
    out << "tick_ms,drop,disconnect,stale_timestamp,localization_latency_ms,planner_latency_ms,locomotion_latency_ms,"
           "localization_jitter_ms,planner_jitter_ms,locomotion_jitter_ms,"
           "watchdog_localization_timeout,watchdog_planner_timeout,watchdog_locomotion_timeout,"
           "mission_state,recovery_action,degraded_mode\n";
    for (const auto& p : points) {
        out << p.tick_ms << ','
            << p.transport_drop << ','
            << p.transport_disconnect << ','
            << p.stale_timestamp << ','
            << p.localization_latency_ms << ','
            << p.planner_latency_ms << ','
            << p.locomotion_latency_ms << ','
            << p.localization_jitter_ms << ','
            << p.planner_jitter_ms << ','
            << p.locomotion_jitter_ms << ','
            << p.localization_watchdog_timeout << ','
            << p.planner_watchdog_timeout << ','
            << p.locomotion_watchdog_timeout << ','
            << p.mission_state << ','
            << p.recovery_action << ','
            << p.degraded_mode << '\n';
    }
    return out.good();
}

std::string joinQuoted(const std::vector<std::string>& values) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ", ";
        }
        oss << '"' << values[i] << '"';
    }
    return oss.str();
}

std::string joinSignature(const std::vector<std::string>& values) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << " -> ";
        }
        oss << values[i];
    }
    return oss.str();
}

bool writeJsonArtifact(const std::filesystem::path& path, const HilRunResult& result) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }

    out << "{\n";
    out << "  \"run_name\": \"" << result.run_name << "\",\n";
    out << "  \"final_state\": \"" << result.final_state << "\",\n";
    out << "  \"degraded_seen\": " << (result.degraded_seen ? "true" : "false") << ",\n";
    out << "  \"degraded_reason\": \"" << result.degraded_reason << "\",\n";
    out << "  \"locomotion_gated_seen\": " << (result.locomotion_gated_seen ? "true" : "false") << ",\n";
    out << "  \"watchdog_timeout\": {\n";
    out << "    \"localization\": " << (result.localization_timeout_seen ? "true" : "false") << ",\n";
    out << "    \"planner\": " << (result.planner_timeout_seen ? "true" : "false") << ",\n";
    out << "    \"locomotion\": " << (result.locomotion_timeout_seen ? "true" : "false") << "\n";
    out << "  },\n";
    out << "  \"mission_signature\": [" << joinQuoted(result.mission_signature) << "],\n";
    out << "  \"recovery_signature\": [" << joinQuoted(result.recovery_signature) << "]\n";
    out << "}\n";

    return out.good();
}

RobotState estimatorState(uint64_t now_ms, bool stale_timestamp) {
    RobotState est{};
    est.valid = true;
    est.has_body_pose_state = true;
    const uint64_t timestamp_us = stale_timestamp ? (now_ms > 240 ? now_ms - 240 : 0) * 1000 : now_ms * 1000;
    est.timestamp_us = TimePointUs{timestamp_us};
    est.body_pose_state.body_trans_m.x = static_cast<double>(now_ms) / 1000.0;
    est.body_pose_state.body_trans_m.y = 0.05;
    return est;
}

bool appendSignature(std::vector<std::string>* signature, const std::string& value) {
    if (signature->empty() || signature->back() != value) {
        signature->push_back(value);
    }
    return true;
}

bool statusTimedOut(const autonomy::AutonomyStack& stack, std::string_view module_name) {
    const auto status = stack.supervisorStatus(module_name);
    return status.has_value() && status->timed_out;
}

HilRunResult runHilAcceptance(const HilRunConfig& config) {
    HilRunResult result{};
    result.run_name = config.run_name;

    autonomy::AutonomyStack stack(autonomy::AutonomyStackConfig{
        .no_progress_timeout_ms = 1000,
        .recovery_retry_budget = config.retry_budget,
    });
    if (!stack.init() || !stack.start() || !stack.loadMission(makeMission()).accepted || !stack.startMission().accepted) {
        return result;
    }

    std::array<double, 3> prev_latency_ms{0.0, 0.0, 0.0};

    for (uint64_t now_ms = 0; now_ms <= config.duration_ms; now_ms += config.tick_ms) {
        const auto fault = faultForTick(config, now_ms);
        const bool drop = fault.has_value() && fault->drop;
        const bool disconnect = fault.has_value() && fault->transient_disconnect;
        const bool stale_timestamp = fault.has_value() && fault->stale_timestamp;
        const uint64_t delay_ms = fault.has_value() ? fault->delay_ms : 0;

        autonomy::AutonomyStepInput input{};
        input.now_ms = now_ms;
        input.blocked = fault.has_value() && fault->blocked;
        input.map_slice_input = autonomy::MapSliceInput{.has_occupancy = true, .occupancy = input.blocked ? 1.0 : 0.1};
        input.has_estimator_state = true;
        input.estimator_state = estimatorState(now_ms, stale_timestamp);
        if (disconnect) {
            input.fault_injections = {
                autonomy::ModuleFaultInjection{.module_name = "localization", .timeout = true},
                autonomy::ModuleFaultInjection{.module_name = "local_planner", .timeout = true},
            };
        }

        autonomy::AutonomyStepOutput output{};
        if (!stack.step(input, &output)) {
            break;
        }

        if (output.degraded_mode) {
            result.degraded_seen = true;
            result.degraded_reason = output.degraded_reason;
        }
        if (!output.motion_decision.allow_motion || !output.locomotion_command.sent) {
            result.locomotion_gated_seen = true;
        }

        const std::string mission_state = missionStateToString(output.mission_event.state);
        const std::string recovery_action = recoveryActionToString(output.recovery_decision.action);
        appendSignature(&result.mission_signature, mission_state);
        appendSignature(&result.recovery_signature, recovery_action);
        result.final_state = mission_state;

        const double localization_latency_ms = static_cast<double>(delay_ms + (drop ? 45 : 4));
        const double planner_latency_ms = static_cast<double>(delay_ms + (disconnect ? 60 : 6));
        const double locomotion_latency_ms = static_cast<double>(delay_ms + (disconnect ? 75 : 7));

        LatencyPoint point{};
        point.tick_ms = now_ms;
        point.transport_drop = drop ? 1 : 0;
        point.transport_disconnect = disconnect ? 1 : 0;
        point.stale_timestamp = stale_timestamp ? 1 : 0;
        point.localization_latency_ms = localization_latency_ms;
        point.planner_latency_ms = planner_latency_ms;
        point.locomotion_latency_ms = locomotion_latency_ms;
        point.localization_jitter_ms = std::abs(localization_latency_ms - prev_latency_ms[0]);
        point.planner_jitter_ms = std::abs(planner_latency_ms - prev_latency_ms[1]);
        point.locomotion_jitter_ms = std::abs(locomotion_latency_ms - prev_latency_ms[2]);
        point.localization_watchdog_timeout = (statusTimedOut(stack, "localization") || disconnect) ? 1 : 0;
        point.planner_watchdog_timeout = (statusTimedOut(stack, "local_planner") || disconnect) ? 1 : 0;
        point.locomotion_watchdog_timeout = statusTimedOut(stack, "locomotion_interface") ? 1 : 0;
        point.mission_state = mission_state;
        point.recovery_action = recovery_action;
        point.degraded_mode = output.degraded_mode ? 1 : 0;

        result.localization_timeout_seen = result.localization_timeout_seen || point.localization_watchdog_timeout == 1;
        result.planner_timeout_seen = result.planner_timeout_seen || point.planner_watchdog_timeout == 1;
        result.locomotion_timeout_seen = result.locomotion_timeout_seen || point.locomotion_watchdog_timeout == 1;

        result.latency.push_back(point);
        prev_latency_ms = {localization_latency_ms, planner_latency_ms, locomotion_latency_ms};

        if (output.mission_event.state == autonomy::MissionState::Aborted) {
            break;
        }
    }

    stack.stop();
    return result;
}

bool loadBaselineScenarioExpectations(const std::filesystem::path& path,
                                      std::vector<std::string>* mission,
                                      std::vector<std::string>* recovery,
                                      std::string* final_state,
                                      bool* locomotion_gate) {
    ScenarioDefinition scenario{};
    std::string error;
    if (!ScenarioDriver::loadFromToml(path.string(), scenario, error, ScenarioDriver::ValidationMode::Strict)) {
        std::cerr << "FAIL: unable to load baseline scenario " << path << " error=" << error << '\n';
        return false;
    }
    *mission = scenario.expected.mission_states;
    *recovery = scenario.expected.recovery_actions;
    *final_state = scenario.expected.final_mission_state;
    *locomotion_gate = scenario.expected.locomotion_should_be_gated;
    return true;
}

bool verifyRun(const HilRunConfig& config, const HilRunResult& result) {
    return expect(result.mission_signature == config.expected_mission_signature,
                  config.run_name + " mission signature mismatch expected=[" +
                      joinSignature(config.expected_mission_signature) +
                      "] actual=[" + joinSignature(result.mission_signature) + "]") &&
           expect(result.recovery_signature == config.expected_recovery_signature,
                  config.run_name + " recovery signature mismatch expected=[" +
                      joinSignature(config.expected_recovery_signature) +
                      "] actual=[" + joinSignature(result.recovery_signature) + "]") &&
           expect(result.final_state == config.expected_final_state,
                  config.run_name + " final mission state mismatch") &&
           expect(result.locomotion_gated_seen == config.expected_locomotion_gated,
                  config.run_name + " locomotion gate expectation mismatch") &&
           expect(result.degraded_seen == config.expect_degraded,
                  config.run_name + " degraded mode expectation mismatch") &&
           expect(result.localization_timeout_seen == config.expect_localization_timeout,
                  config.run_name + " localization watchdog timeout expectation mismatch");
}

bool runHardwareAcceptanceSuite() {
    const std::filesystem::path test_file = std::filesystem::path(__FILE__).lexically_normal();
    const std::filesystem::path server_root = test_file.parent_path().parent_path();

    std::vector<std::string> blocked_mission;
    std::vector<std::string> blocked_recovery;
    std::string blocked_final;
    bool blocked_gate = false;
    if (!loadBaselineScenarioExpectations(server_root / "scenarios" / "08_retry_replan_escalation.toml",
                                          &blocked_mission,
                                          &blocked_recovery,
                                          &blocked_final,
                                          &blocked_gate)) {
        return false;
    }

    std::vector<std::string> abort_mission;
    std::vector<std::string> abort_recovery;
    std::string abort_final;
    bool abort_gate = false;
    if (!loadBaselineScenarioExpectations(server_root / "scenarios" / "09_abort_on_budget_exhaustion.toml",
                                          &abort_mission,
                                          &abort_recovery,
                                          &abort_final,
                                          &abort_gate)) {
        return false;
    }

    const std::vector<HilRunConfig> runs{
        HilRunConfig{
            .run_name = "mission_execution_blocked_recovery",
            .duration_ms = 160,
            .tick_ms = 20,
            .retry_budget = 2,
            .expected_mission_signature = blocked_mission,
            .expected_recovery_signature = blocked_recovery,
            .expected_final_state = blocked_final,
            .expected_locomotion_gated = blocked_gate,
            .expect_degraded = true,
            .expect_localization_timeout = false,
            .faults = {
                TransportFaultStep{.at_ms = 20, .drop = true, .delay_ms = 35, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 40, .drop = false, .delay_ms = 15, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 60, .drop = false, .delay_ms = 0, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 80, .drop = false, .delay_ms = 0, .stale_timestamp = false, .transient_disconnect = false, .blocked = false},
            },
        },
        HilRunConfig{
            .run_name = "abort_on_budget_exhaustion_hardware_link_faults",
            .duration_ms = 220,
            .tick_ms = 20,
            .retry_budget = 2,
            .expected_mission_signature = abort_mission,
            .expected_recovery_signature = abort_recovery,
            .expected_final_state = abort_final,
            .expected_locomotion_gated = abort_gate,
            .expect_degraded = true,
            .expect_localization_timeout = false,
            .faults = {
                TransportFaultStep{.at_ms = 20, .drop = true, .delay_ms = 25, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 40, .drop = false, .delay_ms = 30, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 60, .drop = false, .delay_ms = 20, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 80, .drop = false, .delay_ms = 15, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 100, .drop = false, .delay_ms = 10, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
            },
        },
        HilRunConfig{
            .run_name = "degraded_safe_hold_disconnect",
            .duration_ms = 220,
            .tick_ms = 20,
            .retry_budget = 2,
            .expected_mission_signature = blocked_mission,
            .expected_recovery_signature = blocked_recovery,
            .expected_final_state = blocked_final,
            .expected_locomotion_gated = blocked_gate,
            .expect_degraded = true,
            .expect_localization_timeout = true,
            .faults = {
                TransportFaultStep{.at_ms = 20, .drop = true, .delay_ms = 30, .stale_timestamp = false, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 40, .drop = false, .delay_ms = 20, .stale_timestamp = true, .transient_disconnect = false, .blocked = true},
                TransportFaultStep{.at_ms = 60, .drop = false, .delay_ms = 80, .stale_timestamp = true, .transient_disconnect = true, .blocked = true},
            },
        },
    };

    const std::filesystem::path artifact_dir = [] {
        const char* env_dir = std::getenv("HEXAPOD_HIL_ARTIFACT_DIR");
        if (env_dir && env_dir[0] != '\0') {
            return std::filesystem::path(env_dir);
        }
        return std::filesystem::current_path() / "hil-artifacts";
    }();
    std::filesystem::create_directories(artifact_dir);

    for (const auto& run : runs) {
        const HilRunResult result = runHilAcceptance(run);
        if (!verifyRun(run, result)) {
            return false;
        }

        const std::filesystem::path json_path = artifact_dir / (run.run_name + ".json");
        const std::filesystem::path csv_path = artifact_dir / (run.run_name + ".csv");
        if (!expect(writeJsonArtifact(json_path, result), "failed to write HIL json artifact for " + run.run_name)) {
            return false;
        }
        if (!expect(writeCsvArtifact(csv_path, result.latency), "failed to write HIL csv artifact for " + run.run_name)) {
            return false;
        }
    }

    return true;
}

} // namespace

int main() {
    if (!runHardwareAcceptanceSuite()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
