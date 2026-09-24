#include "cli_options.hpp"
#include "command_channel.hpp"
#include "motion_intent_utils.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

bool expect(bool cond, const std::string& message) {
    if (!cond) {
        std::cerr << "[FAIL] " << message << '\n';
        ++g_failures;
        return false;
    }
    return true;
}

bool testParseScenarioList() {
    command_channel::ParsedCommand command{};
    std::string error;
    const bool ok = command_channel::parseCommandJson(
        R"({"schema_version":1,"type":"scenario.list","ref":"a"})", command, error);
    return expect(ok, "scenario.list should parse: " + error) &&
           expect(command.type == command_channel::CommandType::ScenarioList, "type scenario.list") &&
           expect(command.ref == "a", "ref preserved");
}

bool testParseScenarioRun() {
    command_channel::ParsedCommand command{};
    std::string error;
    const bool ok = command_channel::parseCommandJson(
        R"({"type":"scenario.run","id":"01_nominal_stand_walk","ref":"r1"})", command, error);
    return expect(ok, "scenario.run should parse: " + error) &&
           expect(command.scenario_id == "01_nominal_stand_walk", "scenario id") &&
           expect(command.ref == "r1", "ref");
}

bool testParseNavGoto() {
    command_channel::ParsedCommand command{};
    std::string error;
    const bool ok = command_channel::parseCommandJson(
        R"({"type":"nav.goto","goal_x_m":1.5,"goal_y_m":-0.25,"goal_yaw_rad":0.1,"gait":"RIPPLE"})",
        command,
        error);
    if (!expect(ok, "nav.goto should parse: " + error) ||
        !expect(command.goal.has_yaw && std::abs(command.goal.yaw_rad - 0.1) < 1e-9,
                "explicit nav.goto yaw should be retained")) {
        return false;
    }
    command_channel::ParsedCommand position_only{};
    error.clear();
    if (!expect(command_channel::parseCommandJson(
                    R"({"type":"nav.goto","goal_x_m":1.5,"goal_y_m":-0.25})",
                    position_only, error),
                "position-only nav.goto should parse: " + error) ||
        !expect(!position_only.goal.has_yaw,
                "omitted nav.goto yaw must not silently become world yaw zero")) {
        return false;
    }
    command_channel::ParsedCommand nonfinite{};
    error.clear();
    if (!expect(!command_channel::parseCommandJson(
                    R"({"type":"nav.goto","goal_x_m":1e999,"goal_y_m":0})",
                    nonfinite, error),
                "non-finite navigation coordinates must be rejected")) {
        return false;
    }
    return
           expect(command.type == command_channel::CommandType::NavGoto, "type") &&
           expect(std::abs(command.goal.x_m - 1.5) < 1e-9, "goal x") &&
           expect(std::abs(command.goal.y_m + 0.25) < 1e-9, "goal y") &&
           expect(command.has_gait && command.gait == GaitType::RIPPLE, "gait");
}

bool testInteractiveNavigationProfile() {
    const auto params = command_channel::interactiveNavigationParams();
    return expect(!params.go_to.rotate_first, "interactive nav should not demand an entry pivot") &&
           expect(params.go_to.drive.max_v_mps <= 0.05,
                  "interactive nav speed should not exceed the passing live-physics profile") &&
           expect(params.go_to.drive.position_gain <= 0.22,
                  "interactive nav position gain should match the passing profile") &&
           expect(params.go_to.rotate.yaw_rate_limit_radps < 0.45,
                  "interactive nav should stay below the rapid body-rate rule") &&
           expect(params.stall_timeout_s >= 6.0,
                  "interactive nav should allow the tested progress window");
}

bool testParseMotionSetRejectedFields() {
    command_channel::ParsedCommand command{};
    std::string error;
    const bool ok = command_channel::parseCommandJson(
        R"({"type":"motion.set","mode":"WALK","speed_mps":0.05,"heading_rad":0.0})",
        command,
        error);
    return expect(ok, "motion.set should parse: " + error) &&
           expect(command.motion.mode == RobotMode::WALK, "mode") &&
           expect(std::abs(command.motion.speed_mps - 0.05) < 1e-9, "speed");
}

bool testParseStandHold() {
    command_channel::ParsedCommand command{};
    std::string error;
    const bool ok = command_channel::parseCommandJson(
        R"({"schema_version":1,"type":"motion.set","ref":"stand-1","mode":"STAND","speed_mps":0,"yaw_rate_radps":0,"body_height_m":0.14})",
        command,
        error);
    return expect(ok, "stand-and-hold should parse: " + error) &&
           expect(command.ref == "stand-1", "stand ref preserved") &&
           expect(command.motion.mode == RobotMode::STAND, "stand mode") &&
           expect(command.motion.speed_mps == 0.0, "stand speed zero") &&
           expect(command.motion.yaw_rate_radps == 0.0, "stand yaw rate zero");
}

bool testUnknownTypeRejected() {
    command_channel::ParsedCommand command{};
    std::string error;
    const bool ok = command_channel::parseCommandJson(R"({"type":"explode"})", command, error);
    return expect(!ok, "unknown type should fail") &&
           expect(error.find("unknown") != std::string::npos, "error mentions unknown");
}

bool testAuthorityOrdering() {
    ScenarioSession idle_session;
    const auto idle = command_channel::computeAuthority(idle_session, nullptr);
    if (!expect(idle.level == command_channel::AuthorityLevel::Idle, "idle authority")) {
        return false;
    }

    // ScenarioSession without start stays inactive; authority is idle/nav only here.
    // Full scenario>nav matrix needs a live RobotControl; parse/authority helpers cover the
    // priority naming used by telemetry.
    const std::string idle_name = command_channel::authorityLevelName(command_channel::AuthorityLevel::Idle);
    const std::string nav_name = command_channel::authorityLevelName(command_channel::AuthorityLevel::Nav);
    const std::string scenario_name =
        command_channel::authorityLevelName(command_channel::AuthorityLevel::Scenario);
    return expect(idle_name == std::string("idle"), "idle name") &&
           expect(nav_name == std::string("nav"), "nav name") &&
           expect(scenario_name == std::string("scenario"), "scenario name") &&
           expect(static_cast<int>(command_channel::AuthorityLevel::Scenario) >
                      static_cast<int>(command_channel::AuthorityLevel::Nav),
                  "scenario > nav ordinal") &&
           expect(static_cast<int>(command_channel::AuthorityLevel::Nav) >
                      static_cast<int>(command_channel::AuthorityLevel::Idle),
                  "nav > idle ordinal");
}

bool testSerializeResult() {
    command_channel::CommandResult result{};
    result.ok = true;
    result.ref = "x";
    result.reason = "listed";
    result.scenario_ids = {"a", "b"};
    const std::string payload = command_channel::serializeCommandResult(result);
    return expect(payload.find("\"ok\":true") != std::string::npos, "ok true") &&
           expect(payload.find("\"scenarios\":[\"a\",\"b\"]") != std::string::npos, "scenarios array") &&
           expect(payload.find("command_result") != std::string::npos, "type");
}

bool testResolveScenarioPath() {
    std::string resolved;
    std::string error;
    const bool ok = command_channel::resolveScenarioPath(
        "scenarios", "does_not_exist_zzz", resolved, error);
    return expect(!ok, "missing scenario should fail") &&
           expect(!error.empty(), "error populated");
}

bool testCliCommandFlags() {
    CliOptions options{};
    std::string error;
    std::vector<std::string> args{
        "hexapod-server",
        "--command-enable",
        "--command-host",
        "127.0.0.1",
        "--command-port",
        "9872",
        "--command-scenarios-dir",
        "scenarios",
    };
    std::vector<char*> argv;
    for (auto& arg : args) {
        argv.push_back(arg.data());
    }
    const bool ok = parseCliOptions(static_cast<int>(argv.size()), argv.data(), options, error);
    return expect(ok, "command CLI should parse: " + error) &&
           expect(options.commandEnabledOverride.has_value() && *options.commandEnabledOverride,
                  "command enable") &&
           expect(options.commandHostOverride == std::string("127.0.0.1"), "host") &&
           expect(options.commandPortOverride == 9872, "port") &&
           expect(options.commandScenariosDirOverride == std::string("scenarios"), "scenarios dir");
}

} // namespace

int main() {
    const bool ok = testParseScenarioList() && testParseScenarioRun() && testParseNavGoto() &&
                    testInteractiveNavigationProfile() &&
                    testParseMotionSetRejectedFields() && testParseStandHold() &&
                    testUnknownTypeRejected() &&
                    testAuthorityOrdering() && testSerializeResult() && testResolveScenarioPath() &&
                    testCliCommandFlags();
    if (!ok || g_failures != 0) {
        std::cerr << g_failures << " failure(s)\n";
        return 1;
    }
    std::cout << "command_channel tests passed\n";
    return 0;
}
