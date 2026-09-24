#pragma once

#include "logger.hpp"
#include "nav_primitives.hpp"
#include "robot_control.hpp"
#include "scenario_driver.hpp"
#include "scenario_session.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <netinet/in.h>

namespace command_channel {

inline constexpr int kDefaultUdpPort = 9872;
inline constexpr int kSchemaVersion = 1;

enum class AuthorityLevel {
    Idle = 0,
    Nav = 1,
    Scenario = 2,
};

struct AuthoritySnapshot {
    AuthorityLevel level{AuthorityLevel::Idle};
    std::string scenario_name{};
    bool nav_active{false};
};

inline const char* authorityLevelName(AuthorityLevel level) {
    switch (level) {
    case AuthorityLevel::Idle:
        return "idle";
    case AuthorityLevel::Nav:
        return "nav";
    case AuthorityLevel::Scenario:
        return "scenario";
    }
    return "idle";
}

enum class CommandType {
    Unknown,
    ScenarioList,
    ScenarioRun,
    ScenarioStop,
    NavGoto,
    NavWaypoints,
    NavCancel,
    MotionSet,
};

struct NavPoseCommand {
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
    bool has_yaw{false};
};

struct ParsedCommand {
    CommandType type{CommandType::Unknown};
    std::string ref{};
    std::string scenario_id{};
    std::string scenario_path{};
    NavPoseCommand goal{};
    std::vector<NavPoseCommand> waypoints{};
    GaitType gait{GaitType::TRIPOD};
    double body_height_m{0.14};
    bool has_gait{false};
    bool has_body_height{false};
    ScenarioMotionIntent motion{};
};

struct CommandResult {
    bool ok{false};
    std::string ref{};
    std::string reason{};
    std::vector<std::string> scenario_ids{};
};

struct CommandChannelConfig {
    bool enabled{false};
    std::string bind_host{"127.0.0.1"};
    int udp_port{kDefaultUdpPort};
    std::string scenarios_dir{"scenarios"};
};

[[nodiscard]] bool parseCommandJson(std::string_view payload, ParsedCommand& out, std::string& error);
[[nodiscard]] std::string serializeCommandResult(const CommandResult& result);
[[nodiscard]] std::vector<std::string> listScenarioIds(const std::string& scenarios_dir);
[[nodiscard]] bool resolveScenarioPath(const std::string& scenarios_dir,
                                       const std::string& id_or_path,
                                       std::string& resolved_path,
                                       std::string& error);

AuthoritySnapshot computeAuthority(const ScenarioSession& session, const NavigationManager* nav);

/** Conservative follow settings used by interactive navigation commands. */
FollowWaypoints::Params interactiveNavigationParams();

struct PendingCommand {
    ParsedCommand command{};
    sockaddr_in peer{};
    socklen_t peer_len{sizeof(sockaddr_in)};
    bool reply{true};
};

struct PendingReply {
    sockaddr_in peer{};
    socklen_t peer_len{sizeof(sockaddr_in)};
    CommandResult result{};
};

class CommandIngress {
public:
    explicit CommandIngress(CommandChannelConfig config,
                            std::shared_ptr<logging::AsyncLogger> logger = nullptr);

    void enqueue(PendingCommand command);
    /** Apply queued commands and advance an active scenario session. */
    void poll(RobotControl& robot);
    void stopScenario(RobotControl& robot);
    [[nodiscard]] std::vector<PendingReply> takeReplies();

    [[nodiscard]] AuthoritySnapshot authority(const RobotControl& robot) const;
    [[nodiscard]] bool scenarioActive() const { return session_.active(); }
    [[nodiscard]] bool allowGamepadMotion() const { return !session_.active(); }

private:
    CommandResult apply(RobotControl& robot, const ParsedCommand& command);
    CommandResult runScenario(RobotControl& robot, const ParsedCommand& command);
    CommandResult applyNavGoto(RobotControl& robot, const ParsedCommand& command);
    CommandResult applyNavWaypoints(RobotControl& robot, const ParsedCommand& command);
    CommandResult applyMotionSet(RobotControl& robot, const ParsedCommand& command);

    CommandChannelConfig config_{};
    std::shared_ptr<logging::AsyncLogger> logger_;
    ScenarioSession session_{};
    mutable std::mutex queue_mutex_;
    std::deque<PendingCommand> queue_{};
    std::deque<PendingReply> replies_{};
};

class UdpCommandListener {
public:
    UdpCommandListener(CommandChannelConfig config,
                       CommandIngress& ingress,
                       std::shared_ptr<logging::AsyncLogger> logger = nullptr);
    ~UdpCommandListener();

    UdpCommandListener(const UdpCommandListener&) = delete;
    UdpCommandListener& operator=(const UdpCommandListener&) = delete;

    [[nodiscard]] bool start();
    void stop();

private:
    void threadMain();

    CommandChannelConfig config_{};
    CommandIngress& ingress_;
    std::shared_ptr<logging::AsyncLogger> logger_;
    int socket_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread thread_{};
};

} // namespace command_channel
