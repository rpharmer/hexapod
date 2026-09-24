#include "command_channel.hpp"

#include "motion_intent_utils.hpp"
#include "navigation_manager.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string_view>
#include <vector>

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace command_channel {
namespace {

std::size_t skipWs(std::string_view s, std::size_t i) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {
        ++i;
    }
    return i;
}

std::optional<std::string> extractQuoted(std::string_view payload, std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const std::size_t key_pos = payload.find(needle);
    if (key_pos == std::string_view::npos) {
        return std::nullopt;
    }
    std::size_t i = skipWs(payload, key_pos + needle.size());
    if (i >= payload.size() || payload[i] != ':') {
        return std::nullopt;
    }
    i = skipWs(payload, i + 1);
    if (i >= payload.size() || payload[i] != '"') {
        return std::nullopt;
    }
    ++i;
    std::string out;
    while (i < payload.size() && payload[i] != '"') {
        if (payload[i] == '\\' && i + 1 < payload.size()) {
            out.push_back(payload[i + 1]);
            i += 2;
            continue;
        }
        out.push_back(payload[i++]);
    }
    return out;
}

std::optional<double> extractNumber(std::string_view payload, std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const std::size_t key_pos = payload.find(needle);
    if (key_pos == std::string_view::npos) {
        return std::nullopt;
    }
    std::size_t i = skipWs(payload, key_pos + needle.size());
    if (i >= payload.size() || payload[i] != ':') {
        return std::nullopt;
    }
    i = skipWs(payload, i + 1);
    char* end = nullptr;
    const std::string token(payload.substr(i));
    const double value = std::strtod(token.c_str(), &end);
    if (end == token.c_str()) {
        return std::nullopt;
    }
    return value;
}

std::optional<std::string_view> extractArray(std::string_view payload, std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const std::size_t key_pos = payload.find(needle);
    if (key_pos == std::string_view::npos) {
        return std::nullopt;
    }
    std::size_t i = skipWs(payload, key_pos + needle.size());
    if (i >= payload.size() || payload[i] != ':') {
        return std::nullopt;
    }
    i = skipWs(payload, i + 1);
    if (i >= payload.size() || payload[i] != '[') {
        return std::nullopt;
    }
    int depth = 0;
    for (std::size_t j = i; j < payload.size(); ++j) {
        if (payload[j] == '[') {
            ++depth;
        } else if (payload[j] == ']') {
            --depth;
            if (depth == 0) {
                return payload.substr(i + 1, j - i - 1);
            }
        }
    }
    return std::nullopt;
}

std::optional<GaitType> parseGait(std::string_view gait) {
    if (gait == "TRIPOD") {
        return GaitType::TRIPOD;
    }
    if (gait == "RIPPLE") {
        return GaitType::RIPPLE;
    }
    if (gait == "WAVE") {
        return GaitType::WAVE;
    }
    if (gait == "CRAWL") {
        return GaitType::CRAWL;
    }
    if (gait == "TURN_IN_PLACE") {
        return GaitType::TURN_IN_PLACE;
    }
    return std::nullopt;
}

std::optional<RobotMode> parseMode(std::string_view mode) {
    if (mode == "SAFE_IDLE") {
        return RobotMode::SAFE_IDLE;
    }
    if (mode == "STAND") {
        return RobotMode::STAND;
    }
    if (mode == "WALK") {
        return RobotMode::WALK;
    }
    return std::nullopt;
}

CommandType parseType(std::string_view type) {
    if (type == "scenario.list") {
        return CommandType::ScenarioList;
    }
    if (type == "scenario.run") {
        return CommandType::ScenarioRun;
    }
    if (type == "scenario.stop") {
        return CommandType::ScenarioStop;
    }
    if (type == "nav.goto") {
        return CommandType::NavGoto;
    }
    if (type == "nav.waypoints") {
        return CommandType::NavWaypoints;
    }
    if (type == "nav.cancel") {
        return CommandType::NavCancel;
    }
    if (type == "motion.set") {
        return CommandType::MotionSet;
    }
    return CommandType::Unknown;
}

bool parsePoseObject(std::string_view object, NavPoseCommand& pose) {
    if (const auto x = extractNumber(object, "x_m")) {
        pose.x_m = *x;
    } else if (const auto x = extractNumber(object, "goal_x_m")) {
        pose.x_m = *x;
    } else {
        return false;
    }
    if (const auto y = extractNumber(object, "y_m")) {
        pose.y_m = *y;
    } else if (const auto y = extractNumber(object, "goal_y_m")) {
        pose.y_m = *y;
    } else {
        return false;
    }
    if (const auto yaw = extractNumber(object, "yaw_rad")) {
        pose.yaw_rad = *yaw;
        pose.has_yaw = true;
    } else if (const auto yaw = extractNumber(object, "goal_yaw_rad")) {
        pose.yaw_rad = *yaw;
        pose.has_yaw = true;
    }
    return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
           (!pose.has_yaw || std::isfinite(pose.yaw_rad));
}

std::vector<NavPoseCommand> parsePoseArray(std::string_view array_payload) {
    std::vector<NavPoseCommand> poses;
    std::size_t i = 0;
    while (i < array_payload.size()) {
        i = skipWs(array_payload, i);
        if (i >= array_payload.size()) {
            break;
        }
        if (array_payload[i] != '{') {
            ++i;
            continue;
        }
        int depth = 0;
        std::size_t j = i;
        for (; j < array_payload.size(); ++j) {
            if (array_payload[j] == '{') {
                ++depth;
            } else if (array_payload[j] == '}') {
                --depth;
                if (depth == 0) {
                    break;
                }
            }
        }
        if (j >= array_payload.size()) {
            break;
        }
        NavPoseCommand pose{};
        if (parsePoseObject(array_payload.substr(i, j - i + 1), pose)) {
            poses.push_back(pose);
        }
        i = j + 1;
    }
    return poses;
}

std::string jsonEscape(std::string_view value) {
    std::string out;
    out.reserve(value.size());
    for (char c : value) {
        if (c == '\\' || c == '"') {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    return out;
}

} // namespace

FollowWaypoints::Params interactiveNavigationParams() {
    FollowWaypoints::Params params{};
    params.stall_timeout_s = 6.0;
    params.go_to.rotate_first = false;
    params.go_to.drive.max_v_mps = 0.05;
    params.go_to.drive.position_gain = 0.22;
    params.go_to.drive.position_tol_m = 0.035;
    params.go_to.drive.settle_cycles_required = 5;
    params.go_to.drive.yaw_hold_kp = 0.0;
    params.go_to.rotate.yaw_rate_limit_radps = 0.25;
    params.go_to.rotate.error_threshold_rad = 0.20;
    params.go_to.rotate.settle_cycles_required = 4;
    return params;
}

bool parseCommandJson(std::string_view payload, ParsedCommand& out, std::string& error) {
    out = {};
    const auto type = extractQuoted(payload, "type");
    if (!type.has_value()) {
        error = "missing type";
        return false;
    }
    out.type = parseType(*type);
    if (out.type == CommandType::Unknown) {
        error = "unknown type '" + *type + "'";
        return false;
    }
    if (const auto ref = extractQuoted(payload, "ref")) {
        out.ref = *ref;
    }
    if (const auto schema = extractNumber(payload, "schema_version")) {
        if (static_cast<int>(*schema) != kSchemaVersion) {
            error = "unsupported schema_version";
            return false;
        }
    }

    switch (out.type) {
    case CommandType::ScenarioList:
    case CommandType::ScenarioStop:
    case CommandType::NavCancel:
        return true;
    case CommandType::ScenarioRun: {
        if (const auto id = extractQuoted(payload, "id")) {
            out.scenario_id = *id;
        }
        if (const auto path = extractQuoted(payload, "path")) {
            out.scenario_path = *path;
        }
        if (out.scenario_id.empty() && out.scenario_path.empty()) {
            error = "scenario.run requires id or path";
            return false;
        }
        return true;
    }
    case CommandType::NavGoto: {
        if (!parsePoseObject(payload, out.goal)) {
            error = "nav.goto requires goal_x_m/goal_y_m or x_m/y_m";
            return false;
        }
        if (const auto gait = extractQuoted(payload, "gait")) {
            const auto parsed = parseGait(*gait);
            if (!parsed) {
                error = "invalid gait";
                return false;
            }
            out.gait = *parsed;
            out.has_gait = true;
        }
        if (const auto height = extractNumber(payload, "body_height_m")) {
            out.body_height_m = *height;
            out.has_body_height = true;
        }
        return true;
    }
    case CommandType::NavWaypoints: {
        const auto poses = extractArray(payload, "poses");
        if (!poses) {
            error = "nav.waypoints requires poses array";
            return false;
        }
        out.waypoints = parsePoseArray(*poses);
        if (out.waypoints.empty()) {
            error = "nav.waypoints poses array is empty";
            return false;
        }
        if (const auto gait = extractQuoted(payload, "gait")) {
            const auto parsed = parseGait(*gait);
            if (!parsed) {
                error = "invalid gait";
                return false;
            }
            out.gait = *parsed;
            out.has_gait = true;
        }
        if (const auto height = extractNumber(payload, "body_height_m")) {
            out.body_height_m = *height;
            out.has_body_height = true;
        }
        return true;
    }
    case CommandType::MotionSet: {
        out.motion.enabled = true;
        if (const auto mode = extractQuoted(payload, "mode")) {
            const auto parsed = parseMode(*mode);
            if (!parsed) {
                error = "invalid mode";
                return false;
            }
            out.motion.mode = *parsed;
        } else {
            out.motion.mode = RobotMode::WALK;
        }
        if (const auto gait = extractQuoted(payload, "gait")) {
            const auto parsed = parseGait(*gait);
            if (!parsed) {
                error = "invalid gait";
                return false;
            }
            out.motion.gait = *parsed;
        }
        if (const auto height = extractNumber(payload, "body_height_m")) {
            out.motion.body_height_m = *height;
        }
        if (const auto speed = extractNumber(payload, "speed_mps")) {
            out.motion.speed_mps = *speed;
        }
        if (const auto heading = extractNumber(payload, "heading_rad")) {
            out.motion.heading_rad = *heading;
        }
        if (const auto yaw = extractNumber(payload, "yaw_rad")) {
            out.motion.yaw_rad = *yaw;
        }
        if (const auto yaw_rate = extractNumber(payload, "yaw_rate_radps")) {
            out.motion.yaw_rate_radps = *yaw_rate;
        }
        if (const auto vx = extractNumber(payload, "vx_mps")) {
            out.motion.has_direct_velocity = true;
            out.motion.vx_mps = *vx;
        }
        if (const auto vy = extractNumber(payload, "vy_mps")) {
            out.motion.has_direct_velocity = true;
            out.motion.vy_mps = *vy;
        }
        return true;
    }
    case CommandType::Unknown:
        break;
    }
    error = "unhandled command";
    return false;
}

std::string serializeCommandResult(const CommandResult& result) {
    std::ostringstream payload;
    payload << "{\"type\":\"command_result\",\"schema_version\":" << kSchemaVersion
            << ",\"ok\":" << (result.ok ? "true" : "false");
    if (!result.ref.empty()) {
        payload << ",\"ref\":\"" << jsonEscape(result.ref) << "\"";
    }
    if (!result.reason.empty()) {
        payload << ",\"reason\":\"" << jsonEscape(result.reason) << "\"";
    }
    if (!result.scenario_ids.empty()) {
        payload << ",\"scenarios\":[";
        for (std::size_t i = 0; i < result.scenario_ids.size(); ++i) {
            if (i > 0) {
                payload << ',';
            }
            payload << '"' << jsonEscape(result.scenario_ids[i]) << '"';
        }
        payload << ']';
    }
    payload << '}';
    return payload.str();
}

std::vector<std::string> listScenarioIds(const std::string& scenarios_dir) {
    std::vector<std::string> ids;
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::is_directory(scenarios_dir, ec)) {
        return ids;
    }
    for (const auto& entry : fs::directory_iterator(scenarios_dir, ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_regular_file()) {
            continue;
        }
        if (entry.path().extension() != ".toml") {
            continue;
        }
        ids.push_back(entry.path().stem().string());
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

bool resolveScenarioPath(const std::string& scenarios_dir,
                         const std::string& id_or_path,
                         std::string& resolved_path,
                         std::string& error) {
    namespace fs = std::filesystem;
    if (id_or_path.empty()) {
        error = "empty scenario id/path";
        return false;
    }

    const fs::path as_path(id_or_path);
    if (as_path.extension() == ".toml" || id_or_path.find('/') != std::string::npos) {
        if (fs::is_regular_file(as_path)) {
            resolved_path = as_path.string();
            return true;
        }
        error = "scenario file not found: " + id_or_path;
        return false;
    }

    const fs::path candidate = fs::path(scenarios_dir) / (id_or_path + ".toml");
    if (fs::is_regular_file(candidate)) {
        resolved_path = candidate.string();
        return true;
    }
    error = "scenario id not found in " + scenarios_dir + ": " + id_or_path;
    return false;
}

AuthoritySnapshot computeAuthority(const ScenarioSession& session, const NavigationManager* nav) {
    AuthoritySnapshot snap{};
    snap.nav_active = nav != nullptr && nav->active();
    if (session.active()) {
        snap.level = AuthorityLevel::Scenario;
        snap.scenario_name = session.name();
        return snap;
    }
    if (snap.nav_active) {
        snap.level = AuthorityLevel::Nav;
        return snap;
    }
    snap.level = AuthorityLevel::Idle;
    return snap;
}

CommandIngress::CommandIngress(CommandChannelConfig config,
                               std::shared_ptr<logging::AsyncLogger> logger)
    : config_(std::move(config)), logger_(std::move(logger)) {}

void CommandIngress::enqueue(PendingCommand command) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(std::move(command));
}

std::vector<PendingReply> CommandIngress::takeReplies() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::vector<PendingReply> out;
    out.reserve(replies_.size());
    while (!replies_.empty()) {
        out.push_back(std::move(replies_.front()));
        replies_.pop_front();
    }
    return out;
}

void CommandIngress::stopScenario(RobotControl& robot) {
    session_.stop(robot);
}

AuthoritySnapshot CommandIngress::authority(const RobotControl& robot) const {
    return computeAuthority(session_, robot.navigationManager());
}

void CommandIngress::poll(RobotControl& robot) {
    std::deque<PendingCommand> local;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        local.swap(queue_);
    }
    for (PendingCommand& pending : local) {
        CommandResult result = apply(robot, pending.command);
        if (pending.reply) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            replies_.push_back(PendingReply{pending.peer, pending.peer_len, std::move(result)});
        }
    }
    if (session_.active()) {
        (void)session_.tick(robot, std::chrono::steady_clock::now());
    }
}

CommandResult CommandIngress::apply(RobotControl& robot, const ParsedCommand& command) {
    CommandResult result{};
    result.ref = command.ref;

    switch (command.type) {
    case CommandType::ScenarioList:
        result.ok = true;
        result.scenario_ids = listScenarioIds(config_.scenarios_dir);
        result.reason = "listed";
        break;
    case CommandType::ScenarioRun:
        result = runScenario(robot, command);
        break;
    case CommandType::ScenarioStop:
        session_.stop(robot);
        result.ok = true;
        result.reason = "stopped";
        break;
    case CommandType::NavGoto:
        result = applyNavGoto(robot, command);
        break;
    case CommandType::NavWaypoints:
        result = applyNavWaypoints(robot, command);
        break;
    case CommandType::NavCancel:
        if (session_.active()) {
            result.ok = false;
            result.reason = "rejected: scenario authority owns navigation";
            break;
        }
        if (robot.navigationManager() == nullptr) {
            result.ok = false;
            result.reason = "navigation manager unavailable";
            break;
        }
        robot.navigationManager()->cancel();
        result.ok = true;
        result.reason = "cancelled";
        break;
    case CommandType::MotionSet:
        result = applyMotionSet(robot, command);
        break;
    case CommandType::Unknown:
        result.ok = false;
        result.reason = "unknown command";
        break;
    }

    if (logger_) {
        LOG_INFO(logger_,
                 "command ",
                 result.ok ? "accepted" : "rejected",
                 " ref=",
                 result.ref,
                 " reason=",
                 result.reason);
    }
    return result;
}

CommandResult CommandIngress::runScenario(RobotControl& robot, const ParsedCommand& command) {
    CommandResult result{};
    result.ref = command.ref;
    const std::string id_or_path =
        !command.scenario_path.empty() ? command.scenario_path : command.scenario_id;
    std::string path;
    std::string error;
    if (!resolveScenarioPath(config_.scenarios_dir, id_or_path, path, error)) {
        result.ok = false;
        result.reason = error;
        return result;
    }

    ScenarioDefinition scenario{};
    if (!ScenarioDriver::loadFromToml(path, scenario, error, ScenarioDriver::ValidationMode::Strict)) {
        result.ok = false;
        result.reason = "failed to load scenario: " + error;
        return result;
    }

    if (!session_.start(robot, std::move(scenario), logger_, error)) {
        result.ok = false;
        result.reason = error;
        return result;
    }
    result.ok = true;
    result.reason = "scenario started";
    return result;
}

CommandResult CommandIngress::applyNavGoto(RobotControl& robot, const ParsedCommand& command) {
    CommandResult result{};
    result.ref = command.ref;
    if (session_.active()) {
        result.ok = false;
        result.reason = "rejected: scenario authority active";
        return result;
    }
    if (robot.navigationManager() == nullptr) {
        result.ok = false;
        result.reason = "navigation manager unavailable";
        return result;
    }
    const GaitType gait = command.has_gait ? command.gait : GaitType::TRIPOD;
    const double height = command.has_body_height ? command.body_height_m : 0.14;
    MotionIntent walk_base = makeMotionIntent(RobotMode::WALK, gait, height);
    stampIntentStreamMotionFields(walk_base);
    double goal_yaw_rad = command.goal.yaw_rad;
    if (!command.goal.has_yaw) {
        const RobotState est = robot.estimatedSnapshot();
        if (!est.valid || !est.has_body_twist_state ||
            !std::isfinite(est.body_twist_state.twist_pos_rad.z)) {
            result.ok = false;
            result.reason = "current heading unavailable for position-only nav.goto";
            return result;
        }
        goal_yaw_rad = est.body_twist_state.twist_pos_rad.z;
    }
    robot.navigationManager()->startNavigateToPose(
        walk_base, NavPose2d{command.goal.x_m, command.goal.y_m, goal_yaw_rad},
        interactiveNavigationParams());
    result.ok = true;
    result.reason = "nav.goto accepted";
    return result;
}

CommandResult CommandIngress::applyNavWaypoints(RobotControl& robot, const ParsedCommand& command) {
    CommandResult result{};
    result.ref = command.ref;
    if (session_.active()) {
        result.ok = false;
        result.reason = "rejected: scenario authority active";
        return result;
    }
    if (robot.navigationManager() == nullptr) {
        result.ok = false;
        result.reason = "navigation manager unavailable";
        return result;
    }
    const GaitType gait = command.has_gait ? command.gait : GaitType::TRIPOD;
    const double height = command.has_body_height ? command.body_height_m : 0.14;
    MotionIntent walk_base = makeMotionIntent(RobotMode::WALK, gait, height);
    stampIntentStreamMotionFields(walk_base);
    std::vector<NavPose2d> waypoints;
    waypoints.reserve(command.waypoints.size());
    double inherited_yaw_rad = 0.0;
    if (std::any_of(command.waypoints.begin(), command.waypoints.end(),
                    [](const NavPoseCommand& pose) { return !pose.has_yaw; })) {
        const RobotState est = robot.estimatedSnapshot();
        if (!est.valid || !est.has_body_twist_state ||
            !std::isfinite(est.body_twist_state.twist_pos_rad.z)) {
            result.ok = false;
            result.reason = "current heading unavailable for position-only nav.waypoints";
            return result;
        }
        inherited_yaw_rad = est.body_twist_state.twist_pos_rad.z;
    }
    for (const NavPoseCommand& pose : command.waypoints) {
        if (pose.has_yaw) inherited_yaw_rad = pose.yaw_rad;
        waypoints.push_back(NavPose2d{pose.x_m, pose.y_m, inherited_yaw_rad});
    }
    robot.navigationManager()->startRawFollowWaypoints(
        walk_base, std::move(waypoints), interactiveNavigationParams());
    result.ok = true;
    result.reason = "nav.waypoints accepted";
    return result;
}

CommandResult CommandIngress::applyMotionSet(RobotControl& robot, const ParsedCommand& command) {
    CommandResult result{};
    result.ref = command.ref;
    if (session_.active()) {
        result.ok = false;
        result.reason = "rejected: scenario authority active";
        return result;
    }
    if (robot.navigationManager() != nullptr && robot.navigationManager()->active()) {
        result.ok = false;
        result.reason = "rejected: nav authority active";
        return result;
    }
    MotionIntent intent = makeMotionIntent(command.motion);
    stampIntentStreamMotionFields(intent);
    robot.setMotionIntent(intent);
    result.ok = true;
    result.reason = "motion.set accepted";
    return result;
}

UdpCommandListener::UdpCommandListener(CommandChannelConfig config,
                                       CommandIngress& ingress,
                                       std::shared_ptr<logging::AsyncLogger> logger)
    : config_(std::move(config)), ingress_(ingress), logger_(std::move(logger)) {}

UdpCommandListener::~UdpCommandListener() {
    stop();
}

bool UdpCommandListener::start() {
    if (!config_.enabled) {
        return false;
    }
    if (running_.load()) {
        return true;
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        if (logger_) {
            LOG_WARN(logger_, "command channel disabled: failed to create UDP socket");
        }
        return false;
    }

    const int reuse = 1;
    (void)::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(config_.udp_port));
    if (::inet_pton(AF_INET, config_.bind_host.c_str(), &address.sin_addr) != 1) {
        if (logger_) {
            LOG_WARN(logger_, "command channel disabled: invalid bind host '", config_.bind_host, "'");
        }
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        if (logger_) {
            LOG_WARN(logger_,
                     "command channel disabled: bind failed on ",
                     config_.bind_host,
                     ":",
                     config_.udp_port,
                     " errno=",
                     errno);
        }
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags >= 0) {
        (void)::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    running_.store(true);
    thread_ = std::thread([this]() { threadMain(); });
    if (logger_) {
        LOG_INFO(logger_,
                 "command channel listening on ",
                 config_.bind_host,
                 ":",
                 config_.udp_port);
    }
    return true;
}

void UdpCommandListener::stop() {
    if (!running_.exchange(false)) {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
        return;
    }
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

void UdpCommandListener::threadMain() {
    std::array<char, 8192> buffer{};
    while (running_.load()) {
        for (const PendingReply& reply_item : ingress_.takeReplies()) {
            const std::string reply = serializeCommandResult(reply_item.result);
            (void)::sendto(socket_fd_,
                           reply.data(),
                           reply.size(),
                           0,
                           reinterpret_cast<const sockaddr*>(&reply_item.peer),
                           reply_item.peer_len);
        }

        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        const ssize_t received = ::recvfrom(socket_fd_,
                                            buffer.data(),
                                            buffer.size() - 1,
                                            0,
                                            reinterpret_cast<sockaddr*>(&peer),
                                            &peer_len);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            if (!running_.load()) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        buffer[static_cast<std::size_t>(received)] = '\0';
        const std::string_view payload(buffer.data(), static_cast<std::size_t>(received));
        ParsedCommand command{};
        std::string error;
        CommandResult result{};
        if (!parseCommandJson(payload, command, error)) {
            result.ok = false;
            result.reason = error;
            if (const auto ref = extractQuoted(payload, "ref")) {
                result.ref = *ref;
            }
            const std::string reply = serializeCommandResult(result);
            (void)::sendto(socket_fd_,
                           reply.data(),
                           reply.size(),
                           0,
                           reinterpret_cast<const sockaddr*>(&peer),
                           peer_len);
        } else if (command.type == CommandType::ScenarioList) {
            result.ok = true;
            result.ref = command.ref;
            result.scenario_ids = listScenarioIds(config_.scenarios_dir);
            result.reason = "listed";
            const std::string reply = serializeCommandResult(result);
            (void)::sendto(socket_fd_,
                           reply.data(),
                           reply.size(),
                           0,
                           reinterpret_cast<const sockaddr*>(&peer),
                           peer_len);
        } else {
            PendingCommand pending{};
            pending.command = std::move(command);
            pending.peer = peer;
            pending.peer_len = peer_len;
            pending.reply = true;
            ingress_.enqueue(std::move(pending));
        }
    }
}

} // namespace command_channel
