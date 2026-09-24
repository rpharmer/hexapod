#include "visualiser/net/udp_command_client.hpp"

#include "visualiser/parsing/json_extract.hpp"

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <cerrno>
#include <cstring>
#include <sstream>

namespace visualiser::net {
namespace {

std::string MakeRef(std::uint64_t value) {
  return "viz-" + std::to_string(value);
}

CommandClientResult ParseResult(const std::string& payload) {
  CommandClientResult result{};
  result.raw = payload;
  if (const auto ok = visualiser::parsing::ExtractBoolField(payload, "ok")) {
    result.ok = *ok;
  }
  if (const auto ref = visualiser::parsing::ExtractStringField(payload, "ref")) {
    result.ref = *ref;
  }
  if (const auto reason = visualiser::parsing::ExtractStringField(payload, "reason")) {
    result.reason = *reason;
  }
  if (const auto scenarios = visualiser::parsing::ExtractArrayField(payload, "scenarios")) {
    std::string_view array = *scenarios;
    std::size_t i = 0;
    while (i < array.size()) {
      while (i < array.size() && (array[i] == ' ' || array[i] == ',' || array[i] == '\n')) {
        ++i;
      }
      if (i >= array.size() || array[i] != '"') {
        break;
      }
      ++i;
      std::string item;
      while (i < array.size() && array[i] != '"') {
        if (array[i] == '\\' && i + 1 < array.size()) {
          item.push_back(array[i + 1]);
          i += 2;
          continue;
        }
        item.push_back(array[i++]);
      }
      result.scenarios.push_back(item);
      if (i < array.size() && array[i] == '"') {
        ++i;
      }
    }
  }
  return result;
}

}  // namespace

CommandClient::CommandClient(CommandClientConfig config) : config_(std::move(config)) {
#ifndef _WIN32
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    return;
  }
  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
#else
  (void)config_;
#endif
}

CommandClient::~CommandClient() {
#ifndef _WIN32
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
#endif
}

CommandClientResult CommandClient::send(std::string_view payload,
                                        const std::string& ref,
                                        std::string_view request_type) {
  CommandClientResult result{};
  result.ref = ref;
  result.request_type = std::string(request_type);
#ifndef _WIN32
  if (socket_fd_ < 0) {
    result.reason = "command socket unavailable";
    return result;
  }
  if (pending_.size() >= 64) {
    result.reason = "too many pending commands";
    return result;
  }

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(static_cast<std::uint16_t>(config_.port));
  if (::inet_pton(AF_INET, config_.host.c_str(), &destination.sin_addr) != 1) {
    result.reason = "invalid command host";
    return result;
  }

  const ssize_t sent = ::sendto(socket_fd_,
                                payload.data(),
                                payload.size(),
                                0,
                                reinterpret_cast<const sockaddr*>(&destination),
                                sizeof(destination));
  if (sent < 0 || static_cast<std::size_t>(sent) != payload.size()) {
    result.reason = "sendto failed";
    return result;
  }
  const int timeout_ms = config_.reply_timeout_ms > 0 ? config_.reply_timeout_ms : 2000;
  pending_.emplace(ref, PendingRequest{std::string(request_type),
                                       std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms)});
  result.ok = true;
  result.reason = "sent";
#else
  (void)payload;
  result.reason = "command client unsupported on Windows build";
#endif
  return result;
}

std::vector<CommandClientResult> CommandClient::poll() {
  std::vector<CommandClientResult> results;
#ifndef _WIN32
  if (socket_fd_ < 0) {
    return results;
  }
  sockaddr_in expected{};
  expected.sin_family = AF_INET;
  expected.sin_port = htons(static_cast<std::uint16_t>(config_.port));
  if (::inet_pton(AF_INET, config_.host.c_str(), &expected.sin_addr) != 1) {
    return results;
  }
  // Bound work per frame even if an endpoint floods the UDP socket.
  for (int i = 0; i < 32; ++i) {
    char buffer[8192];
    sockaddr_in sender{};
    socklen_t sender_len = sizeof(sender);
    const ssize_t received = ::recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                        reinterpret_cast<sockaddr*>(&sender), &sender_len);
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      break;
    }
    if (sender.sin_family != AF_INET || sender.sin_port != expected.sin_port ||
        sender.sin_addr.s_addr != expected.sin_addr.s_addr) {
      continue;
    }
    const std::string payload(buffer, static_cast<std::size_t>(received));
    const auto type = visualiser::parsing::ExtractStringField(payload, "type");
    const auto schema = visualiser::parsing::ExtractIntField(payload, "schema_version");
    const auto ref = visualiser::parsing::ExtractStringField(payload, "ref");
    const auto ok = visualiser::parsing::ExtractBoolField(payload, "ok");
    if (!type || *type != "command_result" || !schema || *schema != 1 || !ref || !ok) {
      continue;
    }
    const auto pending = pending_.find(*ref);
    if (pending == pending_.end()) {
      continue;
    }
    CommandClientResult result = ParseResult(payload);
    result.request_type = pending->second.type;
    pending_.erase(pending);
    results.push_back(std::move(result));
  }
  const auto now = std::chrono::steady_clock::now();
  for (auto pending = pending_.begin(); pending != pending_.end();) {
    if (pending->second.deadline > now) {
      ++pending;
      continue;
    }
    CommandClientResult result{};
    result.ref = pending->first;
    result.request_type = pending->second.type;
    result.reason = "command_result timed out";
    results.push_back(std::move(result));
    pending = pending_.erase(pending);
  }
#endif
  return results;
}

CommandClientResult CommandClient::scenarioList() {
  const std::string ref = MakeRef(next_ref_++);
  std::ostringstream payload;
  payload << "{\"schema_version\":1,\"type\":\"scenario.list\",\"ref\":\"" << ref << "\"}";
  return send(payload.str(), ref, "scenario.list");
}

CommandClientResult CommandClient::scenarioRun(std::string_view id_or_path) {
  const std::string ref = MakeRef(next_ref_++);
  std::ostringstream payload;
  payload << "{\"schema_version\":1,\"type\":\"scenario.run\",\"ref\":\"" << ref
          << "\",\"id\":\"" << id_or_path << "\"}";
  return send(payload.str(), ref, "scenario.run");
}

CommandClientResult CommandClient::scenarioStop() {
  const std::string ref = MakeRef(next_ref_++);
  std::ostringstream payload;
  payload << "{\"schema_version\":1,\"type\":\"scenario.stop\",\"ref\":\"" << ref << "\"}";
  return send(payload.str(), ref, "scenario.stop");
}

CommandClientResult CommandClient::navGoto(const NavPose2d& goal,
                                          std::string_view gait,
                                          double body_height_m) {
  const std::string ref = MakeRef(next_ref_++);
  std::ostringstream payload;
  payload << "{\"schema_version\":1,\"type\":\"nav.goto\",\"ref\":\"" << ref
          << "\",\"goal_x_m\":" << goal.x_m << ",\"goal_y_m\":" << goal.y_m
          << ",\"goal_yaw_rad\":" << goal.yaw_rad << ",\"gait\":\"" << gait
          << "\",\"body_height_m\":" << body_height_m << '}';
  return send(payload.str(), ref, "nav.goto");
}

CommandClientResult CommandClient::navWaypoints(const std::vector<NavPose2d>& poses,
                                                std::string_view gait,
                                                double body_height_m) {
  const std::string ref = MakeRef(next_ref_++);
  std::ostringstream payload;
  payload << "{\"schema_version\":1,\"type\":\"nav.waypoints\",\"ref\":\"" << ref
          << "\",\"gait\":\"" << gait << "\",\"body_height_m\":" << body_height_m
          << ",\"poses\":[";
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (i > 0) {
      payload << ',';
    }
    payload << "{\"x_m\":" << poses[i].x_m << ",\"y_m\":" << poses[i].y_m
            << ",\"yaw_rad\":" << poses[i].yaw_rad << '}';
  }
  payload << "]}";
  return send(payload.str(), ref, "nav.waypoints");
}

CommandClientResult CommandClient::navCancel() {
  const std::string ref = MakeRef(next_ref_++);
  std::ostringstream payload;
  payload << "{\"schema_version\":1,\"type\":\"nav.cancel\",\"ref\":\"" << ref << "\"}";
  return send(payload.str(), ref, "nav.cancel");
}

CommandClientResult CommandClient::motionSet(const MotionSetCommand& motion) {
  const std::string ref = MakeRef(next_ref_++);
  std::ostringstream payload;
  payload << "{\"schema_version\":1,\"type\":\"motion.set\",\"ref\":\"" << ref
          << "\",\"mode\":\"" << motion.mode << "\",\"gait\":\"" << motion.gait
          << "\",\"body_height_m\":" << motion.body_height_m
          << ",\"speed_mps\":" << motion.speed_mps
          << ",\"heading_rad\":" << motion.heading_rad
          << ",\"yaw_rate_radps\":" << motion.yaw_rate_radps;
  if (motion.has_direct_velocity) {
    payload << ",\"vx_mps\":" << motion.vx_mps << ",\"vy_mps\":" << motion.vy_mps;
  }
  payload << '}';
  return send(payload.str(), ref, "motion.set");
}

CommandClientResult CommandClient::standHold(double body_height_m) {
  MotionSetCommand motion{};
  motion.mode = "STAND";
  motion.speed_mps = 0.0;
  motion.yaw_rate_radps = 0.0;
  motion.body_height_m = body_height_m;
  return motionSet(motion);
}

}  // namespace visualiser::net
