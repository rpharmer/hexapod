#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace visualiser::net {

struct CommandClientConfig {
  std::string host{"127.0.0.1"};
  int port{9872};
  int reply_timeout_ms{2000};
};

struct CommandClientResult {
  bool ok{false};
  std::string ref{};
  std::string request_type{};
  std::string reason{};
  std::vector<std::string> scenarios{};
  std::string raw{};
};

struct NavPose2d {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

struct MotionSetCommand {
  std::string mode{"WALK"};
  std::string gait{"TRIPOD"};
  double body_height_m{0.14};
  double speed_mps{0.0};
  double heading_rad{0.0};
  double yaw_rate_radps{0.0};
  bool has_direct_velocity{false};
  double vx_mps{0.0};
  double vy_mps{0.0};
};

class CommandClient {
public:
  explicit CommandClient(CommandClientConfig config = {});
  ~CommandClient();

  CommandClient(const CommandClient&) = delete;
  CommandClient& operator=(const CommandClient&) = delete;

  [[nodiscard]] bool valid() const { return socket_fd_ >= 0; }
  [[nodiscard]] std::size_t pendingCount() const { return pending_.size(); }
  // Submit without waiting for UDP; poll() yields only replies whose ref
  // matches an outstanding request, plus local timeout results.
  [[nodiscard]] std::vector<CommandClientResult> poll();
  [[nodiscard]] CommandClientResult scenarioList();
  [[nodiscard]] CommandClientResult scenarioRun(std::string_view id_or_path);
  [[nodiscard]] CommandClientResult scenarioStop();
  [[nodiscard]] CommandClientResult navGoto(const NavPose2d& goal,
                                           std::string_view gait = "TRIPOD",
                                           double body_height_m = 0.14);
  [[nodiscard]] CommandClientResult navWaypoints(const std::vector<NavPose2d>& poses,
                                                 std::string_view gait = "TRIPOD",
                                                 double body_height_m = 0.14);
  [[nodiscard]] CommandClientResult navCancel();
  [[nodiscard]] CommandClientResult motionSet(const MotionSetCommand& motion);
  [[nodiscard]] CommandClientResult standHold(double body_height_m = 0.14);

private:
  struct PendingRequest {
    std::string type{};
    std::chrono::steady_clock::time_point deadline{};
  };
  [[nodiscard]] CommandClientResult send(std::string_view payload,
                                         const std::string& ref,
                                         std::string_view request_type);
  CommandClientConfig config_{};
  int socket_fd_{-1};
  std::uint64_t next_ref_{1};
  std::map<std::string, PendingRequest> pending_{};
};

}  // namespace visualiser::net
