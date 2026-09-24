#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include "visualiser/gl/debug.hpp"
#include "visualiser/gl/loader.hpp"
#include "visualiser/net/udp_command_client.hpp"
#include "visualiser/render/camera.hpp"
#include "visualiser/render/line_renderer.hpp"
#include "visualiser/render/mesh_renderer.hpp"
#include "visualiser/render/point_renderer.hpp"
#include "visualiser/render/primitive_draw.hpp"
#include "visualiser/render/shader_sources.hpp"
#include "visualiser/robot/enum_names.hpp"
#include "visualiser/robot/kinematics.hpp"
#include "visualiser/scene/visibility.hpp"
#include "visualiser/scene/ground_pick.hpp"
#include "visualiser/scene/ground_reference.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "minphys_viz_protocol.hpp"
#include "visualiser_frame_math.hpp"

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

constexpr int kDefaultWindowWidth = 1280;
constexpr int kDefaultWindowHeight = 720;
constexpr int kDefaultUdpPort = 9870;
constexpr float kPi = 3.14159265358979323846f;

visualiser::render::LineRenderer g_line_renderer;
visualiser::render::MeshRenderer g_mesh_renderer;
visualiser::render::PointRenderer g_point_renderer;
bool g_modern_renderer_ok = false;

bool RunningInWsl() {
  return std::getenv("WSL_INTEROP") != nullptr || std::getenv("WSL_DISTRO_NAME") != nullptr;
}

void ConfigureWslWindowPlatform() {
#if defined(__linux__) && defined(GLFW_PLATFORM) && defined(GLFW_PLATFORM_X11)
  const bool x11_available = std::getenv("DISPLAY") != nullptr;
  if (RunningInWsl() && x11_available) {
    // GLFW 3.4 prefers Wayland when WSLg exposes both backends. Wayland deliberately prevents
    // clients from activating their own windows, which leaves the visualiser visible but unable
    // to take keyboard focus reliably. WSLg's XWayland path supports the activation request and
    // also avoids the noisy Mesa/EGL device-probe fallback seen on the Wayland path.
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
    std::cout << "WSL detected: using GLFW X11 backend for reliable window focus\n";
  }
#endif
}

// One-shot focus often loses to the launching terminal under WSLg/Windows focus-stealing rules.
// Keep requesting activation for a short window after show until the WM grants focus.
void RequestWindowFocus(GLFWwindow* window) {
  if (window == nullptr) {
    return;
  }
  glfwShowWindow(window);
  glfwRestoreWindow(window);
  // Nudge onto a known on-screen position in case a previous WSLg session left it off-screen.
  glfwSetWindowPos(window, 80, 80);
  glfwFocusWindow(window);
  glfwRequestWindowAttention(window);
}

void PollStartupWindowFocus(GLFWwindow* window, double shown_at_s, bool& focus_settled) {
  if (focus_settled || window == nullptr) {
    return;
  }
  if (glfwGetWindowAttrib(window, GLFW_FOCUSED)) {
    focus_settled = true;
    return;
  }
  constexpr double kFocusRetrySeconds = 2.5;
  if ((glfwGetTime() - shown_at_s) > kFocusRetrySeconds) {
    focus_settled = true;
    return;
  }
  RequestWindowFocus(window);
}

bool InitModernRenderer() {
  if (g_modern_renderer_ok) {
    return true;
  }
  if (!g_line_renderer.Init(visualiser::render::kWireVertGlsl, visualiser::render::kWireFragGlsl)) {
    return false;
  }
  if (!g_mesh_renderer.Init(visualiser::render::kMeshVertGlsl, visualiser::render::kMeshFragGlsl)) {
    return false;
  }
  if (!g_point_renderer.Init(visualiser::render::kPointVertGlsl, visualiser::render::kPointFragGlsl)) {
    return false;
  }
  g_modern_renderer_ok = true;
  return true;
}

constexpr std::array<const char*, 6> kLegKeys = {"LF", "LM", "LR", "RF", "RM", "RR"};
// Server telemetry leg order is LF, LM, LR, RF, RM, RR -> LegID L1, L2, L3, R1, R2, R3.
constexpr std::array<float, 6> kDefaultMountAnglesDeg = {323.0f, 270.0f, 217.0f, 37.0f, 90.0f, 143.0f};
constexpr std::array<float, 6> kDefaultCoxaAttachDeg = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
constexpr std::array<float, 6> kDefaultFemurAttachDeg = {-35.0f, -35.0f, -35.0f, 35.0f, 35.0f, 35.0f};
constexpr std::array<float, 6> kDefaultTibiaAttachDeg = {-83.0f, -83.0f, -83.0f, 83.0f, 83.0f, 83.0f};
constexpr std::array<std::array<float, 3>, 6> kDefaultBodyCoxaOffsets = {{
    {{-0.0835f, -0.063f, -0.007f}}, // LF = L1
    {{0.0f, -0.0815f, -0.007f}},    // LM = L2
    {{0.0835f, -0.063f, -0.007f}},  // LR = L3
    {{-0.0835f, 0.063f, -0.007f}},  // RF = R1
    {{0.0f, 0.0815f, -0.007f}},     // RM = R2
    {{0.0835f, 0.063f, -0.007f}},   // RR = R3
}};

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Quat {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

enum class ShapeType {
  kUnknown,
  kSphere,
  kBox,
  kPlane,
  kCapsule,
  kCylinder,
  kHalfCylinder,
  kCompound,
};

struct CompoundChildState {
  ShapeType shape = ShapeType::kUnknown;
  float radius = 0.5f;
  float half_height = 0.5f;
  Vec3 half_extents{0.5f, 0.5f, 0.5f};
  Vec3 local_position{};
  Quat local_rotation{};
};

struct EntityState {
  std::uint32_t id = 0;
  ShapeType shape = ShapeType::kUnknown;
  float radius = 0.5f;
  float half_height = 0.5f;
  Vec3 half_extents{0.5f, 0.5f, 0.5f};
  Vec3 plane_normal{0.0f, 1.0f, 0.0f};
  float plane_offset = 0.0f;
  std::vector<CompoundChildState> compound_children{};
  Vec3 position{};
  Quat rotation{};
  bool has_static = false;
  bool has_frame = false;
};

struct TerrainPatchState {
  bool valid = false;
  int schema_version = 1;
  int frame = 0;
  float sim_time_s = 0.0f;
  int rows = 0;
  int cols = 0;
  float cell_size_m = 0.0f;
  float base_margin_m = 0.0f;
  float min_cell_thickness_m = 0.0f;
  float influence_sigma_m = 0.0f;
  float plane_confidence = 0.0f;
  float confidence_half_life_s = 0.0f;
  float base_update_blend = 0.0f;
  float decay_update_boost = 0.0f;
  Vec3 center{};
  bool has_grid_origin_xz = false;
  float grid_origin_x = 0.0f;
  float grid_origin_z = 0.0f;
  float base_height_m = 0.0f;
  float plane_height_m = 0.0f;
  Vec3 plane_normal{0.0f, 1.0f, 0.0f};
  std::vector<float> heights{};
  std::vector<float> confidences{};
  std::vector<float> collision_heights{};
};

struct HexapodLegLayout {
  std::string key;
  Vec3 body_coxa_offset{0.0f, 0.0f, 0.0f};
  float mount_angle_rad = 0.0f;
  float coxa_mm = 43.0f;
  float femur_mm = 60.0f;
  float tibia_mm = 104.0f;
  float coxa_attach_deg = 0.0f;
  float femur_attach_deg = 0.0f;
  float tibia_attach_deg = 0.0f;
  float coxa_sign = 1.0f;
  float femur_sign = 1.0f;
  float tibia_sign = 1.0f;
};

struct HexapodGeometryState {
  bool valid = false;
  float coxa_mm = 43.0f;
  float femur_mm = 60.0f;
  float tibia_mm = 104.0f;
  float body_radius_mm = 60.0f;
  std::array<HexapodLegLayout, 6> legs{};
};

struct HexapodStatusState {
  bool valid = false;
  uint64_t timestamp_ms = 0;
  int loop_counter = 0;
  int active_mode = 0;
  int active_fault = -1;
  bool bus_ok = true;
  bool estimator_valid = true;
  float voltage = 0.0f;
  float current = 0.0f;
  std::optional<int> nav_lifecycle{};
  std::optional<int> nav_block_reason{};
  std::optional<int> nav_planner_status{};
  std::optional<bool> nav_map_fresh{};
  std::optional<std::size_t> nav_replan_count{};
  std::optional<double> nav_active_segment_length_m{};
  std::optional<std::size_t> nav_active_segment_waypoint_count{};
  std::optional<double> nav_nearest_obstacle_distance_m{};
  std::optional<double> fusion_model_trust{};
  std::optional<bool> fusion_resync_requested{};
  std::optional<bool> fusion_hard_reset_requested{};
  std::optional<bool> fusion_predictive_mode{};
  std::optional<double> fusion_max_body_position_error_m{};
  std::optional<double> fusion_max_body_orientation_error_rad{};
  std::optional<double> fusion_contact_mismatch_ratio{};
  std::optional<double> fusion_terrain_residual_m{};
  std::optional<double> requested_planar_speed_mps{};
  std::optional<double> governed_planar_speed_mps{};
  std::optional<double> physics_peak_servo_torque_utilization{};
  std::optional<std::string> command_authority{};
  std::optional<std::string> command_scenario{};
  std::optional<bool> command_nav_active{};
  std::optional<float> nav_goal_x_m{};
  std::optional<float> nav_goal_y_m{};
  std::optional<float> nav_goal_yaw_rad{};
  std::vector<std::array<float, 3>> nav_active_segment{};
  std::optional<int> nav_active_waypoint_index{};
  std::optional<double> nav_distance_to_active_waypoint_m{};
};

struct LocalMapOverlayState {
  bool valid = false;
  bool fresh = false;
  int width_cells = 0;
  int height_cells = 0;
  int cell_step = 1;
  float resolution_m = 0.05f;
  float center_x_m = 0.0f;
  float center_y_m = 0.0f;
  float center_yaw_rad = 0.0f;
  std::vector<std::uint8_t> cells{};
};

struct LocomotionOverlayState {
  bool valid = false;
  bool has_commanded_feet = false;
  bool has_measured_feet = false;
  bool has_planned_targets = false;
  bool has_planned_stance = false;
  bool has_raw_contact = false;
  bool has_fused_support = false;
  bool has_tracking_error = false;
  std::optional<float> max_post_clamp_distortion_m{};
  std::array<Vec3, 6> commanded_foot_world_m{};
  std::array<Vec3, 6> measured_foot_world_m{};
  std::array<Vec3, 6> planned_leg_target_body_m{};
  std::array<bool, 6> planned_stance{};
  std::array<bool, 6> raw_contact{};
  std::array<bool, 6> fused_support{};
  std::array<float, 6> commanded_tracking_error_m{};
};

struct HexapodBodyPoseState {
  bool valid = false;
  Vec3 position{};
  float yaw_rad = 0.0f;
  Vec3 orientation_rad{};
};

struct HexapodTelemetryState {
  bool has_geometry = false;
  bool has_joints = false;
  HexapodGeometryState geometry{};
  HexapodStatusState status{};
  HexapodBodyPoseState body_pose{};
  std::array<std::array<float, 3>, 6> angles_deg{};
  LocalMapOverlayState local_map{};
  LocomotionOverlayState locomotion{};
};

struct AppUiState {
  bool show_scene = true;
  bool show_robot = true;
  bool overlay_command_robot = false;
  bool show_terrain = true;
  bool show_nav_path = true;
  bool show_local_map = true;
  bool show_feet = true;
  bool click_goal_mode = false;
  bool waypoint_edit_mode = false;
  bool follow_active = true;
  bool rotate_scene = false;
  bool show_overlay = true;
  bool show_debug = false;
};

struct CameraState {
  float yaw_deg = 28.0f;
  float pitch_deg = 18.0f;
  float distance_scale = 6.0f;
  float pan_x = 0.0f;
  float pan_y = 0.0f;
  float spin_deg_per_s = 8.0f;
};

struct Options {
  int udp_port = kDefaultUdpPort;
  bool log_joint_positions = false;
  std::string command_host = "127.0.0.1";
  int command_port = 9872;
};

struct CommandUiState {
  struct HistoryEntry {
    std::string ref{};
    std::string type{};
    std::string state{};
    std::string reason{};
  };
  std::vector<std::string> scenarios{};
  int selected_index = 0;
  std::string last_result{};
  std::vector<HistoryEntry> history{};
  double last_server_reply_time_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<visualiser::net::NavPose2d> draft_waypoints{};
  std::size_t draft_revision = 0;
  std::string pending_waypoints_ref{};
  std::size_t pending_waypoints_revision = 0;
  float motion_speed_mps = 0.05f;
  float motion_heading_rad = 0.0f;
  float motion_yaw_rate = 0.0f;
  float motion_body_height_m = 0.14f;
};

void ShowCommandSubmission(CommandUiState& ui,
                           const visualiser::net::CommandClientResult& submitted) {
  ui.last_result = submitted.ok
      ? (submitted.request_type + " pending (" + submitted.ref + ")")
      : (submitted.request_type + " send failed: " + submitted.reason);
  ui.history.insert(ui.history.begin(), CommandUiState::HistoryEntry{
      submitted.ref, submitted.request_type,
      submitted.ok ? "pending" : "send failed", submitted.reason});
  if (ui.history.size() > 8) {
    ui.history.resize(8);
  }
}

void ApplyCommandReply(AppUiState& ui,
                       CommandUiState& command_ui,
                       const visualiser::net::CommandClientResult& reply) {
  const bool local_timeout = reply.raw.empty();
  if (!local_timeout) {
    command_ui.last_server_reply_time_s = glfwGetTime();
  }
  if (reply.request_type == "scenario.list" && reply.ok) {
    command_ui.scenarios = reply.scenarios;
    if (command_ui.selected_index >= static_cast<int>(reply.scenarios.size())) {
      command_ui.selected_index = 0;
    }
  }
  if (reply.request_type == "nav.waypoints" &&
      reply.ref == command_ui.pending_waypoints_ref) {
    if (reply.ok && command_ui.draft_revision == command_ui.pending_waypoints_revision) {
      command_ui.draft_waypoints.clear();
      ++command_ui.draft_revision;
      ui.waypoint_edit_mode = false;
    }
    command_ui.pending_waypoints_ref.clear();
  }
  const bool navigation_request = reply.request_type == "nav.goto" ||
                                  reply.request_type == "nav.waypoints";
  const std::string state = local_timeout ? "timed out" :
      (reply.ok ? (navigation_request ? "accepted (not completed)" : "applied") : "rejected");
  command_ui.last_result = reply.request_type + " " + state +
      " (" + reply.ref + "): " + reply.reason;
  const auto history = std::find_if(command_ui.history.begin(), command_ui.history.end(),
      [&](const CommandUiState::HistoryEntry& entry) { return entry.ref == reply.ref; });
  if (history != command_ui.history.end()) {
    history->state = state;
    history->reason = reply.reason;
  }
  if (reply.request_type == "scenario.list" && reply.ok) {
    command_ui.last_result = "listed " + std::to_string(reply.scenarios.size()) +
        " scenarios (" + reply.ref + ")";
  }
}

struct ScenePickContext {
  bool valid = false;
  visualiser::render::Mat4 inv_view_proj{};
  float ground_y = 0.0f;
  int viewport_width = 1;
  int viewport_height = 1;
};

struct SceneBounds {
  Vec3 min{};
  Vec3 max{};
  bool valid = false;
};

bool ParsePositiveInt(const char* text, int& out_value) {
  const char* end = text;
  while (*end != '\0') {
    ++end;
  }
  int value = 0;
  const auto result = std::from_chars(text, end, value);
  if (result.ec != std::errc{} || result.ptr != end || value <= 0) {
    return false;
  }
  out_value = value;
  return true;
}

bool ParseUdpPort(const char* text, int& out_value) {
  if (!ParsePositiveInt(text, out_value)) {
    return false;
  }
  return out_value <= 65535;
}

float Clamp(float value, float lo, float hi) {
  return std::max(lo, std::min(value, hi));
}

std::size_t SkipWhitespace(std::string_view payload, std::size_t index) {
  while (index < payload.size()
         && std::isspace(static_cast<unsigned char>(payload[index])) != 0) {
    ++index;
  }
  return index;
}

Vec3 Normalize(const Vec3& value) {
  const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  if (length <= 1e-6f) {
    return {0.0f, 1.0f, 0.0f};
  }
  return {value.x / length, value.y / length, value.z / length};
}

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

float Dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Quat NormalizeQuat(const Quat& quat) {
  const float length = std::sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
  if (length <= 1e-6f) {
    return {};
  }
  return {quat.w / length, quat.x / length, quat.y / length, quat.z / length};
}

visualiser::render::Mat4 LegacyMatFromQuat(const Quat& q) {
  const Quat n = NormalizeQuat(q);
  return visualiser::render::Mat4::FromQuat(n.w, n.x, n.y, n.z);
}

Quat MultiplyQuat(const Quat& a, const Quat& b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

Vec3 RotateVector(const Quat& quat, const Vec3& vector) {
  const Quat q = NormalizeQuat(quat);
  const Quat p{0.0f, vector.x, vector.y, vector.z};
  const Quat qc{q.w, -q.x, -q.y, -q.z};
  const Quat rotated = MultiplyQuat(MultiplyQuat(q, p), qc);
  return {rotated.x, rotated.y, rotated.z};
}

Vec3 ServerToSceneVec(const Vec3& value);

Vec3 SceneToServerVec(const Vec3& value) {
  return Vec3{-value.z, value.x, value.y};
}

Vec3 RotateAroundSceneX(const Vec3& value, float roll_rad) {
  const float c = std::cos(roll_rad);
  const float s = std::sin(roll_rad);
  return Vec3{value.x, value.y * c - value.z * s, value.y * s + value.z * c};
}

Vec3 RotateAroundSceneZ(const Vec3& value, float yaw_rad) {
  const float c = std::cos(yaw_rad);
  const float s = std::sin(yaw_rad);
  return Vec3{value.x * c - value.y * s, value.x * s + value.y * c, value.z};
}

Vec3 RotateAroundSceneY(const Vec3& value, float yaw_rad) {
  const float c = std::cos(yaw_rad);
  const float s = std::sin(yaw_rad);
  return Vec3{value.x * c + value.z * s, value.y, -value.x * s + value.z * c};
}

Vec3 EffectiveOrientationRad(const HexapodBodyPoseState& pose) {
  if (pose.orientation_rad.x != 0.0f || pose.orientation_rad.y != 0.0f || pose.orientation_rad.z != 0.0f) {
    return pose.orientation_rad;
  }
  return Vec3{0.0f, 0.0f, pose.yaw_rad};
}

Vec3 RotateBodyVectorToScene(const Vec3& value, const HexapodBodyPoseState& pose) {
  const Vec3 orientation = EffectiveOrientationRad(pose);
  Vec3 rotated = ServerToSceneVec(value);
  rotated = RotateAroundSceneZ(rotated, orientation.x);
  rotated = RotateAroundSceneX(rotated, -orientation.y);
  rotated = RotateAroundSceneY(rotated, -orientation.z);
  return rotated;
}

Vec3 TransformBodyPoint(const Vec3& point, const HexapodBodyPoseState& pose) {
  if (!pose.valid) {
    return point;
  }
  const Vec3 rotated = RotateBodyVectorToScene(point, pose);
  const Vec3 origin = ServerToSceneVec(pose.position);
  return Vec3{origin.x + rotated.x, origin.y + rotated.y, origin.z + rotated.z};
}

Vec3 TransformSceneBodyPoint(const Vec3& point, const HexapodBodyPoseState& pose) {
  if (!pose.valid) {
    return point;
  }
  return TransformBodyPoint(SceneToServerVec(point), pose);
}

Vec3 TransformSceneBodyDirection(const Vec3& direction, const HexapodBodyPoseState& pose) {
  if (!pose.valid) {
    return direction;
  }
  return RotateBodyVectorToScene(SceneToServerVec(direction), pose);
}

void ExpandBounds(SceneBounds& bounds, const Vec3& point) {
  if (!bounds.valid) {
    bounds.min = point;
    bounds.max = point;
    bounds.valid = true;
    return;
  }
  bounds.min.x = std::min(bounds.min.x, point.x);
  bounds.min.y = std::min(bounds.min.y, point.y);
  bounds.min.z = std::min(bounds.min.z, point.z);
  bounds.max.x = std::max(bounds.max.x, point.x);
  bounds.max.y = std::max(bounds.max.y, point.y);
  bounds.max.z = std::max(bounds.max.z, point.z);
}

Vec3 PrimitiveBoundsHalfExtents(
    ShapeType shape,
    float radius,
    float half_height,
    const Vec3& half_extents) {
  switch (shape) {
    case ShapeType::kSphere:
      return {radius, radius, radius};
    case ShapeType::kCapsule:
      return {radius, half_height + radius, radius};
    case ShapeType::kCylinder:
    case ShapeType::kHalfCylinder:
      return {radius, half_height, radius};
    case ShapeType::kBox:
    case ShapeType::kUnknown:
    case ShapeType::kPlane:
    case ShapeType::kCompound:
      return half_extents;
  }
  return half_extents;
}

void ExpandPrimitiveBounds(
    SceneBounds& bounds,
    ShapeType shape,
    const Vec3& position,
    const Quat& rotation,
    float radius,
    float half_height,
    const Vec3& half_extents) {
  if (shape == ShapeType::kPlane || shape == ShapeType::kUnknown) {
    return;
  }
  const Vec3 extents_local = PrimitiveBoundsHalfExtents(shape, radius, half_height, half_extents);
  const Vec3 ax = RotateVector(rotation, {1.0f, 0.0f, 0.0f});
  const Vec3 ay = RotateVector(rotation, {0.0f, 1.0f, 0.0f});
  const Vec3 az = RotateVector(rotation, {0.0f, 0.0f, 1.0f});
  const Vec3 extents_world{
      std::abs(ax.x) * extents_local.x + std::abs(ay.x) * extents_local.y + std::abs(az.x) * extents_local.z,
      std::abs(ax.y) * extents_local.x + std::abs(ay.y) * extents_local.y + std::abs(az.y) * extents_local.z,
      std::abs(ax.z) * extents_local.x + std::abs(ay.z) * extents_local.y + std::abs(az.z) * extents_local.z,
  };
  ExpandBounds(bounds, {position.x - extents_world.x, position.y - extents_world.y, position.z - extents_world.z});
  ExpandBounds(bounds, {position.x + extents_world.x, position.y + extents_world.y, position.z + extents_world.z});
}

SceneBounds ComputeSceneBounds(const std::map<std::uint32_t, EntityState>& entities) {
  SceneBounds bounds;
  for (const auto& [id, entity] : entities) {
    (void)id;
    if (!entity.has_static || !entity.has_frame || entity.shape == ShapeType::kPlane) {
      continue;
    }
    if (entity.shape == ShapeType::kCompound && !entity.compound_children.empty()) {
      for (const CompoundChildState& child : entity.compound_children) {
        const Vec3 child_offset = RotateVector(entity.rotation, child.local_position);
        const Vec3 child_position{
            entity.position.x + child_offset.x,
            entity.position.y + child_offset.y,
            entity.position.z + child_offset.z,
        };
        const Quat child_rotation = MultiplyQuat(entity.rotation, child.local_rotation);
        ExpandPrimitiveBounds(
            bounds,
            child.shape,
            child_position,
            child_rotation,
            child.radius,
            child.half_height,
            child.half_extents);
      }
      continue;
    }
    ExpandPrimitiveBounds(
        bounds,
        entity.shape,
        entity.position,
        entity.rotation,
        entity.radius,
        entity.half_height,
        entity.half_extents);
  }
  return bounds;
}

void ExpandTerrainPatchBounds(SceneBounds& bounds, const TerrainPatchState& terrain) {
  if (!terrain.valid || terrain.rows <= 0 || terrain.cols <= 0 || terrain.cell_size_m <= 0.0f) {
    return;
  }

  const float half_span_x = 0.5f * static_cast<float>(std::max(0, terrain.cols - 1)) * terrain.cell_size_m;
  const float half_span_z = 0.5f * static_cast<float>(std::max(0, terrain.rows - 1)) * terrain.cell_size_m;
  const float origin_x = terrain.has_grid_origin_xz ? terrain.grid_origin_x : terrain.center.x - half_span_x;
  const float origin_z = terrain.has_grid_origin_xz ? terrain.grid_origin_z : terrain.center.z - half_span_z;
  const std::size_t expected = static_cast<std::size_t>(terrain.rows * terrain.cols);
  const bool has_heights = terrain.heights.size() >= expected;

  for (int row = 0; row < terrain.rows; ++row) {
    for (int col = 0; col < terrain.cols; ++col) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
      const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
      const float y = has_heights ? terrain.heights[index] : terrain.base_height_m;
      ExpandBounds(bounds, {x, y, z});
    }
  }
}

std::optional<std::string> ExtractStringField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":\"";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find('"', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }

  return std::string(payload.substr(value_start, value_end - value_start));
}

std::optional<std::string_view> ExtractStructuredField(
    std::string_view payload,
    std::string_view key,
    char open_char,
    char close_char) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  std::size_t cursor = SkipWhitespace(payload, start + needle.size());
  if (cursor >= payload.size() || payload[cursor] != open_char) {
    return std::nullopt;
  }

  const std::size_t value_start = cursor + 1;
  int depth = 0;
  for (; cursor < payload.size(); ++cursor) {
    if (payload[cursor] == open_char) {
      ++depth;
    } else if (payload[cursor] == close_char) {
      --depth;
      if (depth == 0) {
        return payload.substr(value_start, cursor - value_start);
      }
    }
  }

  return std::nullopt;
}

std::optional<std::string_view> ExtractObjectField(std::string_view payload, std::string_view key) {
  return ExtractStructuredField(payload, key, '{', '}');
}

std::optional<std::string_view> ExtractArrayField(std::string_view payload, std::string_view key) {
  return ExtractStructuredField(payload, key, '[', ']');
}

std::optional<std::uint32_t> ExtractUintField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return static_cast<std::uint32_t>(parsed);
}

std::optional<float> ExtractFloatField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const float parsed = std::strtof(token.c_str(), &end_ptr);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<std::array<float, 3>> ExtractFloat3Field(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":[";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find(']', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }

  std::string values(payload.substr(value_start, value_end - value_start));
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::array<float, 3> out{};
  if (!(in >> out[0] >> out[1] >> out[2])) {
    return std::nullopt;
  }
  return out;
}

std::optional<std::array<float, 4>> ExtractFloat4Field(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":[";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find(']', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }

  std::string values(payload.substr(value_start, value_end - value_start));
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::array<float, 4> out{};
  if (!(in >> out[0] >> out[1] >> out[2] >> out[3])) {
    return std::nullopt;
  }
  return out;
}

std::optional<int> ExtractIntField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const long parsed = std::strtol(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return static_cast<int>(parsed);
}

std::optional<double> ExtractDoubleField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const double parsed = std::strtod(token.c_str(), &end_ptr);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<bool> ExtractBoolField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  if (payload.substr(value_start, 4) == "true") {
    return true;
  }
  if (payload.substr(value_start, 5) == "false") {
    return false;
  }
  return std::nullopt;
}

std::optional<std::vector<float>> ExtractFloatArrayField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":[";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find(']', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }

  std::string values(payload.substr(value_start, value_end - value_start));
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::vector<float> out;
  float value = 0.0f;
  while (in >> value) {
    out.push_back(value);
  }
  return out;
}

ShapeType ParseShapeType(const std::optional<std::string>& shape_name) {
  if (!shape_name.has_value()) {
    return ShapeType::kUnknown;
  }
  if (*shape_name == "sphere") {
    return ShapeType::kSphere;
  }
  if (*shape_name == "box") {
    return ShapeType::kBox;
  }
  if (*shape_name == "plane") {
    return ShapeType::kPlane;
  }
  if (*shape_name == "capsule") {
    return ShapeType::kCapsule;
  }
  if (*shape_name == "cylinder") {
    return ShapeType::kCylinder;
  }
  if (*shape_name == "half_cylinder") {
    return ShapeType::kHalfCylinder;
  }
  if (*shape_name == "half cylinder" || *shape_name == "half-cylinder") {
    return ShapeType::kHalfCylinder;
  }
  if (*shape_name == "compound") {
    return ShapeType::kCompound;
  }
  return ShapeType::kUnknown;
}

std::vector<CompoundChildState> ParseCompoundChildren(std::string_view payload) {
  std::vector<CompoundChildState> children;
  const auto array_payload = ExtractArrayField(payload, "compound_children");
  if (!array_payload.has_value()) {
    return children;
  }

  std::size_t cursor = 0;
  while (cursor < array_payload->size()) {
    const std::size_t object_open = array_payload->find('{', cursor);
    if (object_open == std::string_view::npos) {
      break;
    }

    int depth = 0;
    std::size_t object_close = object_open;
    for (; object_close < array_payload->size(); ++object_close) {
      if ((*array_payload)[object_close] == '{') {
        ++depth;
      } else if ((*array_payload)[object_close] == '}') {
        --depth;
        if (depth == 0) {
          break;
        }
      }
    }
    if (object_close >= array_payload->size()) {
      break;
    }

    const std::string_view object_payload =
        array_payload->substr(object_open + 1, object_close - object_open - 1);
    CompoundChildState child;
    child.shape = ParseShapeType(ExtractStringField(object_payload, "shape_type"));
    if (const auto local_position = ExtractFloat3Field(object_payload, "local_position")) {
      child.local_position = {(*local_position)[0], (*local_position)[1], (*local_position)[2]};
    }
    if (const auto local_rotation = ExtractFloat4Field(object_payload, "local_rotation")) {
      child.local_rotation = {(*local_rotation)[0], (*local_rotation)[1], (*local_rotation)[2], (*local_rotation)[3]};
    }
    if (const auto radius = ExtractFloatField(object_payload, "radius")) {
      child.radius = *radius;
    }
    if (const auto half_height = ExtractFloatField(object_payload, "half_height")) {
      child.half_height = *half_height;
    }
    if (const auto half_extents = ExtractFloat3Field(object_payload, "half_extents")) {
      child.half_extents = {(*half_extents)[0], (*half_extents)[1], (*half_extents)[2]};
    }
    if (child.shape != ShapeType::kUnknown) {
      children.push_back(child);
    }

    cursor = object_close + 1;
  }

  return children;
}

HexapodGeometryState MakeDefaultGeometryState() {
  HexapodGeometryState state;
  for (std::size_t i = 0; i < state.legs.size(); ++i) {
    state.legs[i].key = kLegKeys[i];
    state.legs[i].body_coxa_offset = {
        kDefaultBodyCoxaOffsets[i][0],
        kDefaultBodyCoxaOffsets[i][1],
        kDefaultBodyCoxaOffsets[i][2],
    };
    state.legs[i].mount_angle_rad = kDefaultMountAnglesDeg[i] * kPi / 180.0f;
    state.legs[i].coxa_mm = 43.0f;
    state.legs[i].femur_mm = 60.0f;
    state.legs[i].tibia_mm = 104.0f;
    state.legs[i].coxa_attach_deg = kDefaultCoxaAttachDeg[i];
    state.legs[i].femur_attach_deg = kDefaultFemurAttachDeg[i];
    state.legs[i].tibia_attach_deg = kDefaultTibiaAttachDeg[i];
    state.legs[i].coxa_sign = (i < 3) ? -1.0f : 1.0f;
    state.legs[i].femur_sign = (i < 3) ? -1.0f : 1.0f;
    state.legs[i].tibia_sign = (i < 3) ? -1.0f : 1.0f;
  }
  state.valid = true;
  return state;
}

std::array<float, 3> ParseFloat3OrDefault(std::string_view payload, std::string_view key, const std::array<float, 3>& fallback) {
  if (const auto values = ExtractFloat3Field(payload, key)) {
    return {(*values)[0], (*values)[1], (*values)[2]};
  }
  return fallback;
}

bool ParseHexapodGeometryPacket(std::string_view payload, HexapodGeometryState& geometry) {
  const auto root_geometry = ExtractObjectField(payload, "geometry");
  if (!root_geometry.has_value()) {
    return false;
  }

  HexapodGeometryState next = geometry.valid ? geometry : MakeDefaultGeometryState();
  if (const auto coxa = ExtractDoubleField(*root_geometry, "coxa")) {
    next.coxa_mm = static_cast<float>(*coxa);
  }
  if (const auto femur = ExtractDoubleField(*root_geometry, "femur")) {
    next.femur_mm = static_cast<float>(*femur);
  }
  if (const auto tibia = ExtractDoubleField(*root_geometry, "tibia")) {
    next.tibia_mm = static_cast<float>(*tibia);
  }
  if (const auto body_radius = ExtractDoubleField(*root_geometry, "body_radius")) {
    next.body_radius_mm = static_cast<float>(*body_radius);
  }

  if (const auto legs_payload = ExtractArrayField(*root_geometry, "legs")) {
    std::size_t cursor = 0;
    std::size_t leg_index = 0;
    while (cursor < legs_payload->size() && leg_index < next.legs.size()) {
      const std::size_t object_open = legs_payload->find('{', cursor);
      if (object_open == std::string_view::npos) {
        break;
      }

      int depth = 0;
      std::size_t object_close = object_open;
      for (; object_close < legs_payload->size(); ++object_close) {
        if ((*legs_payload)[object_close] == '{') {
          ++depth;
        } else if ((*legs_payload)[object_close] == '}') {
          --depth;
          if (depth == 0) {
            break;
          }
        }
      }
      if (object_close >= legs_payload->size()) {
        break;
      }

      const std::string_view object_payload =
          legs_payload->substr(object_open + 1, object_close - object_open - 1);
      HexapodLegLayout leg = next.legs[leg_index];
      if (const auto key = ExtractStringField(object_payload, "key")) {
        leg.key = *key;
      }
      if (const auto offset = ExtractFloat3Field(object_payload, "body_coxa_offset")) {
        leg.body_coxa_offset = {(*offset)[0], (*offset)[1], (*offset)[2]};
      }
      if (const auto mount_angle = ExtractDoubleField(object_payload, "mount_angle_deg")) {
        leg.mount_angle_rad = static_cast<float>(*mount_angle) * kPi / 180.0f;
      } else if (const auto mount_angle_rad = ExtractDoubleField(object_payload, "mount_angle_rad")) {
        leg.mount_angle_rad = static_cast<float>(*mount_angle_rad);
      }
      if (const auto coxa_mm = ExtractDoubleField(object_payload, "coxa_mm")) {
        leg.coxa_mm = static_cast<float>(*coxa_mm);
      }
      if (const auto femur_mm = ExtractDoubleField(object_payload, "femur_mm")) {
        leg.femur_mm = static_cast<float>(*femur_mm);
      }
      if (const auto tibia_mm = ExtractDoubleField(object_payload, "tibia_mm")) {
        leg.tibia_mm = static_cast<float>(*tibia_mm);
      }
      if (const auto coxa_attach_deg = ExtractDoubleField(object_payload, "coxa_attach_deg")) {
        leg.coxa_attach_deg = static_cast<float>(*coxa_attach_deg);
      }
      if (const auto femur_attach_deg = ExtractDoubleField(object_payload, "femur_attach_deg")) {
        leg.femur_attach_deg = static_cast<float>(*femur_attach_deg);
      }
      if (const auto tibia_attach_deg = ExtractDoubleField(object_payload, "tibia_attach_deg")) {
        leg.tibia_attach_deg = static_cast<float>(*tibia_attach_deg);
      }
      if (const auto coxa_sign = ExtractDoubleField(object_payload, "coxa_sign")) {
        leg.coxa_sign = static_cast<float>(*coxa_sign);
      }
      if (const auto femur_sign = ExtractDoubleField(object_payload, "femur_sign")) {
        leg.femur_sign = static_cast<float>(*femur_sign);
      }
      if (const auto tibia_sign = ExtractDoubleField(object_payload, "tibia_sign")) {
        leg.tibia_sign = static_cast<float>(*tibia_sign);
      }

      next.legs[leg_index] = leg;
      ++leg_index;
      cursor = object_close + 1;
    }
  }

  next.valid = true;
  geometry = std::move(next);
  return true;
}

bool ParseAnglesPacket(std::string_view payload, std::array<std::array<float, 3>, 6>& angles_deg) {
  const auto angles_payload = ExtractObjectField(payload, "angles_deg");
  if (!angles_payload.has_value()) {
    return false;
  }

  bool any = false;
  for (std::size_t i = 0; i < kLegKeys.size(); ++i) {
    if (const auto leg_angles = ExtractFloatArrayField(*angles_payload, kLegKeys[i])) {
      if (leg_angles->size() >= 3) {
        angles_deg[i] = {(*leg_angles)[0], (*leg_angles)[1], (*leg_angles)[2]};
        any = true;
      }
    }
  }
  return any;
}

bool ParseNavigationSummary(std::string_view payload, HexapodStatusState& status) {
  const auto nav_payload = ExtractObjectField(payload, "nav");
  if (!nav_payload.has_value()) {
    return false;
  }

  if (const auto lifecycle = ExtractIntField(*nav_payload, "lifecycle")) {
    status.nav_lifecycle = lifecycle;
  }
  if (const auto planner_status = ExtractIntField(*nav_payload, "planner_status")) {
    status.nav_planner_status = planner_status;
  }
  if (const auto block_reason = ExtractIntField(*nav_payload, "block_reason")) {
    status.nav_block_reason = block_reason;
  }
  if (const auto map_fresh = ExtractBoolField(*nav_payload, "map_fresh")) {
    status.nav_map_fresh = map_fresh;
  }
  if (const auto replan_count = ExtractIntField(*nav_payload, "replan_count")) {
    status.nav_replan_count = static_cast<std::size_t>(*replan_count);
  }
  if (const auto segment_length = ExtractDoubleField(*nav_payload, "active_segment_length_m")) {
    status.nav_active_segment_length_m = *segment_length;
  }
  if (const auto waypoint_count = ExtractIntField(*nav_payload, "active_segment_waypoint_count")) {
    status.nav_active_segment_waypoint_count = static_cast<std::size_t>(*waypoint_count);
  }
  if (const auto obstacle_distance = ExtractDoubleField(*nav_payload, "nearest_obstacle_distance_m")) {
    status.nav_nearest_obstacle_distance_m = *obstacle_distance;
  }
  if (const auto idx = ExtractIntField(*nav_payload, "active_waypoint_index")) {
    status.nav_active_waypoint_index = *idx;
  }
  if (const auto dist = ExtractDoubleField(*nav_payload, "distance_to_active_waypoint_m")) {
    status.nav_distance_to_active_waypoint_m = *dist;
  }
  if (const auto has_goal = ExtractBoolField(*nav_payload, "has_goal"); has_goal && *has_goal) {
    if (const auto goal = ExtractObjectField(*nav_payload, "goal")) {
      if (const auto x = ExtractDoubleField(*goal, "x_m")) {
        status.nav_goal_x_m = static_cast<float>(*x);
      }
      if (const auto y = ExtractDoubleField(*goal, "y_m")) {
        status.nav_goal_y_m = static_cast<float>(*y);
      }
      if (const auto yaw = ExtractDoubleField(*goal, "yaw_rad")) {
        status.nav_goal_yaw_rad = static_cast<float>(*yaw);
      }
    }
  } else {
    status.nav_goal_x_m.reset();
    status.nav_goal_y_m.reset();
    status.nav_goal_yaw_rad.reset();
  }
  status.nav_active_segment.clear();
  if (const auto segment = ExtractArrayField(*nav_payload, "active_segment")) {
    std::size_t i = 0;
    while (i < segment->size()) {
      i = SkipWhitespace(*segment, i);
      if (i >= segment->size() || (*segment)[i] != '{') {
        break;
      }
      int depth = 0;
      std::size_t j = i;
      for (; j < segment->size(); ++j) {
        if ((*segment)[j] == '{') {
          ++depth;
        } else if ((*segment)[j] == '}') {
          --depth;
          if (depth == 0) {
            break;
          }
        }
      }
      if (j >= segment->size()) {
        break;
      }
      const std::string_view object = segment->substr(i, j - i + 1);
      std::array<float, 3> pose{0.0f, 0.0f, 0.0f};
      if (const auto x = ExtractDoubleField(object, "x_m")) {
        pose[0] = static_cast<float>(*x);
      }
      if (const auto y = ExtractDoubleField(object, "y_m")) {
        pose[1] = static_cast<float>(*y);
      }
      if (const auto yaw = ExtractDoubleField(object, "yaw_rad")) {
        pose[2] = static_cast<float>(*yaw);
      }
      status.nav_active_segment.push_back(pose);
      i = j + 1;
    }
  }
  return true;
}

bool ParseCommandSummary(std::string_view payload, HexapodStatusState& status) {
  const auto command_payload = ExtractObjectField(payload, "command");
  if (!command_payload.has_value()) {
    return false;
  }
  if (const auto authority = ExtractStringField(*command_payload, "authority")) {
    status.command_authority = authority;
  }
  if (const auto scenario = ExtractStringField(*command_payload, "scenario")) {
    status.command_scenario = scenario;
  }
  if (const auto nav_active = ExtractBoolField(*command_payload, "nav_active")) {
    status.command_nav_active = nav_active;
  }
  return status.command_authority.has_value() || status.command_scenario.has_value() ||
         status.command_nav_active.has_value();
}

bool ParseLocalMapOverlay(std::string_view payload, LocalMapOverlayState& map) {
  const auto map_payload = ExtractObjectField(payload, "local_map");
  if (!map_payload.has_value()) {
    return false;
  }
  LocalMapOverlayState next{};
  if (const auto fresh = ExtractBoolField(*map_payload, "fresh")) {
    next.fresh = *fresh;
  }
  if (const auto w = ExtractIntField(*map_payload, "width_cells")) {
    next.width_cells = *w;
  }
  if (const auto h = ExtractIntField(*map_payload, "height_cells")) {
    next.height_cells = *h;
  }
  if (const auto step = ExtractIntField(*map_payload, "cell_step")) {
    next.cell_step = std::max(1, *step);
  }
  if (const auto res = ExtractDoubleField(*map_payload, "resolution_m")) {
    next.resolution_m = static_cast<float>(*res);
  }
  if (const auto center = ExtractObjectField(*map_payload, "center_pose")) {
    if (const auto x = ExtractDoubleField(*center, "x_m")) {
      next.center_x_m = static_cast<float>(*x);
    }
    if (const auto y = ExtractDoubleField(*center, "y_m")) {
      next.center_y_m = static_cast<float>(*y);
    }
    if (const auto yaw = ExtractDoubleField(*center, "yaw_rad")) {
      next.center_yaw_rad = static_cast<float>(*yaw);
    }
  }
  if (const auto cells = ExtractArrayField(*map_payload, "cells")) {
    std::size_t i = 0;
    while (i < cells->size()) {
      i = SkipWhitespace(*cells, i);
      if (i >= cells->size()) {
        break;
      }
      const std::string token(cells->substr(i));
      char* end = nullptr;
      const long value = std::strtol(token.c_str(), &end, 10);
      if (end == token.c_str()) {
        ++i;
        continue;
      }
      next.cells.push_back(static_cast<std::uint8_t>(std::clamp(value, 0L, 255L)));
      i += static_cast<std::size_t>(end - token.c_str());
    }
  }
  const int out_w = (next.width_cells + next.cell_step - 1) / next.cell_step;
  const int out_h = (next.height_cells + next.cell_step - 1) / next.cell_step;
  next.valid = out_w > 0 && out_h > 0 &&
               next.cells.size() >= static_cast<std::size_t>(out_w * out_h);
  if (next.valid) {
    map = std::move(next);
  }
  return next.valid;
}

bool ParseLocomotionOverlay(std::string_view payload, LocomotionOverlayState& loco) {
  loco = LocomotionOverlayState{};
  const auto debug = ExtractObjectField(payload, "locomotion_debug");
  if (!debug.has_value() || ExtractBoolField(*debug, "valid") != true) {
    return false;
  }
  LocomotionOverlayState next{};
  auto parse_vec3_array = [&](std::string_view key, std::array<Vec3, 6>& out) {
    const auto arr = ExtractArrayField(*debug, key);
    if (!arr) {
      return false;
    }
    std::size_t leg = 0;
    std::size_t i = 0;
    while (i < arr->size() && leg < out.size()) {
      i = SkipWhitespace(*arr, i);
      if (i >= arr->size() || (*arr)[i] != '[') {
        break;
      }
      const auto close = arr->find(']', i);
      if (close == std::string_view::npos) {
        break;
      }
      std::string triple(arr->substr(i + 1, close - i - 1));
      std::replace(triple.begin(), triple.end(), ',', ' ');
      std::istringstream in(triple);
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      std::string extra;
      if (!(in >> x >> y >> z) || (in >> extra) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return false;
      }
      out[leg++] = Vec3{x, y, z};
      i = close + 1;
      while (i < arr->size() && ((*arr)[i] == ',' || (*arr)[i] == ' ')) {
        ++i;
      }
    }
    return leg == out.size() && SkipWhitespace(*arr, i) == arr->size();
  };
  auto parse_bool_array = [&](std::string_view key, std::array<bool, 6>& out) {
    const auto values = ExtractArrayField(*debug, key);
    if (!values) {
      return false;
    }
    std::size_t leg = 0;
    std::size_t i = 0;
    while (i < values->size() && leg < out.size()) {
      i = SkipWhitespace(*values, i);
      if (values->substr(i, 4) == "true") {
        out[leg++] = true;
        i += 4;
      } else if (values->substr(i, 5) == "false") {
        out[leg++] = false;
        i += 5;
      } else {
        return false;
      }
      i = SkipWhitespace(*values, i);
      if (i < values->size() && (*values)[i] == ',') {
        ++i;
      }
    }
    return leg == out.size() && SkipWhitespace(*values, i) == values->size();
  };
  next.has_commanded_feet = parse_vec3_array("commanded_foot_world_m", next.commanded_foot_world_m);
  next.has_measured_feet = parse_vec3_array("measured_foot_world_m", next.measured_foot_world_m);
  next.has_planned_targets = parse_vec3_array("planned_leg_target_body_m", next.planned_leg_target_body_m);
  next.has_planned_stance = parse_bool_array("planned_stance", next.planned_stance);
  next.has_raw_contact = parse_bool_array("raw_contact", next.raw_contact);
  next.has_fused_support = parse_bool_array("fused_support", next.fused_support);
  if (const auto values = ExtractFloatArrayField(*debug, "commanded_tracking_error_m");
      values && values->size() == next.commanded_tracking_error_m.size() &&
      std::all_of(values->begin(), values->end(), [](float value) { return std::isfinite(value); })) {
    std::copy(values->begin(), values->end(), next.commanded_tracking_error_m.begin());
    next.has_tracking_error = true;
  }
  if (const auto distortion = ExtractFloatField(*debug, "max_post_clamp_distortion_m");
      distortion && std::isfinite(*distortion)) {
    next.max_post_clamp_distortion_m = *distortion;
  }
  next.valid = next.has_commanded_feet || next.has_measured_feet ||
               next.has_planned_targets || next.has_planned_stance ||
               next.has_raw_contact || next.has_fused_support || next.has_tracking_error;
  if (next.valid) {
    loco = next;
  }
  return next.valid;
}

bool ParseFusionSummary(std::string_view payload, HexapodStatusState& status) {
  const auto fusion_payload = ExtractObjectField(payload, "fusion");
  if (!fusion_payload.has_value()) {
    return false;
  }

  if (const auto model_trust = ExtractDoubleField(*fusion_payload, "model_trust")) {
    status.fusion_model_trust = *model_trust;
  }
  if (const auto resync_requested = ExtractBoolField(*fusion_payload, "resync_requested")) {
    status.fusion_resync_requested = *resync_requested;
  }
  if (const auto hard_reset_requested = ExtractBoolField(*fusion_payload, "hard_reset_requested")) {
    status.fusion_hard_reset_requested = *hard_reset_requested;
  }
  if (const auto predictive_mode = ExtractBoolField(*fusion_payload, "predictive_mode")) {
    status.fusion_predictive_mode = *predictive_mode;
  }

  const auto residuals = ExtractObjectField(*fusion_payload, "residuals");
  if (residuals.has_value()) {
    if (const auto value = ExtractDoubleField(*residuals, "max_body_position_error_m")) {
      status.fusion_max_body_position_error_m = *value;
    }
    if (const auto value = ExtractDoubleField(*residuals, "max_body_orientation_error_rad")) {
      status.fusion_max_body_orientation_error_rad = *value;
    }
    if (const auto value = ExtractDoubleField(*residuals, "contact_mismatch_ratio")) {
      status.fusion_contact_mismatch_ratio = *value;
    }
    if (const auto value = ExtractDoubleField(*residuals, "terrain_residual_m")) {
      status.fusion_terrain_residual_m = *value;
    }
  }

  return true;
}

bool ParseHexapodTelemetryPacket(std::string_view payload, HexapodTelemetryState& telemetry) {
  const auto packet_type = ExtractStringField(payload, "type");
  if (!packet_type.has_value()) {
    return false;
  }

  if (*packet_type == "geometry") {
    return ParseHexapodGeometryPacket(payload, telemetry.geometry) ? (telemetry.has_geometry = true, true) : false;
  }

  if (*packet_type != "joints") {
    return false;
  }

  if (ParseHexapodGeometryPacket(payload, telemetry.geometry)) {
    telemetry.has_geometry = true;
  }
  telemetry.has_joints = ParseAnglesPacket(payload, telemetry.angles_deg) || telemetry.has_joints;
  if (const auto body_position = ExtractFloat3Field(payload, "body_position")) {
    telemetry.body_pose.position = {(*body_position)[0], (*body_position)[1], (*body_position)[2]};
    telemetry.body_pose.valid = true;
  }
  if (const auto body_orientation = ExtractFloat3Field(payload, "body_orientation_rad")) {
    telemetry.body_pose.orientation_rad =
        {(*body_orientation)[0], (*body_orientation)[1], (*body_orientation)[2]};
    telemetry.body_pose.yaw_rad = telemetry.body_pose.orientation_rad.z;
    telemetry.body_pose.valid = true;
  } else if (const auto body_yaw = ExtractFloatField(payload, "body_yaw_rad")) {
    telemetry.body_pose.orientation_rad = {0.0f, 0.0f, *body_yaw};
    telemetry.body_pose.yaw_rad = *body_yaw;
    telemetry.body_pose.valid = true;
  }

  if (const auto timestamp_ms = ExtractUintField(payload, "timestamp_ms")) {
    telemetry.status.timestamp_ms = *timestamp_ms;
  }
  if (const auto loop_counter = ExtractIntField(payload, "loop_counter")) {
    telemetry.status.loop_counter = *loop_counter;
  }
  if (const auto active_mode = ExtractIntField(payload, "mode")) {
    telemetry.status.active_mode = *active_mode;
  }
  telemetry.status.active_fault = -1;
  if (const auto active_fault = ExtractIntField(payload, "active_fault")) {
    telemetry.status.active_fault = *active_fault;
  } else if (const auto active_fault_name = ExtractStringField(payload, "active_fault")) {
    if (const auto fault = visualiser::robot::ParseFaultCodeName(*active_fault_name)) {
      telemetry.status.active_fault = *fault;
    }
  }
  if (const auto bus_ok = ExtractBoolField(payload, "bus_ok")) {
    telemetry.status.bus_ok = *bus_ok;
  }
  if (const auto estimator_valid = ExtractBoolField(payload, "estimator_valid")) {
    telemetry.status.estimator_valid = *estimator_valid;
  }
  if (const auto voltage = ExtractDoubleField(payload, "voltage")) {
    telemetry.status.voltage = static_cast<float>(*voltage);
  }
  if (const auto current = ExtractDoubleField(payload, "current")) {
    telemetry.status.current = static_cast<float>(*current);
  }

  telemetry.status.valid = true;
  ParseNavigationSummary(payload, telemetry.status);
  ParseCommandSummary(payload, telemetry.status);
  ParseLocalMapOverlay(payload, telemetry.local_map);
  ParseLocomotionOverlay(payload, telemetry.locomotion);
  ParseFusionSummary(payload, telemetry.status);
  telemetry.status.requested_planar_speed_mps.reset();
  telemetry.status.governed_planar_speed_mps.reset();
  telemetry.status.physics_peak_servo_torque_utilization.reset();
  if (const auto governor = ExtractObjectField(payload, "governor")) {
    if (const auto speed = ExtractDoubleField(*governor, "requested_planar_speed_mps");
        speed && std::isfinite(*speed)) {
      telemetry.status.requested_planar_speed_mps = *speed;
    }
    if (const auto speed = ExtractDoubleField(*governor, "governed_planar_speed_mps");
        speed && std::isfinite(*speed)) {
      telemetry.status.governed_planar_speed_mps = *speed;
    }
  }
  if (const auto physics = ExtractObjectField(payload, "physics_sim")) {
    if (const auto utilization = ExtractDoubleField(*physics, "peak_servo_torque_utilization");
        utilization && std::isfinite(*utilization)) {
      telemetry.status.physics_peak_servo_torque_utilization = *utilization;
    }
  }
  telemetry.status.valid = telemetry.status.valid || telemetry.status.nav_lifecycle.has_value() ||
                          telemetry.status.fusion_model_trust.has_value();
  return true;
}

const char* RobotModeName(int mode) {
  switch (mode) {
    case 0:
      return "SAFE_IDLE";
    case 1:
      return "HOMING";
    case 2:
      return "STAND";
    case 3:
      return "WALK";
    case 4:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

const char* NavigationLifecycleName(int value) {
  switch (value) {
    case 0:
      return "Idle";
    case 1:
      return "Running";
    case 2:
      return "Paused";
    case 3:
      return "Blocked";
    case 4:
      return "MapUnavailable";
    case 5:
      return "Completed";
    case 6:
      return "Failed";
    case 7:
      return "Cancelled";
    default:
      return "Unknown";
  }
}

const char* LocalPlanStatusName(int value) {
  switch (value) {
    case 0:
      return "Ready";
    case 1:
      return "GoalReached";
    case 2:
      return "Blocked";
    case 3:
      return "MapUnavailable";
    default:
      return "Unknown";
  }
}

const char* PlannerBlockReasonName(int value) {
  switch (value) {
    case 0:
      return "None";
    case 1:
      return "StartOccupied";
    case 2:
      return "GoalOccupied";
    case 3:
      return "NoPath";
    case 4:
      return "SearchBudgetExceeded";
    default:
      return "Unknown";
  }
}

const char* ContactPhaseName(int value) {
  switch (value) {
    case 0:
      return "Swing";
    case 1:
      return "ExpectedTouchdown";
    case 2:
      return "ContactCandidate";
    case 3:
      return "ConfirmedStance";
    case 4:
      return "LostCandidate";
    case 5:
      return "Search";
    default:
      return "Unknown";
  }
}

struct RobotKinematics {
  Vec3 anchor{};
  Vec3 shoulder{};
  Vec3 knee{};
  Vec3 foot{};
};

Vec3 ServerToSceneVec(const Vec3& value) {
  return Vec3{value.y, value.z, -value.x};
}

bool PickGroundServerXY(const ScenePickContext& pick,
                        float mouse_x,
                        float mouse_y,
                        int window_width,
                        int window_height,
                        float& out_x_m,
                        float& out_y_m) {
  if (!pick.valid || pick.viewport_width <= 0 || pick.viewport_height <= 0) {
    return false;
  }
  visualiser::math::Vec3 scene;
  if (!visualiser::scene::PickHorizontalGround(pick.inv_view_proj, mouse_x, mouse_y,
                                               window_width, window_height, pick.ground_y, scene)) {
    return false;
  }
  const Vec3 server = SceneToServerVec({scene.x, scene.y, scene.z});
  out_x_m = server.x;
  out_y_m = server.y;
  return true;
}

RobotKinematics ComputeRobotLeg(const HexapodLegLayout& layout, const std::array<float, 3>& angles_deg) {
  visualiser::robot::HexapodLegLayout model_layout;
  model_layout.body_coxa_offset = {layout.body_coxa_offset.x, layout.body_coxa_offset.y,
                                   layout.body_coxa_offset.z};
  model_layout.mount_angle_rad = layout.mount_angle_rad;
  model_layout.coxa_mm = layout.coxa_mm;
  model_layout.femur_mm = layout.femur_mm;
  model_layout.tibia_mm = layout.tibia_mm;
  model_layout.coxa_attach_deg = layout.coxa_attach_deg;
  model_layout.femur_attach_deg = layout.femur_attach_deg;
  model_layout.tibia_attach_deg = layout.tibia_attach_deg;
  model_layout.coxa_sign = layout.coxa_sign;
  model_layout.femur_sign = layout.femur_sign;
  model_layout.tibia_sign = layout.tibia_sign;
  const auto leg = visualiser::robot::ComputeRobotLeg(model_layout, angles_deg);
  return {{leg.coxa.x, leg.coxa.y, leg.coxa.z},
          {leg.femur.x, leg.femur.y, leg.femur.z},
          {leg.tibia.x, leg.tibia.y, leg.tibia.z},
          {leg.foot.x, leg.foot.y, leg.foot.z}};
}

RobotKinematics ComputeRobotLegScene(const HexapodLegLayout& layout,
                                     const std::array<float, 3>& angles_deg) {
  const RobotKinematics server = ComputeRobotLeg(layout, angles_deg);
  return RobotKinematics{
      ServerToSceneVec(server.anchor),
      ServerToSceneVec(server.shoulder),
      ServerToSceneVec(server.knee),
      ServerToSceneVec(server.foot),
  };
}

void ExpandServerBoundsInScene(SceneBounds& bounds, const Vec3& min_corner, const Vec3& max_corner) {
  const std::array<Vec3, 8> corners = {{
      {min_corner.x, min_corner.y, min_corner.z},
      {max_corner.x, min_corner.y, min_corner.z},
      {max_corner.x, max_corner.y, min_corner.z},
      {min_corner.x, max_corner.y, min_corner.z},
      {min_corner.x, min_corner.y, max_corner.z},
      {max_corner.x, min_corner.y, max_corner.z},
      {max_corner.x, max_corner.y, max_corner.z},
      {min_corner.x, max_corner.y, max_corner.z},
  }};

  for (const Vec3& corner : corners) {
    ExpandBounds(bounds, ServerToSceneVec(corner));
  }
}

SceneBounds ComputeRobotBounds(const HexapodGeometryState& geometry,
                               const std::array<std::array<float, 3>, 6>& angles_deg,
                               const HexapodBodyPoseState& pose) {
  SceneBounds bounds;
  for (std::size_t i = 0; i < geometry.legs.size(); ++i) {
    const RobotKinematics leg = ComputeRobotLeg(geometry.legs[i], angles_deg[i]);
    ExpandBounds(bounds, TransformBodyPoint(leg.anchor, pose));
    ExpandBounds(bounds, TransformBodyPoint(leg.shoulder, pose));
    ExpandBounds(bounds, TransformBodyPoint(leg.knee, pose));
    ExpandBounds(bounds, TransformBodyPoint(leg.foot, pose));
  }
  const float body_radius = geometry.valid ? geometry.body_radius_mm * 0.001f : 0.06f;
  const std::array<Vec3, 8> body_corners = {{
      {-body_radius, -body_radius, -0.04f},
      {body_radius, -body_radius, -0.04f},
      {body_radius, body_radius, -0.04f},
      {-body_radius, body_radius, -0.04f},
      {-body_radius, -body_radius, 0.04f},
      {body_radius, -body_radius, 0.04f},
      {body_radius, body_radius, 0.04f},
      {-body_radius, body_radius, 0.04f},
  }};
  for (const Vec3& corner : body_corners) {
    ExpandBounds(bounds, TransformBodyPoint(corner, pose));
  }
  return bounds;
}

void DrawPrimitiveShape(
    ShapeType shape,
    float radius,
    float half_height,
    const Vec3& half_extents,
    const Vec3& plane_normal,
    float plane_offset,
    const visualiser::render::Mat4& model) {
  using RV = visualiser::render::Vec3;
  switch (shape) {
    case ShapeType::kBox:
      visualiser::render::AppendWireBox(
          g_line_renderer, model, RV{half_extents.x, half_extents.y, half_extents.z}, RV{0.92f, 0.43f, 0.21f});
      break;
    case ShapeType::kSphere:
      visualiser::render::AppendWireSphere(g_line_renderer, model, radius, RV{0.22f, 0.75f, 0.91f});
      break;
    case ShapeType::kCapsule:
      visualiser::render::AppendWireCapsule(g_line_renderer, model, radius, half_height, RV{0.37f, 0.82f, 0.51f});
      break;
    case ShapeType::kCylinder:
      visualiser::render::AppendWireCylinder(g_line_renderer, model, radius, half_height, RV{0.88f, 0.78f, 0.26f});
      break;
    case ShapeType::kHalfCylinder:
      visualiser::render::AppendWireHalfCylinder(g_line_renderer, model, radius, half_height, RV{0.82f, 0.56f, 0.88f});
      break;
    case ShapeType::kPlane:
      visualiser::render::AppendPlane(g_mesh_renderer,
                                      g_line_renderer,
                                      visualiser::render::Mat4::Identity(),
                                      RV{plane_normal.x, plane_normal.y, plane_normal.z},
                                      plane_offset,
                                      RV{0.14f, 0.18f, 0.22f},
                                      RV{0.28f, 0.34f, 0.40f});
      break;
    case ShapeType::kCompound:
    case ShapeType::kUnknown:
      break;
  }
}

void DrawTerrainPatch(const TerrainPatchState& terrain) {
  using RV = visualiser::render::Vec3;
  if (!terrain.valid || terrain.rows <= 0 || terrain.cols <= 0 || terrain.cell_size_m <= 0.0f) {
    return;
  }
  const std::size_t expected = static_cast<std::size_t>(terrain.rows * terrain.cols);
  if (terrain.heights.size() < expected) {
    return;
  }
  const float half_span_x = 0.5f * static_cast<float>(std::max(0, terrain.cols - 1)) * terrain.cell_size_m;
  const float half_span_z = 0.5f * static_cast<float>(std::max(0, terrain.rows - 1)) * terrain.cell_size_m;
  const float origin_x = terrain.has_grid_origin_xz ? terrain.grid_origin_x : terrain.center.x - half_span_x;
  const float origin_z = terrain.has_grid_origin_xz ? terrain.grid_origin_z : terrain.center.z - half_span_z;
  const RV col_main{0.30f, 0.78f, 0.48f};
  for (int row = 0; row < terrain.rows; ++row) {
    RV prev{};
    for (int col = 0; col < terrain.cols; ++col) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
      const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
      const RV cur{x, terrain.heights[index], z};
      if (col > 0) {
        g_line_renderer.AddSegment(prev, cur, col_main);
      }
      prev = cur;
    }
  }
  for (int col = 0; col < terrain.cols; ++col) {
    RV prev{};
    for (int row = 0; row < terrain.rows; ++row) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
      const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
      const RV cur{x, terrain.heights[index], z};
      if (row > 0) {
        g_line_renderer.AddSegment(prev, cur, col_main);
      }
      prev = cur;
    }
  }
  if (terrain.schema_version >= 2 && terrain.collision_heights.size() >= expected) {
    const RV col_col{0.95f, 0.45f, 0.18f};
    for (int row = 0; row < terrain.rows; ++row) {
      RV prev{};
      for (int col = 0; col < terrain.cols; ++col) {
        const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
        const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
        const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
        const RV cur{x, terrain.collision_heights[index], z};
        if (col > 0) {
          g_line_renderer.AddSegment(prev, cur, col_col);
        }
        prev = cur;
      }
    }
    for (int col = 0; col < terrain.cols; ++col) {
      RV prev{};
      for (int row = 0; row < terrain.rows; ++row) {
        const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
        const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
        const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
        const RV cur{x, terrain.collision_heights[index], z};
        if (row > 0) {
          g_line_renderer.AddSegment(prev, cur, col_col);
        }
        prev = cur;
      }
    }
  }
  const Vec3 up = Normalize(terrain.plane_normal);
  const float arrow_scale = std::max(terrain.cell_size_m, 0.08f);
  const RV normal_base{terrain.center.x, terrain.base_height_m, terrain.center.z};
  const RV normal_tip{normal_base.x + up.x * arrow_scale, normal_base.y + up.y * arrow_scale,
                      normal_base.z + up.z * arrow_scale};
  g_line_renderer.AddSegment(normal_base, normal_tip, RV{0.95f, 0.88f, 0.24f});
  g_point_renderer.AddPoint(normal_base, RV{0.95f, 0.88f, 0.24f});
}

void DrawGroundReference(const TerrainPatchState& terrain, const HexapodBodyPoseState& pose) {
  if (!terrain.valid || terrain.rows <= 0 || terrain.cols <= 0 || terrain.cell_size_m <= 0.0f ||
      terrain.heights.size() < static_cast<std::size_t>(terrain.rows * terrain.cols)) {
    return;
  }
  const Vec3 center = pose.valid ? ServerToSceneVec(pose.position) : terrain.center;
  const float half_patch_x = 0.5f * static_cast<float>(terrain.cols - 1) * terrain.cell_size_m;
  const float half_patch_z = 0.5f * static_cast<float>(terrain.rows - 1) * terrain.cell_size_m;
  const float origin_x = terrain.has_grid_origin_xz ? terrain.grid_origin_x : terrain.center.x - half_patch_x;
  const float origin_z = terrain.has_grid_origin_xz ? terrain.grid_origin_z : terrain.center.z - half_patch_z;
  const visualiser::scene::GroundGridExclusion patch{
      true, origin_x, origin_x + 2.0f * half_patch_x,
      origin_z, origin_z + 2.0f * half_patch_z};
  const auto lines = visualiser::scene::BuildGroundReferenceGrid(
      center.x, center.z, terrain.plane_height_m, 1.0f, 0.10f, patch);
  const visualiser::render::Vec3 color{0.16f, 0.40f, 0.33f};
  for (const auto& line : lines) {
    g_line_renderer.AddSegment({line.a.x, line.a.y, line.a.z},
                               {line.b.x, line.b.y, line.b.z}, color);
  }
}

void DrawHexapodModel(const HexapodGeometryState& geometry,
                      const std::array<std::array<float, 3>, 6>& angles_deg,
                      const HexapodStatusState& status,
                      const HexapodBodyPoseState& pose) {
  using RV = visualiser::render::Vec3;
  if (!geometry.valid) {
    return;
  }
  const SceneBounds bounds = ComputeRobotBounds(geometry, angles_deg, pose);
  if (!bounds.valid) {
    return;
  }
  const std::array<std::size_t, 6> body_loop = {0, 1, 2, 5, 4, 3};
  const float body_height = 0.015f;
  const bool healthy = status.bus_ok && status.estimator_valid && status.active_fault == 0;
  const RV fill_c{healthy ? 0.18f : 0.35f, healthy ? 0.32f : 0.18f, healthy ? 0.48f : 0.12f};
  const RV line_c{healthy ? 0.44f : 0.90f, healthy ? 0.74f : 0.32f, healthy ? 1.00f : 0.18f};
  const auto transform_scene_point = [&pose](const RV& local_point) {
    const Vec3 world = TransformSceneBodyPoint(Vec3{local_point.x, local_point.y, local_point.z}, pose);
    return RV{world.x, world.y, world.z};
  };
  const auto transform_scene_dir = [&pose](const RV& direction) {
    const Vec3 world = TransformSceneBodyDirection(Vec3{direction.x, direction.y, direction.z}, pose);
    return RV{world.x, world.y, world.z};
  };
  std::array<RV, 6> hull{};
  for (std::size_t i = 0; i < body_loop.size(); ++i) {
    const RobotKinematics leg = ComputeRobotLegScene(geometry.legs[body_loop[i]], angles_deg[body_loop[i]]);
    hull[i] = transform_scene_point(RV{leg.anchor.x, body_height, leg.anchor.z});
  }
  const RV body_n = transform_scene_dir(RV{0.0f, 1.0f, 0.0f});
  for (std::size_t k = 1; k + 1 < hull.size(); ++k) {
    g_mesh_renderer.AddTriangle(hull[0], hull[k], hull[k + 1], body_n, fill_c);
  }
  for (std::size_t k = 0; k < hull.size(); ++k) {
    const std::size_t next = (k + 1) % hull.size();
    g_line_renderer.AddSegment(hull[k], hull[next], line_c);
  }
  g_point_renderer.AddPoint(transform_scene_point(RV{0.0f, body_height, 0.0f}), RV{0.95f, 0.93f, 0.85f});
  for (std::size_t i = 0; i < geometry.legs.size(); ++i) {
    const RobotKinematics leg = ComputeRobotLegScene(geometry.legs[i], angles_deg[i]);
    const RV a = transform_scene_point(RV{leg.anchor.x, leg.anchor.y, leg.anchor.z});
    const RV s = transform_scene_point(RV{leg.shoulder.x, leg.shoulder.y, leg.shoulder.z});
    const RV kk = transform_scene_point(RV{leg.knee.x, leg.knee.y, leg.knee.z});
    const RV f = transform_scene_point(RV{leg.foot.x, leg.foot.y, leg.foot.z});
    g_line_renderer.AddSegment(a, s, RV{0.95f, 0.58f, 0.18f});
    g_line_renderer.AddSegment(s, kk, RV{0.30f, 0.80f, 0.42f});
    g_line_renderer.AddSegment(kk, f, RV{0.25f, 0.72f, 0.95f});
    g_point_renderer.AddPoint(a, RV{0.96f, 0.94f, 0.90f});
    g_point_renderer.AddPoint(s, RV{0.96f, 0.94f, 0.90f});
    g_point_renderer.AddPoint(kk, RV{0.96f, 0.94f, 0.90f});
    g_point_renderer.AddPoint(f, RV{0.96f, 0.94f, 0.90f});
  }
}
void LogJointPositions(const HexapodGeometryState& geometry,
                       const std::array<std::array<float, 3>, 6>& angles_deg,
                       const HexapodBodyPoseState& pose,
                       double now_s) {
  std::ostringstream line;
  line << std::fixed << std::setprecision(3);
  line << "[viz-joints t=" << now_s << "s]";
  for (std::size_t leg = 0; leg < geometry.legs.size() && leg < kLegKeys.size(); ++leg) {
    const RobotKinematics kinematics = ComputeRobotLeg(geometry.legs[leg], angles_deg[leg]);
    const Vec3 anchor_scene = TransformBodyPoint(kinematics.anchor, pose);
    const Vec3 shoulder_scene = TransformBodyPoint(kinematics.shoulder, pose);
    const Vec3 knee_scene = TransformBodyPoint(kinematics.knee, pose);
    const Vec3 foot_scene = TransformBodyPoint(kinematics.foot, pose);
    line << " " << kLegKeys[leg] << ".coxa=(" << anchor_scene.x << "," << anchor_scene.y << "," << anchor_scene.z
         << ")";
    line << " " << kLegKeys[leg] << ".femur=(" << shoulder_scene.x << "," << shoulder_scene.y << ","
         << shoulder_scene.z << ")";
    line << " " << kLegKeys[leg] << ".knee=(" << knee_scene.x << "," << knee_scene.y << "," << knee_scene.z
         << ")";
    line << " " << kLegKeys[leg] << ".foot=(" << foot_scene.x << "," << foot_scene.y << "," << foot_scene.z
         << ")";
  }
  std::cout << line.str() << "\n";
}

bool ParseTerrainPatchPacket(const std::string& payload, TerrainPatchState& terrain_patch) {
  const auto message_type = ExtractStringField(payload, "message_type");
  if (!message_type.has_value() || *message_type != "terrain_patch") {
    return false;
  }

  TerrainPatchState next = terrain_patch;
  if (const auto schema = ExtractUintField(payload, "schema_version")) {
    next.schema_version = static_cast<int>(*schema);
  }
  if (const auto frame = ExtractUintField(payload, "frame")) {
    next.frame = static_cast<int>(*frame);
  }
  if (const auto sim_time_s = ExtractFloatField(payload, "sim_time_s")) {
    next.sim_time_s = *sim_time_s;
  }
  if (const auto rows = ExtractUintField(payload, "rows")) {
    next.rows = static_cast<int>(*rows);
  }
  if (const auto cols = ExtractUintField(payload, "cols")) {
    next.cols = static_cast<int>(*cols);
  }
  if (const auto cell_size = ExtractFloatField(payload, "cell_size_m")) {
    next.cell_size_m = *cell_size;
  }
  if (const auto base_margin = ExtractFloatField(payload, "base_margin_m")) {
    next.base_margin_m = *base_margin;
  }
  if (const auto min_cell = ExtractFloatField(payload, "min_cell_thickness_m")) {
    next.min_cell_thickness_m = *min_cell;
  }
  if (const auto sigma = ExtractFloatField(payload, "influence_sigma_m")) {
    next.influence_sigma_m = *sigma;
  }
  if (const auto plane_conf = ExtractFloatField(payload, "plane_confidence")) {
    next.plane_confidence = *plane_conf;
  }
  if (const auto half_life = ExtractFloatField(payload, "confidence_half_life_s")) {
    next.confidence_half_life_s = *half_life;
  }
  if (const auto base_blend = ExtractFloatField(payload, "base_update_blend")) {
    next.base_update_blend = *base_blend;
  }
  if (const auto decay_boost = ExtractFloatField(payload, "decay_update_boost")) {
    next.decay_update_boost = *decay_boost;
  }
  if (const auto center = ExtractFloat3Field(payload, "center")) {
    next.center = {(*center)[0], (*center)[1], (*center)[2]};
  }
  if (const auto origin = ExtractFloatArrayField(payload, "grid_origin_xz")) {
    if (origin->size() >= 2) {
      next.grid_origin_x = (*origin)[0];
      next.grid_origin_z = (*origin)[1];
      next.has_grid_origin_xz = true;
    }
  } else {
    next.has_grid_origin_xz = false;
  }
  if (const auto base_height = ExtractFloatField(payload, "base_height_m")) {
    next.base_height_m = *base_height;
  }
  if (const auto plane_height = ExtractFloatField(payload, "plane_height_m")) {
    next.plane_height_m = *plane_height;
  }
  if (const auto normal = ExtractFloat3Field(payload, "plane_normal")) {
    next.plane_normal = {(*normal)[0], (*normal)[1], (*normal)[2]};
  }
  if (const auto heights = ExtractFloatArrayField(payload, "heights")) {
    next.heights = *heights;
  }
  if (const auto confidences = ExtractFloatArrayField(payload, "confidences")) {
    next.confidences = *confidences;
  }
  if (const auto collision = ExtractFloatArrayField(payload, "collision_heights")) {
    next.collision_heights = *collision;
  } else {
    next.collision_heights.clear();
  }

  next.valid = next.rows > 0 && next.cols > 0 && next.cell_size_m > 0.0f;
  terrain_patch = std::move(next);
  return terrain_patch.valid;
}

ShapeType ShapeFromViz(minphys_viz::VizShapeType v) {
  switch (v) {
    case minphys_viz::VizShapeType::Sphere:
      return ShapeType::kSphere;
    case minphys_viz::VizShapeType::Box:
      return ShapeType::kBox;
    case minphys_viz::VizShapeType::Plane:
      return ShapeType::kPlane;
    case minphys_viz::VizShapeType::Capsule:
      return ShapeType::kCapsule;
    case minphys_viz::VizShapeType::Cylinder:
      return ShapeType::kCylinder;
    case minphys_viz::VizShapeType::HalfCylinder:
      return ShapeType::kHalfCylinder;
    case minphys_viz::VizShapeType::Compound:
      return ShapeType::kCompound;
    default:
      break;
  }
  return ShapeType::kUnknown;
}

struct VizTerrainReassembly {
  minphys_viz::VizTerrainPatchMetaBody meta{};
  bool active = false;
  std::uint32_t watermark = 0;
  std::vector<std::uint8_t> blob;

  void on_meta(const minphys_viz::VizTerrainPatchMetaBody& m) {
    if (m.expected_float_blob_bytes == 0u || (m.expected_float_blob_bytes % 4u) != 0u) {
      reset();
      return;
    }
    meta = m;
    blob.assign(static_cast<std::size_t>(m.expected_float_blob_bytes), std::uint8_t{0});
    watermark = 0;
    active = true;
  }

  void reset() {
    active = false;
    watermark = 0;
    blob.clear();
    meta = {};
  }

  bool has_collision_layer() const {
    return (meta.flags & minphys_viz::kTerrainFlagHasCollisionHeights) != 0u;
  }

  bool on_floats(const minphys_viz::VizTerrainPatchFloatsHeader& fh,
                 const std::uint8_t* chunk,
                 std::size_t chunk_len) {
    if (!active || fh.terrain_seq != meta.terrain_seq) {
      return false;
    }
    if (chunk_len != fh.chunk_bytes || (fh.chunk_bytes % 4u) != 0u) {
      return false;
    }
    if (fh.byte_offset != watermark) {
      reset();
      return false;
    }
    if (static_cast<std::uint64_t>(fh.byte_offset) + static_cast<std::uint64_t>(fh.chunk_bytes)
        > static_cast<std::uint64_t>(meta.expected_float_blob_bytes)) {
      reset();
      return false;
    }
    std::memcpy(blob.data() + fh.byte_offset, chunk, chunk_len);
    watermark = fh.byte_offset + fh.chunk_bytes;
    return true;
  }

  bool is_done() const {
    return active && watermark == meta.expected_float_blob_bytes && meta.expected_float_blob_bytes > 0u;
  }

  bool finalize_into(TerrainPatchState& out) {
    if (!is_done()) {
      return false;
    }
    const int rows = meta.rows;
    const int cols = meta.cols;
    const std::size_t ncell = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
    const std::size_t layer_bytes = ncell * sizeof(float);
    const std::size_t layers = has_collision_layer() ? 3u : 2u;
    if (blob.size() != layer_bytes * layers) {
      reset();
      return false;
    }
    out.frame = meta.frame;
    out.sim_time_s = meta.sim_time_s;
    out.rows = rows;
    out.cols = cols;
    out.cell_size_m = meta.cell_size_m;
    out.base_margin_m = meta.base_margin_m;
    out.min_cell_thickness_m = meta.min_cell_thickness_m;
    out.influence_sigma_m = meta.influence_sigma_m;
    out.plane_confidence = meta.plane_confidence;
    out.confidence_half_life_s = meta.confidence_half_life_s;
    out.base_update_blend = meta.base_update_blend;
    out.decay_update_boost = meta.decay_update_boost;
    out.center = {meta.center_world.x, meta.center_world.y, meta.center_world.z};
    out.has_grid_origin_xz = true;
    out.grid_origin_x = meta.grid_origin_x;
    out.grid_origin_z = meta.grid_origin_z;
    out.base_height_m = meta.base_height_m;
    out.plane_height_m = meta.plane_height_m;
    out.plane_normal = {meta.plane_normal.x, meta.plane_normal.y, meta.plane_normal.z};
    out.heights.resize(ncell);
    out.confidences.resize(ncell);
    std::memcpy(out.heights.data(), blob.data(), layer_bytes);
    std::memcpy(out.confidences.data(), blob.data() + layer_bytes, layer_bytes);
    if (has_collision_layer()) {
      out.collision_heights.resize(ncell);
      std::memcpy(out.collision_heights.data(), blob.data() + 2u * layer_bytes, layer_bytes);
      out.schema_version = 2;
    } else {
      out.collision_heights.clear();
      out.schema_version = 1;
    }
    out.valid = rows > 0 && cols > 0 && out.cell_size_m > 0.0f;
    reset();
    return out.valid;
  }
};

bool ParseVizBinaryPacket(const std::uint8_t* data,
                          std::size_t len,
                          std::map<std::uint32_t, EntityState>& entities,
                          TerrainPatchState& terrain_patch,
                          std::string& packet_kind,
                          VizTerrainReassembly& terrain_asm) {
  if (!minphys_viz::IsVizBinaryPayload(data, len)) {
    return false;
  }
  if (len < sizeof(minphys_viz::VizWireHeader)) {
    return false;
  }
  const auto* hdr = reinterpret_cast<const minphys_viz::VizWireHeader*>(data);
  const auto kind = static_cast<minphys_viz::VizMessageKind>(hdr->message_kind);
  switch (kind) {
    case minphys_viz::VizMessageKind::SceneClear: {
      if (len < sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizSceneClearBody)) {
        return false;
      }
      entities.clear();
      terrain_patch = {};
      terrain_asm.reset();
      packet_kind = "viz.scene_clear";
      return true;
    }
    case minphys_viz::VizMessageKind::EntityFrame: {
      if (len < sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizEntityFrameBody)) {
        return false;
      }
      minphys_viz::VizEntityFrameBody body{};
      std::memcpy(&body, data + sizeof(minphys_viz::VizWireHeader), sizeof(body));
      EntityState& entity = entities[body.entity_id];
      entity.id = body.entity_id;
      entity.position = {body.position.x, body.position.y, body.position.z};
      entity.rotation = {body.rotation.w, body.rotation.x, body.rotation.y, body.rotation.z};
      entity.has_frame = true;
      packet_kind = "viz.entity_frame";
      return true;
    }
    case minphys_viz::VizMessageKind::EntityStatic: {
      if (len < sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizEntityStaticBody)) {
        return false;
      }
      minphys_viz::VizEntityStaticBody fixed{};
      std::memcpy(&fixed, data + sizeof(minphys_viz::VizWireHeader), sizeof(fixed));
      if (fixed.compound_child_count > 4096u) {
        return false;
      }
      const std::size_t tail_bytes =
          static_cast<std::size_t>(fixed.compound_child_count) * sizeof(minphys_viz::VizCompoundChildWire);
      if (len < sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizEntityStaticBody) + tail_bytes) {
        return false;
      }
      EntityState& entity = entities[fixed.entity_id];
      entity.id = fixed.entity_id;
      entity.shape = ShapeFromViz(fixed.shape);
      entity.radius = fixed.radius;
      entity.half_height = fixed.half_height;
      entity.half_extents = {fixed.half_extents.x, fixed.half_extents.y, fixed.half_extents.z};
      entity.plane_normal = {fixed.plane_normal.x, fixed.plane_normal.y, fixed.plane_normal.z};
      entity.plane_offset = fixed.plane_offset;
      entity.compound_children.clear();
      entity.compound_children.reserve(static_cast<std::size_t>(fixed.compound_child_count));
      const std::uint8_t* child_ptr =
          data + sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizEntityStaticBody);
      for (std::uint32_t i = 0; i < fixed.compound_child_count; ++i) {
        minphys_viz::VizCompoundChildWire cw{};
        std::memcpy(&cw, child_ptr + static_cast<std::size_t>(i) * sizeof(minphys_viz::VizCompoundChildWire),
                    sizeof(cw));
        CompoundChildState cs{};
        cs.shape = ShapeFromViz(cw.shape);
        cs.radius = cw.radius;
        cs.half_height = cw.half_height;
        cs.half_extents = {cw.half_extents.x, cw.half_extents.y, cw.half_extents.z};
        cs.local_position = {cw.local_position.x, cw.local_position.y, cw.local_position.z};
        cs.local_rotation = {cw.local_orientation.w,
                             cw.local_orientation.x,
                             cw.local_orientation.y,
                             cw.local_orientation.z};
        entity.compound_children.push_back(cs);
      }
      entity.has_static = true;
      packet_kind = "viz.entity_static";
      return true;
    }
    case minphys_viz::VizMessageKind::TerrainPatchMeta: {
      if (len < sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizTerrainPatchMetaBody)) {
        return false;
      }
      minphys_viz::VizTerrainPatchMetaBody meta{};
      std::memcpy(&meta, data + sizeof(minphys_viz::VizWireHeader), sizeof(meta));
      terrain_asm.on_meta(meta);
      terrain_patch.valid = false;
      packet_kind = "viz.terrain_patch_meta";
      return true;
    }
    case minphys_viz::VizMessageKind::TerrainPatchFloats: {
      if (len < sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizTerrainPatchFloatsHeader)) {
        return false;
      }
      minphys_viz::VizTerrainPatchFloatsHeader fh{};
      std::memcpy(&fh, data + sizeof(minphys_viz::VizWireHeader), sizeof(fh));
      const std::size_t header_total = sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizTerrainPatchFloatsHeader);
      if (len < header_total + static_cast<std::size_t>(fh.chunk_bytes)) {
        return false;
      }
      const std::uint8_t* chunk = data + header_total;
      if (!terrain_asm.on_floats(fh, chunk, static_cast<std::size_t>(fh.chunk_bytes))) {
        return false;
      }
      if (terrain_asm.is_done()) {
        (void)terrain_asm.finalize_into(terrain_patch);
      }
      packet_kind = "viz.terrain_floats";
      return true;
    }
    default:
      return false;
  }
}

bool ParsePacket(const std::string& payload,
                 std::map<std::uint32_t, EntityState>& entities,
                 TerrainPatchState& terrain_patch,
                 HexapodTelemetryState& telemetry,
                 std::string& packet_kind,
                 VizTerrainReassembly* terrain_asm_reset = nullptr) {
  if (ParseHexapodTelemetryPacket(payload, telemetry)) {
    const auto type = ExtractStringField(payload, "type");
    packet_kind = type.has_value() ? *type : "telemetry";
    return true;
  }

  const auto message_type = ExtractStringField(payload, "message_type");
  if (!message_type.has_value()) {
    return false;
  }

  packet_kind = *message_type;

  if (*message_type == "scene_clear") {
    entities.clear();
    terrain_patch = {};
    if (terrain_asm_reset != nullptr) {
      terrain_asm_reset->reset();
    }
    return true;
  }

  if (*message_type == "terrain_patch") {
    return ParseTerrainPatchPacket(payload, terrain_patch);
  }

  const auto entity_id = ExtractUintField(payload, "entity_id");
  if (!entity_id.has_value()) {
    return false;
  }

  EntityState& entity = entities[*entity_id];
  entity.id = *entity_id;

  if (*message_type == "entity_static") {
    entity.shape = ParseShapeType(ExtractStringField(payload, "shape_type"));
    entity.compound_children.clear();

    const std::string_view dimensions_payload =
        ExtractObjectField(payload, "dimensions").value_or(std::string_view{});

    if (const auto radius = ExtractFloatField(dimensions_payload, "radius")) {
      entity.radius = *radius;
    }
    if (const auto half_height = ExtractFloatField(dimensions_payload, "half_height")) {
      entity.half_height = *half_height;
    }
    if (const auto half_extents = ExtractFloat3Field(dimensions_payload, "half_extents")) {
      entity.half_extents = {(*half_extents)[0], (*half_extents)[1], (*half_extents)[2]};
    }
    if (const auto plane_normal = ExtractFloat3Field(dimensions_payload, "plane_normal")) {
      entity.plane_normal = {(*plane_normal)[0], (*plane_normal)[1], (*plane_normal)[2]};
    }
    if (const auto plane_offset = ExtractFloatField(dimensions_payload, "plane_offset")) {
      entity.plane_offset = *plane_offset;
    }
    if (entity.shape == ShapeType::kCompound) {
      entity.compound_children = ParseCompoundChildren(payload);
    }
    entity.has_static = true;
    return true;
  }

  if (*message_type == "entity_frame") {
    if (const auto position = ExtractFloat3Field(payload, "position")) {
      entity.position = {(*position)[0], (*position)[1], (*position)[2]};
    }
    if (const auto rotation = ExtractFloat4Field(payload, "rotation")) {
      entity.rotation = {(*rotation)[0], (*rotation)[1], (*rotation)[2], (*rotation)[3]};
    }
    entity.has_frame = true;
    return true;
  }

  return false;
}

#ifndef _WIN32
class UdpReceiver {
 public:
  explicit UdpReceiver(int port) {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      std::cerr << "Failed to create UDP socket on port " << port << ": " << std::strerror(errno) << "\n";
      return;
    }

    const int reuse = 1;
    (void)::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(static_cast<std::uint16_t>(port));
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(socket_fd_, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
      std::cerr << "Failed to bind UDP socket on port " << port << ": " << std::strerror(errno) << "\n";
      ::close(socket_fd_);
      socket_fd_ = -1;
      return;
    }

    const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags >= 0) {
      (void)::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    valid_ = true;
  }

  ~UdpReceiver() {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
    }
  }

  bool valid() const { return valid_; }

  int Pump(std::map<std::uint32_t, EntityState>& entities,
           TerrainPatchState& terrain_patch,
           HexapodTelemetryState& telemetry,
           uint64_t& accepted_packets,
           uint64_t& rejected_packets,
           std::string& last_packet_kind) {
    if (!valid_) {
      return 0;
    }

    int packets = 0;
    for (;;) {
      std::array<char, 65536> buffer{};
      const ssize_t bytes = ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
      if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        std::cerr << "UDP receive error: " << std::strerror(errno) << "\n";
        break;
      }
      if (bytes == 0) {
        break;
      }

      const std::size_t n = static_cast<std::size_t>(bytes);
      std::string packet_kind;
      if (n >= sizeof(minphys_viz::VizWireHeader)
          && minphys_viz::IsVizBinaryPayload(reinterpret_cast<const std::uint8_t*>(buffer.data()), n)) {
        if (ParseVizBinaryPacket(reinterpret_cast<const std::uint8_t*>(buffer.data()),
                                   n,
                                   entities,
                                   terrain_patch,
                                   packet_kind,
                                   terrain_reassembly_)) {
          last_packet_kind = packet_kind;
          ++accepted_packets;
          ++packets;
        } else {
          ++rejected_packets;
        }
      } else {
        const std::string text_payload(buffer.data(), n);
        if (ParsePacket(text_payload, entities, terrain_patch, telemetry, packet_kind, &terrain_reassembly_)) {
          last_packet_kind = packet_kind;
          ++accepted_packets;
          ++packets;
        } else {
          ++rejected_packets;
        }
      }
    }

    return packets;
  }

 private:
  int socket_fd_ = -1;
  bool valid_ = false;
  VizTerrainReassembly terrain_reassembly_{};
};
#endif

Options ParseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--udp-port") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --udp-port\n";
        std::exit(1);
      }
      const std::string value = argv[++i];
      if (!ParseUdpPort(value.c_str(), options.udp_port)) {
        std::cerr << "Invalid UDP port: " << value << "\n";
        std::exit(1);
      }
      continue;
    }
    if (arg == "--command-host") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --command-host\n";
        std::exit(1);
      }
      options.command_host = argv[++i];
      continue;
    }
    if (arg == "--command-port") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --command-port\n";
        std::exit(1);
      }
      const std::string value = argv[++i];
      if (!ParseUdpPort(value.c_str(), options.command_port)) {
        std::cerr << "Invalid command UDP port: " << value << "\n";
        std::exit(1);
      }
      continue;
    }
    if (arg == "--log-joint-positions") {
      options.log_joint_positions = true;
      continue;
    }

    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: hexapod-opengl-visualiser [--udp-port <port>] "
                   "[--command-host <host>] [--command-port <port>] [--log-joint-positions]\n"
                << "  --udp-port              Telemetry/MPV1 listen port (default 9870).\n"
                << "  --command-host/port     Server command channel (default 127.0.0.1:9872).\n"
                << "  --log-joint-positions   Print per-joint (x,y,z) once per second to stdout.\n"
                << "Press F1 while running to toggle the overlay panel.\n";
      std::exit(0);
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    std::exit(1);
  }

  return options;
}

bool HasMeasuredSceneGeometry(const std::map<std::uint32_t, EntityState>& entities) {
  return visualiser::scene::HasDrawableMeasuredGeometry(entities);
}

bool ScenarioAuthorityActive(const HexapodStatusState& status) {
  return status.command_authority.has_value() && *status.command_authority == "scenario";
}

bool NavAuthorityActive(const HexapodStatusState& status) {
  return status.command_authority.has_value() && *status.command_authority == "nav";
}

void DrawNavPathOverlay(const HexapodStatusState& status, const AppUiState& ui) {
  using RV = visualiser::render::Vec3;
  if (!ui.show_nav_path) {
    return;
  }
  const RV path_color{0.25f, 0.95f, 0.55f};
  const RV goal_color{1.0f, 0.35f, 0.2f};
  if (status.nav_active_segment.size() >= 2) {
    for (std::size_t i = 1; i < status.nav_active_segment.size(); ++i) {
      const auto& a = status.nav_active_segment[i - 1];
      const auto& b = status.nav_active_segment[i];
      const Vec3 sa = ServerToSceneVec(Vec3{a[0], a[1], 0.02f});
      const Vec3 sb = ServerToSceneVec(Vec3{b[0], b[1], 0.02f});
      g_line_renderer.AddSegment(RV{sa.x, sa.y, sa.z}, RV{sb.x, sb.y, sb.z}, path_color);
    }
  }
  for (const auto& pose : status.nav_active_segment) {
    const Vec3 p = ServerToSceneVec(Vec3{pose[0], pose[1], 0.02f});
    g_point_renderer.AddPoint(RV{p.x, p.y, p.z}, path_color);
  }
  if (status.nav_goal_x_m.has_value() && status.nav_goal_y_m.has_value()) {
    const float yaw = status.nav_goal_yaw_rad.value_or(0.0f);
    const Vec3 goal = ServerToSceneVec(Vec3{*status.nav_goal_x_m, *status.nav_goal_y_m, 0.04f});
    g_point_renderer.AddPoint(RV{goal.x, goal.y, goal.z}, goal_color);
    const float tip_x = *status.nav_goal_x_m + 0.12f * std::cos(yaw);
    const float tip_y = *status.nav_goal_y_m + 0.12f * std::sin(yaw);
    const Vec3 tip = ServerToSceneVec(Vec3{tip_x, tip_y, 0.04f});
    g_line_renderer.AddSegment(RV{goal.x, goal.y, goal.z}, RV{tip.x, tip.y, tip.z}, goal_color);
  }
}

void DrawLocalMapOverlay(const LocalMapOverlayState& map, const AppUiState& ui) {
  using RV = visualiser::render::Vec3;
  if (!ui.show_local_map || !map.valid) {
    return;
  }
  const int out_w = (map.width_cells + map.cell_step - 1) / map.cell_step;
  const int out_h = (map.height_cells + map.cell_step - 1) / map.cell_step;
  if (out_w <= 0 || out_h <= 0) {
    return;
  }
  const float half_w = 0.5f * static_cast<float>(map.width_cells - 1);
  const float half_h = 0.5f * static_cast<float>(map.height_cells - 1);
  const float cell_m = map.resolution_m * static_cast<float>(map.cell_step);
  const float half_cell = 0.45f * cell_m;
  const RV occ{0.95f, 0.28f, 0.18f};
  const RV free_c{0.35f, 0.55f, 0.95f};
  for (int oy = 0; oy < out_h; ++oy) {
    for (int ox = 0; ox < out_w; ++ox) {
      const std::size_t idx = static_cast<std::size_t>(oy * out_w + ox);
      if (idx >= map.cells.size()) {
        continue;
      }
      const std::uint8_t state = map.cells[idx];
      if (state == 0) {
        continue;
      }
      const int cell_x = ox * map.cell_step;
      const int cell_y = oy * map.cell_step;
      const float local_x = (static_cast<float>(cell_x) - half_w) * map.resolution_m;
      const float local_y = (static_cast<float>(cell_y) - half_h) * map.resolution_m;
      const float wx = map.center_x_m + local_x;
      const float wy = map.center_y_m + local_y;
      const Vec3 c00 = ServerToSceneVec(Vec3{wx - half_cell, wy - half_cell, 0.015f});
      const Vec3 c10 = ServerToSceneVec(Vec3{wx + half_cell, wy - half_cell, 0.015f});
      const Vec3 c11 = ServerToSceneVec(Vec3{wx + half_cell, wy + half_cell, 0.015f});
      const Vec3 c01 = ServerToSceneVec(Vec3{wx - half_cell, wy + half_cell, 0.015f});
      const RV color = (state == 2) ? occ : free_c;
      g_line_renderer.AddSegment(RV{c00.x, c00.y, c00.z}, RV{c10.x, c10.y, c10.z}, color);
      g_line_renderer.AddSegment(RV{c10.x, c10.y, c10.z}, RV{c11.x, c11.y, c11.z}, color);
      g_line_renderer.AddSegment(RV{c11.x, c11.y, c11.z}, RV{c01.x, c01.y, c01.z}, color);
      g_line_renderer.AddSegment(RV{c01.x, c01.y, c01.z}, RV{c00.x, c00.y, c00.z}, color);
    }
  }
}

void DrawFeetOverlay(const HexapodTelemetryState& telemetry, const AppUiState& ui,
                     bool telemetry_fresh) {
  using RV = visualiser::render::Vec3;
  if (!ui.show_feet || !telemetry_fresh || !telemetry.locomotion.valid) {
    return;
  }
  const RV commanded{0.98f, 0.82f, 0.2f};
  const RV planned_stance{0.2f, 0.95f, 0.85f};
  const RV planned_swing{0.75f, 0.45f, 0.95f};
  for (std::size_t i = 0; i < 6; ++i) {
    if (telemetry.locomotion.has_commanded_feet) {
      const Vec3 cmd = ServerToSceneVec(telemetry.locomotion.commanded_foot_world_m[i]);
      g_point_renderer.AddPoint(RV{cmd.x, cmd.y, cmd.z}, commanded);
    }
    if (telemetry.locomotion.has_planned_targets && telemetry.body_pose.valid) {
      const Vec3 plan =
          TransformBodyPoint(telemetry.locomotion.planned_leg_target_body_m[i], telemetry.body_pose);
      const RV plan_c = !telemetry.locomotion.has_planned_stance
          ? RV{0.6f, 0.6f, 0.6f}
          : (telemetry.locomotion.planned_stance[i] ? planned_stance : planned_swing);
      g_point_renderer.AddPoint(RV{plan.x, plan.y, plan.z}, plan_c);
      if (telemetry.locomotion.has_commanded_feet) {
        const Vec3 cmd = ServerToSceneVec(telemetry.locomotion.commanded_foot_world_m[i]);
        g_line_renderer.AddSegment(RV{cmd.x, cmd.y, cmd.z}, RV{plan.x, plan.y, plan.z}, RV{0.55f, 0.55f, 0.55f});
      }
    }
  }
}

void DrawDraftWaypoints(const CommandUiState& command_ui, const AppUiState& ui) {
  using RV = visualiser::render::Vec3;
  if (!ui.waypoint_edit_mode || command_ui.draft_waypoints.empty()) {
    return;
  }
  const RV color{0.95f, 0.75f, 0.2f};
  for (std::size_t i = 0; i < command_ui.draft_waypoints.size(); ++i) {
    const auto& pose = command_ui.draft_waypoints[i];
    const Vec3 p = ServerToSceneVec(Vec3{static_cast<float>(pose.x_m), static_cast<float>(pose.y_m), 0.03f});
    g_point_renderer.AddPoint(RV{p.x, p.y, p.z}, color);
    if (i > 0) {
      const auto& prev = command_ui.draft_waypoints[i - 1];
      const Vec3 a =
          ServerToSceneVec(Vec3{static_cast<float>(prev.x_m), static_cast<float>(prev.y_m), 0.03f});
      g_line_renderer.AddSegment(RV{a.x, a.y, a.z}, RV{p.x, p.y, p.z}, color);
    }
  }
}

void DrawScene(const std::map<std::uint32_t, EntityState>& entities,
               const TerrainPatchState& terrain_patch,
               const HexapodTelemetryState& telemetry,
               const AppUiState& ui,
               const CommandUiState& command_ui,
               const CameraState& camera,
               bool telemetry_fresh,
               float time_s,
               int viewport_width,
               int viewport_height,
               ScenePickContext& pick_out) {
  using RM = visualiser::render::Mat4;
  using RV = visualiser::render::Vec3;

  g_line_renderer.Clear();
  g_mesh_renderer.Clear();
  g_point_renderer.Clear();
  pick_out = ScenePickContext{};

  // The physics stream contains measured link poses, while the JSON robot is reconstructed
  // from commanded joint angles. Showing both in the same place makes normal servo lag look
  // like reversed leg motion, so command geometry is automatic fallback unless explicitly
  // requested as an overlay.
  const bool measured_scene_available = HasMeasuredSceneGeometry(entities);
  const bool draw_command_robot =
      ui.show_robot && (!ui.show_scene || !measured_scene_available || ui.overlay_command_robot);

  glViewport(0, 0, viewport_width, viewport_height);
  glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  SceneBounds bounds;
  if (ui.show_scene) {
    bounds = ComputeSceneBounds(entities);
  }
  if (ui.show_terrain) {
    ExpandTerrainPatchBounds(bounds, terrain_patch);
  }
  if (draw_command_robot && telemetry.has_geometry && telemetry.has_joints) {
    const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
    const SceneBounds robot_bounds = ComputeRobotBounds(robot_geometry, telemetry.angles_deg, telemetry.body_pose);
    if (robot_bounds.valid) {
      if (!bounds.valid) {
        bounds = robot_bounds;
      } else {
        ExpandBounds(bounds, robot_bounds.min);
        ExpandBounds(bounds, robot_bounds.max);
      }
    }
  }
  if (ui.show_nav_path) {
    for (const auto& pose : telemetry.status.nav_active_segment) {
      ExpandBounds(bounds, ServerToSceneVec(Vec3{pose[0], pose[1], 0.0f}));
    }
    if (telemetry.status.nav_goal_x_m.has_value() && telemetry.status.nav_goal_y_m.has_value()) {
      ExpandBounds(bounds,
                   ServerToSceneVec(Vec3{*telemetry.status.nav_goal_x_m, *telemetry.status.nav_goal_y_m, 0.0f}));
    }
  }

  const Vec3 center = ui.follow_active && bounds.valid
      ? Vec3{
            0.5f * (bounds.min.x + bounds.max.x),
            0.5f * (bounds.min.y + bounds.max.y),
            0.5f * (bounds.min.z + bounds.max.z),
        }
      : Vec3{};
  const Vec3 diagonal = bounds.valid
      ? Vec3{
            bounds.max.x - bounds.min.x,
            bounds.max.y - bounds.min.y,
            bounds.max.z - bounds.min.z,
        }
      : Vec3{1.0f, 1.0f, 1.0f};
  const float scene_radius =
      std::max(0.25f, 0.5f * std::sqrt(Dot(diagonal, diagonal)));
  const float camera_distance = std::max(2.0f, scene_radius * camera.distance_scale);
  const float yaw = camera.yaw_deg + (ui.rotate_scene ? time_s * camera.spin_deg_per_s : 0.0f);

  const RM proj = visualiser::render::ProjectionFromLegacyFrustum(viewport_width, viewport_height);
  const RM view = visualiser::render::LegacyViewMatrix(scene_radius,
                                                       camera_distance,
                                                       camera.pitch_deg,
                                                       yaw,
                                                       center.x,
                                                       center.y,
                                                       center.z,
                                                       camera.pan_x,
                                                       camera.pan_y);
  const RM vp = RM::Mul(proj, view);
  pick_out.inv_view_proj = RM::Inverse(vp);
  pick_out.ground_y = terrain_patch.valid ? terrain_patch.plane_height_m : 0.0f;
  if (!terrain_patch.valid) {
    for (const auto& [id, entity] : entities) {
      (void)id;
      if (entity.has_static && entity.shape == ShapeType::kPlane) {
        pick_out.ground_y = entity.plane_offset;
        break;
      }
    }
  }
  pick_out.viewport_width = viewport_width;
  pick_out.viewport_height = viewport_height;
  pick_out.valid = true;

  if (ui.show_terrain) {
    DrawTerrainPatch(terrain_patch);
    DrawGroundReference(terrain_patch, telemetry.body_pose);
  }

  if (ui.show_scene) {
    for (const auto& [id, entity] : entities) {
      (void)id;
      if (!entity.has_static) {
        continue;
      }

      if (entity.shape == ShapeType::kPlane) {
        // The terrain patch already draws the local contact surface. Layering
        // the infinite blue plane grid over it makes two ground surfaces appear
        // to sit at different heights as the patch scrolls or deforms.
        if (ui.show_terrain && terrain_patch.valid && terrain_patch.rows > 0 && terrain_patch.cols > 0
            && terrain_patch.cell_size_m > 0.0f
            && terrain_patch.heights.size() >= static_cast<std::size_t>(terrain_patch.rows * terrain_patch.cols)) {
          continue;
        }
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset,
            RM::Identity());
        continue;
      }

      if (!entity.has_frame) {
        continue;
      }

      RM model = RM::Mul(RM::Translate(entity.position.x, entity.position.y, entity.position.z),
                         LegacyMatFromQuat(entity.rotation));

      if (entity.shape == ShapeType::kCompound) {
        if (entity.compound_children.empty()) {
          DrawPrimitiveShape(
              ShapeType::kBox,
              entity.radius,
              entity.half_height,
              entity.half_extents,
              entity.plane_normal,
              entity.plane_offset,
              model);
        } else {
          for (const CompoundChildState& child : entity.compound_children) {
            const RM child_model =
                RM::Mul(model,
                        RM::Mul(RM::Translate(child.local_position.x, child.local_position.y, child.local_position.z),
                                LegacyMatFromQuat(child.local_rotation)));
            DrawPrimitiveShape(
                child.shape,
                child.radius,
                child.half_height,
                child.half_extents,
                entity.plane_normal,
                entity.plane_offset,
                child_model);
          }
        }
      } else {
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset,
            model);
      }
    }
  }

  DrawLocalMapOverlay(telemetry.local_map, ui);
  DrawNavPathOverlay(telemetry.status, ui);
  DrawDraftWaypoints(command_ui, ui);

  if (draw_command_robot && telemetry.has_joints) {
    const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
    DrawHexapodModel(robot_geometry, telemetry.angles_deg, telemetry.status, telemetry.body_pose);
  }
  DrawFeetOverlay(telemetry, ui, telemetry_fresh);

  const RV light_dir = visualiser::render::Normalize(RV{0.35f, 1.0f, 0.25f});
  g_mesh_renderer.FlushWorld(proj, view, light_dir);
  g_line_renderer.Flush(vp);
  g_point_renderer.Flush(vp, 6.0f);
}

void DrawUi(AppUiState& ui,
            CameraState& camera,
            const HexapodTelemetryState& telemetry,
            const std::string& source_label,
            uint64_t packets_received,
            uint64_t packets_rejected,
            double last_packet_age_s,
            double telemetry_age_s,
            std::size_t entity_count,
            bool measured_scene_available,
            bool terrain_available,
            visualiser::net::CommandClient* command_client,
            CommandUiState& command_ui) {
  if (!ui.show_overlay) {
    return;
  }

  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  const float max_width = std::max(200.0f, display_size.x - 20.0f);
  const float max_height = std::max(180.0f, display_size.y - 20.0f);
  ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(std::min(440.0f, max_width),
                                  std::min(620.0f, max_height)), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(320.0f, max_width),
                                            std::min(260.0f, max_height)),
                                      ImVec2(max_width, max_height));
  ImGui::SetNextWindowBgAlpha(0.90f);
  ImGui::Begin("Hexapod Control Room");

  ImGui::Text("Source: %s", source_label.c_str());
  ImGui::Text("Packets: %llu accepted, %llu rejected",
              static_cast<unsigned long long>(packets_received),
              static_cast<unsigned long long>(packets_rejected));
  if (std::isfinite(last_packet_age_s)) {
    ImGui::Text("Last packet age: %.2fs", last_packet_age_s);
  } else {
    ImGui::TextUnformatted("Last packet age: n/a");
  }
  ImGui::Text("Scene entities: %zu", entity_count);
  ImGui::Text("Terrain: %s", terrain_available ? "available" : "none");
  const bool measured_robot_visible = ui.show_scene && measured_scene_available;
  const bool command_robot_visible = ui.show_robot && telemetry.has_joints &&
      (!measured_robot_visible || ui.overlay_command_robot);
  if (measured_robot_visible && command_robot_visible) {
    ImGui::TextUnformatted("Robot view: measured + command wireframe");
  } else if (measured_robot_visible) {
    ImGui::TextUnformatted("Robot view: measured physics wireframe");
  } else if (command_robot_visible) {
    ImGui::TextUnformatted("Robot view: command wireframe fallback");
  } else {
    ImGui::TextDisabled("Robot view: waiting for drawable robot data");
  }
  if (measured_robot_visible && telemetry.has_joints && !command_robot_visible &&
      ImGui::Button("Show command wireframe too")) {
    ui.show_robot = true;
    ui.overlay_command_robot = true;
  }

  const bool telemetry_fresh = telemetry.status.valid &&
      std::isfinite(telemetry_age_s) && telemetry_age_s <= 1.0;
  if (std::isfinite(telemetry_age_s)) {
    ImGui::Text("Server telemetry: %.1fs old%s", telemetry_age_s,
                telemetry_fresh ? "" : " (stale)");
  } else {
    ImGui::TextDisabled("Server telemetry: unavailable");
  }
  if (command_client != nullptr && command_client->valid()) {
    const double reply_age_s = std::isfinite(command_ui.last_server_reply_time_s)
        ? glfwGetTime() - command_ui.last_server_reply_time_s
        : std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(reply_age_s) && reply_age_s <= 5.0) {
      ImGui::Text("Command server: replied %.1fs ago", reply_age_s);
    } else {
      ImGui::TextDisabled("Command server: unconfirmed / stale");
    }
    ImGui::Text("Pending commands: %zu", command_client->pendingCount());
    if (ImGui::Button("Stop scenario")) {
      ShowCommandSubmission(command_ui, command_client->scenarioStop());
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel navigation")) {
      ShowCommandSubmission(command_ui, command_client->navCancel());
    }
  } else {
    ImGui::TextDisabled("Command client unavailable");
  }
  if (!command_ui.last_result.empty()) {
    ImGui::TextWrapped("Last command: %s", command_ui.last_result.c_str());
  }

  ImGui::Separator();
  if (ImGui::CollapsingHeader("View and camera")) {
  ImGui::Checkbox("Show measured scene", &ui.show_scene);
  ImGui::Checkbox("Show command wireframe", &ui.show_robot);
  ImGui::Checkbox("Overlay command on measured scene", &ui.overlay_command_robot);
  if (ui.show_scene && measured_scene_available && ui.show_robot && !ui.overlay_command_robot) {
    ImGui::TextDisabled("Command wireframe hidden; measured scene is active");
  }
  ImGui::Checkbox("Show terrain", &ui.show_terrain);
  ImGui::Checkbox("Show nav path/goal", &ui.show_nav_path);
  ImGui::Checkbox("Show local map", &ui.show_local_map);
  ImGui::Checkbox("Show planned feet", &ui.show_feet);
  ImGui::Checkbox("Rotate scene", &ui.rotate_scene);
  ImGui::Checkbox("Follow active", &ui.follow_active);
  ImGui::Checkbox("Show debug", &ui.show_debug);
  ImGui::SliderFloat("Yaw", &camera.yaw_deg, -180.0f, 180.0f);
  ImGui::SliderFloat("Pitch", &camera.pitch_deg, -89.0f, 89.0f);
  ImGui::SliderFloat("Distance", &camera.distance_scale, 1.5f, 12.0f);
  ImGui::SliderFloat("Pan X", &camera.pan_x, -1.0f, 1.0f);
  ImGui::SliderFloat("Pan Y", &camera.pan_y, -1.0f, 1.0f);
  ImGui::SliderFloat("Spin deg/s", &camera.spin_deg_per_s, 0.0f, 45.0f);
  if (ImGui::Button("Reset View")) {
    camera = CameraState{};
  }
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Telemetry");
  if (telemetry.status.valid) {
    ImGui::Text("Mode: %s (%d)", RobotModeName(telemetry.status.active_mode), telemetry.status.active_mode);
    ImGui::Text("Fault: %s (%d)", visualiser::robot::FaultCodeName(telemetry.status.active_fault),
                telemetry.status.active_fault);
  } else {
    ImGui::TextUnformatted("No server telemetry yet");
  }
  if (telemetry_fresh && telemetry.status.command_authority.has_value()) {
    ImGui::Text("Authority: %s", telemetry.status.command_authority->c_str());
    if (telemetry.status.command_scenario.has_value()) {
      ImGui::Text("Scenario: %s", telemetry.status.command_scenario->c_str());
    }
    if (telemetry.status.command_nav_active.has_value()) {
      ImGui::Text("Nav active: %s", *telemetry.status.command_nav_active ? "yes" : "no");
    }
  } else {
    ImGui::TextDisabled("Authority: unknown (stale or missing)");
  }
  if (telemetry_fresh && telemetry.status.nav_lifecycle.has_value()) {
    ImGui::Text("Navigation: %s", NavigationLifecycleName(*telemetry.status.nav_lifecycle));
    if (telemetry.status.nav_block_reason.has_value() &&
        *telemetry.status.nav_block_reason != 0) {
      ImGui::SameLine();
      ImGui::Text("(%s)", PlannerBlockReasonName(*telemetry.status.nav_block_reason));
    }
  }

  ImGui::Separator();
  if (ImGui::CollapsingHeader("Commands", ImGuiTreeNodeFlags_DefaultOpen)) {
  if (command_client == nullptr || !command_client->valid()) {
    ImGui::TextDisabled("Command client unavailable");
  } else {
    const bool scenario_auth = telemetry_fresh && ScenarioAuthorityActive(telemetry.status);
    const bool nav_auth = telemetry_fresh && NavAuthorityActive(telemetry.status);
    const bool idle_auth = telemetry_fresh && !scenario_auth && !nav_auth;

    if (!telemetry_fresh) {
      ImGui::TextDisabled("Authority unknown: waiting for fresh server telemetry");
      ui.click_goal_mode = false;
      ui.waypoint_edit_mode = false;
    }

    if (ImGui::CollapsingHeader("Scenarios", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("List scenarios")) {
      const auto listed = command_client->scenarioList();
      ShowCommandSubmission(command_ui, listed);
    }
    if (!command_ui.scenarios.empty()) {
      std::vector<const char*> items;
      items.reserve(command_ui.scenarios.size());
      for (const auto& id : command_ui.scenarios) {
        items.push_back(id.c_str());
      }
      ImGui::Combo("Scenario",
                   &command_ui.selected_index,
                   items.data(),
                   static_cast<int>(items.size()));
      if (ImGui::Button("Run")) {
        const auto& id = command_ui.scenarios[static_cast<std::size_t>(command_ui.selected_index)];
        const auto ran = command_client->scenarioRun(id);
        ShowCommandSubmission(command_ui, ran);
      }
    } else {
      ImGui::TextDisabled("No scenarios listed yet");
    }
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Navigation controls")) {
    if (scenario_auth) {
      ImGui::TextDisabled("Click-to-goal disabled (scenario authority)");
      ui.click_goal_mode = false;
      ui.waypoint_edit_mode = false;
    } else {
      ImGui::BeginDisabled(!telemetry_fresh);
      ImGui::Checkbox("Click ground → nav.goto", &ui.click_goal_mode);
      if (ui.click_goal_mode && ui.waypoint_edit_mode) {
        ui.waypoint_edit_mode = false;
      }
      ImGui::Checkbox("Click ground → draft waypoints", &ui.waypoint_edit_mode);
      if (ui.waypoint_edit_mode && ui.click_goal_mode) {
        ui.click_goal_mode = false;
      }
      ImGui::EndDisabled();
    }
    ImGui::Text("Draft waypoints: %zu", command_ui.draft_waypoints.size());
    ImGui::BeginDisabled(!telemetry_fresh || scenario_auth || command_ui.draft_waypoints.size() < 2 ||
                         !command_ui.pending_waypoints_ref.empty());
    if (ImGui::Button("Send waypoints")) {
      const auto sent = command_client->navWaypoints(command_ui.draft_waypoints);
      ShowCommandSubmission(command_ui, sent);
      if (sent.ok) {
        command_ui.pending_waypoints_ref = sent.ref;
        command_ui.pending_waypoints_revision = command_ui.draft_revision;
      }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Clear draft")) {
      command_ui.draft_waypoints.clear();
      ++command_ui.draft_revision;
    }
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Motion controls")) {
    ImGui::BeginDisabled(!idle_auth);
    ImGui::SliderFloat("Speed m/s", &command_ui.motion_speed_mps, 0.0f, 0.2f);
    ImGui::SliderFloat("Heading rad", &command_ui.motion_heading_rad, -3.14f, 3.14f);
    ImGui::SliderFloat("Yaw rate", &command_ui.motion_yaw_rate, -1.0f, 1.0f);
    ImGui::SliderFloat("Body height m", &command_ui.motion_body_height_m, 0.08f, 0.2f);
    ImGui::TextDisabled("WALK / TRIPOD; %.3f m/s, heading %.2f rad, yaw %.2f rad/s, height %.3f m",
                        command_ui.motion_speed_mps, command_ui.motion_heading_rad,
                        command_ui.motion_yaw_rate, command_ui.motion_body_height_m);
    if (ImGui::Button("Walk (TRIPOD) with settings")) {
      visualiser::net::MotionSetCommand motion{};
      motion.mode = "WALK";
      motion.gait = "TRIPOD";
      motion.speed_mps = command_ui.motion_speed_mps;
      motion.heading_rad = command_ui.motion_heading_rad;
      motion.yaw_rate_radps = command_ui.motion_yaw_rate;
      motion.body_height_m = command_ui.motion_body_height_m;
      const auto applied = command_client->motionSet(motion);
      ShowCommandSubmission(command_ui, applied);
    }
    ImGui::SameLine();
    if (ImGui::Button("Stand & hold")) {
      const auto applied = command_client->standHold(command_ui.motion_body_height_m);
      ShowCommandSubmission(command_ui, applied);
    }
    ImGui::EndDisabled();
    if (!idle_auth) {
      ImGui::TextDisabled(telemetry_fresh
          ? "Motion controls require idle authority"
          : "Motion controls require fresh server telemetry");
    }
    }

    if (ImGui::TreeNode("Recent commands")) {
      for (const auto& entry : command_ui.history) {
        ImGui::TextWrapped("%s %s (%s): %s", entry.type.c_str(),
                           entry.state.c_str(), entry.ref.c_str(), entry.reason.c_str());
      }
      ImGui::TreePop();
    }
  }
  }

  if (ImGui::CollapsingHeader("Locomotion observation")) {
    if (!telemetry_fresh || !telemetry.locomotion.valid) {
      ImGui::TextDisabled("Foot data: n/a (missing or stale server telemetry)");
    } else {
      const auto& loco = telemetry.locomotion;
      ImGui::Text("Foot data: live (%.2f s old); heights in world frame", telemetry_age_s);
      if (telemetry.status.requested_planar_speed_mps &&
          telemetry.status.governed_planar_speed_mps) {
        ImGui::Text("Requested / governed speed: %.3f / %.3f m/s",
                    *telemetry.status.requested_planar_speed_mps,
                    *telemetry.status.governed_planar_speed_mps);
      } else {
        ImGui::TextDisabled("Requested / governed speed: n/a");
      }
      ImGui::TextDisabled("Actual speed: n/a (not in telemetry)");
      if (telemetry.status.physics_peak_servo_torque_utilization) {
        ImGui::Text("Peak servo torque use: %.0f%% (sim aggregate)",
                    *telemetry.status.physics_peak_servo_torque_utilization * 100.0);
      } else {
        ImGui::TextDisabled("Servo torque use: n/a (no physics-sim metric)");
      }
      if (loco.max_post_clamp_distortion_m) {
        ImGui::Text("Max target clamp distortion: %.1f mm",
                    *loco.max_post_clamp_distortion_m * 1000.0f);
      } else {
        ImGui::TextDisabled("Target clamp distortion: n/a");
      }
      ImGui::TextDisabled("Foot reach margin: n/a (not in telemetry)");
      ImGui::TextDisabled("Plan / raw / fused; foot Z in m; tracking error in mm");
      constexpr std::array<const char*, 6> kInternalLegNames =
          {"R3", "L3", "R2", "L2", "R1", "L1"};
      if (ImGui::BeginTable("##foot_observation", 7,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        for (const char* label : {"Leg", "Plan", "Raw", "Fused", "Cmd Z", "Meas Z", "Err"}) {
          ImGui::TableSetupColumn(label);
        }
        ImGui::TableHeadersRow();
        for (std::size_t leg = 0; leg < kInternalLegNames.size(); ++leg) {
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::TextUnformatted(kInternalLegNames[leg]);
          ImGui::TableSetColumnIndex(1);
          ImGui::TextUnformatted(loco.has_planned_stance ? (loco.planned_stance[leg] ? "stance" : "swing") : "n/a");
          ImGui::TableSetColumnIndex(2);
          ImGui::TextUnformatted(loco.has_raw_contact ? (loco.raw_contact[leg] ? "yes" : "no") : "n/a");
          ImGui::TableSetColumnIndex(3);
          ImGui::TextUnformatted(loco.has_fused_support ? (loco.fused_support[leg] ? "yes" : "no") : "n/a");
          ImGui::TableSetColumnIndex(4);
          if (loco.has_commanded_feet) ImGui::Text("%.3f", loco.commanded_foot_world_m[leg].z);
          else ImGui::TextUnformatted("n/a");
          ImGui::TableSetColumnIndex(5);
          if (loco.has_measured_feet) ImGui::Text("%.3f", loco.measured_foot_world_m[leg].z);
          else ImGui::TextUnformatted("n/a");
          ImGui::TableSetColumnIndex(6);
          if (loco.has_tracking_error) ImGui::Text("%.0f", loco.commanded_tracking_error_m[leg] * 1000.0f);
          else ImGui::TextUnformatted("n/a");
        }
        ImGui::EndTable();
      }
    }
  }

  if (telemetry.status.valid) {
    ImGui::Separator();
    ImGui::Text("Bus: %s | Estimator: %s",
                telemetry.status.bus_ok ? "OK" : "FAULT",
                telemetry.status.estimator_valid ? "valid" : "invalid");
    ImGui::Text("Loop: %d | Timestamp: %llu ms",
                telemetry.status.loop_counter,
                static_cast<unsigned long long>(telemetry.status.timestamp_ms));
    ImGui::Text("Voltage: %.2f V | Current: %.2f A", telemetry.status.voltage, telemetry.status.current);
  }

  if (ImGui::CollapsingHeader("Navigation diagnostics") &&
      (telemetry.status.nav_lifecycle.has_value() ||
       telemetry.status.nav_planner_status.has_value() ||
       telemetry.status.nav_block_reason.has_value())) {
    ImGui::Separator();
    ImGui::TextUnformatted("Navigation");
    if (telemetry.status.nav_lifecycle.has_value()) {
      ImGui::Text("Lifecycle: %s (%d)",
                  NavigationLifecycleName(*telemetry.status.nav_lifecycle),
                  *telemetry.status.nav_lifecycle);
    }
    if (telemetry.status.nav_planner_status.has_value()) {
      ImGui::Text("Planner: %s (%d)",
                  LocalPlanStatusName(*telemetry.status.nav_planner_status),
                  *telemetry.status.nav_planner_status);
    }
    if (telemetry.status.nav_block_reason.has_value()) {
      ImGui::Text("Block reason: %s (%d)",
                  PlannerBlockReasonName(*telemetry.status.nav_block_reason),
                  *telemetry.status.nav_block_reason);
    }
    if (telemetry.status.nav_map_fresh.has_value()) {
      ImGui::Text("Map fresh: %s", *telemetry.status.nav_map_fresh ? "yes" : "no");
    }
    if (telemetry.status.nav_replan_count.has_value()) {
      ImGui::Text("Replans: %zu", *telemetry.status.nav_replan_count);
    }
    if (telemetry.status.nav_active_segment_waypoint_count.has_value()) {
      ImGui::Text("Waypoints: %zu", *telemetry.status.nav_active_segment_waypoint_count);
    }
    if (telemetry.status.nav_active_segment_length_m.has_value()) {
      ImGui::Text("Segment length: %.2fm", *telemetry.status.nav_active_segment_length_m);
    }
    if (telemetry.status.nav_nearest_obstacle_distance_m.has_value()) {
      ImGui::Text("Nearest obstacle: %.2fm", *telemetry.status.nav_nearest_obstacle_distance_m);
    }
    if (telemetry.status.nav_goal_x_m.has_value() && telemetry.status.nav_goal_y_m.has_value()) {
      ImGui::Text("Goal: (%.2f, %.2f)", *telemetry.status.nav_goal_x_m, *telemetry.status.nav_goal_y_m);
    }
    if (telemetry.status.nav_active_waypoint_index.has_value()) {
      ImGui::Text("Active waypoint idx: %d", *telemetry.status.nav_active_waypoint_index);
    }
    if (telemetry.status.nav_distance_to_active_waypoint_m.has_value()) {
      ImGui::Text("Dist to waypoint: %.2fm", *telemetry.status.nav_distance_to_active_waypoint_m);
    }
    if (telemetry.local_map.valid) {
      ImGui::Text("Local map: %dx%d step=%d %s",
                  telemetry.local_map.width_cells,
                  telemetry.local_map.height_cells,
                  telemetry.local_map.cell_step,
                  telemetry.local_map.fresh ? "fresh" : "stale");
    }
  }

  if (ImGui::CollapsingHeader("Fusion diagnostics") &&
      (telemetry.status.fusion_model_trust.has_value() ||
       telemetry.status.fusion_contact_mismatch_ratio.has_value())) {
    ImGui::Separator();
    ImGui::TextUnformatted("Fusion");
    if (telemetry.status.fusion_model_trust.has_value()) {
      ImGui::ProgressBar(static_cast<float>(*telemetry.status.fusion_model_trust), ImVec2(-1.0f, 0.0f), "model trust");
    }
    if (telemetry.status.fusion_resync_requested.has_value()) {
      ImGui::Text("Resync: %s", *telemetry.status.fusion_resync_requested ? "yes" : "no");
    }
    if (telemetry.status.fusion_hard_reset_requested.has_value()) {
      ImGui::Text("Hard reset: %s", *telemetry.status.fusion_hard_reset_requested ? "yes" : "no");
    }
    if (telemetry.status.fusion_predictive_mode.has_value()) {
      ImGui::Text("Predictive mode: %s", *telemetry.status.fusion_predictive_mode ? "yes" : "no");
    }
    if (telemetry.status.fusion_max_body_position_error_m.has_value()) {
      ImGui::Text("Fusion residual: %.3fm", *telemetry.status.fusion_max_body_position_error_m);
    }
    if (telemetry.status.fusion_max_body_orientation_error_rad.has_value()) {
      ImGui::Text("Max orientation error: %.3frad", *telemetry.status.fusion_max_body_orientation_error_rad);
    }
    if (telemetry.status.fusion_contact_mismatch_ratio.has_value()) {
      ImGui::Text("Contact mismatch: %.3f", *telemetry.status.fusion_contact_mismatch_ratio);
    }
    if (telemetry.status.fusion_terrain_residual_m.has_value()) {
      ImGui::Text("Terrain residual: %.3fm", *telemetry.status.fusion_terrain_residual_m);
    }
  }

  ImGui::Separator();
  if (ImGui::CollapsingHeader("Robot geometry")) {
  if (telemetry.has_geometry) {
    ImGui::Text("Coxa: %.1fmm  Femur: %.1fmm  Tibia: %.1fmm  Body radius: %.1fmm",
                telemetry.geometry.coxa_mm,
                telemetry.geometry.femur_mm,
                telemetry.geometry.tibia_mm,
                telemetry.geometry.body_radius_mm);
    for (std::size_t i = 0; i < telemetry.geometry.legs.size(); ++i) {
      ImGui::Text("%s: mount %.1fdeg", telemetry.geometry.legs[i].key.c_str(),
                  telemetry.geometry.legs[i].mount_angle_rad * 180.0f / kPi);
    }
  } else {
    ImGui::TextUnformatted("Waiting for geometry packet");
  }
  }

  ImGui::End();

  if (ui.show_debug) {
    ImGui::SetNextWindowBgAlpha(0.80f);
    ImGui::Begin("Leg Angles", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    if (telemetry.has_joints) {
      for (std::size_t i = 0; i < kLegKeys.size(); ++i) {
        ImGui::Text("%s: [%5.1f, %5.1f, %5.1f]",
                    kLegKeys[i],
                    telemetry.angles_deg[i][0],
                    telemetry.angles_deg[i][1],
                    telemetry.angles_deg[i][2]);
      }
    } else {
      ImGui::TextUnformatted("Waiting for joint packet");
    }
    ImGui::End();
  }
}

}  // namespace

namespace visualiser::app {

int RunApplication(int argc, char** argv) {
  const Options options = ParseArgs(argc, argv);

  ConfigureWslWindowPlatform();
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);
  // Keep the window hidden until GL/ImGui/UDP are ready, but ask to focus on first show.
  // Under WSLg the host may still prefix the title with "[WARN:COPY MODE]".
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_FOCUSED, GLFW_TRUE);
  glfwWindowHint(GLFW_FOCUS_ON_SHOW, GLFW_TRUE);

  GLFWwindow* window = glfwCreateWindow(
      kDefaultWindowWidth, kDefaultWindowHeight, "Hexapod OpenGL Visualiser", nullptr, nullptr);
  if (window == nullptr) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if (!visualiser::gl::InitGlad()) {
    std::cerr << "Failed to load OpenGL entry points (GLAD)\n";
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  visualiser::gl::SetupDebugCallback();

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glLineWidth(1.0f);

  if (!InitModernRenderer()) {
    std::cerr << "Failed to initialize shader-based renderers\n";
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGuiIO& imgui_io = ImGui::GetIO();
  imgui_io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");

#ifndef _WIN32
  UdpReceiver receiver(options.udp_port);
  if (!receiver.valid()) {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }
  visualiser::net::CommandClient command_client(
      visualiser::net::CommandClientConfig{options.command_host, options.command_port});
  CommandUiState command_ui{};
  if (command_client.valid()) {
    const auto listed = command_client.scenarioList();
    ShowCommandSubmission(command_ui, listed);
  }
#else
  std::cerr << "UDP receiver is not implemented on Windows in this build\n";
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 1;
#endif

  std::map<std::uint32_t, EntityState> entities;
  TerrainPatchState terrain_patch;
  HexapodTelemetryState telemetry;
  AppUiState ui;
  CameraState camera;
  uint64_t accepted_packets = 0;
  uint64_t rejected_packets = 0;
  std::string last_packet_kind = "waiting";
  double last_packet_time_s = std::numeric_limits<double>::quiet_NaN();
  double last_status_time_s = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t last_status_timestamp_ms = 0;
  double last_title_update_s = -1.0;
  double last_joint_log_s = -1.0;
  bool overlay_toggle_down = false;
  bool startup_focus_settled = false;
  const double window_shown_at_s = glfwGetTime();

  ScenePickContext scene_pick{};
  bool mouse_left_was_down = false;
  // Show only after OpenGL, ImGui, and UDP input are ready. Retry activation for a short period:
  // background launches from run_physics_stack.sh often lose the first focus request to the
  // terminal / Cursor window under WSLg.
  RequestWindowFocus(window);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    PollStartupWindowFocus(window, window_shown_at_s, startup_focus_settled);
    for (const auto& reply : command_client.poll()) {
      ApplyCommandReply(ui, command_ui, reply);
    }

    const bool overlay_toggle_now = glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS;
    if (overlay_toggle_now && !overlay_toggle_down) {
      ui.show_overlay = !ui.show_overlay;
    }
    overlay_toggle_down = overlay_toggle_now;

    const int accepted_this_frame =
        receiver.Pump(entities, terrain_patch, telemetry, accepted_packets, rejected_packets, last_packet_kind);
    if (accepted_this_frame > 0) {
      last_packet_time_s = glfwGetTime();
    }
    if (telemetry.status.valid &&
        (!std::isfinite(last_status_time_s) ||
         telemetry.status.timestamp_ms != last_status_timestamp_ms)) {
      last_status_time_s = glfwGetTime();
      last_status_timestamp_ms = telemetry.status.timestamp_ms;
    }

    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

    const double now_s = glfwGetTime();
    const float time_s = static_cast<float>(now_s);
    const double telemetry_age_s = std::isfinite(last_status_time_s) ? now_s - last_status_time_s
                                                                      : std::numeric_limits<double>::quiet_NaN();
    const bool telemetry_fresh = telemetry.status.valid &&
        std::isfinite(telemetry_age_s) && telemetry_age_s <= 1.0;
    DrawScene(entities,
              terrain_patch,
              telemetry,
              ui,
              command_ui,
              camera,
              telemetry_fresh,
              time_s,
              framebuffer_width,
              framebuffer_height,
              scene_pick);
    if (options.log_joint_positions
        && telemetry.has_joints
        && (last_joint_log_s < 0.0 || now_s - last_joint_log_s >= 1.0)) {
      const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
      LogJointPositions(robot_geometry, telemetry.angles_deg, telemetry.body_pose, now_s);
      last_joint_log_s = now_s;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    const double packet_age_s = std::isfinite(last_packet_time_s) ? now_s - last_packet_time_s
                                                                  : std::numeric_limits<double>::quiet_NaN();
    DrawUi(ui,
           camera,
           telemetry,
           last_packet_kind,
           accepted_packets,
           rejected_packets,
           packet_age_s,
           telemetry_age_s,
           entities.size(),
           HasMeasuredSceneGeometry(entities),
           terrain_patch.valid,
           &command_client,
           command_ui);

    const bool mouse_left_down = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool mouse_clicked = mouse_left_down && !mouse_left_was_down;
    mouse_left_was_down = mouse_left_down;
    if (mouse_clicked && !ImGui::GetIO().WantCaptureMouse && command_client.valid() &&
        std::isfinite(telemetry_age_s) && telemetry_age_s <= 1.0 &&
        !ScenarioAuthorityActive(telemetry.status) &&
        (ui.click_goal_mode || ui.waypoint_edit_mode)) {
      double mx = 0.0;
      double my = 0.0;
      glfwGetCursorPos(window, &mx, &my);
      float sx = 0.0f;
      float sy = 0.0f;
      int window_width = 0;
      int window_height = 0;
      glfwGetWindowSize(window, &window_width, &window_height);
      if (PickGroundServerXY(scene_pick, static_cast<float>(mx), static_cast<float>(my),
                             window_width, window_height, sx, sy)) {
        if (ui.click_goal_mode) {
          const auto result = command_client.navGoto(
              visualiser::net::NavPose2d{sx, sy, telemetry.body_pose.yaw_rad},
              "TRIPOD", command_ui.motion_body_height_m);
          ShowCommandSubmission(command_ui, result);
        } else if (ui.waypoint_edit_mode) {
          command_ui.draft_waypoints.push_back(
              visualiser::net::NavPose2d{sx, sy, telemetry.body_pose.yaw_rad});
          ++command_ui.draft_revision;
          command_ui.last_result =
              "draft waypoint #" + std::to_string(command_ui.draft_waypoints.size());
        }
      } else {
        command_ui.last_result = "ground pick missed";
      }
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (last_title_update_s < 0.0 || now_s - last_title_update_s > 0.25) {
      std::ostringstream title;
      title << "Hexapod OpenGL Visualiser | " << last_packet_kind << " | UDP " << options.udp_port
            << " | entities " << entities.size() << " | packets " << accepted_packets;
      glfwSetWindowTitle(window, title.str().c_str());
      last_title_update_s = now_s;
    }

    glfwSwapBuffers(window);
  }

  if (GLAD_GL_KHR_debug) {
    glDebugMessageCallback(nullptr, nullptr);
  }
  g_point_renderer.Shutdown();
  g_mesh_renderer.Shutdown();
  g_line_renderer.Shutdown();
  g_modern_renderer_ok = false;

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

}  // namespace visualiser::app
