#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl2.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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

constexpr std::array<const char*, 6> kLegKeys = {"LF", "LM", "LR", "RF", "RM", "RR"};
// Defaults match `geometry_config::buildDefaultHexapodGeometry` in telemetry order (LF..RR).
constexpr std::array<float, 6> kDefaultMountAnglesDeg = {323.0f, 270.0f, 217.0f, 37.0f, 90.0f, 143.0f};
constexpr std::array<float, 6> kDefaultCoxaAttachDeg = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
constexpr std::array<float, 6> kDefaultFemurAttachDeg = {35.0f, -35.0f, -35.0f, -35.0f, 35.0f, -35.0f};
constexpr std::array<float, 6> kDefaultTibiaAttachDeg = {83.0f, -83.0f, -83.0f, -83.0f, 83.0f, -83.0f};
constexpr std::array<std::array<float, 3>, 6> kDefaultBodyCoxaOffsets = {{
    {{-0.063f, 0.0835f, -0.007f}},
    {{-0.0815f, 0.0f, -0.007f}},
    {{-0.063f, -0.0835f, -0.007f}},
    {{0.063f, 0.0835f, -0.007f}},
    {{0.0815f, 0.0f, -0.007f}},
    {{0.063f, -0.0835f, -0.007f}},
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
  float coxa_mm = 35.0f;
  float femur_mm = 70.0f;
  float tibia_mm = 110.0f;
  float coxa_attach_deg = 0.0f;
  float femur_attach_deg = 0.0f;
  float tibia_attach_deg = 0.0f;
  float coxa_sign = 1.0f;
  float femur_sign = 1.0f;
  float tibia_sign = 1.0f;
};

struct HexapodGeometryState {
  bool valid = false;
  float coxa_mm = 35.0f;
  float femur_mm = 70.0f;
  float tibia_mm = 110.0f;
  float body_radius_mm = 60.0f;
  std::array<HexapodLegLayout, 6> legs{};
};

struct HexapodStatusState {
  bool valid = false;
  uint64_t timestamp_ms = 0;
  int loop_counter = 0;
  int active_mode = 0;
  int active_fault = 0;
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
};

struct HexapodBodyPoseState {
  bool valid = false;
  Vec3 position{};
  float roll_rad = 0.0f;
  float pitch_rad = 0.0f;
  float yaw_rad = 0.0f;
};

struct HexapodTelemetryState {
  bool has_geometry = false;
  bool has_joints = false;
  HexapodGeometryState geometry{};
  HexapodStatusState status{};
  HexapodBodyPoseState body_pose{};
  std::array<std::array<float, 3>, 6> angles_deg{};
};

struct AppUiState {
  bool show_scene = true;
  bool show_robot = true;
  bool show_terrain = true;
  bool follow_active = true;
  bool rotate_scene = false;
  bool show_overlay = true;
  bool show_debug = false;
  bool log_kinematics = false;
  int source_mode = 0;
};

struct KinematicsLogState {
  bool enabled = false;
  bool header_written = false;
  std::string path = "/tmp/hexapod_visualiser_kinematics.csv";
  std::string last_error{};
  std::ofstream stream{};
  std::uint64_t rows_written = 0;
};

struct JointAnglesRad {
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
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
  bool start_kinematics_log = false;
  std::string kinematics_log_path = "/tmp/hexapod_visualiser_kinematics.csv";
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

Vec3 GeoToSceneVec(const Vec3& value);
Vec3 LocoToSceneVec(const Vec3& value);
Vec3 SimVecToServerVec(const Vec3& sim);
Vec3 SimToSceneVec(const Vec3& sim);

Vec3 RotateServerPointByPose(const Vec3& point, const HexapodBodyPoseState& pose) {
  // Body Euler angles (roll/pitch/yaw) are in the locomotion frame (+X=fwd, +Y=left, +Z=up).
  // `point` is in the geometry frame (+X=right, +Y=fwd, +Z=up).
  // Convert geo->loco, apply rotation, convert loco->geo so the axes align correctly.
  // geo->loco: loco = { geo.y, -geo.x, geo.z }
  const float px = point.y;
  const float py = -point.x;
  const float pz = point.z;

  const float cx = std::cos(pose.roll_rad),  sx = std::sin(pose.roll_rad);
  const float cy = std::cos(pose.pitch_rad), sy = std::sin(pose.pitch_rad);
  const float cz = std::cos(pose.yaw_rad),   sz = std::sin(pose.yaw_rad);

  // Rx: roll around loco-X (forward)
  const float x1 = px;
  const float y1 = cx * py - sx * pz;
  const float z1 = sx * py + cx * pz;

  // Ry: pitch around loco-Y (left)
  const float x2 = cy * x1 + sy * z1;
  const float y2 = y1;
  const float z2 = -sy * x1 + cy * z1;

  // Rz: yaw around Z (shared axis)
  const float x3 = cz * x2 - sz * y2;
  const float y3 = sz * x2 + cz * y2;
  const float z3 = z2;

  // loco->geo: geo = { -loco.y, loco.x, loco.z }
  return Vec3{-y3, x3, z3};
}

Vec3 TransformBodyPoint(const Vec3& point, const HexapodBodyPoseState& pose) {
  if (!pose.valid) {
    return GeoToSceneVec(point);
  }
  const Vec3 rotated_server = RotateServerPointByPose(point, pose);
  const Vec3 rotated = GeoToSceneVec(rotated_server);
  const Vec3 origin = LocoToSceneVec(pose.position);
  return Vec3{origin.x + rotated.x, origin.y + rotated.y, origin.z + rotated.z};
}

void ApplyQuaternion(const Quat& quat) {
  const Quat q = NormalizeQuat(quat);

  const float angle = 2.0f * std::acos(Clamp(q.w, -1.0f, 1.0f));
  const float sin_half = std::sqrt(std::max(0.0f, 1.0f - q.w * q.w));
  if (sin_half <= 1e-5f) {
    return;
  }

  const float axis_x = q.x / sin_half;
  const float axis_y = q.y / sin_half;
  const float axis_z = q.z / sin_half;
  glRotatef(angle * 180.0f / kPi, axis_x, axis_y, axis_z);
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
        const Vec3 child_position_sim{
            entity.position.x + child_offset.x,
            entity.position.y + child_offset.y,
            entity.position.z + child_offset.z,
        };
        const Vec3 child_position = SimToSceneVec(child_position_sim);
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
        SimToSceneVec(entity.position),
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
      ExpandBounds(bounds, SimToSceneVec(Vec3{x, y, z}));
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
    // Left legs (LF, LM, LR) use -1; right legs +1 — same as server `side_sign` for L1/L2/L3 vs R1/R2/R3.
    const float side = (i < 3) ? -1.0f : 1.0f;
    state.legs[i].coxa_sign = side;
    state.legs[i].femur_sign = side;
    state.legs[i].tibia_sign = side;
  }
  state.coxa_mm = 43.0f;
  state.femur_mm = 60.0f;
  state.tibia_mm = 104.0f;
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
  return true;
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

  // Handle server restarts before merging this message: if we cleared after parsing, we would drop
  // the first useful joints+geometry packet from the new run.
  std::optional<int> new_loop_counter;
  if (const auto loop_counter = ExtractIntField(payload, "loop_counter")) {
    new_loop_counter = *loop_counter;
    if (telemetry.status.valid && *loop_counter < telemetry.status.loop_counter) {
      telemetry = HexapodTelemetryState{};
    }
  }

  if (ParseHexapodGeometryPacket(payload, telemetry.geometry)) {
    telemetry.has_geometry = true;
  }
  const bool parsed_angles = ParseAnglesPacket(payload, telemetry.angles_deg);
  telemetry.has_joints = parsed_angles;
  bool has_body_pose = telemetry.body_pose.valid;
  if (const auto has_pose = ExtractBoolField(payload, "has_body_pose")) {
    has_body_pose = *has_pose;
    if (!has_body_pose) {
      telemetry.body_pose = {};
    }
  }
  if (const auto body_position = ExtractFloat3Field(payload, "body_position")) {
    telemetry.body_pose.position = {(*body_position)[0], (*body_position)[1], (*body_position)[2]};
    has_body_pose = true;
  }
  if (const auto body_roll = ExtractFloatField(payload, "body_roll_rad")) {
    telemetry.body_pose.roll_rad = *body_roll;
    has_body_pose = true;
  }
  if (const auto body_pitch = ExtractFloatField(payload, "body_pitch_rad")) {
    telemetry.body_pose.pitch_rad = *body_pitch;
    has_body_pose = true;
  }
  if (const auto body_yaw = ExtractFloatField(payload, "body_yaw_rad")) {
    telemetry.body_pose.yaw_rad = *body_yaw;
    has_body_pose = true;
  }
  telemetry.body_pose.valid = has_body_pose;

  if (const auto timestamp_ms = ExtractUintField(payload, "timestamp_ms")) {
    telemetry.status.timestamp_ms = *timestamp_ms;
  }
  if (new_loop_counter.has_value()) {
    telemetry.status.loop_counter = *new_loop_counter;
  }
  if (const auto active_mode = ExtractIntField(payload, "mode")) {
    telemetry.status.active_mode = *active_mode;
  }
  if (const auto active_fault = ExtractIntField(payload, "active_fault")) {
    telemetry.status.active_fault = *active_fault;
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
  ParseFusionSummary(payload, telemetry.status);
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

const char* FaultCodeName(int fault) {
  switch (fault) {
    case 0:
      return "NONE";
    case 1:
      return "BUS_TIMEOUT";
    case 2:
      return "ESTOP";
    case 3:
      return "TIP_OVER";
    case 4:
      return "ESTIMATOR_INVALID";
    case 5:
      return "MOTOR_FAULT";
    case 6:
      return "JOINT_LIMIT";
    case 7:
      return "COMMAND_TIMEOUT";
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

// Geometry frame (+X=right, +Y=fwd, +Z=up) → scene frame (+X=right, +Y=up, -Z=fwd).
// Used for joint positions produced by ComputeRobotLeg.
Vec3 GeoToSceneVec(const Vec3& value) {
  return Vec3{value.x, value.z, -value.y};
}

// Locomotion frame (+X=fwd, +Y=left, +Z=up) → scene frame (+X=right, +Y=up, -Z=fwd).
// Used for body position and any vectors already in locomotion frame.
Vec3 LocoToSceneVec(const Vec3& value) {
  return Vec3{-value.y, value.z, -value.x};
}

// Physics sim / UDP terrain use Bullet-style axes (+Y up). Same mapping as `physics_sim_bridge::simVecToServer`.
Vec3 SimVecToServerVec(const Vec3& sim) {
  return Vec3{-sim.z, sim.x, sim.y};
}

Vec3 SimToSceneVec(const Vec3& sim) {
  return LocoToSceneVec(SimVecToServerVec(sim));
}

JointAnglesRad ComputeJointAnglesRad(const HexapodLegLayout& layout, const std::array<float, 3>& angles_deg) {
  // Incoming angles are servo-space (same values written to hardware by server).
  // Convert back to joint-space with the true inverse of
  // servo = sign * joint + offset  =>  joint = sign * (servo - offset)
  const float coxa_servo_rad = angles_deg[0] * kPi / 180.0f;
  const float femur_servo_rad = angles_deg[1] * kPi / 180.0f;
  const float tibia_servo_rad = angles_deg[2] * kPi / 180.0f;

  const float coxa_offset_rad = layout.coxa_attach_deg * kPi / 180.0f;
  const float femur_offset_rad = layout.femur_attach_deg * kPi / 180.0f;
  const float tibia_offset_rad = layout.tibia_attach_deg * kPi / 180.0f;

  return JointAnglesRad{
      layout.coxa_sign * (coxa_servo_rad - coxa_offset_rad),
      layout.femur_sign * (femur_servo_rad - femur_offset_rad),
      layout.tibia_sign * (tibia_servo_rad - tibia_offset_rad),
  };
}

RobotKinematics ComputeRobotLeg(const HexapodLegLayout& layout, const std::array<float, 3>& angles_deg) {
  const JointAnglesRad joint = ComputeJointAnglesRad(layout, angles_deg);
  const float q1 = joint.q1;
  const float q2 = joint.q2;
  const float q3 = joint.q3;
  const float body_radius = std::sqrt(layout.body_coxa_offset.x * layout.body_coxa_offset.x
                                      + layout.body_coxa_offset.y * layout.body_coxa_offset.y);
  const Vec3 anchor{
      layout.body_coxa_offset.x != 0.0f || layout.body_coxa_offset.y != 0.0f ? layout.body_coxa_offset.x
                                                                              : body_radius * std::sin(layout.mount_angle_rad),
      layout.body_coxa_offset.x != 0.0f || layout.body_coxa_offset.y != 0.0f ? layout.body_coxa_offset.y
                                                                              : body_radius * std::cos(layout.mount_angle_rad),
      layout.body_coxa_offset.z,
  };

  const float coxa = layout.coxa_mm * 0.001f;
  const float femur = layout.femur_mm * 0.001f;
  const float tibia = layout.tibia_mm * 0.001f;

  // Leg-local +X should point radially outward (center -> coxa mount) in body XY.
  // Derive basis from anchor when available; fall back to mount-angle interpretation only if needed.
  const float radial_norm = std::hypot(anchor.x, anchor.y);
  const float x_axis_x = (radial_norm > 1e-6f) ? (anchor.x / radial_norm) : std::sin(layout.mount_angle_rad);
  const float x_axis_y = (radial_norm > 1e-6f) ? (anchor.y / radial_norm) : std::cos(layout.mount_angle_rad);
  const float y_axis_x = -x_axis_y;
  const float y_axis_y = x_axis_x;
  const auto legToBody = [&](float x_leg, float y_leg, float z_leg) -> Vec3 {
    return Vec3{
        anchor.x + (x_axis_x * x_leg + y_axis_x * y_leg),
        anchor.y + (x_axis_y * x_leg + y_axis_y * y_leg),
        anchor.z + z_leg,
    };
  };

  // Coxa axis is leg-local +X (in body plane), so q1 rotates distal chain in y/z.
  const Vec3 shoulder = legToBody(coxa, 0.0f, 0.0f);

  const float femur_proj = femur * std::cos(q2);
  const float femur_vert = femur * std::sin(q2);
  // Keep lateral sweep sign consistent with the original gait appearance, while flipping
  // vertical sign so distal joints stay below the body for the current q1 branch.
  const Vec3 knee = legToBody(
      coxa + femur_proj,
      -std::sin(q1) * femur_vert,
      -std::cos(q1) * femur_vert);

  const float tibia_total = q2 + q3;
  const float tibia_proj = tibia * std::cos(tibia_total);
  const float tibia_vert = tibia * std::sin(tibia_total);
  const float z_plane = femur_vert + tibia_vert;
  const Vec3 foot = legToBody(
      coxa + femur_proj + tibia_proj,
      -std::sin(q1) * z_plane,
      -std::cos(q1) * z_plane);

  return {anchor, shoulder, knee, foot};
}

// Unit vector in body frame (+Z up) along the coxa rotation axis (leg-local +X mapped to body XY).
Vec3 CoxaRotationAxisBodyUnit(const HexapodLegLayout& layout) {
  const float ox = layout.body_coxa_offset.x;
  const float oy = layout.body_coxa_offset.y;
  const float h = std::hypot(ox, oy);
  if (h > 1e-5f) {
    // Radial direction from body center to mount point = leg-local +X axis in body frame.
    return Vec3{ox / h, oy / h, 0.0f};
  }
  return Vec3{std::sin(layout.mount_angle_rad), std::cos(layout.mount_angle_rad), 0.0f};
}

RobotKinematics ComputeRobotLegScene(const HexapodLegLayout& layout,
                                     const std::array<float, 3>& angles_deg) {
  const RobotKinematics server = ComputeRobotLeg(layout, angles_deg);
  return RobotKinematics{
      GeoToSceneVec(server.anchor),
      GeoToSceneVec(server.shoulder),
      GeoToSceneVec(server.knee),
      GeoToSceneVec(server.foot),
  };
}

bool StartKinematicsLog(KinematicsLogState& log) {
  log.stream.close();
  log.stream.clear();
  log.stream.open(log.path, std::ios::out | std::ios::trunc);
  if (!log.stream.is_open()) {
    log.enabled = false;
    log.last_error = "failed to open CSV path";
    return false;
  }
  log.stream << "time_s,loop_counter,leg,servo_coxa_deg,servo_femur_deg,servo_tibia_deg,"
                "joint_q1_rad,joint_q2_rad,joint_q3_rad,"
                "anchor_body_x,anchor_body_y,anchor_body_z,"
                "shoulder_body_x,shoulder_body_y,shoulder_body_z,"
                "knee_body_x,knee_body_y,knee_body_z,"
                "foot_body_x,foot_body_y,foot_body_z,"
                "anchor_scene_x,anchor_scene_y,anchor_scene_z,"
                "shoulder_scene_x,shoulder_scene_y,shoulder_scene_z,"
                "knee_scene_x,knee_scene_y,knee_scene_z,"
                "foot_scene_x,foot_scene_y,foot_scene_z\n";
  log.stream.flush();
  if (!log.stream.good()) {
    log.enabled = false;
    log.last_error = "failed to write CSV header";
    return false;
  }
  log.enabled = true;
  log.header_written = true;
  log.rows_written = 0;
  log.last_error.clear();
  return true;
}

void StopKinematicsLog(KinematicsLogState& log) {
  if (log.stream.is_open()) {
    log.stream.flush();
    log.stream.close();
  }
  log.enabled = false;
}

void LogKinematicsFrame(KinematicsLogState& log,
                        const HexapodGeometryState& geometry,
                        const HexapodTelemetryState& telemetry,
                        double time_s) {
  if (!log.enabled || !log.stream.is_open() || !telemetry.has_joints) {
    return;
  }
  if (!log.header_written) {
    log.last_error = "CSV header missing";
    log.enabled = false;
    return;
  }

  log.stream << std::fixed << std::setprecision(6);
  for (std::size_t i = 0; i < geometry.legs.size(); ++i) {
    const RobotKinematics leg_body = ComputeRobotLeg(geometry.legs[i], telemetry.angles_deg[i]);
    const JointAnglesRad joint = ComputeJointAnglesRad(geometry.legs[i], telemetry.angles_deg[i]);
    const Vec3 anchor_scene = TransformBodyPoint(leg_body.anchor, telemetry.body_pose);
    const Vec3 shoulder_scene = TransformBodyPoint(leg_body.shoulder, telemetry.body_pose);
    const Vec3 knee_scene = TransformBodyPoint(leg_body.knee, telemetry.body_pose);
    const Vec3 foot_scene = TransformBodyPoint(leg_body.foot, telemetry.body_pose);

    log.stream << time_s << ','
               << telemetry.status.loop_counter << ','
               << kLegKeys[i] << ','
               << telemetry.angles_deg[i][0] << ','
               << telemetry.angles_deg[i][1] << ','
               << telemetry.angles_deg[i][2] << ','
               << joint.q1 << ','
               << joint.q2 << ','
               << joint.q3 << ','
               << leg_body.anchor.x << ',' << leg_body.anchor.y << ',' << leg_body.anchor.z << ','
               << leg_body.shoulder.x << ',' << leg_body.shoulder.y << ',' << leg_body.shoulder.z << ','
               << leg_body.knee.x << ',' << leg_body.knee.y << ',' << leg_body.knee.z << ','
               << leg_body.foot.x << ',' << leg_body.foot.y << ',' << leg_body.foot.z << ','
               << anchor_scene.x << ',' << anchor_scene.y << ',' << anchor_scene.z << ','
               << shoulder_scene.x << ',' << shoulder_scene.y << ',' << shoulder_scene.z << ','
               << knee_scene.x << ',' << knee_scene.y << ',' << knee_scene.z << ','
               << foot_scene.x << ',' << foot_scene.y << ',' << foot_scene.z << '\n';
    ++log.rows_written;
  }

  log.stream.flush();
  if (!log.stream.good()) {
    log.last_error = "write failed (disk/path permissions?)";
    log.enabled = false;
  }
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
    ExpandBounds(bounds, LocoToSceneVec(corner));
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

void DrawWireCube(const Vec3& half_extents) {
  const std::array<Vec3, 8> vertices = {{
      {-half_extents.x, -half_extents.y, -half_extents.z},
      {half_extents.x, -half_extents.y, -half_extents.z},
      {half_extents.x, half_extents.y, -half_extents.z},
      {-half_extents.x, half_extents.y, -half_extents.z},
      {-half_extents.x, -half_extents.y, half_extents.z},
      {half_extents.x, -half_extents.y, half_extents.z},
      {half_extents.x, half_extents.y, half_extents.z},
      {-half_extents.x, half_extents.y, half_extents.z},
  }};

  const std::array<std::array<int, 2>, 12> edges = {{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}},
  }};

  glBegin(GL_LINES);
  for (const auto& edge : edges) {
    const Vec3& a = vertices[edge[0]];
    const Vec3& b = vertices[edge[1]];
    glVertex3f(a.x, a.y, a.z);
    glVertex3f(b.x, b.y, b.z);
  }
  glEnd();
}

void DrawWireSphere(float radius, int slices = 18, int stacks = 10) {
  for (int stack = 1; stack < stacks; ++stack) {
    const float phi = -0.5f * kPi + kPi * static_cast<float>(stack) / static_cast<float>(stacks);
    glBegin(GL_LINE_LOOP);
    for (int slice = 0; slice < slices; ++slice) {
      const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
      const float x = radius * std::cos(phi) * std::cos(theta);
      const float y = radius * std::sin(phi);
      const float z = radius * std::cos(phi) * std::sin(theta);
      glVertex3f(x, y, z);
    }
    glEnd();
  }

  for (int slice = 0; slice < slices; ++slice) {
    const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
    glBegin(GL_LINE_STRIP);
    for (int stack = 0; stack <= stacks; ++stack) {
      const float phi = -0.5f * kPi + kPi * static_cast<float>(stack) / static_cast<float>(stacks);
      const float x = radius * std::cos(phi) * std::cos(theta);
      const float y = radius * std::sin(phi);
      const float z = radius * std::cos(phi) * std::sin(theta);
      glVertex3f(x, y, z);
    }
    glEnd();
  }
}

void DrawWireCapsule(float radius, float half_height) {
  glBegin(GL_LINES);
  for (int i = 0; i < 4; ++i) {
    const float angle = static_cast<float>(i) * 0.5f * kPi;
    const float x = std::cos(angle) * radius;
    const float z = std::sin(angle) * radius;
    glVertex3f(x, -half_height, z);
    glVertex3f(x, half_height, z);
  }
  glEnd();

  glPushMatrix();
  glTranslatef(0.0f, half_height, 0.0f);
  DrawWireSphere(radius, 18, 6);
  glPopMatrix();

  glPushMatrix();
  glTranslatef(0.0f, -half_height, 0.0f);
  DrawWireSphere(radius, 18, 6);
  glPopMatrix();
}

void DrawWireCylinder(float radius, float half_height, int slices = 24) {
  glBegin(GL_LINE_LOOP);
  for (int slice = 0; slice < slices; ++slice) {
    const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
    glVertex3f(radius * std::cos(theta), half_height, radius * std::sin(theta));
  }
  glEnd();

  glBegin(GL_LINE_LOOP);
  for (int slice = 0; slice < slices; ++slice) {
    const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
    glVertex3f(radius * std::cos(theta), -half_height, radius * std::sin(theta));
  }
  glEnd();

  glBegin(GL_LINES);
  for (int i = 0; i < 4; ++i) {
    const float theta = 0.5f * kPi * static_cast<float>(i);
    const float x = radius * std::cos(theta);
    const float z = radius * std::sin(theta);
    glVertex3f(x, -half_height, z);
    glVertex3f(x, half_height, z);
  }
  glEnd();
}

void DrawWireHalfCylinder(float radius, float half_height, int slices = 16) {
  glBegin(GL_LINE_STRIP);
  for (int slice = 0; slice <= slices; ++slice) {
    const float theta = kPi * static_cast<float>(slice) / static_cast<float>(slices);
    glVertex3f(radius * std::cos(theta), half_height, radius * std::sin(theta));
  }
  glEnd();

  glBegin(GL_LINE_STRIP);
  for (int slice = 0; slice <= slices; ++slice) {
    const float theta = kPi * static_cast<float>(slice) / static_cast<float>(slices);
    glVertex3f(radius * std::cos(theta), -half_height, radius * std::sin(theta));
  }
  glEnd();

  glBegin(GL_LINES);
  for (int i = 0; i <= 2; ++i) {
    const float theta = 0.5f * kPi * static_cast<float>(i);
    const float x = radius * std::cos(theta);
    const float z = radius * std::sin(theta);
    glVertex3f(x, -half_height, z);
    glVertex3f(x, half_height, z);
  }

  glVertex3f(-radius, -half_height, 0.0f);
  glVertex3f(radius, -half_height, 0.0f);
  glVertex3f(-radius, half_height, 0.0f);
  glVertex3f(radius, half_height, 0.0f);
  glVertex3f(-radius, -half_height, 0.0f);
  glVertex3f(-radius, half_height, 0.0f);
  glVertex3f(radius, -half_height, 0.0f);
  glVertex3f(radius, half_height, 0.0f);
  glEnd();
}

void DrawPlane(const Vec3& normal, float offset) {
  const Vec3 up = Normalize(normal);
  const Vec3 tangent_seed = std::fabs(up.y) > 0.9f ? Vec3{1.0f, 0.0f, 0.0f} : Vec3{0.0f, 1.0f, 0.0f};
  const Vec3 tangent = Normalize(Cross(tangent_seed, up));
  const Vec3 bitangent = Normalize(Cross(up, tangent));
  const Vec3 center{up.x * offset, up.y * offset, up.z * offset};
  constexpr float scale = 12.0f;

  const Vec3 corners[4] = {
      {center.x + (tangent.x + bitangent.x) * scale, center.y + (tangent.y + bitangent.y) * scale,
       center.z + (tangent.z + bitangent.z) * scale},
      {center.x + (tangent.x - bitangent.x) * scale, center.y + (tangent.y - bitangent.y) * scale,
       center.z + (tangent.z - bitangent.z) * scale},
      {center.x + (-tangent.x - bitangent.x) * scale, center.y + (-tangent.y - bitangent.y) * scale,
       center.z + (-tangent.z - bitangent.z) * scale},
      {center.x + (-tangent.x + bitangent.x) * scale, center.y + (-tangent.y + bitangent.y) * scale,
       center.z + (-tangent.z + bitangent.z) * scale},
  };

  glColor4f(0.14f, 0.18f, 0.22f, 1.0f);
  glBegin(GL_QUADS);
  for (const Vec3& corner : corners) {
    glVertex3f(corner.x, corner.y, corner.z);
  }
  glEnd();

  glColor3f(0.28f, 0.34f, 0.40f);
  glBegin(GL_LINES);
  for (int i = -6; i <= 6; ++i) {
    const float d = static_cast<float>(i) * 2.0f;
    const Vec3 a{center.x + tangent.x * scale + bitangent.x * d, center.y + tangent.y * scale + bitangent.y * d,
                 center.z + tangent.z * scale + bitangent.z * d};
    const Vec3 b{center.x - tangent.x * scale + bitangent.x * d, center.y - tangent.y * scale + bitangent.y * d,
                 center.z - tangent.z * scale + bitangent.z * d};
    const Vec3 c{center.x + bitangent.x * scale + tangent.x * d, center.y + bitangent.y * scale + tangent.y * d,
                 center.z + bitangent.z * scale + tangent.z * d};
    const Vec3 d2{center.x - bitangent.x * scale + tangent.x * d, center.y - bitangent.y * scale + tangent.y * d,
                  center.z - bitangent.z * scale + tangent.z * d};
    glVertex3f(a.x, a.y, a.z);
    glVertex3f(b.x, b.y, b.z);
    glVertex3f(c.x, c.y, c.z);
    glVertex3f(d2.x, d2.y, d2.z);
  }
  glEnd();
}

void DrawPrimitiveShape(
    ShapeType shape,
    float radius,
    float half_height,
    const Vec3& half_extents,
    const Vec3& plane_normal,
    float plane_offset) {
  switch (shape) {
    case ShapeType::kBox:
      glColor3f(0.92f, 0.43f, 0.21f);
      DrawWireCube(half_extents);
      break;
    case ShapeType::kSphere:
      glColor3f(0.22f, 0.75f, 0.91f);
      DrawWireSphere(radius);
      break;
    case ShapeType::kCapsule:
      glColor3f(0.37f, 0.82f, 0.51f);
      DrawWireCapsule(radius, half_height);
      break;
    case ShapeType::kCylinder:
      glColor3f(0.88f, 0.78f, 0.26f);
      DrawWireCylinder(radius, half_height);
      break;
    case ShapeType::kHalfCylinder:
      glColor3f(0.82f, 0.56f, 0.88f);
      DrawWireHalfCylinder(radius, half_height);
      break;
    case ShapeType::kPlane:
      DrawPlane(plane_normal, plane_offset);
      break;
    case ShapeType::kCompound:
    case ShapeType::kUnknown:
      break;
  }
}

void DrawTerrainPatch(const TerrainPatchState& terrain) {
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

  glColor3f(0.30f, 0.78f, 0.48f);
  for (int row = 0; row < terrain.rows; ++row) {
    glBegin(GL_LINE_STRIP);
    for (int col = 0; col < terrain.cols; ++col) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
      const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
      const Vec3 p = SimToSceneVec(Vec3{x, terrain.heights[index], z});
      glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
  }

  for (int col = 0; col < terrain.cols; ++col) {
    glBegin(GL_LINE_STRIP);
    for (int row = 0; row < terrain.rows; ++row) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
      const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
      const Vec3 p = SimToSceneVec(Vec3{x, terrain.heights[index], z});
      glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
  }

  if (terrain.schema_version >= 2 && terrain.collision_heights.size() >= expected) {
    glColor3f(0.95f, 0.45f, 0.18f);
    for (int row = 0; row < terrain.rows; ++row) {
      glBegin(GL_LINE_STRIP);
      for (int col = 0; col < terrain.cols; ++col) {
        const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
        const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
        const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
        const Vec3 p = SimToSceneVec(Vec3{x, terrain.collision_heights[index], z});
        glVertex3f(p.x, p.y, p.z);
      }
      glEnd();
    }
    for (int col = 0; col < terrain.cols; ++col) {
      glBegin(GL_LINE_STRIP);
      for (int row = 0; row < terrain.rows; ++row) {
        const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
        const float x = origin_x + (static_cast<float>(col) * terrain.cell_size_m);
        const float z = origin_z + (static_cast<float>(row) * terrain.cell_size_m);
        const Vec3 p = SimToSceneVec(Vec3{x, terrain.collision_heights[index], z});
        glVertex3f(p.x, p.y, p.z);
      }
      glEnd();
    }
  }

  const Vec3 up_sim = Normalize(terrain.plane_normal);
  const float arrow_scale = std::max(terrain.cell_size_m, 0.08f);
  const Vec3 normal_base = SimToSceneVec(Vec3{terrain.center.x, terrain.base_height_m, terrain.center.z});
  const Vec3 up_scene = Normalize(SimToSceneVec(up_sim));
  const Vec3 normal_tip{
      normal_base.x + up_scene.x * arrow_scale,
      normal_base.y + up_scene.y * arrow_scale,
      normal_base.z + up_scene.z * arrow_scale};
  glColor3f(0.95f, 0.88f, 0.24f);
  glBegin(GL_LINES);
  glVertex3f(normal_base.x, normal_base.y, normal_base.z);
  glVertex3f(normal_tip.x, normal_tip.y, normal_tip.z);
  glEnd();

  glPointSize(5.0f);
  glBegin(GL_POINTS);
  glVertex3f(normal_base.x, normal_base.y, normal_base.z);
  glEnd();
}

void DrawHexapodModel(const HexapodGeometryState& geometry,
                      const std::array<std::array<float, 3>, 6>& angles_deg,
                      const HexapodStatusState& status,
                      const HexapodBodyPoseState& pose,
                      bool draw_debug_coxa_axes) {
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

  glColor4f(healthy ? 0.18f : 0.35f, healthy ? 0.32f : 0.18f, healthy ? 0.48f : 0.12f, 0.60f);
  glBegin(GL_POLYGON);
  for (const std::size_t index : body_loop) {
    const RobotKinematics leg = ComputeRobotLeg(geometry.legs[index], angles_deg[index]);
    const Vec3 body_vertex = TransformBodyPoint(Vec3{leg.anchor.x, leg.anchor.y, leg.anchor.z + body_height}, pose);
    glVertex3f(body_vertex.x, body_vertex.y, body_vertex.z);
  }
  glEnd();

  glColor3f(healthy ? 0.44f : 0.90f, healthy ? 0.74f : 0.32f, healthy ? 1.00f : 0.18f);
  glBegin(GL_LINE_LOOP);
  for (const std::size_t index : body_loop) {
    const RobotKinematics leg = ComputeRobotLeg(geometry.legs[index], angles_deg[index]);
    const Vec3 body_vertex = TransformBodyPoint(Vec3{leg.anchor.x, leg.anchor.y, leg.anchor.z + body_height}, pose);
    glVertex3f(body_vertex.x, body_vertex.y, body_vertex.z);
  }
  glEnd();

  glPointSize(6.0f);
  glColor3f(0.95f, 0.93f, 0.85f);
  glBegin(GL_POINTS);
  const Vec3 body_center = TransformBodyPoint(Vec3{0.0f, 0.0f, body_height}, pose);
  glVertex3f(body_center.x, body_center.y, body_center.z);
  glEnd();

  for (std::size_t i = 0; i < geometry.legs.size(); ++i) {
    const RobotKinematics leg = ComputeRobotLeg(geometry.legs[i], angles_deg[i]);
    const Vec3 anchor = TransformBodyPoint(leg.anchor, pose);
    const Vec3 shoulder = TransformBodyPoint(leg.shoulder, pose);
    const Vec3 knee = TransformBodyPoint(leg.knee, pose);
    const Vec3 foot = TransformBodyPoint(leg.foot, pose);

    glColor3f(0.95f, 0.58f, 0.18f);
    glBegin(GL_LINES);
    glVertex3f(anchor.x, anchor.y, anchor.z);
    glVertex3f(shoulder.x, shoulder.y, shoulder.z);
    glEnd();

    glColor3f(0.30f, 0.80f, 0.42f);
    glBegin(GL_LINES);
    glVertex3f(shoulder.x, shoulder.y, shoulder.z);
    glVertex3f(knee.x, knee.y, knee.z);
    glEnd();

    glColor3f(0.25f, 0.72f, 0.95f);
    glBegin(GL_LINES);
    glVertex3f(knee.x, knee.y, knee.z);
    glVertex3f(foot.x, foot.y, foot.z);
    glEnd();

    glPointSize(5.0f);
    glColor3f(0.96f, 0.94f, 0.90f);
    glBegin(GL_POINTS);
    glVertex3f(anchor.x, anchor.y, anchor.z);
    glVertex3f(shoulder.x, shoulder.y, shoulder.z);
    glVertex3f(knee.x, knee.y, knee.z);
    glVertex3f(foot.x, foot.y, foot.z);
    glEnd();

    if (draw_debug_coxa_axes) {
      const Vec3 k_body = CoxaRotationAxisBodyUnit(geometry.legs[i]);
      constexpr float axis_len_m = 0.045f;
      const Vec3 shoulder_body = leg.shoulder;
      const Vec3 tip_body = Vec3{
          shoulder_body.x + k_body.x * axis_len_m,
          shoulder_body.y + k_body.y * axis_len_m,
          shoulder_body.z + k_body.z * axis_len_m,
      };
      const Vec3 tip = TransformBodyPoint(tip_body, pose);
      glLineWidth(2.0f);
      glColor3f(0.92f, 0.35f, 0.95f);
      glBegin(GL_LINES);
      glVertex3f(shoulder.x, shoulder.y, shoulder.z);
      glVertex3f(tip.x, tip.y, tip.z);
      glEnd();
      glLineWidth(1.0f);
    }
  }
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

bool ParsePacket(const std::string& payload,
                 std::map<std::uint32_t, EntityState>& entities,
                 TerrainPatchState& terrain_patch,
                 HexapodTelemetryState& telemetry,
                 std::string& packet_kind) {
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
    telemetry = {};
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
      const ssize_t bytes = ::recvfrom(socket_fd_, buffer.data(), buffer.size() - 1, 0, nullptr, nullptr);
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

      buffer[static_cast<std::size_t>(bytes)] = '\0';
      std::string packet_kind;
      if (ParsePacket(buffer.data(), entities, terrain_patch, telemetry, packet_kind)) {
        last_packet_kind = packet_kind;
        ++accepted_packets;
        ++packets;
      } else {
        ++rejected_packets;
      }
    }

    return packets;
  }

 private:
  int socket_fd_ = -1;
  bool valid_ = false;
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

    if (arg == "--log-kinematics") {
      options.start_kinematics_log = true;
      continue;
    }

    if (arg == "--log-kinematics-path") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --log-kinematics-path\n";
        std::exit(1);
      }
      options.kinematics_log_path = argv[++i];
      options.start_kinematics_log = true;
      continue;
    }

    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: hexapod-opengl-visualiser [--udp-port <port>] [--log-kinematics] "
                   "[--log-kinematics-path <csv-path>]\n"
                << "Press F1 while running to toggle the overlay panel.\n";
      std::exit(0);
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    std::exit(1);
  }

  return options;
}

void ConfigureProjection(int width, int height) {
  const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
  constexpr float near_plane = 0.1f;
  constexpr float far_plane = 100.0f;
  constexpr float fov_y_degrees = 50.0f;
  const float top = std::tan(fov_y_degrees * 0.5f * kPi / 180.0f) * near_plane;
  const float right = top * aspect;

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-right, right, -top, top, near_plane, far_plane);
}

void DrawScene(const std::map<std::uint32_t, EntityState>& entities,
               const TerrainPatchState& terrain_patch,
               const HexapodTelemetryState& telemetry,
               const AppUiState& ui,
               const CameraState& camera,
               float time_s) {
  glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  SceneBounds bounds;
  if (ui.show_scene) {
    bounds = ComputeSceneBounds(entities);
  }
  if (ui.show_terrain) {
    ExpandTerrainPatchBounds(bounds, terrain_patch);
  }
  if (ui.show_robot && telemetry.has_geometry && telemetry.has_joints) {
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
  glTranslatef(0.0f, -0.35f * scene_radius, -camera_distance);
  glRotatef(camera.pitch_deg, 1.0f, 0.0f, 0.0f);
  glRotatef(yaw, 0.0f, 1.0f, 0.0f);
  glTranslatef(-(center.x + camera.pan_x), -(center.y + camera.pan_y), -center.z);

  if (ui.show_terrain) {
    DrawTerrainPatch(terrain_patch);
  }

  if (ui.show_scene) {
    for (const auto& [id, entity] : entities) {
      (void)id;
      if (!entity.has_static) {
        continue;
      }

      if (entity.shape == ShapeType::kPlane) {
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset);
        continue;
      }

      if (!entity.has_frame) {
        continue;
      }

      const Vec3 entity_pos_scene = SimToSceneVec(entity.position);
      glPushMatrix();
      glTranslatef(entity_pos_scene.x, entity_pos_scene.y, entity_pos_scene.z);
      ApplyQuaternion(entity.rotation);

      if (entity.shape == ShapeType::kCompound) {
        if (entity.compound_children.empty()) {
          DrawPrimitiveShape(
              ShapeType::kBox,
              entity.radius,
              entity.half_height,
              entity.half_extents,
              entity.plane_normal,
              entity.plane_offset);
        } else {
          for (const CompoundChildState& child : entity.compound_children) {
            glPushMatrix();
            glTranslatef(child.local_position.x, child.local_position.y, child.local_position.z);
            ApplyQuaternion(child.local_rotation);
            DrawPrimitiveShape(
                child.shape,
                child.radius,
                child.half_height,
                child.half_extents,
                entity.plane_normal,
                entity.plane_offset);
            glPopMatrix();
          }
        }
      } else {
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset);
      }

      glPopMatrix();
    }
  }

  if (ui.show_robot && telemetry.has_joints) {
    const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
    DrawHexapodModel(
        robot_geometry, telemetry.angles_deg, telemetry.status, telemetry.body_pose, ui.show_debug);
  }
}

void DrawUi(AppUiState& ui,
            CameraState& camera,
            const HexapodTelemetryState& telemetry,
            KinematicsLogState& kinematics_log,
            const std::string& source_label,
            uint64_t packets_received,
            uint64_t packets_rejected,
            double last_packet_age_s,
            std::size_t entity_count,
            bool terrain_available) {
  if (!ui.show_overlay) {
    return;
  }

  ImGui::SetNextWindowBgAlpha(0.90f);
  ImGui::Begin("Hexapod Control Room", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

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

  ImGui::Separator();
  ImGui::TextUnformatted("View");
  ImGui::Checkbox("Show scene", &ui.show_scene);
  ImGui::Checkbox("Show robot", &ui.show_robot);
  ImGui::Checkbox("Show terrain", &ui.show_terrain);
  ImGui::Checkbox("Rotate scene", &ui.rotate_scene);
  ImGui::Checkbox("Follow active", &ui.follow_active);
  ImGui::Checkbox("Show debug", &ui.show_debug);
  bool want_log = ui.log_kinematics;
  if (ImGui::Checkbox("Log kinematics CSV", &want_log)) {
    if (want_log) {
      if (StartKinematicsLog(kinematics_log)) {
        ui.log_kinematics = true;
      } else {
        ui.log_kinematics = false;
      }
    } else {
      StopKinematicsLog(kinematics_log);
      ui.log_kinematics = false;
    }
  }
  ImGui::Text("Log path: %s", kinematics_log.path.c_str());
  ImGui::Text("Rows written: %llu", static_cast<unsigned long long>(kinematics_log.rows_written));
  if (!kinematics_log.last_error.empty()) {
    ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f), "Log error: %s", kinematics_log.last_error.c_str());
  }
  ImGui::SliderFloat("Yaw", &camera.yaw_deg, -180.0f, 180.0f);
  ImGui::SliderFloat("Pitch", &camera.pitch_deg, -89.0f, 89.0f);
  ImGui::SliderFloat("Distance", &camera.distance_scale, 1.5f, 12.0f);
  ImGui::SliderFloat("Pan X", &camera.pan_x, -1.0f, 1.0f);
  ImGui::SliderFloat("Pan Y", &camera.pan_y, -1.0f, 1.0f);
  ImGui::SliderFloat("Spin deg/s", &camera.spin_deg_per_s, 0.0f, 45.0f);
  if (ImGui::Button("Reset View")) {
    camera = CameraState{};
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Telemetry");
  if (telemetry.status.valid) {
    ImGui::Text("Mode: %s (%d)", RobotModeName(telemetry.status.active_mode), telemetry.status.active_mode);
    ImGui::Text("Fault: %s (%d)", FaultCodeName(telemetry.status.active_fault), telemetry.status.active_fault);
    ImGui::Text("Bus: %s | Estimator: %s",
                telemetry.status.bus_ok ? "OK" : "FAULT",
                telemetry.status.estimator_valid ? "valid" : "invalid");
    ImGui::Text("Loop: %d | Timestamp: %llu ms",
                telemetry.status.loop_counter,
                static_cast<unsigned long long>(telemetry.status.timestamp_ms));
    ImGui::Text("Voltage: %.2f V | Current: %.2f A", telemetry.status.voltage, telemetry.status.current);
  } else {
    ImGui::TextUnformatted("No server telemetry yet");
  }

  if (telemetry.status.nav_lifecycle.has_value() || telemetry.status.nav_planner_status.has_value() ||
      telemetry.status.nav_block_reason.has_value()) {
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
  }

  if (telemetry.status.fusion_model_trust.has_value() || telemetry.status.fusion_contact_mismatch_ratio.has_value()) {
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
  ImGui::TextUnformatted("Robot geometry");
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

int main(int argc, char** argv) {
  const Options options = ParseArgs(argc, argv);

  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  GLFWwindow* window = glfwCreateWindow(
      kDefaultWindowWidth, kDefaultWindowHeight, "Hexapod OpenGL Visualiser", nullptr, nullptr);
  if (window == nullptr) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glLineWidth(2.0f);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGuiIO& imgui_io = ImGui::GetIO();
  imgui_io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();

#ifndef _WIN32
  UdpReceiver receiver(options.udp_port);
  if (!receiver.valid()) {
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }
#else
  std::cerr << "UDP receiver is not implemented on Windows in this build\n";
  ImGui_ImplOpenGL2_Shutdown();
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
  KinematicsLogState kinematics_log;
  kinematics_log.path = options.kinematics_log_path;
  if (options.start_kinematics_log) {
    ui.log_kinematics = StartKinematicsLog(kinematics_log);
    if (!ui.log_kinematics && !kinematics_log.last_error.empty()) {
      std::cerr << "Failed to start kinematics log: " << kinematics_log.last_error << "\n";
    }
  }
  CameraState camera;
  uint64_t accepted_packets = 0;
  uint64_t rejected_packets = 0;
  std::string last_packet_kind = "waiting";
  double last_packet_time_s = std::numeric_limits<double>::quiet_NaN();
  double last_title_update_s = -1.0;
  bool overlay_toggle_down = false;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

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

    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
    ConfigureProjection(framebuffer_width, framebuffer_height);

    const float time_s = static_cast<float>(glfwGetTime());
    if (ui.log_kinematics) {
      const HexapodGeometryState robot_geometry =
          telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
      LogKinematicsFrame(kinematics_log, robot_geometry, telemetry, glfwGetTime());
      if (!kinematics_log.enabled) {
        ui.log_kinematics = false;
      }
    }
    DrawScene(entities, terrain_patch, telemetry, ui, camera, time_s);

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    const double packet_age_s = std::isfinite(last_packet_time_s) ? glfwGetTime() - last_packet_time_s
                                                                  : std::numeric_limits<double>::quiet_NaN();
    DrawUi(ui,
           camera,
           telemetry,
           kinematics_log,
           last_packet_kind,
           accepted_packets,
           rejected_packets,
           packet_age_s,
           entities.size(),
           terrain_patch.valid);
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    if (last_title_update_s < 0.0 || glfwGetTime() - last_title_update_s > 0.25) {
      std::ostringstream title;
      title << "Hexapod OpenGL Visualiser | " << last_packet_kind << " | UDP " << options.udp_port
            << " | entities " << entities.size() << " | packets " << accepted_packets;
      glfwSetWindowTitle(window, title.str().c_str());
      last_title_update_s = glfwGetTime();
    }

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
