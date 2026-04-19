#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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
  float base_height_m = 0.0f;
  float plane_height_m = 0.0f;
  Vec3 plane_normal{0.0f, 1.0f, 0.0f};
  std::vector<float> heights{};
  std::vector<float> confidences{};
};

struct Options {
  int udp_port = kDefaultUdpPort;
};

struct SceneBounds {
  Vec3 min{};
  Vec3 max{};
  bool valid = false;
};

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
  const std::size_t expected = static_cast<std::size_t>(terrain.rows * terrain.cols);
  const bool has_heights = terrain.heights.size() >= expected;

  for (int row = 0; row < terrain.rows; ++row) {
    for (int col = 0; col < terrain.cols; ++col) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = terrain.center.x + (static_cast<float>(col) * terrain.cell_size_m) - half_span_x;
      const float z = terrain.center.z + (static_cast<float>(row) * terrain.cell_size_m) - half_span_z;
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

  glColor3f(0.30f, 0.78f, 0.48f);
  for (int row = 0; row < terrain.rows; ++row) {
    glBegin(GL_LINE_STRIP);
    for (int col = 0; col < terrain.cols; ++col) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = terrain.center.x + (static_cast<float>(col) * terrain.cell_size_m) - half_span_x;
      const float z = terrain.center.z + (static_cast<float>(row) * terrain.cell_size_m) - half_span_z;
      glVertex3f(x, terrain.heights[index], z);
    }
    glEnd();
  }

  for (int col = 0; col < terrain.cols; ++col) {
    glBegin(GL_LINE_STRIP);
    for (int row = 0; row < terrain.rows; ++row) {
      const std::size_t index = static_cast<std::size_t>(row * terrain.cols + col);
      const float x = terrain.center.x + (static_cast<float>(col) * terrain.cell_size_m) - half_span_x;
      const float z = terrain.center.z + (static_cast<float>(row) * terrain.cell_size_m) - half_span_z;
      glVertex3f(x, terrain.heights[index], z);
    }
    glEnd();
  }

  const Vec3 up = Normalize(terrain.plane_normal);
  const float arrow_scale = std::max(terrain.cell_size_m, 0.08f);
  const Vec3 normal_base{terrain.center.x, terrain.base_height_m, terrain.center.z};
  const Vec3 normal_tip{
      normal_base.x + up.x * arrow_scale,
      normal_base.y + up.y * arrow_scale,
      normal_base.z + up.z * arrow_scale};
  glColor3f(0.95f, 0.88f, 0.24f);
  glBegin(GL_LINES);
  glVertex3f(normal_base.x, normal_base.y, normal_base.z);
  glVertex3f(normal_tip.x, normal_tip.y, normal_tip.z);
  glEnd();

  glPointSize(5.0f);
  glBegin(GL_POINTS);
  glVertex3f(terrain.center.x, terrain.base_height_m, terrain.center.z);
  glEnd();
}

bool ParseTerrainPatchPacket(const std::string& payload, TerrainPatchState& terrain_patch) {
  const auto message_type = ExtractStringField(payload, "message_type");
  if (!message_type.has_value() || *message_type != "terrain_patch") {
    return false;
  }

  TerrainPatchState next = terrain_patch;
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

  next.valid = next.rows > 0 && next.cols > 0 && next.cell_size_m > 0.0f;
  terrain_patch = std::move(next);
  return terrain_patch.valid;
}

bool ParsePacket(const std::string& payload,
                 std::map<std::uint32_t, EntityState>& entities,
                 TerrainPatchState& terrain_patch) {
  const auto message_type = ExtractStringField(payload, "message_type");
  if (!message_type.has_value()) {
    return false;
  }

  if (*message_type == "scene_clear") {
    entities.clear();
    terrain_patch = {};
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

  int Pump(std::map<std::uint32_t, EntityState>& entities, TerrainPatchState& terrain_patch) {
    if (!valid_) {
      return 0;
    }

    int packets = 0;
    for (;;) {
      std::array<char, 4096> buffer{};
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
      if (ParsePacket(buffer.data(), entities, terrain_patch)) {
        ++packets;
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
      options.udp_port = std::atoi(argv[++i]);
      if (options.udp_port <= 0 || options.udp_port > 65535) {
        std::cerr << "Invalid UDP port: " << options.udp_port << "\n";
        std::exit(1);
      }
      continue;
    }

    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: hexapod-opengl-visualiser [--udp-port <port>]\n";
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
               float time_s) {
  glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  SceneBounds bounds = ComputeSceneBounds(entities);
  ExpandTerrainPatchBounds(bounds, terrain_patch);
  const Vec3 center = bounds.valid
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
  const float camera_distance = std::max(2.0f, scene_radius * 6.0f);
  glTranslatef(0.0f, -0.35f * scene_radius, -camera_distance);
  glRotatef(18.0f, 1.0f, 0.0f, 0.0f);
  glRotatef(28.0f + time_s * 8.0f, 0.0f, 1.0f, 0.0f);
  glTranslatef(-center.x, -center.y, -center.z);

  DrawTerrainPatch(terrain_patch);

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

    glPushMatrix();
    glTranslatef(entity.position.x, entity.position.y, entity.position.z);
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

#ifndef _WIN32
  UdpReceiver receiver(options.udp_port);
  if (!receiver.valid()) {
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }
#else
  std::cerr << "UDP receiver is not implemented on Windows in this build\n";
  glfwDestroyWindow(window);
  glfwTerminate();
  return 1;
#endif

  std::map<std::uint32_t, EntityState> entities;
  TerrainPatchState terrain_patch;
  double last_title_update_s = -1.0;

  while (!glfwWindowShouldClose(window)) {
    receiver.Pump(entities, terrain_patch);

    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
    ConfigureProjection(framebuffer_width, framebuffer_height);

    const float time_s = static_cast<float>(glfwGetTime());
    DrawScene(entities, terrain_patch, time_s);

    if (last_title_update_s < 0.0 || glfwGetTime() - last_title_update_s > 0.25) {
      std::ostringstream title;
      title << "Hexapod OpenGL Visualiser | UDP " << options.udp_port << " | entities " << entities.size();
      glfwSetWindowTitle(window, title.str().c_str());
      last_title_update_s = glfwGetTime();
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
