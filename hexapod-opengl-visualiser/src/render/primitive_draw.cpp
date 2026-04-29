#include "visualiser/render/primitive_draw.hpp"

#include <array>
#include <cmath>

namespace visualiser::render {

namespace {

Vec3 T(const Mat4& m, const Vec3& p) { return m.TransformPoint(p); }

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

}  // namespace

void AppendWireBox(LineRenderer& lr, const Mat4& model, const Vec3& half_extents, const Vec3& color) {
  const Mat4 m = Mat4::Mul(model, Mat4::Scale(half_extents.x, half_extents.y, half_extents.z));
  const std::array<Vec3, 8> vertices = {{
      {-1.0f, -1.0f, -1.0f},
      {1.0f, -1.0f, -1.0f},
      {1.0f, 1.0f, -1.0f},
      {-1.0f, 1.0f, -1.0f},
      {-1.0f, -1.0f, 1.0f},
      {1.0f, -1.0f, 1.0f},
      {1.0f, 1.0f, 1.0f},
      {-1.0f, 1.0f, 1.0f},
  }};
  const std::array<std::array<int, 2>, 12> edges = {{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}},
  }};
  for (const auto& edge : edges) {
    lr.AddSegment(T(m, vertices[edge[0]]), T(m, vertices[edge[1]]), color);
  }
}

void AppendWireSphere(LineRenderer& lr, const Mat4& model, float radius, const Vec3& color) {
  const Mat4 m = Mat4::Mul(model, Mat4::Scale(radius, radius, radius));
  constexpr int slices = 18;
  constexpr int stacks = 10;

  for (int stack = 1; stack < stacks; ++stack) {
    const float phi = -0.5f * kPi + kPi * static_cast<float>(stack) / static_cast<float>(stacks);
    Vec3 first{};
    Vec3 prev{};
    for (int slice = 0; slice < slices; ++slice) {
      const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
      const float x = std::cos(phi) * std::cos(theta);
      const float y = std::sin(phi);
      const float z = std::cos(phi) * std::sin(theta);
      const Vec3 cur{x, y, z};
      if (slice == 0) {
        first = cur;
        prev = cur;
      } else {
        lr.AddSegment(T(m, prev), T(m, cur), color);
        prev = cur;
      }
    }
    lr.AddSegment(T(m, prev), T(m, first), color);
  }

  for (int slice = 0; slice < slices; ++slice) {
    const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
    Vec3 prev{};
    for (int stack = 0; stack <= stacks; ++stack) {
      const float phi = -0.5f * kPi + kPi * static_cast<float>(stack) / static_cast<float>(stacks);
      const float x = std::cos(phi) * std::cos(theta);
      const float y = std::sin(phi);
      const float z = std::cos(phi) * std::sin(theta);
      const Vec3 cur{x, y, z};
      if (stack > 0) {
        lr.AddSegment(T(m, prev), T(m, cur), color);
      }
      prev = cur;
    }
  }
}

void AppendWireCapsule(LineRenderer& lr, const Mat4& model, float radius, float half_height, const Vec3& color) {
  for (int i = 0; i < 4; ++i) {
    const float angle = static_cast<float>(i) * 0.5f * kPi;
    const float x = std::cos(angle) * radius;
    const float z = std::sin(angle) * radius;
    lr.AddSegment(T(model, {x, -half_height, z}), T(model, {x, half_height, z}), color);
  }
  const Mat4 top = Mat4::Mul(model, Mat4::Translate(0.0f, half_height, 0.0f));
  const Mat4 bot = Mat4::Mul(model, Mat4::Translate(0.0f, -half_height, 0.0f));
  AppendWireSphere(lr, top, radius, color);
  AppendWireSphere(lr, bot, radius, color);
}

void AppendWireCylinder(LineRenderer& lr, const Mat4& model, float radius, float half_height, const Vec3& color) {
  constexpr int slices = 24;
  Vec3 first_top{};
  Vec3 first_bot{};
  Vec3 prev_top{};
  Vec3 prev_bot{};
  for (int slice = 0; slice <= slices; ++slice) {
    const float theta = 2.0f * kPi * static_cast<float>(slice % slices) / static_cast<float>(slices);
    const float x = radius * std::cos(theta);
    const float z = radius * std::sin(theta);
    const Vec3 top{x, half_height, z};
    const Vec3 bot{x, -half_height, z};
    if (slice == 0) {
      first_top = top;
      first_bot = bot;
      prev_top = top;
      prev_bot = bot;
    } else {
      lr.AddSegment(T(model, prev_top), T(model, top), color);
      lr.AddSegment(T(model, prev_bot), T(model, bot), color);
      prev_top = top;
      prev_bot = bot;
    }
  }
  lr.AddSegment(T(model, prev_top), T(model, first_top), color);
  lr.AddSegment(T(model, prev_bot), T(model, first_bot), color);

  for (int i = 0; i < 4; ++i) {
    const float theta = 0.5f * kPi * static_cast<float>(i);
    const float x = radius * std::cos(theta);
    const float z = radius * std::sin(theta);
    lr.AddSegment(T(model, {x, -half_height, z}), T(model, {x, half_height, z}), color);
  }
}

void AppendWireHalfCylinder(LineRenderer& lr, const Mat4& model, float radius, float half_height, const Vec3& color) {
  constexpr int slices = 16;
  Vec3 first_hi{};
  Vec3 first_lo{};
  Vec3 prev_hi{};
  Vec3 prev_lo{};
  for (int slice = 0; slice <= slices; ++slice) {
    const float theta = kPi * static_cast<float>(slice) / static_cast<float>(slices);
    const float x = radius * std::cos(theta);
    const float z = radius * std::sin(theta);
    const Vec3 hi{x, half_height, z};
    const Vec3 lo{x, -half_height, z};
    if (slice == 0) {
      first_hi = hi;
      first_lo = lo;
      prev_hi = hi;
      prev_lo = lo;
    } else {
      lr.AddSegment(T(model, prev_hi), T(model, hi), color);
      lr.AddSegment(T(model, prev_lo), T(model, lo), color);
      prev_hi = hi;
      prev_lo = lo;
    }
  }
  lr.AddSegment(T(model, prev_hi), T(model, first_hi), color);
  lr.AddSegment(T(model, prev_lo), T(model, first_lo), color);

  for (int i = 0; i <= 2; ++i) {
    const float theta = 0.5f * kPi * static_cast<float>(i);
    const float x = radius * std::cos(theta);
    const float z = radius * std::sin(theta);
    lr.AddSegment(T(model, {x, -half_height, z}), T(model, {x, half_height, z}), color);
  }
  lr.AddSegment(T(model, {-radius, -half_height, 0.0f}), T(model, {radius, -half_height, 0.0f}), color);
  lr.AddSegment(T(model, {-radius, half_height, 0.0f}), T(model, {radius, half_height, 0.0f}), color);
  lr.AddSegment(T(model, {-radius, -half_height, 0.0f}), T(model, {-radius, half_height, 0.0f}), color);
  lr.AddSegment(T(model, {radius, -half_height, 0.0f}), T(model, {radius, half_height, 0.0f}), color);
}

void AppendPlane(MeshRenderer& mesh,
                 LineRenderer& lines,
                 const Mat4& model,
                 const Vec3& plane_normal,
                 float plane_offset,
                 const Vec3& fill_color,
                 const Vec3& grid_color) {
  const Vec3 up = Normalize(plane_normal);
  const Vec3 tangent_seed = std::fabs(up.y) > 0.9f ? Vec3{1.0f, 0.0f, 0.0f} : Vec3{0.0f, 1.0f, 0.0f};
  const Vec3 tangent = Normalize(Cross(tangent_seed, up));
  const Vec3 bitangent = Normalize(Cross(up, tangent));
  const Vec3 center{up.x * plane_offset, up.y * plane_offset, up.z * plane_offset};
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

  const Vec3 p0 = model.TransformPoint(corners[0]);
  const Vec3 p1 = model.TransformPoint(corners[1]);
  const Vec3 p2 = model.TransformPoint(corners[2]);
  const Vec3 p3 = model.TransformPoint(corners[3]);
  mesh.AddTriangle(p0, p1, p2, up, fill_color);
  mesh.AddTriangle(p0, p2, p3, up, fill_color);

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
    lines.AddSegment(model.TransformPoint(a), model.TransformPoint(b), grid_color);
    lines.AddSegment(model.TransformPoint(c), model.TransformPoint(d2), grid_color);
  }
}

}  // namespace visualiser::render
