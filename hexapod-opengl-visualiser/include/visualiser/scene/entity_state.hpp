#pragma once

#include <cstdint>
#include <vector>

#include "visualiser/math/vec3.hpp"
#include "visualiser/scene/shape_type.hpp"

namespace visualiser::scene {

struct CompoundChildState {
  ShapeType shape = ShapeType::kUnknown;
  float radius = 0.5f;
  float half_height = 0.5f;
  visualiser::math::Vec3 half_extents{0.5f, 0.5f, 0.5f};
  visualiser::math::Vec3 local_position{};
  visualiser::math::Quat local_rotation{};
};

struct EntityState {
  std::uint32_t id = 0;
  ShapeType shape = ShapeType::kUnknown;
  float radius = 0.5f;
  float half_height = 0.5f;
  visualiser::math::Vec3 half_extents{0.5f, 0.5f, 0.5f};
  visualiser::math::Vec3 plane_normal{0.0f, 1.0f, 0.0f};
  float plane_offset = 0.0f;
  std::vector<CompoundChildState> compound_children{};
  visualiser::math::Vec3 position{};
  visualiser::math::Quat rotation{};
  bool has_static = false;
  bool has_frame = false;
};

}  // namespace visualiser::scene
