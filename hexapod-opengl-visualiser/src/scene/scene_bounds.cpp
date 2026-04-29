#include "visualiser/scene/scene_bounds.hpp"

#include <algorithm>
#include <cmath>

#include "visualiser/math/geometry.hpp"

namespace visualiser::scene {

using visualiser::math::Quat;
using visualiser::math::RotateVector;
using visualiser::math::Vec3;

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
        const Quat child_rotation = visualiser::math::MultiplyQuat(entity.rotation, child.local_rotation);
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

}  // namespace visualiser::scene
