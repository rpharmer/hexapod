#pragma once

#include <map>

#include "visualiser/math/vec3.hpp"
#include "visualiser/scene/entity_state.hpp"
#include "visualiser/scene/terrain_state.hpp"

namespace visualiser::scene {

struct SceneBounds {
  visualiser::math::Vec3 min{};
  visualiser::math::Vec3 max{};
  bool valid = false;
};

void ExpandBounds(SceneBounds& bounds, const visualiser::math::Vec3& point);
visualiser::math::Vec3 PrimitiveBoundsHalfExtents(
    ShapeType shape,
    float radius,
    float half_height,
    const visualiser::math::Vec3& half_extents);
void ExpandPrimitiveBounds(
    SceneBounds& bounds,
    ShapeType shape,
    const visualiser::math::Vec3& position,
    const visualiser::math::Quat& rotation,
    float radius,
    float half_height,
    const visualiser::math::Vec3& half_extents);
SceneBounds ComputeSceneBounds(const std::map<std::uint32_t, EntityState>& entities);
void ExpandTerrainPatchBounds(SceneBounds& bounds, const TerrainPatchState& terrain);

}  // namespace visualiser::scene
