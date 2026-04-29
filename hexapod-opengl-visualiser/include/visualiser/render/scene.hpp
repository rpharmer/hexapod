#pragma once

#include <map>

#include "visualiser/robot/geometry_state.hpp"
#include "visualiser/scene/entity_state.hpp"
#include "visualiser/scene/terrain_state.hpp"
#include "visualiser/ui/overlay_state.hpp"

namespace visualiser::render {

void DrawScene(const std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
               const visualiser::scene::TerrainPatchState& terrain_patch,
               const visualiser::robot::HexapodTelemetryState& telemetry,
               const visualiser::ui::AppUiState& ui,
               const visualiser::ui::CameraState& camera,
               float time_s);

}  // namespace visualiser::render
