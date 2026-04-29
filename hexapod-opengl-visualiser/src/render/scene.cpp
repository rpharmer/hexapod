#include "visualiser/render/scene.hpp"

#include <GLFW/glfw3.h>

namespace visualiser::render {

void DrawScene(const std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
               const visualiser::scene::TerrainPatchState& terrain_patch,
               const visualiser::robot::HexapodTelemetryState& telemetry,
               const visualiser::ui::AppUiState& ui,
               const visualiser::ui::CameraState& camera,
               float time_s) {
  (void)entities;
  (void)terrain_patch;
  (void)telemetry;
  (void)ui;
  (void)camera;
  (void)time_s;
  glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

}  // namespace visualiser::render
