#pragma once

#include <vector>

#include "visualiser/math/vec3.hpp"

namespace visualiser::scene {

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
  visualiser::math::Vec3 center{};
  bool has_grid_origin_xz = false;
  float grid_origin_x = 0.0f;
  float grid_origin_z = 0.0f;
  float base_height_m = 0.0f;
  float plane_height_m = 0.0f;
  visualiser::math::Vec3 plane_normal{0.0f, 1.0f, 0.0f};
  std::vector<float> heights{};
  std::vector<float> confidences{};
  std::vector<float> collision_heights{};
};

}  // namespace visualiser::scene
