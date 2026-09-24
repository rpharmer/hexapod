#pragma once

#include "visualiser/math/vec3.hpp"

#include <vector>

namespace visualiser::scene {

struct GroundGridSegment {
  visualiser::math::Vec3 a{};
  visualiser::math::Vec3 b{};
};

struct GroundGridExclusion {
  bool valid = false;
  float min_x = 0.0f;
  float max_x = 0.0f;
  float min_z = 0.0f;
  float max_z = 0.0f;
};

// Visual reference only: world-fixed 10 cm cells in a moving window around
// the chassis. The terrain patch, when present, supplies its own measured grid.
std::vector<GroundGridSegment> BuildGroundReferenceGrid(float center_x,
                                                         float center_z,
                                                         float height,
                                                         float half_span,
                                                         float spacing,
                                                         GroundGridExclusion exclude = {});

}  // namespace visualiser::scene
