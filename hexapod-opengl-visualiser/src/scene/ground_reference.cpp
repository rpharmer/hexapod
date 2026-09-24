#include "visualiser/scene/ground_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace visualiser::scene {

std::vector<GroundGridSegment> BuildGroundReferenceGrid(float center_x,
                                                         float center_z,
                                                         float height,
                                                         float half_span,
                                                         float spacing,
                                                         GroundGridExclusion exclude) {
  std::vector<GroundGridSegment> lines;
  if (!std::isfinite(center_x) || !std::isfinite(center_z) || !std::isfinite(height) ||
      !std::isfinite(half_span) || !std::isfinite(spacing) ||
      half_span <= 0.0f || spacing <= 0.0f || half_span / spacing > 128.0f) {
    return lines;
  }
  const double first_x_cell = std::floor((static_cast<double>(center_x) - half_span) / spacing);
  const double last_x_cell = std::ceil((static_cast<double>(center_x) + half_span) / spacing);
  const double first_z_cell = std::floor((static_cast<double>(center_z) - half_span) / spacing);
  const double last_z_cell = std::ceil((static_cast<double>(center_z) + half_span) / spacing);
  if (first_x_cell < std::numeric_limits<int>::min() ||
      last_x_cell >= std::numeric_limits<int>::max() ||
      first_z_cell < std::numeric_limits<int>::min() ||
      last_z_cell >= std::numeric_limits<int>::max()) {
    return lines;
  }
  const int first_x = static_cast<int>(first_x_cell);
  const int last_x = static_cast<int>(last_x_cell);
  const int first_z = static_cast<int>(first_z_cell);
  const int last_z = static_cast<int>(last_z_cell);
  const float min_x = first_x * spacing;
  const float max_x = last_x * spacing;
  const float min_z = first_z * spacing;
  const float max_z = last_z * spacing;
  const auto vertical = [&](float x, float z0, float z1) {
    if (z1 - z0 > 1e-5f) lines.push_back({{x, height, z0}, {x, height, z1}});
  };
  const auto horizontal = [&](float z, float x0, float x1) {
    if (x1 - x0 > 1e-5f) lines.push_back({{x0, height, z}, {x1, height, z}});
  };
  for (int i = first_x; i <= last_x; ++i) {
    const float x = i * spacing;
    if (exclude.valid && x >= exclude.min_x && x <= exclude.max_x) {
      vertical(x, min_z, std::clamp(exclude.min_z, min_z, max_z));
      vertical(x, std::clamp(exclude.max_z, min_z, max_z), max_z);
    } else {
      vertical(x, min_z, max_z);
    }
  }
  for (int i = first_z; i <= last_z; ++i) {
    const float z = i * spacing;
    if (exclude.valid && z >= exclude.min_z && z <= exclude.max_z) {
      horizontal(z, min_x, std::clamp(exclude.min_x, min_x, max_x));
      horizontal(z, std::clamp(exclude.max_x, min_x, max_x), max_x);
    } else {
      horizontal(z, min_x, max_x);
    }
  }
  return lines;
}

}  // namespace visualiser::scene
