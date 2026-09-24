#include "visualiser/scene/ground_reference.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
  const visualiser::scene::GroundGridExclusion patch{true, -0.3f, 0.3f, -0.3f, 0.3f};
  const auto first = visualiser::scene::BuildGroundReferenceGrid(0.0f, 0.0f, 0.02f, 1.0f, 0.1f, patch);
  const auto moved = visualiser::scene::BuildGroundReferenceGrid(0.65f, -0.45f, 0.02f, 1.0f, 0.1f, patch);
  if (first.empty() || moved.empty()) {
    std::cerr << "reference grid should extend with the chassis\n";
    return EXIT_FAILURE;
  }
  bool reaches_new_area = false;
  for (const auto& line : moved) {
    const float mid_x = 0.5f * (line.a.x + line.b.x);
    const float mid_z = 0.5f * (line.a.z + line.b.z);
    reaches_new_area |= line.a.x > 1.5f || line.b.x > 1.5f;
    if (std::abs(line.a.y - 0.02f) > 1e-6f ||
        std::abs(line.b.y - 0.02f) > 1e-6f ||
        (mid_x > patch.min_x && mid_x < patch.max_x &&
         mid_z > patch.min_z && mid_z < patch.max_z)) {
      std::cerr << "reference grid must stay on the plane and outside the measured patch\n";
      return EXIT_FAILURE;
    }
    const bool constant_x = std::abs(line.a.x - line.b.x) < 1e-6f;
    const float fixed_coordinate = constant_x ? line.a.x : line.a.z;
    if (std::abs(fixed_coordinate / 0.1f - std::round(fixed_coordinate / 0.1f)) > 1e-4f) {
      std::cerr << "reference cells must remain world-fixed while the window moves\n";
      return EXIT_FAILURE;
    }
  }
  if (!reaches_new_area) {
    std::cerr << "moving grid failed to cover new ground\n";
    return EXIT_FAILURE;
  }
  if (!visualiser::scene::BuildGroundReferenceGrid(1.0e30f, 0.0f, 0.0f, 1.0f, 0.1f, patch).empty()) {
    std::cerr << "unrepresentable grid cell coordinates must be rejected\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
