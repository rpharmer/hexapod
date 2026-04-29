#include "visualiser/scene/scene_bounds.hpp"

#include <cstdlib>
#include <iostream>
#include <map>

int main() {
  std::map<std::uint32_t, visualiser::scene::EntityState> entities;
  visualiser::scene::EntityState entity{};
  entity.id = 1;
  entity.shape = visualiser::scene::ShapeType::kBox;
  entity.has_static = true;
  entity.has_frame = true;
  entity.half_extents = {1.0f, 1.0f, 1.0f};
  entities.emplace(1, entity);
  auto bounds = visualiser::scene::ComputeSceneBounds(entities);
  if (!bounds.valid) {
    std::cerr << "FAIL: expected valid scene bounds\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
