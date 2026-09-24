#include "visualiser/scene/scene_bounds.hpp"
#include "visualiser/scene/visibility.hpp"

#include <cstdlib>
#include <iostream>
#include <map>

int main() {
  std::map<std::uint32_t, visualiser::scene::EntityState> entities;
  visualiser::scene::EntityState entity{};
  entity.id = 1;
  entity.shape = visualiser::scene::ShapeType::kBox;
  entity.has_frame = true;
  entities.emplace(1, entity);
  if (visualiser::scene::HasDrawableMeasuredGeometry(entities)) {
    std::cerr << "FAIL: a pose without a shape must not hide the command robot\n";
    return EXIT_FAILURE;
  }
  entity.has_static = true;
  entities[1] = entity;
  if (!visualiser::scene::HasDrawableMeasuredGeometry(entities)) {
    std::cerr << "FAIL: a shaped pose should provide measured geometry\n";
    return EXIT_FAILURE;
  }
  entity.shape = visualiser::scene::ShapeType::kPlane;
  entities[1] = entity;
  if (visualiser::scene::HasDrawableMeasuredGeometry(entities)) {
    std::cerr << "FAIL: the ground plane is not robot geometry\n";
    return EXIT_FAILURE;
  }
  entity.shape = visualiser::scene::ShapeType::kBox;
  entities[1] = entity;
  entity.half_extents = {1.0f, 1.0f, 1.0f};
  entities[1] = entity;
  auto bounds = visualiser::scene::ComputeSceneBounds(entities);
  if (!bounds.valid) {
    std::cerr << "FAIL: expected valid scene bounds\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
