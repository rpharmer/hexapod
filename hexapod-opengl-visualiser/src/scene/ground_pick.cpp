#include "visualiser/scene/ground_pick.hpp"

#include <cmath>

namespace visualiser::scene {

bool PickHorizontalGround(const visualiser::render::Mat4& inverse_view_projection,
                          float cursor_x,
                          float cursor_y,
                          int window_width,
                          int window_height,
                          float ground_height,
                          visualiser::math::Vec3& hit) {
  if (window_width <= 0 || window_height <= 0 || !std::isfinite(cursor_x) ||
      !std::isfinite(cursor_y) || !std::isfinite(ground_height) ||
      cursor_x < 0.0f || cursor_y < 0.0f ||
      cursor_x >= static_cast<float>(window_width) ||
      cursor_y >= static_cast<float>(window_height)) {
    return false;
  }
  const float ndc_x = 2.0f * cursor_x / static_cast<float>(window_width) - 1.0f;
  const float ndc_y = 1.0f - 2.0f * cursor_y / static_cast<float>(window_height);
  const auto near_point = inverse_view_projection.TransformPoint({ndc_x, ndc_y, -1.0f});
  const auto far_point = inverse_view_projection.TransformPoint({ndc_x, ndc_y, 1.0f});
  const float dx = far_point.x - near_point.x;
  const float dy = far_point.y - near_point.y;
  const float dz = far_point.z - near_point.z;
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
      std::fabs(dy) < 1e-6f) {
    return false;
  }
  const float t = (ground_height - near_point.y) / dy;
  if (!std::isfinite(t) || t < 0.0f || t > 1.0f) {
    return false;
  }
  hit = {near_point.x + t * dx, ground_height, near_point.z + t * dz};
  return std::isfinite(hit.x) && std::isfinite(hit.z);
}

}  // namespace visualiser::scene
