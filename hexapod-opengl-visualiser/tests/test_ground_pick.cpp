#include "visualiser/render/camera.hpp"
#include "visualiser/scene/ground_pick.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
  using visualiser::math::Vec3;
  using visualiser::render::Mat4;
  const Mat4 projection = visualiser::render::ProjectionFromLegacyFrustum(1280, 720);
  const Mat4 view = visualiser::render::LegacyViewMatrix(
      0.5f, 2.0f, 18.0f, 28.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  const Mat4 view_projection = Mat4::Mul(projection, view);
  const Mat4 inverse = Mat4::Inverse(view_projection);
  const Vec3 expected{0.2f, 0.03f, -0.25f};
  const auto ndc = view_projection.TransformPoint({expected.x, expected.y, expected.z});

  // The framebuffer is 2x the logical window, as on a high-DPI display.
  const float cursor_x = (ndc.x + 1.0f) * 0.5f * 640.0f;
  const float cursor_y = (1.0f - ndc.y) * 0.5f * 360.0f;
  Vec3 picked{};
  const bool ok = visualiser::scene::PickHorizontalGround(
      inverse, cursor_x, cursor_y, 640, 360, expected.y, picked);
  if (!ok || std::abs(picked.x - expected.x) > 1e-3f ||
      std::abs(picked.y - expected.y) > 1e-5f ||
      std::abs(picked.z - expected.z) > 1e-3f) {
    std::cerr << "logical-window pick missed the projected ground point\n";
    return EXIT_FAILURE;
  }
  if (visualiser::scene::PickHorizontalGround(
          inverse, -1.0f, cursor_y, 640, 360, expected.y, picked)) {
    std::cerr << "out-of-window click should not produce a goal\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
