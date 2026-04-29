#include "visualiser/math/frame.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    return false;
  }
  return true;
}

bool nearlyEqual(float a, float b, float eps = 1e-6f) {
  return std::abs(a - b) <= eps;
}

bool test_server_yaw_maps_to_scene_yaw() {
  constexpr float half_pi = visualiser::math::frame_math::kSceneYawOffsetRad;
  return expect(nearlyEqual(visualiser::math::frame_math::ServerYawToSceneYaw(0.0f), -half_pi),
                "server yaw 0 should map to scene yaw -90 degrees") &&
         expect(nearlyEqual(visualiser::math::frame_math::ServerYawToSceneYaw(half_pi), 0.0f),
                "server yaw +90 degrees should map to scene yaw 0") &&
         expect(nearlyEqual(visualiser::math::frame_math::ServerYawToSceneYaw(3.0f * half_pi), 2.0f * half_pi),
                "server yaw +270 degrees should map consistently in the scene basis");
}

}  // namespace

int main() {
  if (!test_server_yaw_maps_to_scene_yaw()) {
    return EXIT_FAILURE;
  }
  std::cout << "test_visualiser_frame_math ok\n";
  return EXIT_SUCCESS;
}
