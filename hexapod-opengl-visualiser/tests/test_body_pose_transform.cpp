#include "visualiser/robot/kinematics.hpp"
#include "visualiser/math/geometry.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
  }
  return condition;
}

bool nearlyEqual(float a, float b, float eps = 1e-5f) {
  return std::abs(a - b) <= eps;
}

}  // namespace

int main() {
  using visualiser::math::Vec3;
  using visualiser::robot::HexapodBodyPoseState;
  using visualiser::robot::TransformBodyPoint;

  bool ok = true;

  HexapodBodyPoseState pose{};
  pose.valid = true;
  pose.position = {0.0f, 0.0f, 0.0f};

  pose.yaw_rad = 0.0f;
  const Vec3 forward_at_zero = TransformBodyPoint({1.0f, 0.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(forward_at_zero.x, 0.0f) &&
                        nearlyEqual(forward_at_zero.y, 0.0f) &&
                        nearlyEqual(forward_at_zero.z, -1.0f),
                    "server +X should land on scene -Z at zero yaw");

  pose.yaw_rad = 0.5f * visualiser::math::kPi;
  const Vec3 forward_at_ninety = TransformBodyPoint({1.0f, 0.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(forward_at_ninety.x, 1.0f) &&
                        nearlyEqual(forward_at_ninety.y, 0.0f) &&
                        nearlyEqual(forward_at_ninety.z, 0.0f),
                    "server +X should stay on scene +X at +90 degrees yaw");

  pose.yaw_rad = 0.0f;
  pose.position = {2.0f, 3.0f, 4.0f};
  const Vec3 origin_translated = TransformBodyPoint({0.0f, 0.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(origin_translated.x, 2.0f) &&
                        nearlyEqual(origin_translated.y, 4.0f) &&
                        nearlyEqual(origin_translated.z, -3.0f),
                    "server origin should map into the scene basis before translation");

  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
