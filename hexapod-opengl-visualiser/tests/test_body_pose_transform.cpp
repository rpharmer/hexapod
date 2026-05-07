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
  pose.orientation_rad = {0.0f, 0.0f, 0.0f};

  pose.yaw_rad = 0.0f;
  pose.orientation_rad = {0.0f, 0.0f, 0.0f};
  const Vec3 forward_at_zero = TransformBodyPoint({1.0f, 0.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(forward_at_zero.x, 0.0f) &&
                        nearlyEqual(forward_at_zero.y, 0.0f) &&
                        nearlyEqual(forward_at_zero.z, -1.0f),
                    "server +X should land on scene -Z at zero yaw");

  pose.yaw_rad = 0.5f * visualiser::math::kPi;
  pose.orientation_rad = {0.0f, 0.0f, pose.yaw_rad};
  const Vec3 forward_at_ninety = TransformBodyPoint({1.0f, 0.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(forward_at_ninety.x, 1.0f) &&
                        nearlyEqual(forward_at_ninety.y, 0.0f) &&
                        nearlyEqual(forward_at_ninety.z, 0.0f),
                    "server +X should stay on scene +X at +90 degrees yaw");

  pose.orientation_rad = {0.5f * visualiser::math::kPi, 0.0f, 0.0f};
  pose.yaw_rad = 0.0f;
  const Vec3 roll_rotated = TransformBodyPoint({0.0f, 1.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(roll_rotated.x, 0.0f) &&
                        nearlyEqual(roll_rotated.y, 1.0f) &&
                        nearlyEqual(roll_rotated.z, 0.0f),
                    "roll should lift server +Y into scene +Y after full pose transform");

  pose.orientation_rad = {0.0f, 0.5f * visualiser::math::kPi, 0.0f};
  pose.position = {2.0f, 3.0f, 4.0f};
  const Vec3 pitch_rotated = TransformBodyPoint({1.0f, 0.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(pitch_rotated.x, 2.0f) &&
                        nearlyEqual(pitch_rotated.y, 3.0f) &&
                        nearlyEqual(pitch_rotated.z, -3.0f),
                    "pitch should rotate server +X into scene -Y before translation");

  pose.orientation_rad = {0.0f, 0.0f, 0.0f};
  pose.yaw_rad = 0.0f;
  const Vec3 origin_translated = TransformBodyPoint({0.0f, 0.0f, 0.0f}, pose);
  ok = ok && expect(nearlyEqual(origin_translated.x, 2.0f) &&
                        nearlyEqual(origin_translated.y, 4.0f) &&
                        nearlyEqual(origin_translated.z, -3.0f),
                    "server origin should map into the scene basis before translation");

  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
