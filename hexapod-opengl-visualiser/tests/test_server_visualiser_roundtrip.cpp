#include "visualiser/math/geometry.hpp"
#include "visualiser/robot/defaults.hpp"
#include "visualiser/robot/kinematics.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

using visualiser::math::Vec3;
using visualiser::robot::HexapodBodyPoseState;
using visualiser::robot::HexapodLegLayout;
using visualiser::robot::RobotKinematics;

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
  }
  return condition;
}

bool nearlyEqual(float a, float b, float eps = 1e-5f) {
  return std::abs(a - b) <= eps;
}

Vec3 SceneToServerVec(const Vec3& value) {
  return Vec3{value.x, -value.z, value.y};
}

Vec3 SceneWorldToServerBody(const Vec3& scene_point, const HexapodBodyPoseState& pose) {
  const Vec3 origin = visualiser::robot::ServerToSceneVec(pose.position);
  const Vec3 relative{
      scene_point.x - origin.x,
      scene_point.y - origin.y,
      scene_point.z - origin.z,
  };
  const Vec3 body_scene = visualiser::math::RotateAroundSceneY(
      relative,
      pose.yaw_rad - (0.5f * visualiser::math::kPi));
  return SceneToServerVec(body_scene);
}

struct SamplePose {
  HexapodBodyPoseState pose{};
  std::array<std::array<float, 3>, 6> angles_deg{};
};

bool almostEqual(const Vec3& a, const Vec3& b, float eps = 1e-5f) {
  return nearlyEqual(a.x, b.x, eps) && nearlyEqual(a.y, b.y, eps) && nearlyEqual(a.z, b.z, eps);
}

}  // namespace

int main() {
  const auto geometry = visualiser::robot::MakeDefaultGeometryState();

  bool ok = true;

  const std::array<SamplePose, 3> samples{{
      {HexapodBodyPoseState{true, {0.0f, 0.0f, 0.0f}, 0.0f},
       {{{0.0f, 0.0f, 0.0f},
         {12.0f, -7.0f, 21.0f},
         {-8.0f, 10.0f, -16.0f},
         {6.0f, -14.0f, 11.0f},
         {-11.0f, 9.0f, -13.0f},
         {4.0f, 5.0f, -9.0f}}}},
      {HexapodBodyPoseState{true, {0.35f, -0.20f, 0.75f}, 0.75f},
       {{{1.0f, 2.0f, 3.0f},
         {14.0f, -5.0f, 17.0f},
         {-10.0f, 8.0f, -18.0f},
         {7.0f, -12.0f, 15.0f},
         {-9.0f, 6.0f, -11.0f},
         {3.0f, 4.0f, -7.0f}}}},
      {HexapodBodyPoseState{true, {-0.55f, 0.15f, -0.30f}, -1.0f},
       {{{-2.0f, 1.0f, -4.0f},
         {9.0f, -11.0f, 19.0f},
         {-7.0f, 13.0f, -15.0f},
         {5.0f, -8.0f, 12.0f},
         {-13.0f, 7.0f, -10.0f},
         {8.0f, 3.0f, -6.0f}}}},
  }};

  for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
    const SamplePose& sample = samples[sample_index];

    for (std::size_t leg = 0; leg < geometry.legs.size(); ++leg) {
      const HexapodLegLayout& layout = geometry.legs[leg];
      const std::array<float, 3>& angles = sample.angles_deg[leg];

      const RobotKinematics body = visualiser::robot::ComputeRobotLeg(layout, angles);
      const RobotKinematics scene = visualiser::robot::ComputeRobotLegScene(layout, angles, sample.pose);

      const Vec3 expected_anchor = visualiser::robot::TransformBodyPoint(body.coxa, sample.pose);
      const Vec3 expected_shoulder = visualiser::robot::TransformBodyPoint(body.femur, sample.pose);
      const Vec3 expected_knee = visualiser::robot::TransformBodyPoint(body.tibia, sample.pose);
      const Vec3 expected_foot = visualiser::robot::TransformBodyPoint(body.foot, sample.pose);

      ok = expect(almostEqual(scene.coxa, expected_anchor),
                  "visualiser scene anchor should match the server round-trip") &&
           ok;
      ok = expect(almostEqual(scene.femur, expected_shoulder),
                  "visualiser scene femur joint should match the server round-trip") &&
           ok;
      ok = expect(almostEqual(scene.tibia, expected_knee),
                  "visualiser scene tibia joint should match the server round-trip") &&
           ok;
      ok = expect(almostEqual(scene.foot, expected_foot),
                  "visualiser scene foot should match the server round-trip") &&
           ok;

      const Vec3 recovered_anchor = SceneWorldToServerBody(scene.coxa, sample.pose);
      const Vec3 recovered_shoulder = SceneWorldToServerBody(scene.femur, sample.pose);
      const Vec3 recovered_knee = SceneWorldToServerBody(scene.tibia, sample.pose);
      const Vec3 recovered_foot = SceneWorldToServerBody(scene.foot, sample.pose);

      ok = expect(almostEqual(recovered_anchor, body.coxa),
                  "server/body anchor should round-trip through the scene transform") &&
           ok;
      ok = expect(almostEqual(recovered_shoulder, body.femur),
                  "server/body shoulder should round-trip through the scene transform") &&
           ok;
      ok = expect(almostEqual(recovered_knee, body.tibia),
                  "server/body knee should round-trip through the scene transform") &&
           ok;
      ok = expect(almostEqual(recovered_foot, body.foot),
                  "server/body foot should round-trip through the scene transform") &&
           ok;
    }
  }

  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
