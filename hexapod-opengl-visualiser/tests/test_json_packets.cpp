#include "visualiser/parsing/json_packets.hpp"

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

bool nearlyEqual(float a, float b, float eps = 1e-5f) {
  return std::abs(a - b) <= eps;
}
}  // namespace

int main() {
  visualiser::robot::HexapodTelemetryState telemetry{};
  const std::string geometry = R"({"type":"geometry","coxa_mm":36,"femur_mm":71,"tibia_mm":111,"body_radius_mm":61})";
  const std::string joints_with_orientation =
      R"({"type":"joints","joints":{"LF":{"angles_deg":[1,2,3]}},"body_position":[1,2,3],"body_orientation_rad":[0.1,0.2,0.3]})";
  const std::string joints_with_yaw = R"({"type":"joints","joints":{"LF":{"angles_deg":[4,5,6]}},"body_position":[7,8,9],"body_yaw_rad":1.25})";

  bool ok = true;
  ok = ok && expect(visualiser::parsing::ParseHexapodTelemetryPacket(geometry, telemetry), "geometry parsed");
  ok = ok && expect(telemetry.has_geometry, "geometry state");
  ok = ok && expect(visualiser::parsing::ParseHexapodTelemetryPacket(joints_with_orientation, telemetry),
                    "joints with orientation parsed");
  ok = ok && expect(telemetry.has_joints, "joints state");
  ok = ok && expect(telemetry.body_pose.valid, "body pose state");
  ok = ok && expect(nearlyEqual(telemetry.body_pose.position.x, 1.0f) &&
                        nearlyEqual(telemetry.body_pose.position.y, 2.0f) &&
                        nearlyEqual(telemetry.body_pose.position.z, 3.0f),
                    "body position parsed");
  ok = ok && expect(nearlyEqual(telemetry.body_pose.orientation_rad.x, 0.1f) &&
                        nearlyEqual(telemetry.body_pose.orientation_rad.y, 0.2f) &&
                        nearlyEqual(telemetry.body_pose.orientation_rad.z, 0.3f) &&
                        nearlyEqual(telemetry.body_pose.yaw_rad, 0.3f),
                    "body orientation parsed");

  visualiser::robot::HexapodTelemetryState fallback{};
  ok = ok && expect(visualiser::parsing::ParseHexapodTelemetryPacket(joints_with_yaw, fallback),
                    "joints with yaw fallback parsed");
  ok = ok && expect(fallback.body_pose.valid, "fallback body pose state");
  ok = ok && expect(nearlyEqual(fallback.body_pose.orientation_rad.x, 0.0f) &&
                        nearlyEqual(fallback.body_pose.orientation_rad.y, 0.0f) &&
                        nearlyEqual(fallback.body_pose.orientation_rad.z, 1.25f) &&
                        nearlyEqual(fallback.body_pose.yaw_rad, 1.25f),
                    "body yaw fallback parsed");
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
