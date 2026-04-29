#include "visualiser/parsing/json_packets.hpp"

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
}  // namespace

int main() {
  visualiser::robot::HexapodTelemetryState telemetry{};
  const std::string geometry = R"({"type":"geometry","coxa_mm":36,"femur_mm":71,"tibia_mm":111,"body_radius_mm":61})";
  const std::string joints = R"({"type":"joints","joints":{"LF":{"angles_deg":[1,2,3]}}})";

  bool ok = true;
  ok = ok && expect(visualiser::parsing::ParseHexapodTelemetryPacket(geometry, telemetry), "geometry parsed");
  ok = ok && expect(telemetry.has_geometry, "geometry state");
  ok = ok && expect(visualiser::parsing::ParseHexapodTelemetryPacket(joints, telemetry), "joints parsed");
  ok = ok && expect(telemetry.has_joints, "joints state");
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
