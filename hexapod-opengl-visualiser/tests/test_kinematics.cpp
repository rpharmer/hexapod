#include "visualiser/robot/defaults.hpp"
#include "visualiser/robot/kinematics.hpp"

#include <cstdlib>
#include <iostream>

int main() {
  auto geometry = visualiser::robot::MakeDefaultGeometryState();
  std::array<std::array<float, 3>, 6> angles{};
  visualiser::robot::HexapodBodyPoseState pose{};
  const auto bounds = visualiser::robot::ComputeRobotBounds(geometry, angles, pose);
  if (!bounds.valid) {
    std::cerr << "FAIL: expected valid robot bounds\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
