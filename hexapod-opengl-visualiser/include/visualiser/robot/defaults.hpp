#pragma once

#include <array>

#include "visualiser/robot/geometry_state.hpp"

namespace visualiser::robot {

constexpr std::array<const char*, 6> kLegKeys = {"LF", "LM", "LR", "RF", "RM", "RR"};

struct HexapodLegDefaults {
  float mount_angle_deg;
  float coxa_attach_deg;
  float femur_attach_deg;
  float tibia_attach_deg;
  std::array<float, 3> body_coxa_offset;
};

constexpr std::array<HexapodLegDefaults, 6> kDefaultLegs = {{
    {323.0f, 0.0f, -35.0f, -83.0f, {-0.063f, 0.0835f, -0.007f}},
    {270.0f, 0.0f, -35.0f, -83.0f, {-0.0815f, 0.0f, -0.007f}},
    {217.0f, 0.0f, -35.0f, -83.0f, {-0.063f, -0.0835f, -0.007f}},
    {37.0f, 0.0f, 35.0f, 83.0f, {0.063f, 0.0835f, -0.007f}},
    {90.0f, 0.0f, 35.0f, 83.0f, {0.0815f, 0.0f, -0.007f}},
    {143.0f, 0.0f, 35.0f, 83.0f, {0.063f, -0.0835f, -0.007f}},
}};

HexapodGeometryState MakeDefaultGeometryState();

}  // namespace visualiser::robot
