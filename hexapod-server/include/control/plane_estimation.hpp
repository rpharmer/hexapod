#pragma once

#include <array>

#include "calibration_probe.hpp"

Vec3 computeFootWorldFromServoAngles(const LegState& servo_angles,
                                     const LegGeometry& leg_geometry,
                                     const BodyPose& body_pose);

int countContacts(const std::array<bool, kNumLegs>& contacts);

double meanContactPlaneHeight(const BaseClearanceSample& sample,
                              const HexapodGeometry& geometry,
                              int& used_contacts);
