#pragma once

#include "calibration_probe.hpp"

double legJoint(const LegState& leg_state, int joint);

SingleJointDynamicsFit fitSingleJointDirection(
    const std::vector<ServoDynamicsSample>& samples,
    int joint,
    bool positive_direction,
    const ServoDynamicsFitOptions& options);
