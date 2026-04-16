#pragma once

#include "types.hpp"

/**
 * If `raw` has no IMU sample but `est` has a contact-plane body estimate (`has_body_twist_state`),
 * fills `est.imu` with synthetic specific-force (1g rotated by estimated roll/pitch/yaw) and
 * angular rate from the plane filter's finite differences. Leaves `est` unchanged when
 * `raw.has_imu` is already true.
 */
void fillSoftwareImuIfNoHardware(const RobotState& raw, RobotState& est);
