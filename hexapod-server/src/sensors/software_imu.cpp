#include "software_imu.hpp"

namespace {

constexpr double kStandardGravity = 9.80665;

} // namespace

void fillSoftwareImuIfNoHardware(const RobotState& raw, RobotState& est) {
    if (raw.has_imu || !est.has_body_twist_state) {
        return;
    }

    const double roll = est.body_twist_state.twist_pos_rad.x;
    const double pitch = est.body_twist_state.twist_pos_rad.y;
    const double yaw = est.body_twist_state.twist_pos_rad.z;
    const Mat3 R_body_to_world = Mat3::rotZ(yaw) * Mat3::rotY(pitch) * Mat3::rotX(roll);
    const Vec3 specific_force_world{0.0, 0.0, kStandardGravity};
    est.imu.accel_mps2 = R_body_to_world.transpose() * specific_force_world;

    est.imu.gyro_radps = AngularVelocityRadPerSec3{est.body_twist_state.twist_vel_radps.x,
                                                   est.body_twist_state.twist_vel_radps.y,
                                                   0.0};
    est.imu.timestamp_us = est.timestamp_us;
    est.imu.valid = true;
    est.has_imu = true;
}
