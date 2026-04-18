#include "imu_physics.hpp"

#include <cmath>

namespace {

constexpr double kStandardGravity = 9.80665;

Vec3 simVecToServer(const float x, const float y, const float z) {
    return Vec3{-static_cast<double>(z), static_cast<double>(x), static_cast<double>(y)};
}

Mat3 quatWxyzToMat3(const double w, const double x, const double y, const double z) {
    const double n = w * w + x * x + y * y + z * z;
    const double s = n > 0.0 ? 2.0 / n : 0.0;
    const double wx = s * w * x;
    const double wy = s * w * y;
    const double wz = s * w * z;
    const double xx = s * x * x;
    const double xy = s * x * y;
    const double xz = s * x * z;
    const double yy = s * y * y;
    const double yz = s * y * z;
    const double zz = s * z * z;
    Mat3 r{};
    r.m[0][0] = 1.0 - (yy + zz);
    r.m[0][1] = xy - wz;
    r.m[0][2] = xz + wy;
    r.m[1][0] = xy + wz;
    r.m[1][1] = 1.0 - (xx + zz);
    r.m[1][2] = yz - wx;
    r.m[2][0] = xz - wy;
    r.m[2][1] = yz + wx;
    r.m[2][2] = 1.0 - (xx + yy);
    return r;
}

Mat3 frameSimToServer() {
    Mat3 r{};
    r.m[0][0] = 0.0;
    r.m[0][1] = 0.0;
    r.m[0][2] = -1.0;
    r.m[1][0] = 1.0;
    r.m[1][1] = 0.0;
    r.m[1][2] = 0.0;
    r.m[2][0] = 0.0;
    r.m[2][1] = 1.0;
    r.m[2][2] = 0.0;
    return r;
}

} // namespace

void imuSampleFromPhysicsStateResponse(const physics_sim::StateResponse& rsp,
                                      const TimePointUs timestamp_us,
                                      ImuSample& out) {
    out.timestamp_us = timestamp_us;

    const Vec3 w_srv = simVecToServer(
        rsp.body_angular_velocity[0], rsp.body_angular_velocity[1], rsp.body_angular_velocity[2]);
    out.gyro_radps = AngularVelocityRadPerSec3{w_srv.x, w_srv.y, w_srv.z};

    const Mat3 R_sim = quatWxyzToMat3(static_cast<double>(rsp.body_orientation[0]),
                                      static_cast<double>(rsp.body_orientation[1]),
                                      static_cast<double>(rsp.body_orientation[2]),
                                      static_cast<double>(rsp.body_orientation[3]));
    const Mat3 R_ws = frameSimToServer();
    const Mat3 R_srv = R_ws * R_sim * R_ws.transpose();

    const Vec3 specific_force_world{0.0, 0.0, kStandardGravity};
    out.accel_mps2 = R_srv.transpose() * specific_force_world;
    out.valid = true;
}
