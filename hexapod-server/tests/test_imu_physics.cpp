#include "dummy_imu.hpp"
#include "estimator.hpp"
#include "imu_physics.hpp"
#include "math_types.hpp"
#include "physics_sim_protocol.hpp"
#include "software_imu.hpp"

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

bool nearlyEqual(double lhs, double rhs, double eps = 1e-5) {
    return std::abs(lhs - rhs) <= eps;
}

} // namespace

int main() {
    physics_sim::StateResponse rsp{};
    rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
    rsp.body_orientation[0] = 1.0f;
    rsp.body_orientation[1] = 0.0f;
    rsp.body_orientation[2] = 0.0f;
    rsp.body_orientation[3] = 0.0f;
    rsp.body_angular_velocity[0] = 0.0f;
    rsp.body_angular_velocity[1] = 0.0f;
    rsp.body_angular_velocity[2] = 0.0f;

    ImuSample imu{};
    imuSampleFromPhysicsStateResponse(rsp, TimePointUs{42}, imu);
    if (!expect(imu.valid, "IMU sample should be marked valid") ||
        !expect(nearlyEqual(imu.accel_mps2.x, 0.0) && nearlyEqual(imu.accel_mps2.y, 0.0) &&
                    nearlyEqual(imu.accel_mps2.z, 9.80665),
                "upright identity pose should read ~+Z gravity specific force") ||
        !expect(nearlyEqual(imu.gyro_radps.x, 0.0) && nearlyEqual(imu.gyro_radps.y, 0.0) &&
                    nearlyEqual(imu.gyro_radps.z, 0.0),
                "zero sim angular velocity should map to zero gyro")) {
        return EXIT_FAILURE;
    }

    const double half = static_cast<double>(std::sqrt(2.0) * 0.5);
    rsp.body_orientation[0] = static_cast<float>(half);
    rsp.body_orientation[1] = static_cast<float>(half);
    rsp.body_orientation[2] = 0.0f;
    rsp.body_orientation[3] = 0.0f;
    rsp.body_angular_velocity[0] = 0.0f;
    rsp.body_angular_velocity[1] = 0.1f;
    rsp.body_angular_velocity[2] = 0.0f;
    imuSampleFromPhysicsStateResponse(rsp, TimePointUs{43}, imu);
    const double an = vecNorm(imu.accel_mps2);
    if (!expect(nearlyEqual(an, 9.80665, 1e-3),
                "rotated rest pose should keep specific-force magnitude near 1g") ||
        !expect(nearlyEqual(imu.gyro_radps.x, 0.0) && nearlyEqual(imu.gyro_radps.y, 0.0) &&
                    nearlyEqual(imu.gyro_radps.z, 0.1),
                "gyro vector should follow simVecToServer axis remap")) {
        return EXIT_FAILURE;
    }

    RobotState st{};
    DummyImu dummy{};
    dummy.sample.gyro_radps = AngularVelocityRadPerSec3{0.2, 0.0, 0.0};
    dummy.sample.accel_mps2 = Vec3{0.0, 1.0, 9.0};
    dummy.sample.valid = true;
    dummy.inject(st, TimePointUs{100});
    if (!expect(st.has_imu && st.imu.valid && st.imu.timestamp_us.value == 100,
                 "dummy IMU should stamp RobotState") ||
        !expect(nearlyEqual(st.imu.accel_mps2.y, 1.0), "dummy should copy accel")) {
        return EXIT_FAILURE;
    }
    dummy.enabled = false;
    dummy.inject(st, TimePointUs{101});
    if (!expect(!st.has_imu, "disabled dummy should clear has_imu")) {
        return EXIT_FAILURE;
    }

    SimpleEstimator est{};
    RobotState raw{};
    raw.sample_id = 7;
    raw.timestamp_us = TimePointUs{9'000'000};
    raw.foot_contacts.fill(true);
    raw.has_imu = true;
    raw.imu.valid = true;
    raw.imu.gyro_radps = AngularVelocityRadPerSec3{0.03, -0.02, 0.01};
    raw.imu.accel_mps2 = Vec3{0.1, -0.2, 9.7};
    const RobotState out = est.update(raw);
    if (!expect(out.has_imu && out.imu.valid && nearlyEqual(out.imu.accel_mps2.z, 9.7),
                 "SimpleEstimator should pass IMU through from raw state")) {
        return EXIT_FAILURE;
    }

    RobotState raw_sw{};
    raw_sw.has_imu = false;
    RobotState est_sw{};
    est_sw.has_body_twist_state = true;
    est_sw.timestamp_us = TimePointUs{500};
    est_sw.body_twist_state.twist_pos_rad = EulerAnglesRad3{0.1, -0.05, 0.0};
    est_sw.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{0.01, -0.02, 0.0};
    fillSoftwareImuIfNoHardware(raw_sw, est_sw);
    if (!expect(est_sw.has_imu && est_sw.imu.valid && est_sw.imu.timestamp_us.value == 500,
                 "fillSoftwareImuIfNoHardware should mark IMU valid") ||
        !expect(nearlyEqual(est_sw.imu.gyro_radps.x, 0.01) && nearlyEqual(est_sw.imu.gyro_radps.y, -0.02) &&
                    nearlyEqual(est_sw.imu.gyro_radps.z, 0.0),
                 "software gyro should copy plane roll/pitch rates (yaw rate forced to zero)") ||
        !expect(nearlyEqual(vecNorm(est_sw.imu.accel_mps2), 9.80665, 2e-3),
                 "software accel magnitude should stay ~1g for tilted plane attitude")) {
        return EXIT_FAILURE;
    }

    RobotState raw_hw{};
    raw_hw.has_imu = true;
    raw_hw.imu.valid = true;
    raw_hw.imu.accel_mps2 = Vec3{1.0, 2.0, 3.0};
    RobotState est_hw{};
    est_hw.has_body_twist_state = true;
    est_hw.body_twist_state.twist_pos_rad = EulerAnglesRad3{0.5, 0.5, 0.0};
    fillSoftwareImuIfNoHardware(raw_hw, est_hw);
    if (!expect(!est_hw.has_imu,
                 "software IMU fill should be skipped when raw already carries IMU")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
