#include "estimator.hpp"
#include "imu_unit.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    hardware::DummyImuUnit imu(std::chrono::milliseconds{1});
    if (!expect(imu.init(), "dummy imu should initialize")) {
        return EXIT_FAILURE;
    }

    hardware::ImuSample first{};
    if (!expect(imu.read(first), "dummy imu should produce first sample") ||
        !expect(first.valid, "dummy imu sample should be marked valid")) {
        return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{2});
    hardware::ImuSample second{};
    if (!expect(imu.read(second), "dummy imu should produce second sample") ||
        !expect(second.sample_id > first.sample_id, "dummy imu sample id should advance")) {
        return EXIT_FAILURE;
    }

    SimpleEstimator estimator{};
    RobotState raw{};
    raw.sample_id = 1;
    raw.timestamp_us = now_us();
    raw.has_body_pose_state = true;
    raw.body_pose_state.orientation_rad = EulerAnglesRad3{0.11, -0.07, 0.42};
    raw.body_pose_state.angular_velocity_radps = AngularVelocityRadPerSec3{0.5, 0.4, 0.3};

    const RobotState est = estimator.update(raw);
    if (!expect(est.has_body_pose_state, "estimator should keep imu body-pose validity") ||
        !expect(std::abs(est.body_pose_state.orientation_rad.x - raw.body_pose_state.orientation_rad.x) < 1e-9,
                "estimator should use imu roll directly") ||
        !expect(std::abs(est.body_pose_state.orientation_rad.y - raw.body_pose_state.orientation_rad.y) < 1e-9,
                "estimator should use imu pitch directly") ||
        !expect(std::abs(est.body_pose_state.angular_velocity_radps.z - raw.body_pose_state.angular_velocity_radps.z) < 1e-9,
                "estimator should use imu yaw rate directly")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
