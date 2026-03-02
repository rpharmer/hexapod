#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>

constexpr int kNumLegs = 6;
constexpr int kJointsPerLeg = 3;
constexpr int kNumJoints = kNumLegs * kJointsPerLeg;

using Clock = std::chrono::steady_clock;

inline uint64_t now_us() {
    const auto t = Clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t).count());
}

enum class RobotMode {
    SAFE_IDLE,
    HOMING,
    STAND,
    WALK,
    FAULT
};

enum class FaultCode : uint32_t {
    NONE = 0,
    BUS_TIMEOUT,
    ESTOP,
    TIP_OVER,
    ESTIMATOR_INVALID,
    MOTOR_FAULT,
    JOINT_LIMIT,
    COMMAND_TIMEOUT
};


enum class GaitType {
    TRIPOD,
    RIPPLE,
    WAVE
};

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct BodyTwistCmd {
    double vx_mps{0.0};
    double vy_mps{0.0};
    double wz_radps{0.0};
    double body_height_m{0.20};
};

struct MotionIntent {
    RobotMode requested_mode{RobotMode::SAFE_IDLE};
    GaitType gait{GaitType::TRIPOD};
    BodyTwistCmd twist{};
    uint64_t timestamp_us{0};
};

struct JointState {
    double pos_rad{0.0};
    double vel_radps{0.0};
    double effort_nm{0.0};
    bool motor_fault{false};
};

struct RawHardwareState {
    std::array<JointState, kNumJoints> joints{};
    Vec3 body_rpy_rad{};
    Vec3 body_omega_radps{};
    std::array<bool, kNumLegs> foot_contact{};
    double battery_v{24.0};
    bool estop{false};
    bool bus_ok{false};
    uint64_t seq_rx{0};
    uint64_t timestamp_us{0};
};

struct EstimatedState {
    std::array<JointState, kNumJoints> joints{};
    Vec3 body_rpy_rad{};
    Vec3 body_omega_radps{};
    std::array<bool, kNumLegs> foot_contact{};
    double body_height_m{0.20};
    bool valid{false};
    uint64_t timestamp_us{0};
};

struct GaitState {
    std::array<double, kNumLegs> phase{};
    std::array<bool, kNumLegs> in_stance{};
    double stride_phase_rate_hz{1.0};
    uint64_t timestamp_us{0};
};
