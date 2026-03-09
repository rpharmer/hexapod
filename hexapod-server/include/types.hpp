#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>

constexpr int COXA = 0;
constexpr int FEMUR = 1;
constexpr int TIBIA = 2;



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


struct GaitState {
    std::array<double, kNumLegs> phase{};
    std::array<bool, kNumLegs> in_stance{};
    double stride_phase_rate_hz{1.0};
    uint64_t timestamp_us{0};
};

enum class FaultCode : uint8_t {
    NONE = 0,
    BUS_TIMEOUT,
    ESTOP,
    TIP_OVER,
    ESTIMATOR_INVALID,
    MOTOR_FAULT,
    JOINT_LIMIT,
    COMMAND_TIMEOUT
};

struct SafetyState {
    bool inhibit_motion{true};
    bool torque_cut{false};
    FaultCode active_fault{FaultCode::NONE};
    std::array<bool, kNumLegs> leg_enabled{true, true, true, true, true, true};
};

struct FootTarget {
    Vec3 pos_body_m{};
    Vec3 vel_body_mps{};
};

struct LegTargets {
    std::array<FootTarget, kNumLegs> feet{};
    uint64_t timestamp_us{0};
};

struct JointRawState {
  double pos_rad {0.0};
};

struct JointState {
  double pos_rad {0.0};
  double vel_radps {0.0};
};

struct LegRawState {
  std::array<JointRawState, kJointsPerLeg> joint_raw_state{};
};

struct LegState {
  std::array<JointState, kJointsPerLeg> joint_state{};
};

struct JointTargets {
  std::array<LegRawState,kNumLegs> leg_raw_states{};
};

struct BodyTwistState {
  Vec3 twist_pos_rad{};
  Vec3 twist_vel_radps{};
  
  double body_height_pos_m{0.0};
  double body_height_vel_mps{0.0};
};

struct RawHardwareState {
  std::array<LegRawState, kNumLegs> leg_states{};
  std::array<bool, kNumLegs> foot_contacts{};
  float voltage{0.0};
  float current{0.0};
  uint64_t timestamp_us{0};
  bool bus_ok{false};
};

struct MotionIntent {
  RobotMode requested_mode{RobotMode::SAFE_IDLE};
  GaitType gait{GaitType::TRIPOD};
  BodyTwistState twist{};
  uint64_t timestamp_us{0};
};

struct EstimatedState {
  std::array<LegRawState, kNumLegs> leg_states{};
  std::array<bool, kNumLegs> foot_contacts{};
  
  BodyTwistState body_twist_state{};
  uint64_t timestamp_us{0};
};

struct TargetBodyState {
  std::array<LegState, kNumLegs> leg_states{};
  std::array<bool, kNumLegs> foot_contacts{};
  BodyTwistState body_twist_state{};
  bool valid{false};
  uint64_t timestamp_us{0};
};

struct ControlStatus {
    RobotMode active_mode{RobotMode::SAFE_IDLE};
    bool estimator_valid{false};
    bool bus_ok{false};
    FaultCode active_fault{FaultCode::NONE};
    uint64_t loop_counter{0};
};