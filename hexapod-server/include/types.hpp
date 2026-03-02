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



// Structures used for Hexapod

struct JointRawState {
  double pos_rad {0.0};
};

struct JointState {
  double pos_rad {0.0};
  double vel_radps {0.0};
};

struct LegRawState {
  std::array<JointRawState, kNumJoints> joint_raw_state{};
};

struct LegState {
  std::array<JointState, kNumJoints> joint_state{};
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
};

struct EstimatedState {
  std::array<LegState, kNumLegs> leg_states{};
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
