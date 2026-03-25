#ifndef PROTOCOL_LAYOUT_H
#define PROTOCOL_LAYOUT_H

#include <cstddef>
#include <cstdint>

inline constexpr uint8_t PROTOCOL_VERSION = 0x04;

inline constexpr std::size_t kProtocolLegCount = 6;
inline constexpr std::size_t kProtocolJointsPerLeg = 3;
inline constexpr std::size_t kProtocolJointCount = kProtocolLegCount * kProtocolJointsPerLeg;
inline constexpr std::size_t kProtocolFootSensorCount = kProtocolLegCount;
inline constexpr std::size_t kProtocolCalibrationPairsPerJoint = 2;
inline constexpr std::size_t kProtocolLedColorChannels = 3;
inline constexpr std::size_t kProtocolLedCount = 6;
inline constexpr std::size_t kProtocolLedColorsPayloadBytes =
    kProtocolLedCount * kProtocolLedColorChannels * sizeof(uint8_t);

// enum to map joints to ints
// [R/L] is Right/ Left handside from point of view of robot
// 1st number is Leg number 1 being front legs, 2 middle legs and 3 being back legs
// 2nd number is 1 -> Coxa, 2 -> Femur, 3 -> Tibia
// order is [2nd (R>L)][1st (descending)][3rd (ascending)]
// 0 - 17 corresponds to physical pin (1 - 18)
enum class JointID : uint8_t {
  R31 = 0,
  R32 = 1,
  R33 = 2,
  L31 = 3,
  L32 = 4,
  L33 = 5,
  R21 = 6,
  R22 = 7,
  R23 = 8,
  L21 = 9,
  L22 = 10,
  L23 = 11,
  R11 = 12,
  R12 = 13,
  R13 = 14,
  L11 = 15,
  L12 = 16,
  L13 = 17
};

enum class LegID : uint8_t {
  R3 = 0,
  L3 = 1,
  R2 = 2,
  L2 = 3,
  R1 = 4,
  L1 = 5
};

enum class LegJointID : uint8_t {
  Coxa = 0,
  Femur = 1,
  Tibia = 2
};

#endif
