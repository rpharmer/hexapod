#pragma once

#include <vector>

#include "types.hpp"

class HardwareStateCodec {
public:
    std::vector<uint8_t> encode_joint_targets(const JointTargets& in) const;

    bool decode_full_hardware_state(const std::vector<uint8_t>& payload,
                                    RobotState& out) const;
};
