#include "hardware_state_codec.hpp"

#include "hexapod-common.hpp"
#include "protocol_codec.hpp"

std::vector<uint8_t> HardwareStateCodec::encode_joint_targets(const JointTargets& in) const {
    protocol::JointTargets target_positions{};

    std::size_t idx = 0;
    for (const auto& leg : in.leg_states) {
        for (int j = 0; j < kJointsPerLeg; ++j) {
            target_positions[idx++] = static_cast<float>(leg.joint_state[j].pos_rad.value);
        }
    }

    return protocol::encode_joint_targets(target_positions);
}

bool HardwareStateCodec::decode_full_hardware_state(const std::vector<uint8_t>& payload,
                                                    RobotState& out) const {
    protocol::FullHardwareState decoded{};
    if (!protocol::decode_full_hardware_state(payload, decoded)) {
        return false;
    }

    std::size_t idx = 0;
    for (auto& leg : out.leg_states) {
        for (int j = 0; j < kJointsPerLeg; ++j) {
            leg.joint_state[j].pos_rad = AngleRad{decoded.joint_positions_rad[idx++]};
        }
    }

    for (std::size_t i = 0; i < kProtocolFootSensorCount; ++i) {
        out.foot_contacts[i] = (decoded.foot_contacts[i] != 0);
    }

    out.voltage = decoded.voltage;
    out.current = decoded.current;
    out.timestamp_us = now_us();
    return true;
}
