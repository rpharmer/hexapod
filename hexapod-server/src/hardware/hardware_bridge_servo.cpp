#include "hardware_bridge.hpp"

#include "bridge_command_api.hpp"
#include "hexapod-common.hpp"
#include "protocol_codec.hpp"

bool SimpleHardwareBridge::set_target_angle(uint8_t servo_id, float angle) {
    std::vector<uint8_t> payload;
    payload.reserve(sizeof(uint8_t) + sizeof(float));
    append_scalar(payload, servo_id);
    append_scalar(payload, angle);
    return complete_command("set_target_angle",
                            withCommandApi("set_target_angle",
                                           [&payload](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_TARGET_ANGLE, payload);
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::set_servos_enabled(const std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled payload{};
    for (std::size_t i = 0; i < enabled.size(); ++i) {
        payload[i] = enabled[i] ? 1 : 0;
    }

    return complete_command("set_servos_enabled",
                            withCommandApi("set_servos_enabled",
                                           [&payload](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_SERVOS_ENABLED,
                                                   protocol::encode_servo_enabled(payload));
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::get_servos_enabled(std::array<bool, kNumJoints>& enabled) {
    protocol::ServoEnabled states{};
    const auto decode_servo_state = [](const std::vector<uint8_t>& payload, protocol::ServoEnabled& decoded) {
        return protocol::decode_servo_enabled(payload, decoded);
    };
    const BridgeError command_error = withCommandApi(
        "get_servos_enabled",
        [&states, &decode_servo_state](BridgeCommandApi& api) {
            return api.request_decoded_with_error(CommandCode::GET_SERVOS_ENABLED, {}, decode_servo_state, states);
        });
    if (!complete_command("get_servos_enabled", command_error, "command execution failed")) {
        return false;
    }

    for (std::size_t i = 0; i < enabled.size(); ++i) {
        enabled[i] = (states[i] != 0);
    }

    return true;
}

bool SimpleHardwareBridge::set_servos_to_mid() {
    return complete_command("set_servos_to_mid",
                            withCommandApi("set_servos_to_mid",
                                           [](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_SERVOS_TO_MID, {});
                                           }),
                            "command execution failed");
}
