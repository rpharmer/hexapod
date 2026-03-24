#include "hardware_bridge.hpp"

#include "bridge_command_api.hpp"
#include "protocol_codec.hpp"

bool SimpleHardwareBridge::get_led_info(bool& present, uint8_t& count) {
    protocol::LedInfo info{};
    const auto decode_led_info = [](const std::vector<uint8_t>& payload, protocol::LedInfo& decoded) {
        return protocol::decode_led_info(payload, decoded);
    };
    const BridgeError command_error = withCommandApi("get_led_info",
                                                     [&info, &decode_led_info](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(
                                                             CommandCode::GET_LED_INFO,
                                                             {},
                                                             decode_led_info,
                                                             info);
                                                     });
    if (!complete_command("get_led_info", command_error, "command execution failed")) {
        return false;
    }

    present = (info.present != 0);
    count = info.count;
    return true;
}

bool SimpleHardwareBridge::set_led_colors(
    const std::array<uint8_t, kProtocolLedColorsPayloadBytes>& colors) {
    protocol::LedColors payload{};
    for (std::size_t i = 0; i < colors.size(); ++i) {
        payload[i] = colors[i];
    }

    return complete_command("set_led_colors",
                            withCommandApi("set_led_colors",
                                           [&payload](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_LED_COLORS,
                                                   protocol::encode_led_colors(payload));
                                           }),
                            "command execution failed");
}
