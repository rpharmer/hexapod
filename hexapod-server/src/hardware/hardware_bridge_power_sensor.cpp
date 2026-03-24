#include "hardware_bridge.hpp"

#include "bridge_command_api.hpp"
#include "hexapod-common.hpp"
#include "protocol_codec.hpp"

namespace {

bool decodeScalarFloatPayload(const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
    return protocol::decode_scalar_float(payload, decoded);
}

}  // namespace

bool SimpleHardwareBridge::set_power_relay(bool enabled) {
    return complete_command("set_power_relay",
                            withCommandApi("set_power_relay",
                                           [enabled](BridgeCommandApi& api) {
                                               return api.request_ack_with_error(
                                                   CommandCode::SET_POWER_RELAY,
                                                   {static_cast<uint8_t>(enabled ? 1 : 0)});
                                           }),
                            "command execution failed");
}

bool SimpleHardwareBridge::get_current(float& out_current) {
    protocol::ScalarFloat current{};
    const BridgeError command_error = withCommandApi("get_current",
                                                     [&current](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(
                                                             CommandCode::GET_CURRENT,
                                                             {},
                                                             decodeScalarFloatPayload,
                                                             current);
                                                     });
    if (!complete_command("get_current", command_error, "command execution failed")) {
        return false;
    }

    out_current = current.value;
    return true;
}

bool SimpleHardwareBridge::get_voltage(float& out_voltage) {
    protocol::ScalarFloat voltage{};
    const BridgeError command_error = withCommandApi("get_voltage",
                                                     [&voltage](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(
                                                             CommandCode::GET_VOLTAGE,
                                                             {},
                                                             decodeScalarFloatPayload,
                                                             voltage);
                                                     });
    if (!complete_command("get_voltage", command_error, "command execution failed")) {
        return false;
    }

    out_voltage = voltage.value;
    return true;
}

bool SimpleHardwareBridge::get_sensor(uint8_t sensor_id, float& out_voltage) {
    std::vector<uint8_t> request_payload;
    request_payload.reserve(sizeof(uint8_t));
    append_scalar(request_payload, sensor_id);

    protocol::ScalarFloat sensor_voltage{};
    const BridgeError command_error = withCommandApi("get_sensor",
                                                     [&request_payload, &sensor_voltage](BridgeCommandApi& api) {
                                                         return api.request_decoded_with_error(CommandCode::GET_SENSOR,
                                                                                              request_payload,
                                                                                              decodeScalarFloatPayload,
                                                                                              sensor_voltage);
                                                     });
    if (!complete_command("get_sensor", command_error, "command execution failed")) {
        return false;
    }

    out_voltage = sensor_voltage.value;
    return true;
}

bool SimpleHardwareBridge::send_diagnostic(const std::vector<uint8_t>& payload,
                                           std::vector<uint8_t>& response_payload) {
    return complete_command("send_diagnostic",
                            withCommandApi("send_diagnostic",
                                           [&payload, &response_payload](BridgeCommandApi& api) {
                                               return api.request_ack_payload_with_error(
                                                   CommandCode::DIAGNOSTIC, payload, response_payload);
                                           }),
                            "command execution failed");
}
