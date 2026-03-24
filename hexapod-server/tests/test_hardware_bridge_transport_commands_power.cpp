#include "hardware_bridge_transport_test_helpers.hpp"

namespace hardware_bridge_transport_test {

bool test_set_target_angle_success_and_nack();
bool test_set_and_get_angle_calibrations_success_and_failures();
bool test_scalar_getters_success_and_malformed_payloads();
bool test_servo_enable_roundtrip_success_and_failures();
bool test_led_info_and_led_color_commands();

bool test_send_diagnostic_success_and_nack() {
    const std::vector<uint8_t> diagnostic_request{0xDE, 0xAD, 0xBE, 0xEF};
    const std::vector<uint8_t> diagnostic_response{0x10, 0x20, 0x30};

    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [&diagnostic_response](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  return ok_hello_ack(request);
                                              }
                                              if (request.cmd == DIAGNOSTIC) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, diagnostic_response}};
                                              }
                                              return std::vector<DecodedPacket>{};
                                          });
    if (!expect(success_init, "init should succeed before send_diagnostic success case")) {
        return false;
    }

    std::vector<uint8_t> observed_response;
    if (!expect(success_bridge->send_diagnostic(diagnostic_request, observed_response),
                "send_diagnostic should succeed with ACK payload")) {
        return false;
    }
    if (!expect(observed_response == diagnostic_response,
                "send_diagnostic should decode/passthrough ACK payload bytes")) {
        return false;
    }

    const auto& success_sent = success_endpoint->sent_packets();
    if (!expect(success_sent.size() == 3, "expected HELLO, HEARTBEAT, DIAGNOSTIC")) {
        return false;
    }
    if (!expect(success_sent.back().cmd == DIAGNOSTIC, "last packet should be DIAGNOSTIC")) {
        return false;
    }
    if (!expect(success_sent.back().payload == diagnostic_request,
                "DIAGNOSTIC payload should be transmitted unchanged")) {
        return false;
    }

    std::unique_ptr<SimpleHardwareBridge> nack_bridge_owner;
    FakePacketEndpoint* nack_endpoint = nullptr;
    SimpleHardwareBridge* nack_bridge = nullptr;
    const bool nack_init = init_bridge(nack_endpoint, nack_bridge, nack_bridge_owner,
                                       [](const DecodedPacket& request) {
                                           if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                               return ok_hello_ack(request);
                                           }
                                           if (request.cmd == DIAGNOSTIC) {
                                               return std::vector<DecodedPacket>{{request.seq, NACK, {UNSUPPORTED_COMMAND}}};
                                           }
                                           return std::vector<DecodedPacket>{};
                                       });
    if (!expect(nack_init, "init should succeed before send_diagnostic NACK case")) {
        return false;
    }

    std::vector<uint8_t> preserved_response{0x55};
    if (!expect(!nack_bridge->send_diagnostic(diagnostic_request, preserved_response),
                "send_diagnostic should fail on NACK")) {
        return false;
    }
    if (!expect(preserved_response == std::vector<uint8_t>{0x55},
                "send_diagnostic should not overwrite response buffer on NACK")) {
        return false;
    }
    if (!expect(nack_endpoint->sent_packets().back().cmd == DIAGNOSTIC,
                "send_diagnostic NACK case should send DIAGNOSTIC opcode")) {
        return false;
    }
    return expect(nack_endpoint->sent_packets().back().payload == diagnostic_request,
                  "send_diagnostic NACK case should preserve outbound payload shape");
}

bool run_command_tests() {
    if (!test_set_target_angle_success_and_nack()) {
        return false;
    }
    if (!test_set_and_get_angle_calibrations_success_and_failures()) {
        return false;
    }
    if (!test_scalar_getters_success_and_malformed_payloads()) {
        return false;
    }
    if (!test_servo_enable_roundtrip_success_and_failures()) {
        return false;
    }
    if (!test_send_diagnostic_success_and_nack()) {
        return false;
    }
    return test_led_info_and_led_color_commands();
}

}  // namespace hardware_bridge_transport_test
