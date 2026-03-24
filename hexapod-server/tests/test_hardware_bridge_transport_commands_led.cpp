#include "hardware_bridge_transport_test_helpers.hpp"

namespace hardware_bridge_transport_test {

bool test_led_info_and_led_color_commands() {
    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  return ok_hello_ack(request);
                                              }
                                              if (request.cmd == GET_LED_INFO) {
                                                  return std::vector<DecodedPacket>{
                                                      {request.seq, ACK, protocol::encode_led_info(protocol::LedInfo{1, 6})}
                                                  };
                                              }
                                              if (request.cmd == SET_LED_COLORS) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                              }
                                              return std::vector<DecodedPacket>{};
                                          });
    if (!expect(success_init, "init should succeed before LED command success cases")) {
        return false;
    }

    bool led_present = false;
    uint8_t led_count = 0;
    if (!expect(success_bridge->get_led_info(led_present, led_count),
                "get_led_info should succeed with decodable ACK payload")) {
        return false;
    }
    if (!expect(success_bridge->last_error() == BridgeError::None,
                "get_led_info success should clear last_error")) {
        return false;
    }
    if (!expect(led_present && led_count == 6, "get_led_info should decode presence and count")) {
        return false;
    }

    std::array<uint8_t, kProtocolLedColorsPayloadBytes> colors{};
    for (std::size_t i = 0; i < colors.size(); ++i) {
        colors[i] = static_cast<uint8_t>(i + 1);
    }
    if (!expect(success_bridge->set_led_colors(colors), "set_led_colors should succeed with ACK")) {
        return false;
    }

    const auto& success_sent = success_endpoint->sent_packets();
    if (!expect(success_sent.size() == 4, "expected HELLO, HEARTBEAT, GET_LED_INFO, SET_LED_COLORS")) {
        return false;
    }
    if (!expect(success_sent[2].cmd == GET_LED_INFO && success_sent[2].payload.empty(),
                "GET_LED_INFO should send empty payload")) {
        return false;
    }
    if (!expect(success_sent[3].cmd == SET_LED_COLORS &&
                    success_sent[3].payload.size() == kProtocolLedColorsPayloadBytes,
                "SET_LED_COLORS should send three bytes per LED")) {
        return false;
    }

    std::unique_ptr<SimpleHardwareBridge> malformed_bridge_owner;
    FakePacketEndpoint* malformed_endpoint = nullptr;
    SimpleHardwareBridge* malformed_bridge = nullptr;
    const bool malformed_init = init_bridge(malformed_endpoint, malformed_bridge, malformed_bridge_owner,
                                            [](const DecodedPacket& request) {
                                                if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                    return ok_hello_ack(request);
                                                }
                                                if (request.cmd == GET_LED_INFO) {
                                                    return std::vector<DecodedPacket>{{request.seq, ACK, {0x01}}};
                                                }
                                                return std::vector<DecodedPacket>{};
                                            });
    if (!expect(malformed_init, "init should succeed before LED malformed payload case")) {
        return false;
    }
    if (!expect(!malformed_bridge->get_led_info(led_present, led_count),
                "get_led_info should fail on malformed ACK payload")) {
        return false;
    }
    if (!expect(malformed_bridge->last_error() == BridgeError::ProtocolFailure,
                "get_led_info malformed payload should map to protocol failure")) {
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
                                           if (request.cmd == SET_LED_COLORS) {
                                               return std::vector<DecodedPacket>{{request.seq, NACK, {BUSY_NOT_READY}}};
                                           }
                                           return std::vector<DecodedPacket>{};
                                       });
    if (!expect(nack_init, "init should succeed before LED NACK case")) {
        return false;
    }
    return expect(!nack_bridge->set_led_colors(colors),
                  "set_led_colors should fail on NACK");
}

}  // namespace hardware_bridge_transport_test
