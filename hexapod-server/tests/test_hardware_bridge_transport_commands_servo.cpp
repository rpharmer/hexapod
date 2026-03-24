#include "hardware_bridge_transport_test_helpers.hpp"

#include <array>

namespace hardware_bridge_transport_test {

bool test_set_target_angle_success_and_nack() {
    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;

    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  return ok_hello_ack(request);
                                              }
                                              if (request.cmd == SET_TARGET_ANGLE) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                              }
                                              return std::vector<DecodedPacket>{};
                                          });
    if (!expect(success_init, "init should succeed before set_target_angle success case")) {
        return false;
    }

    const uint8_t servo_id = 7;
    const float requested_angle = 1.75f;
    if (!expect(success_bridge->set_target_angle(servo_id, requested_angle),
                "set_target_angle should succeed with ACK")) {
        return false;
    }
    if (!expect(success_bridge->last_error() == BridgeError::None,
                "set_target_angle success should clear last_error")) {
        return false;
    }

    const auto& success_sent = success_endpoint->sent_packets();
    if (!expect(success_sent.size() == 3, "expected HELLO, HEARTBEAT, and SET_TARGET_ANGLE")) {
        return false;
    }
    if (!expect(success_sent.back().cmd == SET_TARGET_ANGLE,
                "last packet should be SET_TARGET_ANGLE for success case")) {
        return false;
    }
    if (!expect(success_sent.back().payload.size() == (sizeof(uint8_t) + sizeof(float)),
                "SET_TARGET_ANGLE payload should contain servo_id + float angle")) {
        return false;
    }
    std::size_t offset = 0;
    uint8_t encoded_servo_id = 0;
    float encoded_angle = 0.0f;
    if (!expect(read_scalar(success_sent.back().payload, offset, encoded_servo_id),
                "SET_TARGET_ANGLE payload should decode servo_id")) {
        return false;
    }
    if (!expect(read_scalar(success_sent.back().payload, offset, encoded_angle),
                "SET_TARGET_ANGLE payload should decode angle")) {
        return false;
    }
    if (!expect(offset == success_sent.back().payload.size(),
                "SET_TARGET_ANGLE payload should be fully consumed by decode")) {
        return false;
    }
    if (!expect(encoded_servo_id == servo_id, "SET_TARGET_ANGLE payload should keep servo_id value")) {
        return false;
    }
    if (!expect(encoded_angle == requested_angle, "SET_TARGET_ANGLE payload should keep angle value")) {
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
                                           if (request.cmd == SET_TARGET_ANGLE) {
                                               return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_ARGUMENT}}};
                                           }
                                           return std::vector<DecodedPacket>{};
                                       });
    if (!expect(nack_init, "init should succeed before set_target_angle NACK case")) {
        return false;
    }

    if (!expect(!nack_bridge->set_target_angle(2, -0.5f),
                "set_target_angle should fail on explicit NACK")) {
        return false;
    }
    if (!expect(nack_bridge->last_error() == BridgeError::ProtocolFailure,
                "set_target_angle NACK should map to protocol failure")) {
        return false;
    }
    const auto& nack_sent = nack_endpoint->sent_packets();
    if (!expect(nack_sent.back().cmd == SET_TARGET_ANGLE,
                "last packet should be SET_TARGET_ANGLE for NACK case")) {
        return false;
    }
    return expect(nack_sent.back().payload.size() == (sizeof(uint8_t) + sizeof(float)),
                  "SET_TARGET_ANGLE NACK case should still send expected payload shape");
}

bool test_servo_enable_roundtrip_success_and_failures() {
    std::array<bool, kNumJoints> requested_enabled{};
    for (std::size_t i = 0; i < requested_enabled.size(); ++i) {
        requested_enabled[i] = ((i % 2) == 0);
    }

    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  return ok_hello_ack(request);
                                              }
                                              if (request.cmd == SET_SERVOS_ENABLED) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                              }
                                              if (request.cmd == GET_SERVOS_ENABLED) {
                                                  protocol::ServoEnabled response{};
                                                  for (std::size_t i = 0; i < response.size(); ++i) {
                                                      response[i] = (i % 3 == 0) ? 1 : 0;
                                                  }
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_servo_enabled(response)}};
                                              }
                                              return std::vector<DecodedPacket>{};
                                          });
    if (!expect(success_init, "init should succeed before servo enable success cases")) {
        return false;
    }
    if (!expect(success_bridge->set_servos_enabled(requested_enabled),
                "set_servos_enabled should succeed with ACK")) {
        return false;
    }
    std::array<bool, kNumJoints> observed_enabled{};
    if (!expect(success_bridge->get_servos_enabled(observed_enabled),
                "get_servos_enabled should succeed with decodable ACK payload")) {
        return false;
    }
    if (!expect(observed_enabled[0] && !observed_enabled[1] && !observed_enabled[2] && observed_enabled[3],
                "get_servos_enabled should decode expected bool mapping")) {
        return false;
    }

    const auto& success_sent = success_endpoint->sent_packets();
    if (!expect(success_sent.size() == 4, "expected HELLO, HEARTBEAT, SET_SERVOS_ENABLED, GET_SERVOS_ENABLED")) {
        return false;
    }
    if (!expect(success_sent[2].cmd == SET_SERVOS_ENABLED &&
                    success_sent[2].payload.size() == kProtocolServoEnablePayloadBytes,
                "SET_SERVOS_ENABLED payload should contain one byte per joint")) {
        return false;
    }
    if (!expect(success_sent[3].cmd == GET_SERVOS_ENABLED && success_sent[3].payload.empty(),
                "GET_SERVOS_ENABLED should send empty payload")) {
        return false;
    }

    std::unique_ptr<SimpleHardwareBridge> set_nack_bridge_owner;
    FakePacketEndpoint* set_nack_endpoint = nullptr;
    SimpleHardwareBridge* set_nack_bridge = nullptr;
    const bool set_nack_init = init_bridge(set_nack_endpoint, set_nack_bridge, set_nack_bridge_owner,
                                           [](const DecodedPacket& request) {
                                               if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                   return ok_hello_ack(request);
                                               }
                                               if (request.cmd == SET_SERVOS_ENABLED) {
                                                   return std::vector<DecodedPacket>{{request.seq, NACK, {BUSY_NOT_READY}}};
                                               }
                                               return std::vector<DecodedPacket>{};
                                           });
    if (!expect(set_nack_init, "init should succeed before set_servos_enabled NACK case")) {
        return false;
    }
    if (!expect(!set_nack_bridge->set_servos_enabled(requested_enabled),
                "set_servos_enabled should fail on NACK")) {
        return false;
    }
    if (!expect(set_nack_endpoint->sent_packets().back().cmd == SET_SERVOS_ENABLED,
                "set_servos_enabled NACK case should send correct opcode")) {
        return false;
    }

    std::unique_ptr<SimpleHardwareBridge> get_malformed_bridge_owner;
    FakePacketEndpoint* get_malformed_endpoint = nullptr;
    SimpleHardwareBridge* get_malformed_bridge = nullptr;
    const bool get_malformed_init = init_bridge(get_malformed_endpoint, get_malformed_bridge, get_malformed_bridge_owner,
                                                [](const DecodedPacket& request) {
                                                    if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                        return ok_hello_ack(request);
                                                    }
                                                    if (request.cmd == GET_SERVOS_ENABLED) {
                                                        return std::vector<DecodedPacket>{{request.seq, ACK, {0x01, 0x00}}};
                                                    }
                                                    return std::vector<DecodedPacket>{};
                                                });
    if (!expect(get_malformed_init, "init should succeed before get_servos_enabled malformed case")) {
        return false;
    }
    std::array<bool, kNumJoints> malformed_enabled{};
    if (!expect(!get_malformed_bridge->get_servos_enabled(malformed_enabled),
                "get_servos_enabled should fail for malformed ACK payload")) {
        return false;
    }
    return expect(get_malformed_endpoint->sent_packets().back().cmd == GET_SERVOS_ENABLED,
                  "get_servos_enabled malformed case should send correct opcode");
}

}  // namespace hardware_bridge_transport_test
