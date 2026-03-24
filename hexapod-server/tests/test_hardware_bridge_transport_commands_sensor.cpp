#include "hardware_bridge_transport_test_helpers.hpp"

namespace hardware_bridge_transport_test {

bool test_scalar_getters_success_and_malformed_payloads() {
    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  return ok_hello_ack(request);
                                              }
                                              if (request.cmd == GET_CURRENT) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_scalar_float({4.5f})}};
                                              }
                                              if (request.cmd == GET_VOLTAGE) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_scalar_float({11.8f})}};
                                              }
                                              if (request.cmd == GET_SENSOR) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_scalar_float({2.25f})}};
                                              }
                                              return std::vector<DecodedPacket>{};
                                          });
    if (!expect(success_init, "init should succeed before scalar getter success cases")) {
        return false;
    }

    float current = 0.0f;
    float voltage = 0.0f;
    float sensor = 0.0f;
    if (!expect(success_bridge->get_current(current), "get_current should succeed with scalar ACK")) {
        return false;
    }
    if (!expect(success_bridge->get_voltage(voltage), "get_voltage should succeed with scalar ACK")) {
        return false;
    }
    if (!expect(success_bridge->get_sensor(3, sensor), "get_sensor should succeed with scalar ACK")) {
        return false;
    }
    if (!expect(current == 4.5f, "get_current should decode expected float")) {
        return false;
    }
    if (!expect(voltage == 11.8f, "get_voltage should decode expected float")) {
        return false;
    }
    if (!expect(sensor == 2.25f, "get_sensor should decode expected float")) {
        return false;
    }

    const auto& success_sent = success_endpoint->sent_packets();
    if (!expect(success_sent.size() == 5, "expected HELLO, HEARTBEAT, GET_CURRENT, GET_VOLTAGE, GET_SENSOR")) {
        return false;
    }
    if (!expect(success_sent[2].cmd == GET_CURRENT && success_sent[2].payload.empty(),
                "GET_CURRENT should be sent with empty payload")) {
        return false;
    }
    if (!expect(success_sent[3].cmd == GET_VOLTAGE && success_sent[3].payload.empty(),
                "GET_VOLTAGE should be sent with empty payload")) {
        return false;
    }
    if (!expect(success_sent[4].cmd == GET_SENSOR && success_sent[4].payload.size() == sizeof(uint8_t),
                "GET_SENSOR should include exactly one-byte sensor id payload")) {
        return false;
    }
    if (!expect(success_sent[4].payload[0] == 3, "GET_SENSOR payload should contain requested sensor id")) {
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
                                                if (request.cmd == GET_CURRENT || request.cmd == GET_VOLTAGE || request.cmd == GET_SENSOR) {
                                                    return std::vector<DecodedPacket>{{request.seq, ACK, {0xAA}}};
                                                }
                                                return std::vector<DecodedPacket>{};
                                            });
    if (!expect(malformed_init, "init should succeed before scalar getter malformed cases")) {
        return false;
    }
    if (!expect(!malformed_bridge->get_current(current),
                "get_current should fail for malformed scalar ACK payload")) {
        return false;
    }
    if (!expect(!malformed_bridge->get_voltage(voltage),
                "get_voltage should fail for malformed scalar ACK payload")) {
        return false;
    }
    if (!expect(!malformed_bridge->get_sensor(9, sensor),
                "get_sensor should fail for malformed scalar ACK payload")) {
        return false;
    }
    const auto& malformed_sent = malformed_endpoint->sent_packets();
    return expect(malformed_sent.back().cmd == GET_SENSOR &&
                      malformed_sent.back().payload.size() == sizeof(uint8_t),
                  "get_sensor malformed case should still send correct opcode and payload shape");
}

}  // namespace hardware_bridge_transport_test
