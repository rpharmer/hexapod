#include "hardware_bridge.hpp"

#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "hexapod-common.hpp"
#include "geometry_config.hpp"
#include "geometry_profile_service.hpp"
#include "protocol_codec.hpp"

namespace {

using ResponseBuilder = std::function<std::vector<DecodedPacket>(const DecodedPacket&)>;

class FakePacketEndpoint final : public IPacketEndpoint {
public:
    explicit FakePacketEndpoint(ResponseBuilder response_builder)
        : response_builder_(std::move(response_builder)) {}

    void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) override {
        DecodedPacket request{seq, cmd, payload};
        sent_packets_.push_back(request);
        const auto responses = response_builder_(request);
        for (const auto& response : responses) {
            queued_responses_.push_back(response);
        }
    }

    bool recv_packet(DecodedPacket& packet) override {
        if (queued_responses_.empty()) {
            return false;
        }
        packet = queued_responses_.front();
        queued_responses_.pop_front();
        return true;
    }

    const std::vector<DecodedPacket>& sent_packets() const {
        return sent_packets_;
    }

private:
    ResponseBuilder response_builder_;
    std::deque<DecodedPacket> queued_responses_;
    std::vector<DecodedPacket> sent_packets_;
};

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool test_successful_handshake_and_heartbeat() {
    auto endpoint = std::make_unique<FakePacketEndpoint>([](const DecodedPacket& request) {
        if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
            const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
            return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
        }
        return std::vector<DecodedPacket>{};
    });
    auto* endpoint_view = endpoint.get();

    SimpleHardwareBridge bridge(std::move(endpoint));
    if (!expect(bridge.init(), "bridge init should succeed with valid HELLO + HEARTBEAT ACKs")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 2, "init should send HELLO and HEARTBEAT packets")) {
        return false;
    }

    if (!expect(sent[0].cmd == HELLO, "first packet should be HELLO")) {
        return false;
    }
    protocol::HelloRequest hello{};
    if (!expect(protocol::decode_hello_request(sent[0].payload, hello),
                "HELLO payload should decode as hello request")) {
        return false;
    }
    if (!expect((hello.capabilities & CAPABILITY_ANGULAR_FEEDBACK) != 0,
                "HELLO should request ANGULAR_FEEDBACK capability")) {
        return false;
    }

    if (!expect(sent[1].cmd == HEARTBEAT, "second packet should be HEARTBEAT")) {
        return false;
    }

    return true;
}

bool test_sequence_mismatch_rejected() {
    auto endpoint = std::make_unique<FakePacketEndpoint>([](const DecodedPacket& request) {
        if (request.cmd == HELLO) {
            const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
            return std::vector<DecodedPacket>{{static_cast<uint16_t>(request.seq + 1), ACK, protocol::encode_hello_ack(ack)}};
        }
        return std::vector<DecodedPacket>{};
    });

    SimpleHardwareBridge bridge(std::move(endpoint));
    return expect(!bridge.init(), "init should fail when HELLO ACK sequence does not match");
}

bool init_bridge(FakePacketEndpoint*& endpoint_view, SimpleHardwareBridge*& bridge_view,
                 std::unique_ptr<SimpleHardwareBridge>& bridge_owner,
                 ResponseBuilder response_builder) {
    auto endpoint = std::make_unique<FakePacketEndpoint>(std::move(response_builder));
    endpoint_view = endpoint.get();
    bridge_owner = std::make_unique<SimpleHardwareBridge>(std::move(endpoint));
    bridge_view = bridge_owner.get();
    return bridge_view->init();
}

bool test_malformed_ack_payload_handling() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* ignored_endpoint = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(ignored_endpoint, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == GET_FULL_HARDWARE_STATE) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {0x42}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before malformed payload test")) {
        return false;
    }

    RobotState out{};
    return expect(!bridge->read(out), "read should fail for malformed ACK payload");
}

bool test_explicit_nack_behavior_for_command_methods() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_ARGUMENT}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });

    if (!expect(init_ok, "init should succeed before NACK test")) {
        return false;
    }

    if (!expect(!bridge->set_power_relay(true), "set_power_relay should fail on explicit NACK")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 3, "bridge should send HELLO, HEARTBEAT, then SET_POWER_RELAY")) {
        return false;
    }

    return expect(sent.back().cmd == SET_POWER_RELAY, "last packet should be SET_POWER_RELAY");
}

bool test_timeout_retry_then_success() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    bool dropped_once = false;
    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [&dropped_once](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             if (!dropped_once) {
                                                 dropped_once = true;
                                                 return std::vector<DecodedPacket>{};
                                             }
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });

    if (!expect(init_ok, "init should succeed before timeout retry test")) {
        return false;
    }

    if (!expect(bridge->set_power_relay(true), "set_power_relay should succeed after a retried timeout")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    return expect(sent.size() == 4, "expected HELLO, HEARTBEAT, and two SET_POWER_RELAY attempts");
}

bool test_timeout_retry_exhausted() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });

    if (!expect(init_ok, "init should succeed before retry exhausted test")) {
        return false;
    }

    if (!expect(!bridge->set_power_relay(true), "set_power_relay should fail when all retries timeout")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    return expect(sent.size() == 5, "expected HELLO, HEARTBEAT, and three SET_POWER_RELAY attempts");
}

bool test_protocol_error_retried_and_succeeds() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    bool sent_bad_seq = false;
    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [&sent_bad_seq](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             if (!sent_bad_seq) {
                                                 sent_bad_seq = true;
                                                 return std::vector<DecodedPacket>{{static_cast<uint16_t>(request.seq + 1), ACK, {}}};
                                             }
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });

    if (!expect(init_ok, "init should succeed before protocol error retry test")) {
        return false;
    }

    if (!expect(bridge->set_power_relay(true), "set_power_relay should succeed after protocol error retry")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    return expect(sent.size() == 4, "expected HELLO, HEARTBEAT, and two SET_POWER_RELAY attempts");
}

bool test_set_target_angle_success_and_nack() {
    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;

    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
                                               const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                               return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
    const auto& nack_sent = nack_endpoint->sent_packets();
    if (!expect(nack_sent.back().cmd == SET_TARGET_ANGLE,
                "last packet should be SET_TARGET_ANGLE for NACK case")) {
        return false;
    }
    return expect(nack_sent.back().payload.size() == (sizeof(uint8_t) + sizeof(float)),
                  "SET_TARGET_ANGLE NACK case should still send expected payload shape");
}

bool test_set_and_get_angle_calibrations_success_and_failures() {
    std::vector<float> expected_calibrations(kProtocolJointCount * kProtocolCalibrationPairsPerJoint);
    for (std::size_t i = 0; i < expected_calibrations.size(); ++i) {
        expected_calibrations[i] = static_cast<float>(i) * 0.25f;
    }

    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [&expected_calibrations](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                              }
                                              if (request.cmd == SET_ANGLE_CALIBRATIONS) {
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                              }
                                              if (request.cmd == GET_ANGLE_CALIBRATIONS) {
                                                  protocol::Calibrations ack_calibs{};
                                                  for (std::size_t i = 0; i < ack_calibs.size(); ++i) {
                                                      ack_calibs[i] = expected_calibrations[i];
                                                  }
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_calibrations(ack_calibs)}};
                                              }
                                              return std::vector<DecodedPacket>{};
                                          });
    if (!expect(success_init, "init should succeed before calibration success cases")) {
        return false;
    }

    if (!expect(success_bridge->set_angle_calibrations(expected_calibrations),
                "set_angle_calibrations should succeed with ACK")) {
        return false;
    }

    std::vector<float> observed_calibrations;
    if (!expect(success_bridge->get_angle_calibrations(observed_calibrations),
                "get_angle_calibrations should succeed with decodable ACK payload")) {
        return false;
    }
    if (!expect(observed_calibrations == expected_calibrations,
                "get_angle_calibrations should decode expected values")) {
        return false;
    }

    const auto& success_sent = success_endpoint->sent_packets();
    if (!expect(success_sent.size() == 4, "expected HELLO, HEARTBEAT, SET_ANGLE_CALIBRATIONS, GET_ANGLE_CALIBRATIONS")) {
        return false;
    }
    if (!expect(success_sent[2].cmd == SET_ANGLE_CALIBRATIONS,
                "third packet should be SET_ANGLE_CALIBRATIONS")) {
        return false;
    }
    if (!expect(success_sent[2].payload.size() == kProtocolCalibrationsPayloadBytes,
                "SET_ANGLE_CALIBRATIONS payload should be encoded calibration bytes")) {
        return false;
    }
    if (!expect(success_sent[3].cmd == GET_ANGLE_CALIBRATIONS,
                "fourth packet should be GET_ANGLE_CALIBRATIONS")) {
        return false;
    }
    if (!expect(success_sent[3].payload.empty(),
                "GET_ANGLE_CALIBRATIONS should send an empty payload")) {
        return false;
    }

    std::unique_ptr<SimpleHardwareBridge> set_nack_bridge_owner;
    FakePacketEndpoint* set_nack_endpoint = nullptr;
    SimpleHardwareBridge* set_nack_bridge = nullptr;
    const bool set_nack_init = init_bridge(set_nack_endpoint, set_nack_bridge, set_nack_bridge_owner,
                                           [](const DecodedPacket& request) {
                                               if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                   const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                   return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                               }
                                               if (request.cmd == SET_ANGLE_CALIBRATIONS) {
                                                   return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_PAYLOAD_LENGTH}}};
                                               }
                                               return std::vector<DecodedPacket>{};
                                           });
    if (!expect(set_nack_init, "init should succeed before set_angle_calibrations NACK case")) {
        return false;
    }
    if (!expect(!set_nack_bridge->set_angle_calibrations(expected_calibrations),
                "set_angle_calibrations should fail on NACK")) {
        return false;
    }
    if (!expect(set_nack_endpoint->sent_packets().back().cmd == SET_ANGLE_CALIBRATIONS,
                "set_angle_calibrations NACK case should send correct opcode")) {
        return false;
    }

    std::unique_ptr<SimpleHardwareBridge> get_malformed_bridge_owner;
    FakePacketEndpoint* get_malformed_endpoint = nullptr;
    SimpleHardwareBridge* get_malformed_bridge = nullptr;
    const bool get_malformed_init = init_bridge(get_malformed_endpoint, get_malformed_bridge, get_malformed_bridge_owner,
                                                [](const DecodedPacket& request) {
                                                    if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                        const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                        return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                                    }
                                                    if (request.cmd == GET_ANGLE_CALIBRATIONS) {
                                                        return std::vector<DecodedPacket>{{request.seq, ACK, {0x01, 0x02}}};
                                                    }
                                                    return std::vector<DecodedPacket>{};
                                                });
    if (!expect(get_malformed_init, "init should succeed before get_angle_calibrations malformed ACK case")) {
        return false;
    }
    std::vector<float> malformed_out;
    if (!expect(!get_malformed_bridge->get_angle_calibrations(malformed_out),
                "get_angle_calibrations should fail for malformed ACK payload")) {
        return false;
    }
    return expect(get_malformed_endpoint->sent_packets().back().cmd == GET_ANGLE_CALIBRATIONS,
                  "get_angle_calibrations malformed case should send correct opcode");
}

bool test_scalar_getters_success_and_malformed_payloads() {
    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
                                                    const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                    return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
                                                  const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
                                                   const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                   return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
                                                        const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                        return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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

bool test_send_diagnostic_success_and_nack() {
    const std::vector<uint8_t> diagnostic_request{0xDE, 0xAD, 0xBE, 0xEF};
    const std::vector<uint8_t> diagnostic_response{0x10, 0x20, 0x30};

    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [&diagnostic_response](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
                                               const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                               return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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

bool test_ensure_link_heartbeat_due_sends_heartbeat_then_command() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before heartbeat-due ensure_link case")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(170));

    if (!expect(bridge->set_power_relay(true),
                "set_power_relay should succeed when ensure_link sends idle heartbeat")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 4,
                "expected HELLO, HEARTBEAT, ensure_link HEARTBEAT, then SET_POWER_RELAY")) {
        return false;
    }
    if (!expect(sent[2].cmd == HEARTBEAT, "third packet should be ensure_link HEARTBEAT")) {
        return false;
    }
    return expect(sent[3].cmd == SET_POWER_RELAY, "fourth packet should be SET_POWER_RELAY");
}

bool test_ensure_link_timeout_reestablishes_then_command_succeeds() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before timeout re-establish ensure_link case")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));

    if (!expect(bridge->set_power_relay(true),
                "set_power_relay should succeed after timeout-triggered re-establish")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 4,
                "expected HELLO, HEARTBEAT, re-establish HELLO, then SET_POWER_RELAY")) {
        return false;
    }
    if (!expect(sent[2].cmd == HELLO, "third packet should be timeout-triggered HELLO")) {
        return false;
    }
    return expect(sent[3].cmd == SET_POWER_RELAY, "fourth packet should be SET_POWER_RELAY");
}

bool test_ensure_link_timeout_reestablish_failure_returns_false() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    bool first_hello = true;
    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [&first_hello](const DecodedPacket& request) {
                                         if (request.cmd == HELLO) {
                                             if (first_hello) {
                                                 first_hello = false;
                                                 const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                 return std::vector<DecodedPacket>{
                                                     {request.seq, ACK, protocol::encode_hello_ack(ack)}
                                                 };
                                             }
                                             return std::vector<DecodedPacket>{{request.seq, NACK, {BUSY_NOT_READY}}};
                                         }
                                         if (request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before re-establish failure ensure_link case")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));

    if (!expect(!bridge->set_power_relay(true),
                "set_power_relay should fail when timeout re-establish handshake fails")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 3,
                "expected only HELLO, HEARTBEAT, and failed timeout re-establish HELLO")) {
        return false;
    }
    if (!expect(sent[2].cmd == HELLO, "third packet should be failed timeout re-establish HELLO")) {
        return false;
    }
    return expect(sent.back().cmd != SET_POWER_RELAY,
                  "SET_POWER_RELAY should not be sent when ensure_link fails");
}

bool test_read_falls_back_to_software_joint_estimate_without_angular_feedback() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const HexapodGeometry geometry_before = geometry_config::activeHexapodGeometry();
    HexapodGeometry modified_geometry = geometry_before;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            auto& dyn = modified_geometry.legGeometry[leg].servoDynamics[joint];
            dyn.positive_direction.tau_s = 0.08;
            dyn.negative_direction.tau_s = 0.08;
            dyn.positive_direction.vmax_radps = 0.05;
            dyn.negative_direction.vmax_radps = 0.05;
        }
    }

    std::string geometry_error;
    if (!geometry_profile_service::preview(modified_geometry, &geometry_error) ||
        !geometry_profile_service::apply(&geometry_error)) {
        std::cerr << "FAIL: failed to apply geometry test profile: " << geometry_error << "\n";
        return false;
    }

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01, 0x00};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }
                                         if (request.cmd == SET_JOINT_TARGETS) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         if (request.cmd == GET_FULL_HARDWARE_STATE) {
                                             protocol::FullHardwareState raw{};
                                             raw.joint_positions_rad[0] = 0.35f;
                                             raw.voltage = 12.3f;
                                             raw.current = 1.1f;
                                             return std::vector<DecodedPacket>{
                                                 {request.seq, ACK, protocol::encode_full_hardware_state(raw)}
                                             };
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before software feedback fallback test")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }

    JointTargets command{};
    command.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.8};
    if (!expect(bridge->write(command), "write should succeed before software feedback fallback read")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }

    RobotState first{};
    if (!expect(bridge->read(first), "first read should succeed")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.leg_states[0].joint_state[COXA].pos_rad.value > 0.0,
                "software feedback should move estimate toward commanded target")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.leg_states[0].joint_state[COXA].pos_rad.value < 0.35,
                "software feedback should be handshake-driven and not use reported angular telemetry")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.leg_states[0].joint_state[COXA].pos_rad.value <= 2e-4,
                "software feedback should respect geometry servo dynamics vmax")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.voltage > 0.0f && first.current > 0.0f,
                "fallback should preserve non-angular telemetry from device")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }

    RobotState second{};
    if (!expect(bridge->read(second), "second read should succeed")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    const bool progressed = expect(second.leg_states[0].joint_state[COXA].pos_rad.value >=
                                       first.leg_states[0].joint_state[COXA].pos_rad.value,
                                   "software estimate should progress toward command over successive reads");
    geometry_profile_service::preview(geometry_before);
    geometry_profile_service::apply();
    return progressed;
}

bool test_led_info_and_led_color_commands() {
    std::unique_ptr<SimpleHardwareBridge> success_bridge_owner;
    FakePacketEndpoint* success_endpoint = nullptr;
    SimpleHardwareBridge* success_bridge = nullptr;
    const bool success_init = init_bridge(success_endpoint, success_bridge, success_bridge_owner,
                                          [](const DecodedPacket& request) {
                                              if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                                  const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                  return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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
                                                    const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                                    return std::vector<DecodedPacket>{
                                                        {request.seq, ACK, protocol::encode_hello_ack(ack)}
                                                    };
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

    std::unique_ptr<SimpleHardwareBridge> nack_bridge_owner;
    FakePacketEndpoint* nack_endpoint = nullptr;
    SimpleHardwareBridge* nack_bridge = nullptr;
    const bool nack_init = init_bridge(nack_endpoint, nack_bridge, nack_bridge_owner,
                                       [](const DecodedPacket& request) {
                                           if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                               const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
                                               return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
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

}  // namespace

int main() {
    if (!test_successful_handshake_and_heartbeat()) {
        return EXIT_FAILURE;
    }

    if (!test_sequence_mismatch_rejected()) {
        return EXIT_FAILURE;
    }

    if (!test_malformed_ack_payload_handling()) {
        return EXIT_FAILURE;
    }

    if (!test_explicit_nack_behavior_for_command_methods()) {
        return EXIT_FAILURE;
    }

    if (!test_timeout_retry_then_success()) {
        return EXIT_FAILURE;
    }

    if (!test_timeout_retry_exhausted()) {
        return EXIT_FAILURE;
    }

    if (!test_protocol_error_retried_and_succeeds()) {
        return EXIT_FAILURE;
    }

    if (!test_set_target_angle_success_and_nack()) {
        return EXIT_FAILURE;
    }

    if (!test_set_and_get_angle_calibrations_success_and_failures()) {
        return EXIT_FAILURE;
    }

    if (!test_scalar_getters_success_and_malformed_payloads()) {
        return EXIT_FAILURE;
    }

    if (!test_servo_enable_roundtrip_success_and_failures()) {
        return EXIT_FAILURE;
    }

    if (!test_send_diagnostic_success_and_nack()) {
        return EXIT_FAILURE;
    }

    if (!test_ensure_link_heartbeat_due_sends_heartbeat_then_command()) {
        return EXIT_FAILURE;
    }

    if (!test_ensure_link_timeout_reestablishes_then_command_succeeds()) {
        return EXIT_FAILURE;
    }

    if (!test_ensure_link_timeout_reestablish_failure_returns_false()) {
        return EXIT_FAILURE;
    }

    if (!test_read_falls_back_to_software_joint_estimate_without_angular_feedback()) {
        return EXIT_FAILURE;
    }
    if (!test_led_info_and_led_color_commands()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
