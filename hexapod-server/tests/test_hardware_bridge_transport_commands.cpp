#include "hardware_bridge_transport_test_helpers.hpp"

#include <array>

namespace hardware_bridge_transport_test {
namespace {

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
                                                  return ok_hello_ack(request);
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
                                                   return ok_hello_ack(request);
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
                                                        return ok_hello_ack(request);
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

}  // namespace

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
