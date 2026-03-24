#include "hardware_bridge_transport_test_helpers.hpp"

namespace hardware_bridge_transport_test {

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

}  // namespace hardware_bridge_transport_test
