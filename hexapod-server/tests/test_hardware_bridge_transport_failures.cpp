#include "hardware_bridge_transport_test_helpers.hpp"

namespace hardware_bridge_transport_test {
namespace {

bool test_malformed_ack_payload_handling() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* ignored_endpoint = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(ignored_endpoint, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             return ok_hello_ack(request);
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
                                             return ok_hello_ack(request);
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
                                             return ok_hello_ack(request);
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
                                             return ok_hello_ack(request);
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
                                             return ok_hello_ack(request);
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

}  // namespace

bool run_failure_and_corruption_tests() {
    if (!test_malformed_ack_payload_handling()) {
        return false;
    }
    if (!test_explicit_nack_behavior_for_command_methods()) {
        return false;
    }
    if (!test_timeout_retry_then_success()) {
        return false;
    }
    if (!test_timeout_retry_exhausted()) {
        return false;
    }
    return test_protocol_error_retried_and_succeeds();
}

}  // namespace hardware_bridge_transport_test
