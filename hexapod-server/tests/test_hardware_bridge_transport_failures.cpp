#include "hardware_bridge_transport_test_helpers.hpp"

#include <chrono>
#include <thread>

namespace hardware_bridge_transport_test {
namespace {

bool test_not_ready_error_before_init() {
    auto endpoint = std::make_unique<FakePacketEndpoint>(
        [](const DecodedPacket&) { return std::vector<DecodedPacket>{}; });
    SimpleHardwareBridge bridge(std::move(endpoint));

    if (!expect(!bridge.set_power_relay(true), "set_power_relay should fail before init")) {
        return false;
    }
    return expect(bridge.last_error() == BridgeError::NotReady,
                  "calling command APIs before init should map to BridgeError::NotReady");
}

bool test_transport_failure_during_init() {
    auto endpoint = std::make_unique<FakePacketEndpoint>(
        [](const DecodedPacket&) { return std::vector<DecodedPacket>{}; });
    SimpleHardwareBridge bridge(std::move(endpoint), /*timeout_ms=*/5);

    if (!expect(!bridge.init(), "init should fail when transport never responds")) {
        return false;
    }
    return expect(bridge.last_error() == BridgeError::TransportFailure,
                  "init handshake timeout should map to BridgeError::TransportFailure");
}

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
    if (!expect(!bridge->read(out), "read should fail for malformed ACK payload")) {
        return false;
    }
    const auto metadata = bridge->last_bridge_result();
    if (!expect(metadata.has_value(), "malformed payload failure should expose bridge metadata")) {
        return false;
    }
    if (!expect(metadata->phase == BridgeFailurePhase::CommandDecode,
                "malformed ACK payload should classify failure phase as command decode")) {
        return false;
    }
    return expect(bridge->last_error() == BridgeError::ProtocolFailure,
                  "malformed ACK payload should map to BridgeError::ProtocolFailure");
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
    if (!expect(bridge->last_error() == BridgeError::ProtocolFailure,
                "explicit NACK should map to BridgeError::ProtocolFailure")) {
        return false;
    }
    const auto metadata = bridge->last_bridge_result();
    if (!expect(metadata.has_value(), "explicit NACK should expose bridge metadata")) {
        return false;
    }
    if (!expect(metadata->phase == BridgeFailurePhase::CommandResponse,
                "explicit NACK should classify as command response failure")) {
        return false;
    }
    if (!expect(metadata->domain == BridgeFailureDomain::CommandProtocol,
                "explicit NACK should classify as command protocol domain")) {
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
    if (!expect(bridge->last_error() == BridgeError::Timeout,
                "retry exhaustion should map to BridgeError::Timeout")) {
        return false;
    }
    const auto metadata = bridge->last_bridge_result();
    if (!expect(metadata.has_value(), "retry exhaustion should expose bridge metadata")) {
        return false;
    }
    if (!expect(metadata->phase == BridgeFailurePhase::CommandTransport,
                "retry exhaustion should classify as command transport failure")) {
        return false;
    }
    if (!expect(metadata->domain == BridgeFailureDomain::TransportLink,
                "retry exhaustion should classify as transport/link domain")) {
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
    if (!expect(bridge->last_error() == BridgeError::None,
                "successful retry should clear BridgeError to None")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    return expect(sent.size() == 4, "expected HELLO, HEARTBEAT, and two SET_POWER_RELAY attempts");
}

bool test_unsupported_command_maps_to_unsupported_error() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             return ok_hello_ack(request);
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, NACK, {UNSUPPORTED_COMMAND}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before unsupported command test")) {
        return false;
    }

    if (!expect(!bridge->set_power_relay(true), "set_power_relay should fail when peer reports unsupported")) {
        return false;
    }
    return expect(bridge->last_error() == BridgeError::Unsupported,
                  "UNSUPPORTED_COMMAND should map to BridgeError::Unsupported");
}

bool test_capability_negotiation_failure_classification() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    bool first_hello = true;
    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [&first_hello](const DecodedPacket& request) {
                                         if (request.cmd == HELLO) {
                                             if (first_hello) {
                                                 first_hello = false;
                                                 return ok_hello_ack(request);
                                             }
                                             return std::vector<DecodedPacket>{{request.seq, NACK, {BUSY_NOT_READY}}};
                                         }
                                         if (request.cmd == HEARTBEAT) {
                                             return ok_hello_ack(request);
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before capability negotiation classification test")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));

    if (!expect(!bridge->set_power_relay(true),
                "set_power_relay should fail when timeout re-establish handshake fails")) {
        return false;
    }
    if (!expect(bridge->last_error() == BridgeError::ProtocolFailure,
                "re-establish handshake failure should map to BridgeError::ProtocolFailure")) {
        return false;
    }

    const auto metadata = bridge->last_bridge_result();
    if (!expect(metadata.has_value(), "capability negotiation failure should expose bridge metadata")) {
        return false;
    }
    if (!expect(metadata->phase == BridgeFailurePhase::CapabilityNegotiation,
                "re-establish handshake failure should classify as capability negotiation")) {
        return false;
    }
    return expect(metadata->domain == BridgeFailureDomain::CapabilityProtocol,
                  "re-establish handshake failure should classify as capability/protocol domain");
}

}  // namespace

bool run_failure_and_corruption_tests() {
    if (!test_not_ready_error_before_init()) {
        return false;
    }
    if (!test_transport_failure_during_init()) {
        return false;
    }
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
    if (!test_protocol_error_retried_and_succeeds()) {
        return false;
    }
    if (!test_unsupported_command_maps_to_unsupported_error()) {
        return false;
    }
    return test_capability_negotiation_failure_classification();
}

}  // namespace hardware_bridge_transport_test
