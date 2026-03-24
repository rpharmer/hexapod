#include "hardware_bridge_transport_test_helpers.hpp"

namespace hardware_bridge_transport_test {
namespace {

bool test_successful_handshake_and_heartbeat() {
    auto endpoint = std::make_unique<FakePacketEndpoint>([](const DecodedPacket& request) {
        if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
            return ok_hello_ack(request);
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

    return expect(sent[1].cmd == HEARTBEAT, "second packet should be HEARTBEAT");
}

bool test_sequence_mismatch_rejected() {
    auto endpoint = std::make_unique<FakePacketEndpoint>([](const DecodedPacket& request) {
        if (request.cmd == HELLO) {
            const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
            return std::vector<DecodedPacket>{
                {static_cast<uint16_t>(request.seq + 1), ACK, protocol::encode_hello_ack(ack)}
            };
        }
        return std::vector<DecodedPacket>{};
    });

    SimpleHardwareBridge bridge(std::move(endpoint));
    return expect(!bridge.init(), "init should fail when HELLO ACK sequence does not match");
}

}  // namespace

bool run_handshake_tests() {
    if (!test_successful_handshake_and_heartbeat()) {
        return false;
    }
    return test_sequence_mismatch_rejected();
}

}  // namespace hardware_bridge_transport_test
