#include "hardware_bridge.hpp"

#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "hexapod-common.hpp"
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

    RawHardwareState out{};
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

    return EXIT_SUCCESS;
}
