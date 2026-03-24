#pragma once

#include "hardware_bridge.hpp"

#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "protocol_codec.hpp"

namespace hardware_bridge_transport_test {

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

inline bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

inline std::vector<DecodedPacket> ok_hello_ack(const DecodedPacket& request, uint8_t controller_id = 0x01,
                                               uint8_t capabilities = CAPABILITY_ANGULAR_FEEDBACK) {
    const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, controller_id, capabilities};
    return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
}

inline bool init_bridge(FakePacketEndpoint*& endpoint_view, SimpleHardwareBridge*& bridge_view,
                        std::unique_ptr<SimpleHardwareBridge>& bridge_owner,
                        ResponseBuilder response_builder) {
    auto endpoint = std::make_unique<FakePacketEndpoint>(std::move(response_builder));
    endpoint_view = endpoint.get();
    bridge_owner = std::make_unique<SimpleHardwareBridge>(std::move(endpoint));
    bridge_view = bridge_owner.get();
    return bridge_view->init();
}

bool run_handshake_tests();
bool run_failure_and_corruption_tests();
bool run_command_tests();
bool run_capability_tests();

}  // namespace hardware_bridge_transport_test
