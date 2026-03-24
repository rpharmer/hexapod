#include "bridge_link_manager.hpp"

#include <chrono>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_bridge.hpp"
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

bool test_init_sends_handshake_and_heartbeat() {
    auto endpoint = std::make_unique<FakePacketEndpoint>([](const DecodedPacket& request) {
        if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
            const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x21, CAPABILITY_ANGULAR_FEEDBACK};
            return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
        }
        return std::vector<DecodedPacket>{};
    });
    auto* endpoint_view = endpoint.get();

    BridgeLinkManager manager("unused", 115200, 100, std::move(endpoint));
    if (!expect(manager.init(CAPABILITY_ANGULAR_FEEDBACK), "init should succeed with valid hello and heartbeat")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 2, "init should emit HELLO followed by HEARTBEAT")) {
        return false;
    }
    if (!expect(sent[0].cmd == HELLO, "first packet must be HELLO")) {
        return false;
    }
    if (!expect(sent[1].cmd == HEARTBEAT, "second packet must be HEARTBEAT")) {
        return false;
    }

    return true;
}

bool test_ensure_link_reestablishes_after_timeout() {
    auto endpoint = std::make_unique<FakePacketEndpoint>([](const DecodedPacket& request) {
        if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
            const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x21, CAPABILITY_ANGULAR_FEEDBACK};
            return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
        }
        return std::vector<DecodedPacket>{};
    });
    auto* endpoint_view = endpoint.get();

    BridgeLinkManager manager("unused", 115200, 100, std::move(endpoint), DurationUs{1000}, DurationUs{50000});
    if (!expect(manager.init(CAPABILITY_ANGULAR_FEEDBACK), "init should succeed before timeout re-link")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    if (!expect(manager.ensure_link(CAPABILITY_ANGULAR_FEEDBACK), "ensure_link should re-establish after timeout")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 3, "expected HELLO, HEARTBEAT, and re-establish HELLO")) {
        return false;
    }
    return expect(sent[2].cmd == HELLO, "third packet should be HELLO after timeout re-establish");
}

}  // namespace

int main() {
    if (!test_init_sends_handshake_and_heartbeat()) {
        return EXIT_FAILURE;
    }
    if (!test_ensure_link_reestablishes_after_timeout()) {
        return EXIT_FAILURE;
    }

    std::cout << "PASS: BridgeLinkManager collaborator behavior" << std::endl;
    return EXIT_SUCCESS;
}
