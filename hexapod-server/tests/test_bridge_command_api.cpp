#include "bridge_command_api.hpp"

#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <utility>
#include <vector>

#include "command_client.hpp"
#include "hardware_bridge.hpp"
#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "transport_session.hpp"

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

bool test_ack_and_nack_paths() {
    FakePacketEndpoint endpoint([](const DecodedPacket& request) {
        if (request.cmd == SET_POWER_RELAY) {
            return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
        }
        if (request.cmd == SET_TARGET_ANGLE) {
            return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_ARGUMENT}}};
        }
        return std::vector<DecodedPacket>{};
    });

    TransportSession transport(endpoint, DurationUs{500000}, DurationUs{150000});
    CommandClient command_client(transport);
    BridgeCommandApi api(command_client);

    if (!expect(api.request_ack(SET_POWER_RELAY, {1}), "ACK response should return true")) {
        return false;
    }

    return expect(!api.request_ack(SET_TARGET_ANGLE, {0, 0, 0, 0, 0}), "NACK response should return false");
}

bool test_request_decoded_success_and_decode_failure() {
    FakePacketEndpoint endpoint([](const DecodedPacket& request) {
        if (request.cmd == GET_VOLTAGE) {
            return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_scalar_float(protocol::ScalarFloat{7.2f})}};
        }
        if (request.cmd == GET_CURRENT) {
            return std::vector<DecodedPacket>{{request.seq, ACK, {0x99}}};
        }
        return std::vector<DecodedPacket>{};
    });

    TransportSession transport(endpoint, DurationUs{500000}, DurationUs{150000});
    CommandClient command_client(transport);
    BridgeCommandApi api(command_client);

    protocol::ScalarFloat voltage{};
    const auto decode_scalar = [](const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded) {
        return protocol::decode_scalar_float(payload, decoded);
    };
    if (!expect(api.request_decoded(GET_VOLTAGE, {}, decode_scalar, voltage),
                "decoded helper should return true for valid ACK payload")) {
        return false;
    }
    if (!expect(voltage.value == 7.2f, "decoded value mismatch for GET_VOLTAGE")) {
        return false;
    }

    protocol::ScalarFloat current{};
    return expect(!api.request_decoded(GET_CURRENT, {}, decode_scalar, current),
                  "decoded helper should fail for malformed ACK payload");
}

}  // namespace

int main() {
    if (!test_ack_and_nack_paths()) {
        return EXIT_FAILURE;
    }
    if (!test_request_decoded_success_and_decode_failure()) {
        return EXIT_FAILURE;
    }

    std::cout << "PASS: BridgeCommandApi collaborator behavior" << std::endl;
    return EXIT_SUCCESS;
}
