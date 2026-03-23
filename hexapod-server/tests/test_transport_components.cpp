#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <vector>

#include "command_client.hpp"
#include "handshake_client.hpp"
#include "hardware_bridge.hpp"
#include "hardware_state_codec.hpp"
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

bool test_handshake_failure_then_recovery() {
    bool first_hello = true;
    FakePacketEndpoint endpoint([&first_hello](const DecodedPacket& request) {
        if (request.cmd == HELLO) {
            if (first_hello) {
                first_hello = false;
                return std::vector<DecodedPacket>{{request.seq, NACK, {BUSY_NOT_READY}}};
            }
            const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x01};
            return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
        }
        return std::vector<DecodedPacket>{};
    });

    TransportSession transport(endpoint, DurationUs{500000}, DurationUs{150000});
    CommandClient command_client(transport);
    HandshakeClient handshake(transport, command_client);

    if (!expect(!handshake.establish_link(0), "first HELLO should fail with NACK")) {
        return false;
    }
    if (!expect(handshake.establish_link(0), "second HELLO should recover with ACK")) {
        return false;
    }

    const auto& sent = endpoint.sent_packets();
    return expect(sent.size() == 2 && sent[0].cmd == HELLO && sent[1].cmd == HELLO,
                  "handshake recovery should send HELLO twice");
}

bool test_retry_exhaustion_and_nack_mapping() {
    FakePacketEndpoint timeout_endpoint([](const DecodedPacket&) {
        return std::vector<DecodedPacket>{};
    });

    TransportSession timeout_transport(timeout_endpoint, DurationUs{500000}, DurationUs{150000});
    CommandClient timeout_client(timeout_transport);

    const auto timeout_outcome = timeout_client.transact(SET_POWER_RELAY, {1}, nullptr);
    if (!expect(timeout_outcome.outcome_class == TransportSession::OutcomeClass::RetryExhausted,
                "timeouts should map to retry exhausted after all attempts")) {
        return false;
    }
    if (!expect(timeout_endpoint.sent_packets().size() == 3,
                "retry exhausted should perform three attempts by default")) {
        return false;
    }

    FakePacketEndpoint nack_endpoint([](const DecodedPacket& request) {
        return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_ARGUMENT}}};
    });

    TransportSession nack_transport(nack_endpoint, DurationUs{500000}, DurationUs{150000});
    CommandClient nack_client(nack_transport);
    const auto nack_outcome = nack_client.transact(SET_POWER_RELAY, {1}, nullptr);

    if (!expect(nack_outcome.outcome_class == TransportSession::OutcomeClass::Nack,
                "explicit NACK should map to Nack outcome")) {
        return false;
    }
    if (!expect(nack_outcome.nack_code == INVALID_ARGUMENT,
                "NACK outcome should preserve device error code")) {
        return false;
    }

    return expect(nack_endpoint.sent_packets().size() == 1,
                  "NACK should stop retries immediately");
}

bool test_hardware_state_codec_decode_failure_for_malformed_payload() {
    HardwareStateCodec codec;
    RobotState state{};

    const std::vector<uint8_t> malformed_payload{0x01, 0x02, 0x03};
    return expect(!codec.decode_full_hardware_state(malformed_payload, state),
                  "codec should reject malformed full hardware-state payloads");
}

}  // namespace

int main() {
    if (!test_handshake_failure_then_recovery()) {
        return EXIT_FAILURE;
    }

    if (!test_retry_exhaustion_and_nack_mapping()) {
        return EXIT_FAILURE;
    }

    if (!test_hardware_state_codec_decode_failure_for_malformed_payload()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
