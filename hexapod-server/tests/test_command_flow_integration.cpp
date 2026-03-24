#include "hardware_bridge.hpp"

#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "command_client.hpp"
#include "hexapod-common.hpp"
#include "protocol_codec.hpp"
#include "transport_session.hpp"
#include "types.hpp"

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

bool init_bridge(FakePacketEndpoint*& endpoint_view,
                 SimpleHardwareBridge*& bridge_view,
                 std::unique_ptr<SimpleHardwareBridge>& bridge_owner,
                 ResponseBuilder response_builder) {
    auto endpoint = std::make_unique<FakePacketEndpoint>(std::move(response_builder));
    endpoint_view = endpoint.get();
    bridge_owner = std::make_unique<SimpleHardwareBridge>(std::move(endpoint));
    bridge_view = bridge_owner.get();
    return bridge_view->init();
}

bool test_motion_command_flow_boundary_and_nack() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == as_u8(CommandCode::HELLO) || request.cmd == as_u8(CommandCode::HEARTBEAT)) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x22,
                                                                          CAPABILITY_ANGULAR_FEEDBACK};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }

                                         if (request.cmd == SET_TARGET_ANGLE) {
                                             std::size_t offset = 0;
                                             uint8_t servo_id = 0;
                                             float ignored_angle = 0.0f;
                                             if (!read_scalar(request.payload, offset, servo_id) ||
                                                 !read_scalar(request.payload, offset, ignored_angle) ||
                                                 offset != request.payload.size()) {
                                                 return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_PAYLOAD_LENGTH}}};
                                             }

                                             if (servo_id >= static_cast<uint8_t>(kNumJoints)) {
                                                 return std::vector<DecodedPacket>{{request.seq, NACK, {OUT_OF_RANGE_INDEX}}};
                                             }
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }

                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "motion flow test bridge init should succeed")) {
        return false;
    }

    if (!expect(bridge->set_target_angle(0, -1.2f),
                "motion command should ACK for minimum servo index")) {
        return false;
    }
    if (!expect(bridge->set_target_angle(static_cast<uint8_t>(kNumJoints - 1), 1.8f),
                "motion command should ACK for maximum valid servo index")) {
        return false;
    }
    if (!expect(!bridge->set_target_angle(static_cast<uint8_t>(kNumJoints), 0.2f),
                "motion command should fail for out-of-range servo index")) {
        return false;
    }

    const auto& sent = endpoint->sent_packets();
    if (!expect(sent.size() == 5,
                "expected HELLO, HEARTBEAT, and three SET_TARGET_ANGLE packets")) {
        return false;
    }

    for (std::size_t i = 2; i < sent.size(); ++i) {
        if (!expect(sent[i].cmd == SET_TARGET_ANGLE,
                    "motion test should only send SET_TARGET_ANGLE after init")) {
            return false;
        }
        if (!expect(sent[i].payload.size() == sizeof(uint8_t) + sizeof(float),
                    "SET_TARGET_ANGLE payload should include servo index + angle")) {
            return false;
        }
    }

    return true;
}

bool test_sensing_get_sensor_decode_malformed_and_boundary_index() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == as_u8(CommandCode::HELLO) || request.cmd == as_u8(CommandCode::HEARTBEAT)) {
                                             const protocol::HelloAck ack{PROTOCOL_VERSION, STATUS_OK, 0x34,
                                                                          CAPABILITY_ANGULAR_FEEDBACK};
                                             return std::vector<DecodedPacket>{{request.seq, ACK, protocol::encode_hello_ack(ack)}};
                                         }

                                         if (request.cmd != GET_SENSOR) {
                                             return std::vector<DecodedPacket>{};
                                         }

                                         if (request.payload.size() != sizeof(uint8_t)) {
                                             return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_PAYLOAD_LENGTH}}};
                                         }

                                         const uint8_t sensor_id = request.payload[0];
                                         if (sensor_id == 0) {
                                             return std::vector<DecodedPacket>{{
                                                 request.seq,
                                                 ACK,
                                                 protocol::encode_scalar_float(protocol::ScalarFloat{0.0f})
                                             }};
                                         }

                                         if (sensor_id == static_cast<uint8_t>(kProtocolFootSensorCount - 1)) {
                                             return std::vector<DecodedPacket>{{
                                                 request.seq,
                                                 ACK,
                                                 protocol::encode_scalar_float(protocol::ScalarFloat{9.5f})
                                             }};
                                         }

                                         if (sensor_id == 2) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {0xAB}}};
                                         }

                                         return std::vector<DecodedPacket>{{request.seq, NACK, {OUT_OF_RANGE_INDEX}}};
                                     });
    if (!expect(init_ok, "sensing flow test bridge init should succeed")) {
        return false;
    }

    float sensor_voltage = 0.0f;
    if (!expect(bridge->get_sensor(0, sensor_voltage), "sensor index 0 should decode ACK payload")) {
        return false;
    }
    if (!expect(sensor_voltage == 0.0f, "sensor index 0 decoded value mismatch")) {
        return false;
    }

    if (!expect(bridge->get_sensor(static_cast<uint8_t>(kProtocolFootSensorCount - 1), sensor_voltage),
                "max valid sensor index should decode ACK payload")) {
        return false;
    }
    if (!expect(sensor_voltage == 9.5f, "max sensor index decoded value mismatch")) {
        return false;
    }

    if (!expect(!bridge->get_sensor(2, sensor_voltage),
                "malformed ACK payload should fail sensing decode")) {
        return false;
    }

    if (!expect(!bridge->get_sensor(static_cast<uint8_t>(kProtocolFootSensorCount), sensor_voltage),
                "out-of-range sensor index should fail with NACK mapping")) {
        return false;
    }

    const auto& sent = endpoint->sent_packets();
    if (!expect(sent.size() == 6,
                "expected HELLO, HEARTBEAT, and four GET_SENSOR commands")) {
        return false;
    }

    for (std::size_t i = 2; i < sent.size(); ++i) {
        if (!expect(sent[i].cmd == as_u8(CommandCode::GET_SENSOR),
                    "sensing test should send GET_SENSOR command for each call")) {
            return false;
        }
        if (!expect(sent[i].payload.size() == sizeof(uint8_t),
                    "GET_SENSOR request should contain exactly one sensor index byte")) {
            return false;
        }
    }

    return true;
}

bool test_power_domain_outcome_semantics_and_payload() {
    FakePacketEndpoint endpoint([](const DecodedPacket& request) {
        if (request.cmd == as_u8(CommandCode::SET_POWER_RELAY)) {
            if (request.payload.size() != sizeof(uint8_t)) {
                return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_PAYLOAD_LENGTH}}};
            }
            if (request.payload[0] > 1) {
                return std::vector<DecodedPacket>{{request.seq, NACK, {INVALID_ARGUMENT}}};
            }
            return std::vector<DecodedPacket>{{request.seq, NACK, {BUSY_NOT_READY}}};
        }
        return std::vector<DecodedPacket>{};
    });

    TransportSession transport(endpoint, DurationUs{500000}, DurationUs{150000});
    CommandClient command_client(transport);

    const auto outcome = command_client.transact(CommandCode::SET_POWER_RELAY, {1}, nullptr);
    if (!expect(outcome.outcome_class == TransportSession::OutcomeClass::Nack,
                "power command should map explicit device rejection to Nack outcome")) {
        return false;
    }
    if (!expect(outcome.nack_code == BUSY_NOT_READY,
                "power command should preserve NACK error code semantics")) {
        return false;
    }

    const auto& sent = endpoint.sent_packets();
    if (!expect(sent.size() == 1, "NACK should stop power retries after first attempt")) {
        return false;
    }
    if (!expect(sent[0].cmd == as_u8(CommandCode::SET_POWER_RELAY),
                "power test should send SET_POWER_RELAY command")) {
        return false;
    }
    if (!expect(sent[0].payload.size() == sizeof(uint8_t) && sent[0].payload[0] == 1,
                "power relay request payload should encode enable flag")) {
        return false;
    }

    return true;
}

}  // namespace

int main() {
    if (!test_motion_command_flow_boundary_and_nack()) {
        return EXIT_FAILURE;
    }

    if (!test_sensing_get_sensor_decode_malformed_and_boundary_index()) {
        return EXIT_FAILURE;
    }

    if (!test_power_domain_outcome_semantics_and_payload()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
