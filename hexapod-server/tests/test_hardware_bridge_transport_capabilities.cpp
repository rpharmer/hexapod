#include "hardware_bridge_transport_test_helpers.hpp"

#include <thread>

#include "geometry_config.hpp"
#include "geometry_profile_service.hpp"

namespace hardware_bridge_transport_test {
namespace {

bool test_ensure_link_heartbeat_due_sends_heartbeat_then_command() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             return ok_hello_ack(request);
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before heartbeat-due ensure_link case")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(170));

    if (!expect(bridge->set_power_relay(true),
                "set_power_relay should succeed when ensure_link sends idle heartbeat")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 4,
                "expected HELLO, HEARTBEAT, ensure_link HEARTBEAT, then SET_POWER_RELAY")) {
        return false;
    }
    if (!expect(sent[2].cmd == HEARTBEAT, "third packet should be ensure_link HEARTBEAT")) {
        return false;
    }
    return expect(sent[3].cmd == SET_POWER_RELAY, "fourth packet should be SET_POWER_RELAY");
}

bool test_ensure_link_timeout_reestablishes_then_command_succeeds() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             return ok_hello_ack(request);
                                         }
                                         if (request.cmd == SET_POWER_RELAY) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before timeout re-establish ensure_link case")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));

    if (!expect(bridge->set_power_relay(true),
                "set_power_relay should succeed after timeout-triggered re-establish")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 4,
                "expected HELLO, HEARTBEAT, re-establish HELLO, then SET_POWER_RELAY")) {
        return false;
    }
    if (!expect(sent[2].cmd == HELLO, "third packet should be timeout-triggered HELLO")) {
        return false;
    }
    return expect(sent[3].cmd == SET_POWER_RELAY, "fourth packet should be SET_POWER_RELAY");
}

bool test_ensure_link_timeout_reestablish_failure_returns_false() {
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
    if (!expect(init_ok, "init should succeed before re-establish failure ensure_link case")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));

    if (!expect(!bridge->set_power_relay(true),
                "set_power_relay should fail when timeout re-establish handshake fails")) {
        return false;
    }

    const auto& sent = endpoint_view->sent_packets();
    if (!expect(sent.size() == 3,
                "expected only HELLO, HEARTBEAT, and failed timeout re-establish HELLO")) {
        return false;
    }
    if (!expect(sent[2].cmd == HELLO, "third packet should be failed timeout re-establish HELLO")) {
        return false;
    }
    return expect(sent.back().cmd != SET_POWER_RELAY,
                  "SET_POWER_RELAY should not be sent when ensure_link fails");
}

bool test_read_falls_back_to_software_joint_estimate_without_angular_feedback() {
    std::unique_ptr<SimpleHardwareBridge> bridge_owner;
    FakePacketEndpoint* endpoint_view = nullptr;
    SimpleHardwareBridge* bridge = nullptr;

    const HexapodGeometry geometry_before = geometry_config::activeHexapodGeometry();
    HexapodGeometry modified_geometry = geometry_before;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            auto& dyn = modified_geometry.legGeometry[leg].servoDynamics[joint];
            dyn.positive_direction.tau_s = 0.08;
            dyn.negative_direction.tau_s = 0.08;
            dyn.positive_direction.vmax_radps = 0.05;
            dyn.negative_direction.vmax_radps = 0.05;
        }
    }

    std::string geometry_error;
    if (!geometry_profile_service::preview(modified_geometry, &geometry_error) ||
        !geometry_profile_service::apply(&geometry_error)) {
        std::cerr << "FAIL: failed to apply geometry test profile: " << geometry_error << "\n";
        return false;
    }

    const bool init_ok = init_bridge(endpoint_view, bridge, bridge_owner,
                                     [](const DecodedPacket& request) {
                                         if (request.cmd == HELLO || request.cmd == HEARTBEAT) {
                                             return ok_hello_ack(request, 0x01, 0x00);
                                         }
                                         if (request.cmd == SET_JOINT_TARGETS) {
                                             return std::vector<DecodedPacket>{{request.seq, ACK, {}}};
                                         }
                                         if (request.cmd == GET_FULL_HARDWARE_STATE) {
                                             protocol::FullHardwareState raw{};
                                             raw.joint_positions_rad[0] = 0.35f;
                                             raw.voltage = 12.3f;
                                             raw.current = 1.1f;
                                             return std::vector<DecodedPacket>{
                                                 {request.seq, ACK, protocol::encode_full_hardware_state(raw)}
                                             };
                                         }
                                         return std::vector<DecodedPacket>{};
                                     });
    if (!expect(init_ok, "init should succeed before software feedback fallback test")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }

    JointTargets command{};
    command.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.8};
    if (!expect(bridge->write(command), "write should succeed before software feedback fallback read")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }

    RobotState first{};
    if (!expect(bridge->read(first), "first read should succeed")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.leg_states[0].joint_state[COXA].pos_rad.value > 0.0,
                "software feedback should move estimate toward commanded target")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.leg_states[0].joint_state[COXA].pos_rad.value < 0.35,
                "software feedback should be handshake-driven and not use reported angular telemetry")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.leg_states[0].joint_state[COXA].pos_rad.value <= 2e-4,
                "software feedback should respect geometry servo dynamics vmax")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    if (!expect(first.voltage > 0.0f && first.current > 0.0f,
                "fallback should preserve non-angular telemetry from device")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }

    RobotState second{};
    if (!expect(bridge->read(second), "second read should succeed")) {
        geometry_profile_service::preview(geometry_before);
        geometry_profile_service::apply();
        return false;
    }
    const bool progressed = expect(second.leg_states[0].joint_state[COXA].pos_rad.value >=
                                       first.leg_states[0].joint_state[COXA].pos_rad.value,
                                   "software estimate should progress toward command over successive reads");
    geometry_profile_service::preview(geometry_before);
    geometry_profile_service::apply();
    return progressed;
}

}  // namespace

bool run_capability_tests() {
    if (!test_ensure_link_heartbeat_due_sends_heartbeat_then_command()) {
        return false;
    }
    if (!test_ensure_link_timeout_reestablishes_then_command_succeeds()) {
        return false;
    }
    if (!test_ensure_link_timeout_reestablish_failure_returns_false()) {
        return false;
    }
    return test_read_falls_back_to_software_joint_estimate_without_angular_feedback();
}

}  // namespace hardware_bridge_transport_test
