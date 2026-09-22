#include "control/in_place_turn_hold.hpp"
#include "control/motion_intent_utils.hpp"
#include "hardware/physics_sim_bridge.hpp"
#include "hardware/sim_hardware_bridge.hpp"
#include <iostream>
#include <limits>

int main() {
    bool ok = true;
    InPlaceTurnHold hold;
    RobotState state{};
    state.bus_ok = true;
    state.has_body_twist_state = true;
    auto intent = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, .14);
    intent.cmd_yaw_radps.value = .45;
    BodyTwist command{};
    command.angular_radps.z = .45;
    hold.apply(state, intent, command, true);
    ok &= command.linear_mps.x == 0 && command.linear_mps.y == 0;
    state.body_twist_state.body_trans_m.x = .1;
    state.body_twist_state.twist_pos_rad.z = 1.5707963267948966;
    hold.apply(state, intent, command, true);
    ok &= std::abs(command.linear_mps.x) < 1e-12 && std::abs(command.linear_mps.y-.02) < 1e-12;
    ok &= command.angular_radps.z == .45;
    command.linear_mps = {};
    state.body_twist_state.body_trans_m.x = 5;
    hold.apply(state, intent, command, true);
    ok &= std::abs(std::hypot(command.linear_mps.x,command.linear_mps.y)-.03) < 1e-12;
    // Yaw-dominant arcs remain intentional translations, never position holds.
    intent.cmd_vx_mps.value = .01;
    command.linear_mps = {.01, 0, 0};
    hold.apply(state, intent, command, true);
    ok &= command.linear_mps.x == .01 && command.linear_mps.y == 0;
    intent.cmd_vx_mps.value = 0;
    command.linear_mps = {};
    hold.apply(state, intent, command, true);
    ok &= command.linear_mps.x == 0 && command.linear_mps.y == 0;
    // Invalid feedback must not leave a stale world anchor for recovery.
    state.body_twist_state.twist_pos_rad.z = std::numeric_limits<double>::quiet_NaN();
    hold.apply(state, intent, command, true);
    state.body_twist_state.twist_pos_rad.z = 0;
    state.body_twist_state.body_trans_m.x = 10;
    hold.apply(state, intent, command, true);
    ok &= command.linear_mps.x == 0 && command.linear_mps.y == 0;
    state.body_twist_state.body_trans_m.x = 11;
    hold.apply(state, intent, command, false);
    ok &= command.linear_mps.x == 0 && command.linear_mps.y == 0;
    hold.apply(state, intent, command, true);
    ok &= command.linear_mps.x == 0;
    state.bus_ok = false;
    hold.apply(state, intent, command, true);
    state.bus_ok = true;
    state.body_twist_state.body_trans_m.x = 12;
    hold.apply(state, intent, command, true);
    ok &= command.linear_mps.x == 0;
    intent.requested_mode = RobotMode::STAND;
    hold.apply(state, intent, command, true);
    intent.requested_mode = RobotMode::WALK;
    state.body_twist_state.body_trans_m.x = 13;
    hold.apply(state, intent, command, true);
    ok &= command.linear_mps.x == 0;
    SimHardwareBridge simple;
    PhysicsSimBridge physics("127.0.0.1", 1, 5000, PhysicsSimSolverSettings{});
    ok &= !simple.supportsAbsoluteBodyPositionFeedback() && physics.supportsAbsoluteBodyPositionFeedback();
    if (!ok) std::cerr << "FAIL: in-place hold scope, frame, reset or bound\n";
    return ok ? 0 : 1;
}
