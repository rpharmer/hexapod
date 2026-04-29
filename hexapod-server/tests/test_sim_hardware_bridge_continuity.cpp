#include "sim_hardware_bridge.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    SimHardwareBridge bridge{};
    if (!expect(bridge.init(), "sim hardware bridge should initialize")) {
        return EXIT_FAILURE;
    }

    JointTargets prime{};
    for (auto& leg : prime.leg_states) {
        leg.joint_state[COXA].pos_rad = AngleRad{2.8};
    }
    if (!expect(bridge.write(prime), "bridge should accept a priming target")) {
        return EXIT_FAILURE;
    }

    double previous = -1e9;
    for (int step = 0; step < 8; ++step) {
        RobotState state{};
        if (!expect(bridge.read(state), "bridge should produce a readable state")) {
            return EXIT_FAILURE;
        }
        previous = state.leg_states[0].joint_state[COXA].pos_rad.value;
    }

    JointTargets target{};
    for (auto& leg : target.leg_states) {
        leg.joint_state[COXA].pos_rad = AngleRad{4.0};
    }
    if (!expect(bridge.write(target), "bridge should accept a continuous target")) {
        return EXIT_FAILURE;
    }

    for (int step = 0; step < 32; ++step) {
        RobotState state{};
        if (!expect(bridge.read(state), "bridge should produce a readable state")) {
            return EXIT_FAILURE;
        }

        const double current = state.leg_states[0].joint_state[COXA].pos_rad.value;
        if (!expect(current >= previous - 1e-9,
                    "sim bridge joint positions should not wrap backward while converging")) {
            std::cerr << " step=" << step << " prev=" << previous << " current=" << current << '\n';
            return EXIT_FAILURE;
        }
        previous = current;
    }

    if (!expect(previous > 3.0, "sim bridge should be able to move past the principal-angle boundary")) {
        return EXIT_FAILURE;
    }

    std::cout << "test_sim_hardware_bridge_continuity ok\n";
    return EXIT_SUCCESS;
}
