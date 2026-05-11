#include "servo_observer.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool ok, const char* msg) {
    if (!ok) {
        std::cerr << "FAIL: " << msg << '\n';
    }
    return ok;
}

} // namespace

int main() {
    ServoObserverConfig cfg{};
    cfg.positive_rate_limit_radps = 1.0;
    cfg.negative_rate_limit_radps = 1.0;
    cfg.lag_tau_s = 0.0;
    cfg.deadband_rad = 0.0;
    ServoObserver observer{cfg};

    JointTargets command{};
    command.leg_states[0].joint_state[0].pos_rad = AngleRad{1.0};
    observer.update(command, 0.1);
    const RobotState& first = observer.state();
    if (!expect(first.leg_states[0].joint_state[0].pos_rad.value == 1.0,
                "first update should initialize to command") ||
        !expect(first.joint_state_quality[0].source == JointStateSource::ObserverEstimate,
                "observer state should be marked as observer estimate")) {
        return EXIT_FAILURE;
    }

    command.leg_states[0].joint_state[0].pos_rad = AngleRad{2.0};
    observer.update(command, 0.1);
    const RobotState& second = observer.state();
    if (!expect(second.leg_states[0].joint_state[0].pos_rad.value > 1.09 &&
                    second.leg_states[0].joint_state[0].pos_rad.value < 1.11,
                "observer should rate-limit motion toward command") ||
        !expect(second.joint_state_quality[0].velocity_valid,
                "observer should mark velocity valid after dt update")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
