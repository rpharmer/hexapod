#include "joint_feedback_estimator.hpp"

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

bool test_disabled_estimator_is_noop() {
    JointFeedbackEstimator estimator;
    estimator.reset();
    estimator.set_enabled(false);

    RobotState state{};
    state.timestamp_us = TimePointUs{1000};
    estimator.on_hardware_read(state);

    JointTargets targets{};
    targets.leg_states[0].joint_state[0].pos_rad = AngleRad{1.0};
    estimator.on_write(targets);

    estimator.synthesize(state);
    return expect(state.leg_states[0].joint_state[0].pos_rad.value == 0.0,
                  "disabled estimator should not mutate output state");
}

bool test_enabled_estimator_moves_toward_target() {
    JointFeedbackEstimator estimator;
    estimator.reset();
    estimator.set_enabled(true);

    RobotState baseline{};
    baseline.timestamp_us = TimePointUs{1000};
    estimator.on_hardware_read(baseline);

    JointTargets targets{};
    targets.leg_states[0].joint_state[0].pos_rad = AngleRad{1.0};
    estimator.on_write(targets);

    RobotState out = baseline;
    out.timestamp_us = TimePointUs{5000};
    estimator.synthesize(out);

    return expect(out.leg_states[0].joint_state[0].pos_rad.value > 0.0,
                  "enabled estimator should advance synthetic position toward target");
}

bool test_enabled_estimator_preserves_continuity_past_pi() {
    JointFeedbackEstimator estimator;
    estimator.reset();
    estimator.set_enabled(false);

    RobotState baseline{};
    baseline.timestamp_us = TimePointUs{1000};
    baseline.leg_states[0].joint_state[0].pos_rad = AngleRad{3.20};
    estimator.on_hardware_read(baseline);

    estimator.set_enabled(true);

    JointTargets targets{};
    targets.leg_states[0].joint_state[0].pos_rad = AngleRad{3.30};
    estimator.on_write(targets);

    RobotState out = baseline;
    out.timestamp_us = TimePointUs{5000};
    estimator.synthesize(out);

    return expect(out.leg_states[0].joint_state[0].pos_rad.value > 3.0,
                  "software feedback should not wrap continuous joint state back below pi");
}

}  // namespace

int main() {
    if (!test_disabled_estimator_is_noop()) {
        return EXIT_FAILURE;
    }
    if (!test_enabled_estimator_moves_toward_target()) {
        return EXIT_FAILURE;
    }
    if (!test_enabled_estimator_preserves_continuity_past_pi()) {
        return EXIT_FAILURE;
    }

    std::cout << "PASS: JointFeedbackEstimator collaborator behavior" << std::endl;
    return EXIT_SUCCESS;
}
