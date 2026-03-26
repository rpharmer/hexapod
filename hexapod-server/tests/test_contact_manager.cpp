#include "contact_manager.hpp"

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

RuntimeGaitPolicy makePolicy() {
    RuntimeGaitPolicy policy{};
    policy.cadence_hz = FrequencyHz{1.0};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        policy.per_leg[leg].duty_cycle = 0.6;
        policy.per_leg[leg].step_length_m = LengthM{0.08};
        policy.per_leg[leg].swing_height_m = LengthM{0.03};
    }
    return policy;
}

} // namespace

int main() {
    ContactManager manager;

    RobotState state{};
    GaitState scheduled{};
    RuntimeGaitPolicy policy = makePolicy();

    for (int leg = 0; leg < kNumLegs; ++leg) {
        scheduled.phase[leg] = 0.8;
        scheduled.in_stance[leg] = false;
        state.foot_contacts[leg] = false;
    }

    (void)manager.update(state, scheduled, policy);

    state.foot_contacts[0] = true;
    ContactManagerOutput touchdown = manager.update(state, scheduled, policy);

    if (!expect(touchdown.touchdown_detected[0], "contact rise in swing should trigger touchdown detection") ||
        !expect(touchdown.managed_gait.in_stance[0], "touchdown should complete swing and force stance")) {
        return EXIT_FAILURE;
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        scheduled.in_stance[leg] = true;
        state.foot_contacts[leg] = true;
    }

    state.foot_contacts[0] = false;
    state.foot_contacts[1] = false;

    ContactManagerOutput degraded = manager.update(state, scheduled, policy);
    degraded = manager.update(state, scheduled, degraded.managed_policy);
    degraded = manager.update(state, scheduled, degraded.managed_policy);

    if (!expect(degraded.derating_requested, "persistent slip/missed touchdown should request gait derating") ||
        !expect(degraded.managed_policy.cadence_hz.value < policy.cadence_hz.value,
                "derating should reduce cadence") ||
        !expect(degraded.managed_policy.per_leg[0].swing_height_m.value > policy.per_leg[0].swing_height_m.value,
                "missed touchdown should activate foot-search swing lift")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
