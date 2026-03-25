#include "stability_tracker.hpp"

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

RobotState nominalState() {
    RobotState state{};
    state.timestamp_us = now_us();
    state.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : state.leg_states) {
        leg.joint_state[0].pos_rad = AngleRad{0.0};
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }
    return state;
}

bool testStableWithAllContacts() {
    const RobotState state = nominalState();
    const StabilityAssessment stability = assessStability(state);

    return expect(stability.has_support_polygon, "all contacts should form a support polygon") &&
           expect(stability.com_inside_support_polygon, "nominal stance should keep COM inside support polygon") &&
           expect(stability.support_contact_count == 6, "all contacts should be counted");
}

bool testInvalidSupportWithTwoContacts() {
    RobotState state = nominalState();
    state.foot_contacts = {true, false, true, false, false, false};

    const StabilityAssessment stability = assessStability(state);

    return expect(!stability.has_support_polygon, "fewer than three contacts should not form a support polygon") &&
           expect(!stability.com_inside_support_polygon, "invalid support polygon should report unstable") &&
           expect(stability.support_contact_count == 2, "support contact count should track active contacts");
}

} // namespace

int main() {
    if (!testStableWithAllContacts() ||
        !testInvalidSupportWithTwoContacts()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
