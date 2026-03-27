#include "autonomy/motion_arbiter.hpp"

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

autonomy::NavigationUpdate activeNavUpdate() {
    return autonomy::NavigationUpdate{
        .has_intent = true,
        .intent = autonomy::NavigationIntent{
            .mission_id = "mission-1",
            .waypoint_index = 0,
            .target = autonomy::Waypoint{.frame_id = "map", .x_m = 1.0, .y_m = 0.0, .yaw_rad = 0.0},
        },
        .status = autonomy::NavigationStatus::Active,
        .reason = {},
    };
}

bool testPriorityOrder() {
    autonomy::MotionArbiter arbiter;
    const auto nav_update = activeNavUpdate();

    const auto estop = arbiter.select(true, true, true, nav_update);
    if (!expect(estop.source == autonomy::MotionSource::EStop, "estop should have top priority")) {
        return false;
    }

    const auto hold = arbiter.select(false, true, true, nav_update);
    if (!expect(hold.source == autonomy::MotionSource::Hold, "hold should beat recovery and nav")) {
        return false;
    }

    const auto recovery = arbiter.select(false, false, true, nav_update);
    if (!expect(recovery.source == autonomy::MotionSource::Recovery, "recovery should beat nav")) {
        return false;
    }

    const auto nav = arbiter.select(false, false, false, nav_update);
    return expect(nav.source == autonomy::MotionSource::Nav, "nav should win when no higher-priority sources") &&
           expect(nav.allow_motion, "nav source should allow motion");
}

} // namespace

int main() {
    if (!testPriorityOrder()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
