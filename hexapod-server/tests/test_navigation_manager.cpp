#include "autonomy/navigation_manager.hpp"

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

autonomy::WaypointMission makeMission() {
    autonomy::WaypointMission mission{};
    mission.mission_id = "mission-1";
    mission.waypoints = {
        autonomy::Waypoint{.frame_id = "map", .x_m = 1.0, .y_m = 2.0, .yaw_rad = 0.0},
        autonomy::Waypoint{.frame_id = "map", .x_m = 3.0, .y_m = 4.0, .yaw_rad = 1.0},
    };
    return mission;
}

bool testGeneratesIntentForCurrentWaypoint() {
    autonomy::NavigationManager manager;
    const auto update = manager.planNextIntent(makeMission(), 1, false);
    return expect(update.has_intent, "expected active nav intent") &&
           expect(update.status == autonomy::NavigationStatus::Active, "status should be active") &&
           expect(update.intent.waypoint_index == 1, "waypoint index should match progress");
}

bool testReportsBlockedWithoutIntent() {
    autonomy::NavigationManager manager;
    const auto update = manager.planNextIntent(makeMission(), 0, true);
    return expect(!update.has_intent, "blocked should suppress nav intent") &&
           expect(update.status == autonomy::NavigationStatus::Blocked, "status should be blocked");
}

bool testReportsCompleteWhenDone() {
    autonomy::NavigationManager manager;
    const auto mission = makeMission();
    const auto update = manager.planNextIntent(mission, mission.waypoints.size(), false);
    return expect(!update.has_intent, "complete mission should not emit intent") &&
           expect(update.status == autonomy::NavigationStatus::Complete, "status should be complete");
}

} // namespace

int main() {
    if (!testGeneratesIntentForCurrentWaypoint() ||
        !testReportsBlockedWithoutIntent() ||
        !testReportsCompleteWhenDone()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
