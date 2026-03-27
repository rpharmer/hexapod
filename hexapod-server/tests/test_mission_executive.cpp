#include "autonomy/mission_executive.hpp"

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
    mission.mission_id = "mission-a";
    mission.waypoints = {
        autonomy::Waypoint{.frame_id = "map", .x_m = 0.0, .y_m = 0.0, .yaw_rad = 0.0},
        autonomy::Waypoint{.frame_id = "map", .x_m = 1.0, .y_m = 1.0, .yaw_rad = 0.5},
    };
    return mission;
}

bool testHappyPathToComplete() {
    autonomy::MissionExecutive executive;

    if (!expect(executive.loadMission(makeMission()).accepted, "load mission should succeed")) {
        return false;
    }
    if (!expect(executive.start().accepted, "start should succeed")) {
        return false;
    }
    if (!expect(executive.markWaypointReached().accepted, "first waypoint should complete")) {
        return false;
    }
    const auto final_event = executive.markWaypointReached();
    return expect(final_event.accepted, "second waypoint should complete") &&
           expect(final_event.state == autonomy::MissionState::Complete,
                  "mission should transition to COMPLETE");
}

bool testPauseResumeAndAbort() {
    autonomy::MissionExecutive executive;

    if (!expect(executive.loadMission(makeMission()).accepted, "load mission should succeed")) {
        return false;
    }
    if (!expect(executive.start().accepted, "start should succeed")) {
        return false;
    }
    if (!expect(executive.pause().accepted, "pause should succeed from EXEC")) {
        return false;
    }
    if (!expect(executive.resume().accepted, "resume should succeed from PAUSED")) {
        return false;
    }

    const auto aborted = executive.abort("operator abort");
    return expect(aborted.accepted, "abort should succeed from active mission") &&
           expect(aborted.state == autonomy::MissionState::Aborted,
                  "mission should transition to ABORTED");
}

bool testInvalidTransitionRejected() {
    autonomy::MissionExecutive executive;
    const auto pause_event = executive.pause();
    return expect(!pause_event.accepted, "pause should fail from IDLE") &&
           expect(pause_event.reason == "pause only allowed from EXEC",
                  "invalid transition should provide explicit reason");
}

} // namespace

int main() {
    if (!testHappyPathToComplete() ||
        !testPauseResumeAndAbort() ||
        !testInvalidTransitionRejected()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
