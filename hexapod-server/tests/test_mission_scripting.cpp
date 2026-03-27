#include "autonomy/mission_scripting.hpp"

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

bool nearlyEqual(double lhs, double rhs) {
    return std::fabs(lhs - rhs) < 1e-9;
}

bool testParsesValidMission() {
    const std::string script =
        "mission_id=demo\n"
        "waypoint,map,1.0,2.0,0.25\n"
        "waypoint,odom,2.0,3.0,0.5\n";

    autonomy::MissionScripting parser;
    const auto result = parser.parseWaypointMission(script);

    if (!expect(result.ok, "valid script should parse")) {
        return false;
    }
    if (!expect(result.mission.mission_id == "demo", "mission_id should match")) {
        return false;
    }
    if (!expect(result.mission.waypoints.size() == 2, "two waypoints should parse")) {
        return false;
    }
    return expect(nearlyEqual(result.mission.waypoints[0].yaw_rad, 0.25),
                  "yaw should parse as rad");
}

bool testRejectsUnknownFrame() {
    const std::string script =
        "mission_id=demo\n"
        "waypoint,base_link,1.0,2.0,0.25\n";

    autonomy::MissionScripting parser;
    const auto result = parser.parseWaypointMission(script);

    return expect(!result.ok, "unsupported frame should fail") &&
           expect(result.error.find("unsupported frame_id") != std::string::npos,
                  "error should mention unsupported frame");
}

bool testRejectsMissingMissionId() {
    const std::string script = "waypoint,map,1.0,2.0,0.25\n";

    autonomy::MissionScripting parser;
    const auto result = parser.parseWaypointMission(script);

    return expect(!result.ok, "missing mission id should fail") &&
           expect(result.error == "missing mission_id", "error should be explicit");
}

} // namespace

int main() {
    if (!testParsesValidMission() ||
        !testRejectsUnknownFrame() ||
        !testRejectsMissingMissionId()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
