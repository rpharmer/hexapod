#include "autonomy/mission_scripting.hpp"

#include <sstream>

namespace autonomy {

namespace {

bool isSupportedFrame(const std::string& frame_id) {
    return frame_id == "map" || frame_id == "odom";
}

} // namespace

MissionParseResult MissionScripting::parseWaypointMission(const std::string& source) const {
    MissionParseResult result{};

    std::istringstream stream(source);
    std::string line;
    std::string mission_id;
    std::vector<Waypoint> waypoints;

    uint64_t line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line.empty()) {
            continue;
        }

        if (line.rfind("mission_id=", 0) == 0) {
            mission_id = line.substr(std::string("mission_id=").size());
            continue;
        }

        if (line.rfind("waypoint,", 0) == 0) {
            std::istringstream line_stream(line);
            std::string token;

            std::getline(line_stream, token, ',');
            Waypoint waypoint{};

            if (!std::getline(line_stream, waypoint.frame_id, ',')) {
                result.error = "line " + std::to_string(line_number) + ": missing frame_id";
                return result;
            }
            if (!isSupportedFrame(waypoint.frame_id)) {
                result.error = "line " + std::to_string(line_number) + ": unsupported frame_id";
                return result;
            }

            std::string x_token;
            std::string y_token;
            std::string yaw_token;
            if (!std::getline(line_stream, x_token, ',') ||
                !std::getline(line_stream, y_token, ',') ||
                !std::getline(line_stream, yaw_token, ',')) {
                result.error = "line " + std::to_string(line_number) + ": expected x_m,y_m,yaw_rad";
                return result;
            }

            try {
                waypoint.x_m = std::stod(x_token);
                waypoint.y_m = std::stod(y_token);
                waypoint.yaw_rad = std::stod(yaw_token);
            } catch (...) {
                result.error = "line " + std::to_string(line_number) + ": invalid numeric field";
                return result;
            }

            waypoints.push_back(waypoint);
            continue;
        }

        result.error = "line " + std::to_string(line_number) + ": unknown directive";
        return result;
    }

    if (mission_id.empty()) {
        result.error = "missing mission_id";
        return result;
    }
    if (waypoints.empty()) {
        result.error = "mission has no waypoints";
        return result;
    }

    result.ok = true;
    result.mission.mission_id = mission_id;
    result.mission.waypoints = std::move(waypoints);
    return result;
}

} // namespace autonomy
