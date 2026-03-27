#pragma once

#include "autonomy/mission_types.hpp"

#include <string>

namespace autonomy {

struct MissionParseResult {
    bool ok{false};
    WaypointMission mission{};
    std::string error{};
};

class MissionScripting {
public:
    MissionParseResult parseWaypointMission(const std::string& source) const;
};

} // namespace autonomy
