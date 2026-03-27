#pragma once

#include "autonomy/mission_types.hpp"

#include <cstdint>
#include <string>

namespace autonomy {

enum class MissionState {
    Idle,
    Ready,
    Exec,
    Paused,
    Aborted,
    Complete,
};

struct MissionEvent {
    bool accepted{false};
    MissionState state{MissionState::Idle};
    std::string reason{};
    MissionProgress progress{};
};

class MissionExecutive {
public:
    MissionExecutive() = default;

    MissionState state() const;
    const WaypointMission* activeMission() const;

    MissionEvent loadMission(const WaypointMission& mission);
    MissionEvent start();
    MissionEvent pause();
    MissionEvent resume();
    MissionEvent abort(const std::string& reason);
    MissionEvent markWaypointReached();

private:
    MissionEvent reject(const std::string& reason) const;
    MissionEvent accept();

    MissionState state_{MissionState::Idle};
    WaypointMission mission_{};
    uint64_t next_waypoint_index_{0};
};

} // namespace autonomy
