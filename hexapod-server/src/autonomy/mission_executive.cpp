#include "autonomy/mission_executive.hpp"

namespace autonomy {

MissionState MissionExecutive::state() const {
    return state_;
}

const WaypointMission* MissionExecutive::activeMission() const {
    if (mission_.mission_id.empty()) {
        return nullptr;
    }
    return &mission_;
}

MissionProgress MissionExecutive::currentProgress() const {
    return MissionProgress{
        .mission_id = mission_.mission_id,
        .completed_waypoints = next_waypoint_index_,
        .total_waypoints = mission_.waypoints.size(),
    };
}

MissionEvent MissionExecutive::loadMission(const WaypointMission& mission) {
    if (state_ != MissionState::Idle && state_ != MissionState::Complete && state_ != MissionState::Aborted) {
        return reject("cannot load mission while active");
    }
    if (mission.mission_id.empty()) {
        return reject("mission_id is required");
    }
    if (mission.waypoints.empty()) {
        return reject("mission requires at least one waypoint");
    }

    mission_ = mission;
    next_waypoint_index_ = 0;
    state_ = MissionState::Ready;
    return accept();
}

MissionEvent MissionExecutive::start() {
    if (state_ != MissionState::Ready) {
        return reject("start only allowed from READY");
    }
    state_ = MissionState::Exec;
    return accept();
}

MissionEvent MissionExecutive::pause() {
    if (state_ != MissionState::Exec) {
        return reject("pause only allowed from EXEC");
    }
    state_ = MissionState::Paused;
    return accept();
}

MissionEvent MissionExecutive::resume() {
    if (state_ != MissionState::Paused) {
        return reject("resume only allowed from PAUSED");
    }
    state_ = MissionState::Exec;
    return accept();
}

MissionEvent MissionExecutive::abort(const std::string& reason) {
    if (state_ == MissionState::Idle) {
        return reject("abort not allowed from IDLE");
    }
    state_ = MissionState::Aborted;
    return MissionEvent{
        .accepted = true,
        .state = state_,
        .reason = reason,
        .progress = MissionProgress{
            .mission_id = mission_.mission_id,
            .completed_waypoints = next_waypoint_index_,
            .total_waypoints = mission_.waypoints.size(),
        },
    };
}

MissionEvent MissionExecutive::markWaypointReached() {
    if (state_ != MissionState::Exec) {
        return reject("waypoint completion only allowed from EXEC");
    }
    if (next_waypoint_index_ < mission_.waypoints.size()) {
        ++next_waypoint_index_;
    }

    if (next_waypoint_index_ >= mission_.waypoints.size()) {
        state_ = MissionState::Complete;
    }

    return accept();
}

MissionEvent MissionExecutive::onRecoveryDecision(const RecoveryDecision& decision) {
    if (state_ != MissionState::Exec && state_ != MissionState::Paused) {
        return reject("recovery decisions only allowed from EXEC or PAUSED");
    }

    if (!decision.mission_should_abort) {
        return accept();
    }

    const std::string reason = decision.reason.empty() ? "recovery escalation abort" : decision.reason;
    return abort(reason);
}

MissionEvent MissionExecutive::reject(const std::string& reason) const {
    return MissionEvent{
        .accepted = false,
        .state = state_,
        .reason = reason,
        .progress = MissionProgress{
            .mission_id = mission_.mission_id,
            .completed_waypoints = next_waypoint_index_,
            .total_waypoints = mission_.waypoints.size(),
        },
    };
}

MissionEvent MissionExecutive::accept() {
    return MissionEvent{
        .accepted = true,
        .state = state_,
        .reason = {},
        .progress = MissionProgress{
            .mission_id = mission_.mission_id,
            .completed_waypoints = next_waypoint_index_,
            .total_waypoints = mission_.waypoints.size(),
        },
    };
}

} // namespace autonomy
