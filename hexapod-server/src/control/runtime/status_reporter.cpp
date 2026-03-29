#include "status_reporter.hpp"
#include "autonomy/mission_executive.hpp"

namespace {

const char* toString(RobotMode mode) {
    switch (mode) {
        case RobotMode::SAFE_IDLE: return "SAFE_IDLE";
        case RobotMode::HOMING: return "HOMING";
        case RobotMode::STAND: return "STAND";
        case RobotMode::WALK: return "WALK";
        case RobotMode::FAULT: return "FAULT";
    }
    return "UNKNOWN";
}

const char* toString(FaultCode code) {
    switch (code) {
        case FaultCode::NONE: return "NONE";
        case FaultCode::BUS_TIMEOUT: return "BUS_TIMEOUT";
        case FaultCode::ESTOP: return "ESTOP";
        case FaultCode::TIP_OVER: return "TIP_OVER";
        case FaultCode::ESTIMATOR_INVALID: return "ESTIMATOR_INVALID";
        case FaultCode::MOTOR_FAULT: return "MOTOR_FAULT";
        case FaultCode::JOINT_LIMIT: return "JOINT_LIMIT";
        case FaultCode::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
    }
    return "UNKNOWN";
}

const char* toString(BridgeError error) {
    switch (error) {
        case BridgeError::None: return "none";
        case BridgeError::NotReady: return "not_ready";
        case BridgeError::TransportFailure: return "transport_failure";
        case BridgeError::ProtocolFailure: return "protocol_failure";
        case BridgeError::Timeout: return "timeout";
        case BridgeError::Unsupported: return "unsupported";
    }
    return "unknown";
}

const char* toString(BridgeFailurePhase phase) {
    switch (phase) {
        case BridgeFailurePhase::None: return "none";
        case BridgeFailurePhase::Readiness: return "readiness";
        case BridgeFailurePhase::CapabilityNegotiation: return "capability_negotiation";
        case BridgeFailurePhase::CommandTransport: return "command_transport";
        case BridgeFailurePhase::CommandResponse: return "command_response";
        case BridgeFailurePhase::CommandDecode: return "command_decode";
        case BridgeFailurePhase::CommandExecution: return "command_execution";
        case BridgeFailurePhase::Initialization: return "initialization";
    }
    return "unknown";
}

const char* toString(BridgeFailureDomain domain) {
    switch (domain) {
        case BridgeFailureDomain::None: return "none";
        case BridgeFailureDomain::CapabilityProtocol: return "capability_protocol";
        case BridgeFailureDomain::TransportLink: return "transport_link";
        case BridgeFailureDomain::CommandProtocol: return "command_protocol";
    }
    return "unknown";
}

const char* toString(GaitType gait) {
    switch (gait) {
        case GaitType::TRIPOD: return "TRIPOD";
        case GaitType::RIPPLE: return "RIPPLE";
        case GaitType::WAVE: return "WAVE";
    }
    return "UNKNOWN";
}

const char* toStringDynamicRegion(uint8_t region) {
    switch (region) {
        case 0: return "ARC";
        case 1: return "PIVOT";
        case 2: return "REORIENTATION";
        default: return "UNKNOWN";
    }
}

const char* toStringTurnMode(uint8_t mode) {
    switch (mode) {
        case 0: return "CRAB";
        case 1: return "IN_PLACE";
        default: return "UNKNOWN";
    }
}

const char* toStringFallbackStage(uint8_t stage) {
    switch (stage) {
        case 0: return "NONE";
        case 1: return "STABILITY";
        case 2: return "DEGRADED_LOCOMOTION";
        case 3: return "SAFE_STOP";
        case 4: return "FAULT_HOLD";
        default: return "UNKNOWN";
    }
}

const char* toStringMissionState(uint8_t state) {
    switch (static_cast<autonomy::MissionState>(state)) {
        case autonomy::MissionState::Idle: return "IDLE";
        case autonomy::MissionState::Ready: return "READY";
        case autonomy::MissionState::Exec: return "EXEC";
        case autonomy::MissionState::Paused: return "PAUSED";
        case autonomy::MissionState::Aborted: return "ABORTED";
        case autonomy::MissionState::Complete: return "COMPLETE";
    }
    return "UNKNOWN";
}

}  // namespace

namespace status_reporter {

void logStatus(const std::shared_ptr<logging::AsyncLogger>& logger,
               const ControlStatus& status,
               const std::optional<BridgeCommandResultMetadata>& bridge_result) {
    if (!logger) {
        return;
    }

    LOG_INFO(
        logger,
        "[diag] mode=", toString(status.active_mode),
        " est=", (status.estimator_valid ? "ok" : "bad"),
        " bus=", (status.bus_ok ? "ok" : "bad"),
        " fault=", toString(status.active_fault),
        " loops=", status.loop_counter);

    if (status.dynamic_gait.valid) {
        LOG_INFO(
            logger,
            "[diag.gait] family=", toString(status.dynamic_gait.gait_family),
            " region=", toStringDynamicRegion(status.dynamic_gait.region),
            " turn=", toStringTurnMode(status.dynamic_gait.turn_mode),
            " fallback=", toStringFallbackStage(status.dynamic_gait.fallback_stage),
            " cadence_hz=", status.dynamic_gait.cadence_hz,
            " reach_util=", status.dynamic_gait.reach_utilization,
            " env_speed_max=", status.dynamic_gait.envelope_max_speed_normalized,
            " env_yaw_max=", status.dynamic_gait.envelope_max_yaw_normalized,
            " env_max_rp_rad=", status.dynamic_gait.envelope_max_roll_pitch_rad,
            " env_tripod=", (status.dynamic_gait.envelope_allow_tripod ? "yes" : "no"),
            " suppress_stride=", (status.dynamic_gait.suppress_stride_progression ? "yes" : "no"),
            " suppress_turn=", (status.dynamic_gait.suppress_turning ? "yes" : "no"),
            " prioritize_stability=", (status.dynamic_gait.prioritize_stability ? "yes" : "no"));
    }

    if (status.autonomy.enabled) {
        LOG_INFO(
            logger,
            "[diag.autonomy] step_ok=", (status.autonomy.step_ok ? "yes" : "no"),
            " blocked=", (status.autonomy.blocked ? "yes" : "no"),
            " no_progress=", (status.autonomy.no_progress ? "yes" : "no"),
            " recovery_active=", (status.autonomy.recovery_active ? "yes" : "no"),
            " motion_allowed=", (status.autonomy.motion_allowed ? "yes" : "no"),
            " locomotion_sent=", (status.autonomy.locomotion_sent ? "yes" : "no"),
            " mission_loaded=", (status.autonomy.mission_loaded ? "yes" : "no"),
            " mission_running=", (status.autonomy.mission_running ? "yes" : "no"),
            " mission_state=", toStringMissionState(status.autonomy.mission_state),
            " mission_progress=", status.autonomy.mission_completed_waypoints, "/",
            status.autonomy.mission_total_waypoints);
        LOG_INFO(
            logger,
            "[diag.autonomy.stages] navigation=", (status.autonomy.stages.navigation ? "yes" : "no"),
            " localization=", (status.autonomy.stages.localization ? "yes" : "no"),
            " world_model=", (status.autonomy.stages.world_model ? "yes" : "no"),
            " traversability=", (status.autonomy.stages.traversability ? "yes" : "no"),
            " global_planner=", (status.autonomy.stages.global_planner ? "yes" : "no"),
            " local_planner=", (status.autonomy.stages.local_planner ? "yes" : "no"),
            " progress_monitor=", (status.autonomy.stages.progress_monitor ? "yes" : "no"),
            " recovery_manager=", (status.autonomy.stages.recovery_manager ? "yes" : "no"),
            " motion_arbiter=", (status.autonomy.stages.motion_arbiter ? "yes" : "no"),
            " locomotion_interface=", (status.autonomy.stages.locomotion_interface ? "yes" : "no"));
    }

    if (bridge_result.has_value() && bridge_result->error != BridgeError::None) {
        LOG_WARN(logger,
                 "[diag.bridge] error=", toString(bridge_result->error),
                 " phase=", toString(bridge_result->phase),
                 " domain=", toString(bridge_result->domain),
                 " retryable=", (bridge_result->retryable ? "yes" : "no"),
                 " command_code=", static_cast<unsigned>(bridge_result->command_code),
                 " requested_caps=", static_cast<unsigned>(bridge_result->requested_capabilities),
                 " negotiated_caps=", static_cast<unsigned>(bridge_result->negotiated_capabilities));
    }
}

}  // namespace status_reporter
