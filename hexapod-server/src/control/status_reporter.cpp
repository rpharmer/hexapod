#include "status_reporter.hpp"

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
