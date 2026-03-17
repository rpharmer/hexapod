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

}  // namespace

namespace status_reporter {

void logStatus(const std::shared_ptr<logging::AsyncLogger>& logger,
               const ControlStatus& status) {
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
}

}  // namespace status_reporter
