#include "robot_control.hpp"

namespace {

std::unique_ptr<telemetry::ITelemetryPublisher> makeRuntimeTelemetryPublisher(
    const control_config::ControlConfig& config,
    const std::shared_ptr<logging::AsyncLogger>& logger) {
    if (!config.telemetry.enabled) {
        return telemetry::makeNoopTelemetryPublisher();
    }

    telemetry::TelemetryPublisherConfig telemetry_config{};
    telemetry_config.enabled = config.telemetry.enabled;
    telemetry_config.udp_host = config.telemetry.udp_host;
    telemetry_config.udp_port = config.telemetry.udp_port;
    telemetry_config.publish_period_ms = static_cast<int>(config.telemetry.publish_period.count());
    telemetry_config.geometry_refresh_period_ms =
        static_cast<int>(config.telemetry.geometry_refresh_period.count());
    return telemetry::makeUdpTelemetryPublisher(telemetry_config, logger);
}

} // namespace

RobotControl::RobotControl(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger,
                           control_config::ControlConfig config)
    : config_(config),
      runtime_(std::move(hw),
               std::move(estimator),
               logger,
               config_,
               makeRuntimeTelemetryPublisher(config_, logger)),
      logger_(std::move(logger)) {}

RobotControl::~RobotControl() {
    stop();
}

bool RobotControl::init() {
    return runtime_.init();
}

void RobotControl::start() {
    if (running_.exchange(true)) return;

    if (logger_) {
        LOG_INFO(logger_, "Starting robot control loops");
    }

    runtime_.startTelemetry();

    loops_.start(
        {
            {config_.loop_timing.bus_loop_period, [this]() { runtime_.busStep(); }},
            {config_.loop_timing.estimator_loop_period, [this]() { runtime_.estimatorStep(); }},
            {config_.loop_timing.control_loop_period, [this]() { runtime_.controlStep(); }},
            {config_.loop_timing.safety_loop_period, [this]() { runtime_.safetyStep(); }},
            {config_.loop_timing.diagnostics_period, [this]() { runtime_.diagnosticsStep(); }},
        },
        running_);
}

void RobotControl::stop() {
    if (!running_.exchange(false)) return;

    if (logger_) {
        LOG_INFO(logger_, "Stopping robot control loops");
    }

    loops_.stop();
}

void RobotControl::setMotionIntent(const MotionIntent& intent) {
    runtime_.setMotionIntent(intent);
}

bool RobotControl::setSimFaultToggles(const SimHardwareFaultToggles& toggles) {
    return runtime_.setSimFaultToggles(toggles);
}

ControlStatus RobotControl::getStatus() const {
    return runtime_.getStatus();
}
