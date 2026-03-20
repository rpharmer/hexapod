#include "control_config.hpp"

#include <algorithm>

#include "hexapod-server.hpp"

namespace control_config {

ControlConfig fromParsedToml(const ParsedToml& config) {
    ControlConfig parsed{};
    parsed.loop_timing.bus_loop_period = std::chrono::microseconds{config.busLoopPeriodUs};
    parsed.loop_timing.estimator_loop_period = std::chrono::microseconds{config.estimatorLoopPeriodUs};
    parsed.loop_timing.control_loop_period = std::chrono::microseconds{config.controlLoopPeriodUs};
    parsed.loop_timing.safety_loop_period = std::chrono::microseconds{config.safetyLoopPeriodUs};
    parsed.loop_timing.diagnostics_period = std::chrono::milliseconds{config.diagnosticsPeriodMs};
    parsed.loop_timing.command_refresh_period = std::chrono::milliseconds{config.commandRefreshPeriodMs};
    parsed.loop_timing.stand_settling_delay = std::chrono::milliseconds{config.standSettlingDelayMs};

    parsed.safety.max_tilt_rad = AngleRad{config.maxTiltRad};
    parsed.safety.command_timeout_us = DurationUs{config.commandTimeoutUs};
    parsed.safety.min_bus_voltage_v = static_cast<float>(config.minBusVoltageV);
    parsed.safety.max_bus_current_a = static_cast<float>(config.maxBusCurrentA);
    parsed.safety.min_foot_contacts = config.minFootContacts;
    parsed.safety.max_foot_contacts = config.maxFootContacts;

    parsed.gait.fallback_speed_mag = LinearRateMps{config.fallbackSpeedMag};
    return parsed;
}

} // namespace control_config
