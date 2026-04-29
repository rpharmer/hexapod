#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "control_config.hpp"
#include "estimator.hpp"
#include "geometry_config.hpp"
#include "hardware_bridge.hpp"
#include "hexapod-server.hpp"
#include "logger.hpp"
#include "mode_runners.hpp"
#include "navigation_manager.hpp"
#include "replay_logger.hpp"
#include "robot_control.hpp"
#include "runtime_teardown.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "matrix_lidar_local_map_source.hpp"
#include "physics_sim_local_map_source.hpp"
#include "sim_hardware_bridge.hpp"
#include "toml_parser.hpp"

using namespace logging;

static std::atomic<bool> g_exit{false};

namespace {

int telemetryPublishPeriodMsFromRateHz(double publish_rate_hz)
{
  return std::max(1, static_cast<int>(std::lround(1000.0 / std::max(publish_rate_hz, 0.1))));
}

int telemetryGeometryRefreshPeriodMsFromSeconds(double geometry_resend_interval_sec)
{
  return std::max(1, static_cast<int>(std::lround(geometry_resend_interval_sec * 1000.0)));
}

void syncTelemetryCompatibilityFields(ParsedToml& config)
{
  if (config.telemetryHost.empty()) {
    config.telemetryHost = "127.0.0.1";
  }
  config.telemetryUdpHost = config.telemetryHost;
  config.telemetryUdpPort = config.telemetryPort;
  config.telemetryPublishPeriodMs = telemetryPublishPeriodMsFromRateHz(config.telemetryPublishRateHz);
  config.telemetryGeometryRefreshPeriodMs =
      telemetryGeometryRefreshPeriodMsFromSeconds(config.telemetryGeometryResendIntervalSec);
}

std::shared_ptr<AsyncLogger> makeLogger(bool enableFileLogging,
                                        const std::string& logFilePath,
                                        const std::shared_ptr<AsyncLogger>& fallbackLogger = nullptr)
{
  auto logger = std::make_shared<AsyncLogger>("app", LogLevel::Debug, 10000);
  logger->AddSink(
      std::make_shared<LevelFilterSink>(std::make_shared<ConsoleSink>(), LogLevel::Info));
  if (enableFileLogging) {
    try {
      logger->AddSink(
          std::make_shared<LevelFilterSink>(std::make_shared<FileSink>(logFilePath), LogLevel::Debug));
    } catch (const std::exception& ex) {
      if (fallbackLogger) {
        LOG_WARN(fallbackLogger,
                 "Failed to initialize file logging at '",
                 logFilePath,
                 "'. Falling back to console-only logging: ",
                 ex.what());
      }
    }
  }
  return logger;
}

std::unique_ptr<IHardwareBridge> makeHardwareBridge(const ParsedToml& config,
                                                    const std::shared_ptr<AsyncLogger>& logger)
{
  if (config.runtimeMode == "sim") {
    SimHardwareFaultToggles sim_faults{};
    sim_faults.nominal_voltage = static_cast<float>(config.simInitialVoltageV);
    sim_faults.nominal_current = static_cast<float>(config.simInitialCurrentA);
    sim_faults.drop_bus = config.simDropBus;
    sim_faults.low_voltage = config.simLowVoltage;
    sim_faults.high_current = config.simHighCurrent;
    sim_faults.low_voltage_value = std::max(0.0f, sim_faults.nominal_voltage * 0.5f);
    sim_faults.high_current_value =
        std::max(sim_faults.nominal_current * 2.0f, sim_faults.nominal_current + 1.0f);

    const double read_period_s = 1.0 / std::max(config.simResponseRateHz, 1.0);
    return std::make_unique<SimHardwareBridge>(sim_faults, DurationSec{read_period_s},
                                               DurationSec{0.08});
  }

  if (config.runtimeMode == "physics-sim") {
    return std::make_unique<PhysicsSimBridge>(config.physicsSimHost, config.physicsSimPort,
                                              config.busLoopPeriodUs, config.physicsSimSolverIterations,
                                              logger);
  }

  return std::make_unique<SimpleHardwareBridge>(config.serialDevice, config.baudRate,
                                                config.timeout, config.minMaxPulses, logger);
}

void applyTelemetryCliOverrides(ParsedToml& config,
                                const CliOptions& options,
                                const std::shared_ptr<AsyncLogger>& logger)
{
  if (options.telemetryEnabledOverride.has_value()) {
    config.telemetryEnabled = options.telemetryEnabledOverride.value();
  }
  if (options.telemetryHostOverride.has_value()) {
    config.telemetryHost = options.telemetryHostOverride.value();
  }
  if (options.telemetryPortOverride.has_value()) {
    config.telemetryPort = options.telemetryPortOverride.value();
  }
  if (options.telemetryPublishRateHzOverride.has_value()) {
    config.telemetryPublishRateHz = options.telemetryPublishRateHzOverride.value();
  }
  if (options.telemetryGeometryResendIntervalSecOverride.has_value()) {
    config.telemetryGeometryResendIntervalSec =
        options.telemetryGeometryResendIntervalSecOverride.value();
  }

  if (config.telemetryHost.empty()) {
    LOG_WARN(logger, "Runtime.Telemetry.Host effective value was empty, using 127.0.0.1");
    config.telemetryHost = "127.0.0.1";
  }
  if (config.telemetryPort < 1 || config.telemetryPort > 65535) {
    LOG_WARN(logger,
             "Runtime.Telemetry.Port effective value out of range (1..65535): ",
             config.telemetryPort,
             ", using 9870");
    config.telemetryPort = 9870;
  }
  if (config.telemetryPublishRateHz < 0.1 || config.telemetryPublishRateHz > 1000.0) {
    LOG_WARN(logger,
             "Runtime.Telemetry.PublishRateHz effective value out of range (0.1..1000): ",
             config.telemetryPublishRateHz,
             ", using 30.0");
    config.telemetryPublishRateHz = 30.0;
  }
  if (config.telemetryGeometryResendIntervalSec < 0.1 ||
      config.telemetryGeometryResendIntervalSec > 3600.0) {
    LOG_WARN(logger,
             "Runtime.Telemetry.GeometryResendIntervalSec effective value out of range (0.1..3600): ",
             config.telemetryGeometryResendIntervalSec,
             ", using 1.0");
    config.telemetryGeometryResendIntervalSec = 1.0;
  }

  syncTelemetryCompatibilityFields(config);
}

void applyInvestigationCliOverrides(ParsedToml& config, const CliOptions& options)
{
  if (options.investigationDisableTerrainStanceBiasOverride.has_value()) {
    config.investigationDisableTerrainStanceBias =
        options.investigationDisableTerrainStanceBiasOverride.value();
  }
  if (options.investigationDisableTerrainSwingClearanceOverride.has_value()) {
    config.investigationDisableTerrainSwingClearance =
        options.investigationDisableTerrainSwingClearanceOverride.value();
  }
  if (options.investigationDisableTerrainSwingXYNudgeOverride.has_value()) {
    config.investigationDisableTerrainSwingXYNudge =
        options.investigationDisableTerrainSwingXYNudgeOverride.value();
  }
  if (options.investigationDisableStanceTiltLevelingOverride.has_value()) {
    config.investigationDisableStanceTiltLeveling =
        options.investigationDisableStanceTiltLevelingOverride.value();
  }
  if (options.investigationSuppressFusionCorrectionsOverride.has_value()) {
    config.investigationSuppressFusionCorrections =
        options.investigationSuppressFusionCorrectionsOverride.value();
  }
  if (options.investigationSuppressFusionResetsOverride.has_value()) {
    config.investigationSuppressFusionResets =
        options.investigationSuppressFusionResetsOverride.value();
  }
}

} // namespace

void signalHandler(int)
{
  g_exit.store(true);
}

int main(int argc, char** argv)
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  CliOptions options;
  std::string cli_error;
  if (!parseCliOptions(argc, argv, options, cli_error)) {
    std::cerr << "CLI error: " << cli_error << '\n';
    return 1;
  }

  auto bootstrapLogger = makeLogger(false, "");
  SetDefaultLogger(bootstrapLogger);

  const std::string effectiveConfigPath =
      options.configFile.empty() ? std::string("config.txt") : options.configFile;

  ParsedToml config;
  if (!tomlParser(effectiveConfigPath, config)) {
    bootstrapLogger->Flush();
    bootstrapLogger->Stop();
    return 1;
  }

  const std::string effectiveLogPath =
      options.logFilePath.empty() ? config.logFilePath : options.logFilePath;
  const bool enableFileLogging = config.logToFile && !options.consoleOnlyLogging;

  auto logger = makeLogger(enableFileLogging, effectiveLogPath, bootstrapLogger);
  SetDefaultLogger(logger);
  bootstrapLogger->Flush();
  bootstrapLogger->Stop();

  applyTelemetryCliOverrides(config, options, logger);
  applyInvestigationCliOverrides(config, options);

  control_config::ControlConfig control_cfg = control_config::fromParsedToml(config);
  control_cfg.control_loop_trace_enabled = options.traceControlLoop;
  geometry_config::loadFromParsedToml(config);

  LOG_INFO(logger, "Runtime.Mode=", config.runtimeMode);
  LOG_INFO(logger, "Logging.FileEnabled=", enableFileLogging, ", Path=", effectiveLogPath);
  if (config.runtimeMode == "physics-sim") {
    LOG_INFO(logger,
             "Runtime.PhysicsSim.Host=",
             config.physicsSimHost,
             ", Port=",
             config.physicsSimPort,
             ", SolverIterations=",
             config.physicsSimSolverIterations);
  }
  LOG_INFO(logger,
           "Runtime.Telemetry.Enabled=",
           control_cfg.telemetry.enabled,
           ", Host=",
           control_cfg.telemetry.host,
           ", Port=",
           control_cfg.telemetry.port,
           ", PublishRateHz=",
           control_cfg.telemetry.publish_rate_hz,
           ", GeometryResendIntervalSec=",
           control_cfg.telemetry.geometry_resend_interval_sec);
  LOG_INFO(logger,
           "Runtime.ReplayLog.Enabled=",
           control_cfg.replay_log.enabled,
           ", Path=",
           control_cfg.replay_log.file_path);
  LOG_INFO(logger,
           "Runtime.Trace.ControlLoop=",
           control_cfg.control_loop_trace_enabled ? 1 : 0);
  LOG_INFO(logger,
           "Runtime.Investigation.TerrainStanceBiasEnabled=",
           control_cfg.foot_terrain.enable_stance_plane_bias,
           ", TerrainSwingClearanceEnabled=",
           control_cfg.foot_terrain.enable_swing_clearance,
           ", TerrainSwingXYNudgeEnabled=",
           control_cfg.foot_terrain.enable_swing_xy_nudge,
           ", StanceTiltLevelingEnabled=",
           control_cfg.foot_terrain.enable_stance_tilt_leveling,
           ", SuppressFusionCorrections=",
           !control_cfg.fusion.emit_physics_sim_corrections,
           ", SuppressFusionResets=",
           control_cfg.fusion.suppress_fusion_resets);
  LOG_INFO(logger,
           "Runtime.Freshness.CommandRefreshMs=",
           control_cfg.loop_timing.command_refresh_period.count(),
           ", IntentMaxAgeUs=",
           control_cfg.freshness.intent.max_allowed_age_us.value,
           ", EstimatorMaxAgeUs=",
           control_cfg.freshness.estimator.max_allowed_age_us.value,
           ", CommandTimeoutUs=",
           control_cfg.safety.command_timeout_us.value);

  auto hw = makeHardwareBridge(config, logger);
  auto navigation_manager =
      std::make_unique<NavigationManager>(control_cfg.local_map, control_cfg.local_planner, control_cfg.nav_bridge);
  navigation_manager->addObservationSource(std::make_shared<MatrixLidarLocalMapObservationSource>());
  if (config.runtimeMode == "physics-sim") {
    if (auto* physics_sim_bridge = dynamic_cast<PhysicsSimBridge*>(hw.get())) {
      navigation_manager->addObservationSource(std::make_shared<PhysicsSimLocalMapObservationSource>(
          *physics_sim_bridge, std::max(0.02, control_cfg.local_map.resolution_m * 0.5)));
      LOG_INFO(logger, "Navigation.LocalMap.Sources=matrix-lidar,physics-sim-footprints");
    }
  } else {
    LOG_INFO(logger, "Navigation.LocalMap.Sources=matrix-lidar");
  }

  std::unique_ptr<IEstimator> estimator;
  if (config.runtimeMode == "physics-sim") {
    estimator = std::make_unique<PhysicsSimEstimator>();
  } else {
    estimator = std::make_unique<SimpleEstimator>();
  }
  auto replay_logger = control_cfg.replay_log.enabled
                           ? replay::makeFileReplayLogger(control_cfg.replay_log.file_path, logger)
                           : replay::makeNoopReplayLogger();
  RobotControl robot(std::move(hw), std::move(estimator), logger, control_cfg, std::move(replay_logger));

  if (!robot.init()) {
    return 1;
  }

  robot.setNavigationManager(std::move(navigation_manager));

  robot.start();

  int runner_rc = 0;
  if (options.mode == ServerMode::Scenario) {
    const ScenarioRunner runner;
    runner_rc = runner.run(robot, options, logger);
  } else {
    const InteractiveRunner runner;
    runner_rc = runner.run(robot, options, control_cfg, logger, g_exit);
  }

  app::runRuntimeTeardown(logger, [&robot]() { robot.stop(); }, runner_rc);

  logger->Flush();
  logger->Stop();

  return runner_rc;
}

bool tomlParser(std::string filename, ParsedToml& out)
{
  const TomlParser parser(GetDefaultLogger());
  return parser.parse(filename, out);
}
