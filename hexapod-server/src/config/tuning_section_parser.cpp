#include "tuning_section_parser.hpp"

#include "config_validation.hpp"
#include "control_config.hpp"
#include "logger.hpp"

using namespace logging;

namespace tuning_section_parser {

void parseTuningSection(const toml::value& root,
                        ParsedToml& out,
                        std::shared_ptr<logging::AsyncLogger> logger)
{
  out.busLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.BusLoopPeriodUs", control_config::kDefaultBusLoopPeriodUs, 500, 50000, "tuning", logger);
  out.estimatorLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.EstimatorLoopPeriodUs", control_config::kDefaultEstimatorLoopPeriodUs,
      500, 50000, "tuning", logger);
  out.controlLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.ControlLoopPeriodUs", control_config::kDefaultControlLoopPeriodUs, 500, 50000,
      "tuning", logger);
  out.safetyLoopPeriodUs = config_validation::parseIntWithFallback(
      root, "Tuning.SafetyLoopPeriodUs", control_config::kDefaultSafetyLoopPeriodUs, 500, 50000,
      "tuning", logger);
  out.diagnosticsPeriodMs = config_validation::parseIntWithFallback(
      root, "Tuning.DiagnosticsPeriodMs", control_config::kDefaultDiagnosticsPeriodMs, 100, 10000,
      "tuning", logger);
  out.commandRefreshPeriodMs = config_validation::parseIntWithFallback(
      root, "Tuning.CommandRefreshPeriodMs", control_config::kDefaultCommandRefreshPeriodMs, 10,
      1000, "tuning", logger);
  out.standSettlingDelayMs = config_validation::parseIntWithFallback(
      root, "Tuning.StandSettlingDelayMs", control_config::kDefaultStandSettlingDelayMs, 0, 10000,
      "tuning", logger);
  out.maxTiltRad = config_validation::parseDoubleWithFallback(
      root, "Tuning.MaxTiltRad", control_config::kDefaultMaxTiltRad.value, 0.1, 1.5, "tuning", logger);
  out.commandTimeoutUs = config_validation::parseU64WithFallback(
      root, "Tuning.CommandTimeoutUs", control_config::kDefaultCommandTimeoutUs.value, 10000,
      2000000, "tuning", logger);
  out.estimatorMaxAgeUs = config_validation::parseU64WithFallback(
      root, "Tuning.EstimatorMaxAgeUs", control_config::kDefaultEstimatorMaxAgeUs.value, 1000,
      2000000, "tuning", logger);
  out.intentMaxAgeUs = config_validation::parseU64WithFallback(
      root, "Tuning.IntentMaxAgeUs", control_config::kDefaultIntentMaxAgeUs.value, 1000, 2000000,
      "tuning", logger);
  out.estimatorRequireTimestamp =
      config_validation::parseBoolWithFallback(root, "Tuning.EstimatorRequireTimestamp", true);
  out.estimatorRequireSampleId =
      config_validation::parseBoolWithFallback(root, "Tuning.EstimatorRequireSampleId", true);
  out.estimatorRequireMonotonicSampleId = config_validation::parseBoolWithFallback(
      root, "Tuning.EstimatorRequireMonotonicSampleId", true);
  out.intentRequireTimestamp =
      config_validation::parseBoolWithFallback(root, "Tuning.IntentRequireTimestamp", true);
  out.intentRequireSampleId =
      config_validation::parseBoolWithFallback(root, "Tuning.IntentRequireSampleId", true);
  out.intentRequireMonotonicSampleId =
      config_validation::parseBoolWithFallback(root, "Tuning.IntentRequireMonotonicSampleId", true);
  out.fallbackSpeedMag = config_validation::parseDoubleWithFallback(
      root, "Tuning.FallbackSpeedMag", control_config::kDefaultFallbackSpeedMag.value, 0.0, 1.0,
      "tuning", logger);
  out.minBusVoltageV = config_validation::parseDoubleWithFallback(
      root, "Tuning.MinBusVoltageV", control_config::kDefaultMinBusVoltageV, 5.0, 24.0, "tuning", logger);
  out.maxBusCurrentA = config_validation::parseDoubleWithFallback(
      root, "Tuning.MaxBusCurrentA", control_config::kDefaultMaxBusCurrentA, 0.1, 120.0, "tuning", logger);
  out.minFootContacts = config_validation::parseIntWithFallback(
      root, "Tuning.MinFootContacts", control_config::kDefaultMinFootContacts, 0, kNumLegs, "tuning", logger);
  out.maxFootContacts = config_validation::parseIntWithFallback(
      root, "Tuning.MaxFootContacts", control_config::kDefaultMaxFootContacts, 0, kNumLegs, "tuning", logger);
  if (out.minFootContacts > out.maxFootContacts) {
    if (logger) {
      LOG_WARN(logger, "[tuning] Tuning.MinFootContacts > Tuning.MaxFootContacts, using defaults");
    }
    out.minFootContacts = control_config::kDefaultMinFootContacts;
    out.maxFootContacts = control_config::kDefaultMaxFootContacts;
  }
}

} // namespace tuning_section_parser
