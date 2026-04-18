#include "tuning_section_parser.hpp"

#include <utility>

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
  out.gaitTransitionBlendS = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTransitionBlendS", control_config::kDefaultGaitTransitionBlendS, 0.05, 3.0,
      "tuning", logger);
  out.gaitNominalPlanarSpeedMps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitNominalPlanarSpeedMps", control_config::kDefaultGaitNominalPlanarSpeedMps, 0.05,
      2.0, "tuning", logger);
  out.gaitNominalYawRateRadps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitNominalYawRateRadps", control_config::kDefaultGaitNominalYawRateRadps, 0.05,
      4.0, "tuning", logger);
  out.gaitTurnNominalRadiusM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTurnNominalRadiusM", control_config::kDefaultGaitTurnNominalRadiusM, 0.03, 0.35,
      "tuning", logger);
  out.footEstimatorBlend = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootEstimatorBlend", control_config::kDefaultFootEstimatorBlend, 0.0, 1.0, "tuning",
      logger);
  out.swingHeightScale = config_validation::parseDoubleWithFallback(
      root, "Tuning.SwingHeightScale", control_config::kDefaultSwingHeightScale, 0.25, 2.5, "tuning", logger);
  out.swingEaseMin = config_validation::parseDoubleWithFallback(
      root, "Tuning.SwingEaseMin", control_config::kDefaultSwingEaseMin, 0.0, 1.0, "tuning", logger);
  out.swingEaseMax = config_validation::parseDoubleWithFallback(
      root, "Tuning.SwingEaseMax", control_config::kDefaultSwingEaseMax, 0.0, 1.0, "tuning", logger);
  if (out.swingEaseMin > out.swingEaseMax) {
    if (logger) {
      LOG_WARN(logger, "[tuning] Tuning.SwingEaseMin > Tuning.SwingEaseMax, swapping");
    }
    std::swap(out.swingEaseMin, out.swingEaseMax);
  }
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

  out.navBodyFrameIntegralKiFwdPerS = config_validation::parseDoubleWithFallback(
      root, "Tuning.NavBodyFrameIntegralKiFwdPerS", control_config::kDefaultNavBodyFrameIntegralKiFwdPerS, 0.0, 5.0,
      "tuning", logger);
  out.navBodyFrameIntegralKiLatPerS = config_validation::parseDoubleWithFallback(
      root, "Tuning.NavBodyFrameIntegralKiLatPerS", control_config::kDefaultNavBodyFrameIntegralKiLatPerS, 0.0, 5.0,
      "tuning", logger);
  out.navBodyFrameIntegralAbsCapMetersSeconds = config_validation::parseDoubleWithFallback(
      root, "Tuning.NavBodyFrameIntegralAbsCapMetersSeconds",
      control_config::kDefaultNavBodyFrameIntegralAbsCapMetersSeconds, 0.0, 50.0, "tuning", logger);
  out.localMapWidthCells = config_validation::parseIntWithFallback(
      root, "Tuning.LocalMapWidthCells", kDefaultLocalMapWidthCells, 9, 401, "tuning", logger);
  out.localMapHeightCells = config_validation::parseIntWithFallback(
      root, "Tuning.LocalMapHeightCells", kDefaultLocalMapHeightCells, 9, 401, "tuning", logger);
  out.localMapResolutionM = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalMapResolutionM", kDefaultLocalMapResolutionM, 0.01, 0.5, "tuning", logger);
  out.localMapObstacleInflationRadiusM = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalMapObstacleInflationRadiusM", kDefaultLocalMapObstacleInflationRadiusM, 0.0, 1.5,
      "tuning", logger);
  out.localMapSafetyMarginM = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalMapSafetyMarginM", kDefaultLocalMapSafetyMarginM, 0.0, 1.0, "tuning", logger);
  out.localMapObservationTimeoutS = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalMapObservationTimeoutS", kDefaultLocalMapObservationTimeoutS, 0.01, 10.0, "tuning",
      logger);
  out.localMapObservationDecayS = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalMapObservationDecayS", kDefaultLocalMapObservationDecayS, 0.05, 30.0, "tuning", logger);
  out.localPlannerReplanPeriodS = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalPlannerReplanPeriodS", kDefaultLocalPlannerReplanPeriodS, 0.01, 10.0, "tuning",
      logger);
  out.localPlannerSearchHorizonM = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalPlannerSearchHorizonM", kDefaultLocalPlannerSearchHorizonM, 0.05, 20.0, "tuning",
      logger);
  out.localPlannerSearchNodeBudget = config_validation::parseIntWithFallback(
      root, "Tuning.LocalPlannerSearchNodeBudget", kDefaultLocalPlannerSearchNodeBudget, 32, 200000, "tuning",
      logger);
  out.localPlannerMaxOutputWaypoints = config_validation::parseIntWithFallback(
      root, "Tuning.LocalPlannerMaxOutputWaypoints", kDefaultLocalPlannerMaxOutputWaypoints, 1, 64, "tuning",
      logger);
  out.localPlannerSegmentCellHorizon = config_validation::parseIntWithFallback(
      root, "Tuning.LocalPlannerSegmentCellHorizon", kDefaultLocalPlannerSegmentCellHorizon, 2, 256, "tuning",
      logger);
  out.localPlannerBlockedTimeoutS = config_validation::parseDoubleWithFallback(
      root, "Tuning.LocalPlannerBlockedTimeoutS", kDefaultLocalPlannerBlockedTimeoutS, 0.01, 60.0, "tuning",
      logger);

  out.footTerrainSwingMarginM = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainSwingMarginM", control_config::kDefaultFootTerrainSwingMarginM, 0.0, 0.15, "tuning",
      logger);
  out.footTerrainSwingMaxLiftM = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainSwingMaxLiftM", control_config::kDefaultFootTerrainSwingMaxLiftM, 0.0, 0.12, "tuning",
      logger);
  out.footTerrainSwingBlend = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainSwingBlend", control_config::kDefaultFootTerrainSwingBlend, 0.0, 1.0, "tuning",
      logger);
  out.footTerrainStancePlaneBlend = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainStancePlaneBlend", control_config::kDefaultFootTerrainStancePlaneBlend, 0.0, 1.0,
      "tuning", logger);
  out.footTerrainStancePlaneDzMaxM = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainStancePlaneDzMaxM", control_config::kDefaultFootTerrainStancePlaneDzMaxM, 0.0, 0.05,
      "tuning", logger);
  out.footTerrainStanceGroundMinSamples = config_validation::parseIntWithFallback(
      root, "Tuning.FootTerrainStanceGroundMinSamples", control_config::kDefaultFootTerrainStanceGroundMinSamples, 1,
      kNumLegs, "tuning", logger);
  out.footTerrainSwingXYNudgeMaxM = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainSwingXYNudgeMaxM", control_config::kDefaultFootTerrainSwingXYNudgeMaxM, 0.0, 0.08,
      "tuning", logger);
  out.footTerrainSwingXYNudgeWindowCells = config_validation::parseIntWithFallback(
      root, "Tuning.FootTerrainSwingXYNudgeWindowCells", control_config::kDefaultFootTerrainSwingXYNudgeWindowCells,
      0, 8, "tuning", logger);
  out.footTerrainSwingXYNudgeTauMin = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainSwingXYNudgeTauMin", control_config::kDefaultFootTerrainSwingXYNudgeTauMin, 0.0, 1.0,
      "tuning", logger);
  out.footTerrainSwingXYNudgeBlend = config_validation::parseDoubleWithFallback(
      root, "Tuning.FootTerrainSwingXYNudgeBlend", control_config::kDefaultFootTerrainSwingXYNudgeBlend, 0.0, 1.0,
      "tuning", logger);
}

} // namespace tuning_section_parser
