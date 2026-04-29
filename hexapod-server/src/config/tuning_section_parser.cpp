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
  out.rapidBodyRateRadps = config_validation::parseDoubleWithFallback(
      root, "Tuning.RapidBodyRateRadps", control_config::kDefaultRapidBodyRateRadps, 0.0, 10.0, "tuning",
      logger);
  out.rapidBodyRateMaxContacts = config_validation::parseIntWithFallback(
      root, "Tuning.RapidBodyRateMaxContacts", control_config::kDefaultRapidBodyRateMaxContacts, 0, kNumLegs,
      "tuning", logger);
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
  out.bodyHeightCollapseMarginM = config_validation::parseDoubleWithFallback(
      root, "Tuning.BodyHeightCollapseMarginM", control_config::kDefaultBodyHeightCollapseMarginM, 0.0, 0.25,
      "tuning", logger);
  out.bodyHeightCollapseMaxContacts = config_validation::parseIntWithFallback(
      root, "Tuning.BodyHeightCollapseMaxContacts", control_config::kDefaultBodyHeightCollapseMaxContacts, 0,
      kNumLegs, "tuning", logger);

  out.navBodyFrameIntegralKiFwdPerS = config_validation::parseDoubleWithFallback(
      root, "Tuning.NavBodyFrameIntegralKiFwdPerS", control_config::kDefaultNavBodyFrameIntegralKiFwdPerS, 0.0, 5.0,
      "tuning", logger);
  out.navBodyFrameIntegralKiLatPerS = config_validation::parseDoubleWithFallback(
      root, "Tuning.NavBodyFrameIntegralKiLatPerS", control_config::kDefaultNavBodyFrameIntegralKiLatPerS, 0.0, 5.0,
      "tuning", logger);
  out.navBodyFrameIntegralAbsCapMetersSeconds = config_validation::parseDoubleWithFallback(
      root, "Tuning.NavBodyFrameIntegralAbsCapMetersSeconds",
      control_config::kDefaultNavBodyFrameIntegralAbsCapMetersSeconds, 0.0, 50.0, "tuning", logger);
  out.governorLowSpeedPlanarCutoffMps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorLowSpeedPlanarCutoffMps", control_config::kDefaultGovernorLowSpeedPlanarCutoffMps,
      0.0, 1.0, "tuning", logger);
  out.governorLowSpeedYawCutoffRadps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorLowSpeedYawCutoffRadps", control_config::kDefaultGovernorLowSpeedYawCutoffRadps,
      0.0, 3.14, "tuning", logger);
  out.governorStartupSupportMarginM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorStartupSupportMarginM", control_config::kDefaultGovernorStartupSupportMarginM,
      -0.05, 0.2, "tuning", logger);
  out.governorSupportMarginSoftM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorSupportMarginSoftM", control_config::kDefaultGovernorSupportMarginSoftM,
      -0.05, 0.2, "tuning", logger);
  out.governorSupportMarginHardM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorSupportMarginHardM", control_config::kDefaultGovernorSupportMarginHardM,
      -0.1, 0.1, "tuning", logger);
  out.governorTiltSoftRad = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorTiltSoftRad", control_config::kDefaultGovernorTiltSoftRad, 0.0, 1.0, "tuning",
      logger);
  out.governorTiltHardRad = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorTiltHardRad", control_config::kDefaultGovernorTiltHardRad, 0.05, 2.5, "tuning",
      logger);
  out.governorBodyRateSoftRadps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorBodyRateSoftRadps", control_config::kDefaultGovernorBodyRateSoftRadps, 0.0, 10.0,
      "tuning", logger);
  out.governorBodyRateHardRadps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorBodyRateHardRadps", control_config::kDefaultGovernorBodyRateHardRadps, 0.05, 20.0,
      "tuning", logger);
  out.governorFusionTrustSoft = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorFusionTrustSoft", control_config::kDefaultGovernorFusionTrustSoft, 0.0, 1.0,
      "tuning", logger);
  out.governorFusionTrustHard = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorFusionTrustHard", control_config::kDefaultGovernorFusionTrustHard, 0.0, 1.0,
      "tuning", logger);
  out.governorContactMismatchSoft = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorContactMismatchSoft", control_config::kDefaultGovernorContactMismatchSoft, 0.0, 1.0,
      "tuning", logger);
  out.governorContactMismatchHard = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorContactMismatchHard", control_config::kDefaultGovernorContactMismatchHard, 0.0, 1.0,
      "tuning", logger);
  out.governorCommandAccelSoftMps2 = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorCommandAccelSoftMps2", control_config::kDefaultGovernorCommandAccelSoftMps2, 0.0,
      20.0, "tuning", logger);
  out.governorCommandAccelHardMps2 = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorCommandAccelHardMps2", control_config::kDefaultGovernorCommandAccelHardMps2, 0.05,
      30.0, "tuning", logger);
  out.governorLowSpeedMinScale = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorLowSpeedMinScale", control_config::kDefaultGovernorLowSpeedMinScale, 0.0, 1.0,
      "tuning", logger);
  out.governorActiveMinScale = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorActiveMinScale", control_config::kDefaultGovernorActiveMinScale, 0.0, 1.0,
      "tuning", logger);
  out.governorLowSpeedCadenceMinScale = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorLowSpeedCadenceMinScale", control_config::kDefaultGovernorLowSpeedCadenceMinScale,
      0.0, 1.0, "tuning", logger);
  out.governorActiveCadenceMinScale = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorActiveCadenceMinScale", control_config::kDefaultGovernorActiveCadenceMinScale,
      0.0, 1.0, "tuning", logger);
  out.governorBodyHeightSquatMaxM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorBodyHeightSquatMaxM", control_config::kDefaultGovernorBodyHeightSquatMaxM,
      0.0, 0.1, "tuning", logger);
  out.governorBodyHeightSquatSeverityThreshold = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorBodyHeightSquatSeverityThreshold",
      control_config::kDefaultGovernorBodyHeightSquatSeverityThreshold, 0.0, 1.0, "tuning", logger);
  out.governorSwingFloorBoostM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GovernorSwingFloorBoostM", control_config::kDefaultGovernorSwingFloorBoostM, 0.0, 0.05,
      "tuning", logger);
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

  out.fusionContactDebounceSamples = config_validation::parseIntWithFallback(
      root, "Tuning.FusionContactDebounceSamples", control_config::kDefaultFusionContactDebounceSamples, 1, 16,
      "tuning", logger);
  out.fusionTouchdownWindowMs = config_validation::parseIntWithFallback(
      root, "Tuning.FusionTouchdownWindowMs", control_config::kDefaultFusionTouchdownWindowMs, 1, 2000,
      "tuning", logger);
  out.fusionContactHoldWindowMs = config_validation::parseIntWithFallback(
      root, "Tuning.FusionContactHoldWindowMs", control_config::kDefaultFusionContactHoldWindowMs, 1, 4000,
      "tuning", logger);
  out.fusionTrustDecayPerMismatch = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionTrustDecayPerMismatch", control_config::kDefaultFusionTrustDecayPerMismatch, 0.0, 1.0,
      "tuning", logger);
  out.fusionPredictiveTrustBias = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionPredictiveTrustBias", control_config::kDefaultFusionPredictiveTrustBias, 0.0, 1.0,
      "tuning", logger);
  out.fusionSoftPoseResyncM = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionSoftPoseResyncM", control_config::kDefaultFusionSoftPoseResyncM, 0.0, 0.5,
      "tuning", logger);
  out.fusionHardPoseResyncM = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionHardPoseResyncM", control_config::kDefaultFusionHardPoseResyncM, 0.01, 2.0,
      "tuning", logger);
  out.fusionSoftOrientationResyncRad = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionSoftOrientationResyncRad", control_config::kDefaultFusionSoftOrientationResyncRad,
      0.0, 1.5, "tuning", logger);
  out.fusionHardOrientationResyncRad = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionHardOrientationResyncRad", control_config::kDefaultFusionHardOrientationResyncRad,
      0.05, 3.14, "tuning", logger);
  out.fusionSoftContactMismatchRatio = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionSoftContactMismatchRatio", control_config::kDefaultFusionSoftContactMismatchRatio,
      0.0, 1.0, "tuning", logger);
  out.fusionHardContactMismatchRatio = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionHardContactMismatchRatio", control_config::kDefaultFusionHardContactMismatchRatio,
      0.0, 1.0, "tuning", logger);
  out.fusionCorrectionHoldSamples = config_validation::parseIntWithFallback(
      root, "Tuning.FusionCorrectionHoldSamples", control_config::kDefaultFusionCorrectionHoldSamples,
      1, 32, "tuning", logger);
  out.fusionCorrectionStrongReleaseFactor = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionCorrectionStrongReleaseFactor",
      control_config::kDefaultFusionCorrectionStrongReleaseFactor, 0.1, 1.0, "tuning", logger);
  out.fusionCorrectionSoftReleaseFactor = config_validation::parseDoubleWithFallback(
      root, "Tuning.FusionCorrectionSoftReleaseFactor",
      control_config::kDefaultFusionCorrectionSoftReleaseFactor, 0.1, 1.0, "tuning", logger);
}

} // namespace tuning_section_parser
