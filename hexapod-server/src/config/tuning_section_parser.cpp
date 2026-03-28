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
  out.gaitFrequencyMinHz = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitFrequencyMinHz", control_config::kDefaultGaitFrequencyMinHz, 0.05, 10.0,
      "tuning", logger);
  out.gaitFrequencyMaxHz = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitFrequencyMaxHz", control_config::kDefaultGaitFrequencyMaxHz, 0.05, 10.0,
      "tuning", logger);
  if (out.gaitFrequencyMinHz > out.gaitFrequencyMaxHz) {
    if (logger) {
      LOG_WARN(logger, "[tuning] Gait frequency min > max, using defaults");
    }
    out.gaitFrequencyMinHz = control_config::kDefaultGaitFrequencyMinHz;
    out.gaitFrequencyMaxHz = control_config::kDefaultGaitFrequencyMaxHz;
  }
  out.gaitNominalMaxSpeedMps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitNominalMaxSpeedMps", control_config::kDefaultGaitNominalMaxSpeedMps,
      0.01, 2.0, "tuning", logger);
  out.gaitReachEnvelopeSoftLimit = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitReachEnvelopeSoftLimit", control_config::kDefaultGaitReachEnvelopeSoftLimit,
      0.01, 1.0, "tuning", logger);
  out.gaitReachEnvelopeMinScale = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitReachEnvelopeMinScale", control_config::kDefaultGaitReachEnvelopeMinScale,
      0.01, 1.0, "tuning", logger);
  out.gaitTripodDutyCycle = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTripodDutyCycle", control_config::kDefaultTripodDutyCycle, 0.05, 0.95,
      "tuning", logger);
  out.gaitRippleDutyCycle = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitRippleDutyCycle", control_config::kDefaultRippleDutyCycle, 0.05, 0.95,
      "tuning", logger);
  out.gaitWaveDutyCycle = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitWaveDutyCycle", control_config::kDefaultWaveDutyCycle, 0.05, 0.95,
      "tuning", logger);

  out.gaitTripodPhaseOffsets = config_validation::parseDoubleListWithFallback(
      root, "Tuning.GaitTripodPhaseOffsets", std::vector<double>(control_config::kDefaultTripodPhaseOffsets.begin(), control_config::kDefaultTripodPhaseOffsets.end()),
      kNumLegs, 0.0, 1.0, "tuning", logger);
  out.gaitRipplePhaseOffsets = config_validation::parseDoubleListWithFallback(
      root, "Tuning.GaitRipplePhaseOffsets", std::vector<double>(control_config::kDefaultRipplePhaseOffsets.begin(), control_config::kDefaultRipplePhaseOffsets.end()),
      kNumLegs, 0.0, 1.0, "tuning", logger);
  out.gaitWavePhaseOffsets = config_validation::parseDoubleListWithFallback(
      root, "Tuning.GaitWavePhaseOffsets", std::vector<double>(control_config::kDefaultWavePhaseOffsets.begin(), control_config::kDefaultWavePhaseOffsets.end()),
      kNumLegs, 0.0, 1.0, "tuning", logger);

  out.gaitSwingHeightM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitSwingHeightM", control_config::kDefaultSwingHeightM, 0.0, 0.20, "tuning", logger);
  out.gaitFootholdStepLengthM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitFootholdStepLengthM", control_config::kDefaultFootholdStepLengthM, 0.0, 0.30,
      "tuning", logger);
  out.gaitStanceFieldCenterXM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitStanceFieldCenterXM", control_config::kDefaultStanceFieldCenterXM, -0.25, 0.25,
      "tuning", logger);
  out.gaitStanceFieldCenterYM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitStanceFieldCenterYM", control_config::kDefaultStanceFieldCenterYM, -0.25, 0.25,
      "tuning", logger);
  out.gaitStanceFieldRadiusXM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitStanceFieldRadiusXM", control_config::kDefaultStanceFieldRadiusXM, 0.01, 0.50,
      "tuning", logger);
  out.gaitStanceFieldRadiusYM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitStanceFieldRadiusYM", control_config::kDefaultStanceFieldRadiusYM, 0.01, 0.50,
      "tuning", logger);

  out.gaitStabilityPriority = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitStabilityPriority", control_config::kDefaultStabilityPriority, 0.0, 10.0,
      "tuning", logger);
  out.gaitReachSuppressionGain = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitReachSuppressionGain", control_config::kDefaultReachSuppressionGain, 0.0, 10.0,
      "tuning", logger);
  out.gaitTurnSuppressionGain = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTurnSuppressionGain", control_config::kDefaultTurnSuppressionGain, 0.0, 10.0,
      "tuning", logger);

  out.gaitTurnYawRateEnterRadps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTurnYawRateEnterRadps", control_config::kDefaultTurnYawRateEnterRadps, 0.0, 20.0,
      "tuning", logger);
  out.gaitTurnYawRateExitRadps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTurnYawRateExitRadps", control_config::kDefaultTurnYawRateExitRadps, 0.0, 20.0,
      "tuning", logger);
  out.gaitTurnSpeedEnterMps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTurnSpeedEnterMps", control_config::kDefaultTurnSpeedEnterMps, 0.0, 2.0,
      "tuning", logger);
  out.gaitTurnSpeedExitMps = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitTurnSpeedExitMps", control_config::kDefaultTurnSpeedExitMps, 0.0, 2.0,
      "tuning", logger);
  out.gaitDynamicFeatureFlagEnabled = config_validation::parseBoolWithFallback(
      root, "Tuning.GaitDynamicFeatureFlagEnabled", false);
  out.gaitDynamicSimulatorFirstRequired = config_validation::parseBoolWithFallback(
      root, "Tuning.GaitDynamicSimulatorFirstRequired", true);
  out.gaitDynamicSimulatorValidationRunsRequired = config_validation::parseIntWithFallback(
      root, "Tuning.GaitDynamicSimulatorValidationRunsRequired", 5, 0, 100000, "tuning", logger);
  out.gaitDynamicSimulatorValidationRunsPassed = config_validation::parseIntWithFallback(
      root, "Tuning.GaitDynamicSimulatorValidationRunsPassed", 0, 0, 100000, "tuning", logger);
  out.gaitDynamicMaxControlLatencyP95Ms = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitDynamicMaxControlLatencyP95Ms", 8.0, 0.1, 200.0, "tuning", logger);
  out.gaitDynamicObservedControlLatencyP95Ms = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitDynamicObservedControlLatencyP95Ms", 0.0, 0.0, 200.0, "tuning", logger);
  out.gaitDynamicMaxSafetyFaultsPerHour = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitDynamicMaxSafetyFaultsPerHour", 0.20, 0.0, 100.0, "tuning", logger);
  out.gaitDynamicObservedSafetyFaultsPerHour = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitDynamicObservedSafetyFaultsPerHour", 0.0, 0.0, 100.0, "tuning", logger);
  out.gaitDynamicMinStabilityMarginM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitDynamicMinStabilityMarginM", 0.015, 0.0, 0.50, "tuning", logger);
  out.gaitDynamicObservedMinStabilityMarginM = config_validation::parseDoubleWithFallback(
      root, "Tuning.GaitDynamicObservedMinStabilityMarginM", 0.0, 0.0, 0.50, "tuning", logger);
  if (out.gaitTurnYawRateExitRadps > out.gaitTurnYawRateEnterRadps) {
    if (logger) {
      LOG_WARN(logger, "[tuning] turn yaw-rate exit threshold > enter threshold, using defaults");
    }
    out.gaitTurnYawRateEnterRadps = control_config::kDefaultTurnYawRateEnterRadps;
    out.gaitTurnYawRateExitRadps = control_config::kDefaultTurnYawRateExitRadps;
  }
  if (out.gaitTurnSpeedExitMps > out.gaitTurnSpeedEnterMps) {
    if (logger) {
      LOG_WARN(logger, "[tuning] turn speed exit threshold > enter threshold, using defaults");
    }
    out.gaitTurnSpeedEnterMps = control_config::kDefaultTurnSpeedEnterMps;
    out.gaitTurnSpeedExitMps = control_config::kDefaultTurnSpeedExitMps;
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

  out.motionBodyLinearAccelLimitXYMps2 = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.body_linear_accel_limit_xy_mps2",
      control_config::kDefaultMotionBodyLinearAccelLimitXYMps2, 0.01, 20.0, "tuning", logger);
  out.motionBodyLinearAccelLimitZMps2 = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.body_linear_accel_limit_z_mps2",
      control_config::kDefaultMotionBodyLinearAccelLimitZMps2, 0.01, 20.0, "tuning", logger);
  out.motionBodyAngularAccelLimitXRadps2 = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.body_angular_accel_limit_x_radps2",
      control_config::kDefaultMotionBodyAngularAccelLimitXRadps2, 0.01, 100.0, "tuning", logger);
  out.motionBodyAngularAccelLimitYRadps2 = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.body_angular_accel_limit_y_radps2",
      control_config::kDefaultMotionBodyAngularAccelLimitYRadps2, 0.01, 100.0, "tuning", logger);
  out.motionBodyAngularAccelLimitZRadps2 = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.body_angular_accel_limit_z_radps2",
      control_config::kDefaultMotionBodyAngularAccelLimitZRadps2, 0.01, 100.0, "tuning", logger);
  out.motionBodyYawRateLimitRadps = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.body_yaw_rate_limit_radps",
      control_config::kDefaultMotionBodyYawRateLimitRadps, 0.01, 100.0, "tuning", logger);
  out.motionFootVelocityLimitMps = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.foot_velocity_limit_mps",
      control_config::kDefaultMotionFootVelocityLimitMps, 0.01, 5.0, "tuning", logger);
  out.motionFootAccelLimitMps2 = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.foot_accel_limit_mps2",
      control_config::kDefaultMotionFootAccelLimitMps2, 0.01, 50.0, "tuning", logger);
  out.motionJointSoftVelocityLimitRadps = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.joint_soft_velocity_limit_radps",
      control_config::kDefaultMotionJointSoftVelocityLimitRadps, 0.01, 100.0, "tuning", logger);
  out.motionJointSoftAccelLimitRadps2 = config_validation::parseDoubleWithFallback(
      root, "motion_limiter.joint_soft_accel_limit_radps2",
      control_config::kDefaultMotionJointSoftAccelLimitRadps2, 0.01, 1000.0, "tuning", logger);
  out.motionStartupPhaseThresholdMs = config_validation::parseIntWithFallback(
      root, "motion_limiter.startup_phase_threshold_ms",
      control_config::kDefaultMotionStartupPhaseThresholdMs, 0, 60000, "tuning", logger);
  out.motionShutdownPhaseThresholdMs = config_validation::parseIntWithFallback(
      root, "motion_limiter.shutdown_phase_threshold_ms",
      control_config::kDefaultMotionShutdownPhaseThresholdMs, 0, 60000, "tuning", logger);
}

} // namespace tuning_section_parser
