// Header guard
#ifndef HEXAPOD_SERVER_HPP
#define HEXAPOD_SERVER_HPP
#include "serialCommsServer.hpp"
#include "types.hpp"
#include "config_validation.hpp"


struct ParsedToml
{
  std::string runtimeMode{"serial"};

  std::string serialDevice{"/dev/ttyACM0"};
  int baudRate{115200};
  int timeout{100};
  std::vector<float> minMaxPulses{};

  double simInitialVoltageV{12.0};
  double simInitialCurrentA{1.0};
  double simResponseRateHz{50.0};
  bool simDropBus{false};
  bool simLowVoltage{false};
  bool simHighCurrent{false};
  bool imuEnableReads{false};
  bool autonomyEnabled{false};
  uint64_t autonomyNoProgressTimeoutMs{1000};
  uint64_t autonomyRecoveryRetryBudget{2};
  double autonomyTraversabilityOccupancyRiskWeight{0.65};
  double autonomyTraversabilityGradientRiskWeight{0.35};
  double autonomyTraversabilityObstacleNearRiskWeight{0.75};
  double autonomyTraversabilityObstacleMidRiskWeight{0.45};
  double autonomyTraversabilityObstacleFarRiskWeight{0.25};
  double autonomyTraversabilitySlopeHighRiskWeight{0.8};
  double autonomyTraversabilityConfidenceUnknownPenalty{0.5};
  double autonomyTraversabilityConfidenceCostWeight{1.0};
  double autonomyTraversabilityRiskBlockThreshold{0.85};
  double autonomyTraversabilityConfidenceBlockThreshold{0.3};

  std::string logFilePath{"app.log"};
  bool logToFile{true};
  std::string telemetryHost{"127.0.0.1"};
  int telemetryPort{9870};
  double telemetryPublishRateHz{30.0};
  double telemetryGeometryResendIntervalSec{1.0};

  bool telemetryEnabled{false};
  std::string telemetryUdpHost{"127.0.0.1"};
  int telemetryUdpPort{9870};
  int telemetryPublishPeriodMs{50};
  int telemetryGeometryRefreshPeriodMs{2000};

  int busLoopPeriodUs{2000};
  int estimatorLoopPeriodUs{2000};
  int controlLoopPeriodUs{4000};
  int safetyLoopPeriodUs{2000};
  int diagnosticsPeriodMs{500};
  int commandRefreshPeriodMs{100};
  int standSettlingDelayMs{2000};
  double maxTiltRad{0.70};
  uint64_t commandTimeoutUs{300000};
  uint64_t estimatorMaxAgeUs{300000};
  uint64_t intentMaxAgeUs{300000};
  bool estimatorRequireTimestamp{true};
  bool estimatorRequireSampleId{true};
  bool estimatorRequireMonotonicSampleId{true};
  bool intentRequireTimestamp{true};
  bool intentRequireSampleId{true};
  bool intentRequireMonotonicSampleId{true};
  double fallbackSpeedMag{0.01};
  double gaitFrequencyMinHz{0.5};
  double gaitFrequencyMaxHz{2.5};
  double gaitNominalMaxSpeedMps{0.25};
  double gaitReachEnvelopeSoftLimit{0.15};
  double gaitReachEnvelopeMinScale{0.25};
  double gaitTripodDutyCycle{0.5};
  double gaitRippleDutyCycle{0.5};
  double gaitWaveDutyCycle{0.5};
  std::vector<double> gaitTripodPhaseOffsets{0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
  std::vector<double> gaitRipplePhaseOffsets{0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0};
  std::vector<double> gaitWavePhaseOffsets{0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0};
  double gaitSwingHeightM{0.03};
  double gaitFootholdStepLengthM{0.06};
  double gaitStanceFieldCenterXM{0.0};
  double gaitStanceFieldCenterYM{0.0};
  double gaitStanceFieldRadiusXM{0.12};
  double gaitStanceFieldRadiusYM{0.10};
  double gaitStabilityPriority{1.0};
  double gaitReachSuppressionGain{1.0};
  double gaitTurnSuppressionGain{1.0};
  double gaitTurnYawRateEnterRadps{0.4};
  double gaitTurnYawRateExitRadps{0.3};
  double gaitTurnSpeedEnterMps{0.05};
  double gaitTurnSpeedExitMps{0.03};
  bool gaitDynamicFeatureFlagEnabled{false};
  bool gaitDynamicSimulatorFirstRequired{true};
  int gaitDynamicSimulatorValidationRunsRequired{5};
  int gaitDynamicSimulatorValidationRunsPassed{0};
  double gaitDynamicMaxControlLatencyP95Ms{8.0};
  double gaitDynamicObservedControlLatencyP95Ms{0.0};
  double gaitDynamicMaxSafetyFaultsPerHour{0.20};
  double gaitDynamicObservedSafetyFaultsPerHour{0.0};
  double gaitDynamicMinStabilityMarginM{0.015};
  double gaitDynamicObservedMinStabilityMarginM{0.0};
  double minBusVoltageV{10.5};
  double maxBusCurrentA{25.0};
  int minFootContacts{0};
  int maxFootContacts{kNumLegs};
  double motionBodyLinearAccelLimitXYMps2{0.6};
  double motionBodyLinearAccelLimitZMps2{0.4};
  double motionBodyAngularAccelLimitXRadps2{1.2};
  double motionBodyAngularAccelLimitYRadps2{1.2};
  double motionBodyAngularAccelLimitZRadps2{1.5};
  double motionBodyYawRateLimitRadps{1.5};
  double motionFootVelocityLimitMps{0.30};
  double motionFootAccelLimitMps2{1.0};
  double motionJointSoftVelocityLimitRadps{5.0};
  double motionJointSoftAccelLimitRadps2{30.0};
  int motionStartupPhaseThresholdMs{350};
  int motionShutdownPhaseThresholdMs{450};

  double coxaLengthM{0.043};
  double femurLengthM{0.060};
  double tibiaLengthM{0.104};
  std::vector<double> mountAnglesDeg{};
  std::vector<Vec3> coxaOffsetsM{};
  double coxaAttachDeg{0.0};
  std::vector<double> femurAttachDeg{};
  std::vector<double> tibiaAttachDeg{};
  std::vector<double> sideSign{};
  double bodyToBottomM{0.040};
  std::vector<Vec3> servoDynamicsPositiveTauS{};
  std::vector<Vec3> servoDynamicsPositiveVmaxRadps{};
  std::vector<Vec3> servoDynamicsNegativeTauS{};
  std::vector<Vec3> servoDynamicsNegativeVmaxRadps{};
  std::vector<config_validation::ConfigDiagnostic> diagnostics{};
};

bool tomlParser(std::string filename,  ParsedToml& out);

#endif  // #ifndef HEXAPOD_SERVER_HPP
