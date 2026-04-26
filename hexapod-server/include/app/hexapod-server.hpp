// Header guard
#ifndef HEXAPOD_SERVER_HPP
#define HEXAPOD_SERVER_HPP
#include "serialCommsServer.hpp"
#include "local_map.hpp"
#include "local_planner.hpp"
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

  std::string physicsSimHost{"127.0.0.1"};
  int physicsSimPort{9871};
  int physicsSimSolverIterations{24};

  std::string logFilePath{"app.log"};
  bool logToFile{true};
  std::string replayLogFilePath{};
  bool replayLogToFile{false};
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
  bool investigationDisableTerrainStanceBias{false};
  bool investigationDisableTerrainSwingClearance{false};
  bool investigationDisableTerrainSwingXYNudge{false};
  bool investigationDisableStanceTiltLeveling{false};
  bool investigationSuppressFusionCorrections{false};
  bool investigationSuppressFusionResets{false};
  bool investigationBypassTerrainSnapshotInRuntime{false};
  bool investigationBypassNavStopOnStaleMap{false};
  bool investigationBypassLocomotionFirstOrderFilter{false};
  bool investigationBypassFreshnessGateReject{false};
  bool investigationBypassReachabilityClamp{false};
  double fallbackSpeedMag{0.01};
  double gaitTransitionBlendS{0.35};
  double gaitNominalPlanarSpeedMps{0.25};
  double gaitNominalYawRateRadps{0.5};
  double gaitTurnNominalRadiusM{0.11};
  double footEstimatorBlend{0.35};
  double swingHeightScale{1.0};
  double swingEaseMin{0.40};
  double swingEaseMax{1.0};
  double minBusVoltageV{10.5};
  double maxBusCurrentA{25.0};
  int minFootContacts{0};
  int maxFootContacts{kNumLegs};

  /** Optional NavLocomotionBridge body-frame position I outer loop; 0 = off. Ki in 1/s. */
  double navBodyFrameIntegralKiFwdPerS{0.0};
  double navBodyFrameIntegralKiLatPerS{0.0};
  /** Integral clamp per axis (m*s); 0 = no clamp in bridge. */
  double navBodyFrameIntegralAbsCapMetersSeconds{0.0};
  int localMapWidthCells{kDefaultLocalMapWidthCells};
  int localMapHeightCells{kDefaultLocalMapHeightCells};
  double localMapResolutionM{kDefaultLocalMapResolutionM};
  double localMapObstacleInflationRadiusM{kDefaultLocalMapObstacleInflationRadiusM};
  double localMapSafetyMarginM{kDefaultLocalMapSafetyMarginM};
  double localMapObservationTimeoutS{kDefaultLocalMapObservationTimeoutS};
  double localMapObservationDecayS{kDefaultLocalMapObservationDecayS};
  double localPlannerReplanPeriodS{kDefaultLocalPlannerReplanPeriodS};
  double localPlannerSearchHorizonM{kDefaultLocalPlannerSearchHorizonM};
  int localPlannerSearchNodeBudget{kDefaultLocalPlannerSearchNodeBudget};
  int localPlannerMaxOutputWaypoints{kDefaultLocalPlannerMaxOutputWaypoints};
  int localPlannerSegmentCellHorizon{kDefaultLocalPlannerSegmentCellHorizon};
  double localPlannerBlockedTimeoutS{kDefaultLocalPlannerBlockedTimeoutS};

  double footTerrainSwingMarginM{0.018};
  double footTerrainSwingMaxLiftM{0.040};
  double footTerrainSwingBlend{0.55};
  double footTerrainStancePlaneBlend{0.35};
  double footTerrainStancePlaneDzMaxM{0.012};
  int footTerrainStanceGroundMinSamples{2};
  double footTerrainSwingXYNudgeMaxM{0.018};
  int footTerrainSwingXYNudgeWindowCells{2};
  double footTerrainSwingXYNudgeTauMin{0.55};
  double footTerrainSwingXYNudgeBlend{0.60};
  int fusionContactDebounceSamples{2};
  int fusionTouchdownWindowMs{120};
  int fusionContactHoldWindowMs{250};
  double fusionTrustDecayPerMismatch{0.18};
  double fusionPredictiveTrustBias{0.80};
  double fusionSoftPoseResyncM{0.04};
  double fusionHardPoseResyncM{0.14};
  double fusionSoftOrientationResyncRad{0.18};
  double fusionHardOrientationResyncRad{0.45};
  double fusionSoftContactMismatchRatio{0.17};
  double fusionHardContactMismatchRatio{0.42};
  int fusionCorrectionHoldSamples{3};
  double fusionCorrectionStrongReleaseFactor{0.72};
  double fusionCorrectionSoftReleaseFactor{0.84};

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
