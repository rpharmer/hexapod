# `hexapod-server` Configuration Reference

This is the canonical key reference for `hexapod-server` configuration.

## Source of truth and precedence

1. TOML schema and parsing behavior:
   - `hexapod-server/src/config/toml_parser.cpp`
   - `hexapod-server/src/config/runtime_section_parser.cpp`
   - `hexapod-server/src/config/transport_section_parser.cpp`
   - `hexapod-server/src/config/calibrations_section_parser.cpp`
   - `hexapod-server/src/config/geometry_section_parser.cpp`
   - `hexapod-server/src/config/tuning_section_parser.cpp`
2. Validation/fallback helpers:
   - `hexapod-server/src/config/config_validation.cpp`
3. Runtime mapping:
   - `hexapod-server/src/control/control_config.cpp`
4. Final effective overrides:
   - `hexapod-server/src/app/hexapod-server.cpp` (CLI overrides)

## Required schema header keys

- `title` (must equal `Hexapod Config File`)
- `Schema` (must equal `hexapod.server.config`)
- `SchemaVersion` (must equal `1`)

Parse fails if any required schema header value mismatches.

## Runtime keys (`Runtime.*`)

Primary parser: `runtime_section_parser.cpp`.

### Core mode

- `Runtime.Mode` (required): `serial|sim|physics-sim`

### Sim mode

- `Runtime.Sim.InitialVoltageV` (double, bounds: `0.0..32.0`, default `12.0`)
- `Runtime.Sim.InitialCurrentA` (double, bounds: `0.0..200.0`, default `1.0`)
- `Runtime.Sim.ResponseRateHz` (double, bounds: `1.0..2000.0`, default `50.0`)
- `Runtime.Sim.DropBus` (bool, default `false`)
- `Runtime.Sim.LowVoltage` (bool, default `false`)
- `Runtime.Sim.HighCurrent` (bool, default `false`)

### Physics sim bridge

- `Runtime.PhysicsSim.Host` (string, default `127.0.0.1`)
- `Runtime.PhysicsSim.Port` (int/double parsed, bounds: `1..65535`, default `9871`)
- `Runtime.PhysicsSim.SolverIterations` (int/double parsed, bounds: `1..512`, default `24`)

### Logging and replay

- `Runtime.Log.FilePath` (string, default `app.log`)
- `Runtime.Log.EnableFile` (bool, default `true`)
- `Runtime.ReplayLog.EnableFile` (bool, default `false`)
- `Runtime.ReplayLog.FilePath` (string, default empty)

### Telemetry (primary keys)

- `Runtime.Telemetry.Enable` (bool, default `false`)
- `Runtime.Telemetry.Host` (string, default `127.0.0.1`)
- `Runtime.Telemetry.Port` (int/double parsed, bounds: `1..65535`, default `9870`)
- `Runtime.Telemetry.PublishRateHz` (double, bounds: `0.1..1000.0`, default `30.0`)
- `Runtime.Telemetry.GeometryResendIntervalSec` (double, bounds: `0.1..3600.0`, default `1.0`)

### Telemetry compatibility aliases (accepted)

- `Runtime.Telemetry.Enabled`
- `Runtime.Telemetry.UdpHost`
- `Runtime.Telemetry.UdpPort`
- `Runtime.Telemetry.PublishPeriodMs`
- `Runtime.Telemetry.GeometryRefreshPeriodMs`

Note: current runtime computes effective UDP host/port and periods from primary host/port/rate fields.

### Investigation toggles

- `Runtime.Investigation.DisableTerrainStanceBias` (bool)
- `Runtime.Investigation.DisableTerrainSwingClearance` (bool)
- `Runtime.Investigation.DisableTerrainSwingXYNudge` (bool)
- `Runtime.Investigation.DisableStanceTiltLeveling` (bool)
- `Runtime.Investigation.SuppressFusionCorrections` (bool)
- `Runtime.Investigation.SuppressFusionResets` (bool)

## Transport keys (top-level)

Primary parser: `transport_section_parser.cpp`.

- `SerialDevice` (string; required in `Runtime.Mode=serial`)
- `BaudRate` (int > 0; required in `serial`, fallback otherwise)
- `Timeout_ms` (int > 0; required in `serial`, fallback otherwise)

## Calibration keys (top-level)

Primary parser: `calibrations_section_parser.cpp`.
Normalization/validation: `motor_calibration_validator.cpp`.

- `MotorCalibrations` (array, expected 18 entries)

Validation expectations include:

- complete expected joint set
- no duplicates
- pulse bounds `500 <= min < max <= 2500`

## Geometry keys (`Geometry.*`)

Primary parser: `geometry_section_parser.cpp`.

### Scalar geometry

- `Geometry.CoxaLengthM` (`0.005..0.30`)
- `Geometry.FemurLengthM` (`0.005..0.30`)
- `Geometry.TibiaLengthM` (`0.005..0.40`)
- `Geometry.BodyToBottomM` (`0.005..0.30`)
- `Geometry.CoxaAttachDeg` (`-180..180`)

### Per-leg list geometry (size `kNumLegs`)

- `Geometry.MountAnglesDeg` (`-360..360`)
- `Geometry.FemurAttachDeg` (`-180..180`)
- `Geometry.TibiaAttachDeg` (`-180..180`)
- `Geometry.SideSign` (`-1..1`)
- `Geometry.CoxaOffsetsM` (vec3 list, each component `-0.30..0.30`)

### Per-leg servo dynamics lists (size `kNumLegs`)

- `Geometry.ServoDynamicsPositiveTauS` (vec3 list, `0.0..2.0`)
- `Geometry.ServoDynamicsPositiveVmaxRadps` (vec3 list, `0.0..30.0`)
- `Geometry.ServoDynamicsNegativeTauS` (vec3 list, `0.0..2.0`)
- `Geometry.ServoDynamicsNegativeVmaxRadps` (vec3 list, `0.0..30.0`)

## Tuning keys (`Tuning.*`)

Primary parser: `tuning_section_parser.cpp`.

### Loop timing and freshness

- `Tuning.BusLoopPeriodUs` (`500..50000`)
- `Tuning.EstimatorLoopPeriodUs` (`500..50000`)
- `Tuning.ControlLoopPeriodUs` (`500..50000`)
- `Tuning.SafetyLoopPeriodUs` (`500..50000`)
- `Tuning.DiagnosticsPeriodMs` (`100..10000`)
- `Tuning.CommandRefreshPeriodMs` (`10..1000`)
- `Tuning.StandSettlingDelayMs` (`0..10000`)
- `Tuning.CommandTimeoutUs` (`10000..2000000`)
- `Tuning.EstimatorMaxAgeUs` (`1000..2000000`)
- `Tuning.IntentMaxAgeUs` (`1000..2000000`)
- `Tuning.EstimatorRequireTimestamp` (bool)
- `Tuning.EstimatorRequireSampleId` (bool)
- `Tuning.EstimatorRequireMonotonicSampleId` (bool)
- `Tuning.IntentRequireTimestamp` (bool)
- `Tuning.IntentRequireSampleId` (bool)
- `Tuning.IntentRequireMonotonicSampleId` (bool)

### Safety and contact thresholds

- `Tuning.MaxTiltRad` (`0.1..1.5`)
- `Tuning.RapidBodyRateRadps` (`0.0..10.0`)
- `Tuning.RapidBodyRateMaxContacts` (`0..kNumLegs`)
- `Tuning.MinBusVoltageV` (`5.0..24.0`)
- `Tuning.MaxBusCurrentA` (`0.1..120.0`)
- `Tuning.MinFootContacts` (`0..kNumLegs`)
- `Tuning.MaxFootContacts` (`0..kNumLegs`)
- `Tuning.BodyHeightCollapseMarginM` (`0.0..0.25`)
- `Tuning.BodyHeightCollapseMinSafeM` (`0.0..0.25`)
- `Tuning.BodyHeightCollapseMaxContacts` (`0..kNumLegs`)

### Gait and locomotion command

- `Tuning.FallbackSpeedMag` (`0.0..1.0`)
- `Tuning.GaitTransitionBlendS` (`0.05..3.0`)
- `Tuning.GaitNominalPlanarSpeedMps` (`0.05..2.0`)
- `Tuning.GaitNominalYawRateRadps` (`0.05..4.0`)
- `Tuning.GaitTurnNominalRadiusM` (`0.03..0.35`)
- `Tuning.FootEstimatorBlend` (`0.0..1.0`)
- `Tuning.SwingHeightScale` (`0.25..2.5`)
- `Tuning.SwingEaseMin` (`0.0..1.0`)
- `Tuning.SwingEaseMax` (`0.0..1.0`)

### Navigation bridge gains

- `Tuning.NavBodyFrameIntegralKiFwdPerS` (`0.0..5.0`)
- `Tuning.NavBodyFrameIntegralKiLatPerS` (`0.0..5.0`)
- `Tuning.NavBodyFrameIntegralAbsCapMetersSeconds` (`0.0..50.0`)

### Command governor

- `Tuning.GovernorLowSpeedPlanarCutoffMps` (`0.0..1.0`)
- `Tuning.GovernorLowSpeedYawCutoffRadps` (`0.0..3.14`)
- `Tuning.GovernorStartupSupportMarginM` (`-0.05..0.2`)
- `Tuning.GovernorSupportMarginSoftM` (`-0.05..0.2`)
- `Tuning.GovernorSupportMarginHardM` (`-0.1..0.1`)
- `Tuning.GovernorTiltSoftRad` (`0.0..1.0`)
- `Tuning.GovernorTiltHardRad` (`0.05..2.5`)
- `Tuning.GovernorBodyRateSoftRadps` (`0.0..10.0`)
- `Tuning.GovernorBodyRateHardRadps` (`0.05..20.0`)
- `Tuning.GovernorFusionTrustSoft` (`0.0..1.0`)
- `Tuning.GovernorFusionTrustHard` (`0.0..1.0`)
- `Tuning.GovernorContactMismatchSoft` (`0.0..1.0`)
- `Tuning.GovernorContactMismatchHard` (`0.0..1.0`)
- `Tuning.GovernorCommandAccelSoftMps2` (`0.0..20.0`)
- `Tuning.GovernorCommandAccelHardMps2` (`0.05..30.0`)
- `Tuning.GovernorLowSpeedMinScale` (`0.0..1.0`)
- `Tuning.GovernorActiveMinScale` (`0.0..1.0`)
- `Tuning.GovernorLowSpeedCadenceMinScale` (`0.0..1.0`)
- `Tuning.GovernorActiveCadenceMinScale` (`0.0..1.0`)
- `Tuning.GovernorBodyHeightSquatMaxM` (`0.0..0.1`)
- `Tuning.GovernorBodyHeightSquatSeverityThreshold` (`0.0..1.0`)
- `Tuning.GovernorSwingFloorBoostM` (`0.0..0.05`)

### Local map and planner

- `Tuning.LocalMapWidthCells` (`9..401`)
- `Tuning.LocalMapHeightCells` (`9..401`)
- `Tuning.LocalMapResolutionM` (`0.01..0.5`)
- `Tuning.LocalMapObstacleInflationRadiusM` (`0.0..1.5`)
- `Tuning.LocalMapSafetyMarginM` (`0.0..1.0`)
- `Tuning.LocalMapObservationTimeoutS` (`0.01..10.0`)
- `Tuning.LocalMapObservationDecayS` (`0.05..30.0`)
- `Tuning.LocalPlannerReplanPeriodS` (`0.01..10.0`)
- `Tuning.LocalPlannerSearchHorizonM` (`0.05..20.0`)
- `Tuning.LocalPlannerSearchNodeBudget` (`32..200000`)
- `Tuning.LocalPlannerMaxOutputWaypoints` (`1..64`)
- `Tuning.LocalPlannerSegmentCellHorizon` (`2..256`)
- `Tuning.LocalPlannerBlockedTimeoutS` (`0.01..60.0`)

### Foot terrain

- `Tuning.FootTerrainSwingMarginM` (`0.0..0.15`)
- `Tuning.FootTerrainSwingMaxLiftM` (`0.0..0.12`)
- `Tuning.FootTerrainSwingBlend` (`0.0..1.0`)
- `Tuning.FootTerrainStancePlaneBlend` (`0.0..1.0`)
- `Tuning.FootTerrainStancePlaneDzMaxM` (`0.0..0.05`)
- `Tuning.FootTerrainStanceGroundMinSamples` (`1..kNumLegs`)
- `Tuning.FootTerrainSwingXYNudgeMaxM` (`0.0..0.08`)
- `Tuning.FootTerrainSwingXYNudgeWindowCells` (`0..8`)
- `Tuning.FootTerrainSwingXYNudgeTauMin` (`0.0..1.0`)
- `Tuning.FootTerrainSwingXYNudgeBlend` (`0.0..1.0`)

### Fusion

- `Tuning.FusionContactDebounceSamples` (`1..16`)
- `Tuning.FusionTouchdownWindowMs` (`1..2000`)
- `Tuning.FusionContactHoldWindowMs` (`1..4000`)
- `Tuning.FusionTrustDecayPerMismatch` (`0.0..1.0`)
- `Tuning.FusionPredictiveTrustBias` (`0.0..1.0`)
- `Tuning.FusionSoftPoseResyncM` (`0.0..0.5`)
- `Tuning.FusionHardPoseResyncM` (`0.01..2.0`)
- `Tuning.FusionSoftOrientationResyncRad` (`0.0..1.5`)
- `Tuning.FusionHardOrientationResyncRad` (`0.05..3.14`)
- `Tuning.FusionSoftContactMismatchRatio` (`0.0..1.0`)
- `Tuning.FusionHardContactMismatchRatio` (`0.0..1.0`)
- `Tuning.FusionCorrectionHoldSamples` (`1..32`)
- `Tuning.FusionCorrectionStrongReleaseFactor` (`0.1..1.0`)
- `Tuning.FusionCorrectionSoftReleaseFactor` (`0.1..1.0`)

## Consumer mapping summary

- Transport/calibration/runtime mode: `hexapod-server/src/app/hexapod-server.cpp`
- Control behavior mapping: `hexapod-server/src/control/control_config.cpp`
- Runtime loops and modules: `hexapod-server/src/control/robot_runtime.cpp`, `control_pipeline.cpp`, `navigation_manager.cpp`, `safety_supervisor.cpp`, `command_governor.cpp`

## CLI override precedence (effective config)

After TOML parsing, `hexapod-server.cpp` applies CLI overrides for:

- telemetry enable/host/port/rate/geometry resend interval
- investigation toggles
- log file / console-only behavior

Then effective telemetry compatibility fields (`telemetryUdp*`, periods) are recomputed.
