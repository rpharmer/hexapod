# Hexapod Server Subsystem Contracts

This document captures contract-level expectations for four subsystems:

- control runtime
- hardware bridge/transport
- scenario engine
- telemetry pipeline

Each section defines:

1. **Invariants** (must always hold),
2. **Ownership contracts** (who is responsible for what),
3. **Failure contracts** (how failures are surfaced/contained),
4. **Current test coverage** (specific tests in `hexapod-server/tests/`), and
5. **Uncovered invariants / follow-up tasks**.

---

## 1) Control Runtime Contract

### Invariants

1. **Fresh estimator + fresh intent allow commanded WALK mode.**
   - Enforced by `testEstimatorAndIntentFreshAllowsNominalControl()`.
   - Link: [`hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp`](../hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp)

2. **Stale estimator forces `SAFE_IDLE` with `ESTIMATOR_INVALID`.**
   - Enforced by `testStaleEstimatorTriggersEstimatorInvalidFault()`.
   - Link: [`hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp`](../hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp)

3. **Stale intent forces `SAFE_IDLE` with `COMMAND_TIMEOUT`.**
   - Enforced by `testStaleIntentTriggersCommandTimeoutFault()`.
   - Link: [`hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp`](../hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp)

4. **When both estimator and intent are stale, estimator fault takes precedence.**
   - Enforced by `testStaleEstimatorAndIntentPreferEstimatorFault()`.
   - Link: [`hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp`](../hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp)

5. **Freshness policy checks timestamps, sample-id nonzero, and sample-id monotonicity when enabled.**
   - Enforced by:
     - `testNonMonotonicSampleIds()`
     - `testMissingTimestamps()`
     - `testBoundaryAgeThresholds()`
   - Link: [`hexapod-server/tests/test_freshness_policy.cpp`](../hexapod-server/tests/test_freshness_policy.cpp)

6. **Safety faults override requested motion mode to `FAULT` while preserving finite joint outputs.**
   - Enforced by `test_control_pipeline_sanity.cpp` main-path assertions.
   - Link: [`hexapod-server/tests/test_control_pipeline_sanity.cpp`](../hexapod-server/tests/test_control_pipeline_sanity.cpp)

### Ownership contract

- **`RobotRuntime` owns loop-stage sequencing and gatekeeping** across bus, estimator, safety, and control phases.
- **`FreshnessPolicy` owns data-validity decisions** (timestamp/sample-id/age checks), with runtime consuming that verdict.
- **`ControlPipeline` owns mode arbitration and safe finite output generation** given state/intent/safety inputs.

### Failure contract

- Invalid estimator stream must surface as `ESTIMATOR_INVALID` and force non-walking safe behavior.
- Invalid command/intent stream must surface as `COMMAND_TIMEOUT` and force non-walking safe behavior.
- Active safety faults must be elevated to `FAULT` mode and reflected in status.

### Uncovered invariants / follow-up tasks

1. **Loop ordering contract is implicit, not directly asserted.**
   - Add a runtime-stage-order test to prove `busStep -> estimatorStep -> safetyStep -> controlStep` ordering is required and stable.

2. **Recovery contract after transient stale data is unverified.**
   - Add tests that stale/faulted runtime returns from `SAFE_IDLE` to `WALK` once fresh valid samples resume.

3. **Concurrent intent updates during runtime stepping are untested.**
   - Add synchronization/atomicity tests for `setMotionIntent()` under concurrent producer timing.

---

## 2) Hardware Bridge / Transport Contract

### Invariants

1. **Bridge must handshake (`HELLO`, then `HEARTBEAT`) before command APIs are usable.**
   - Enforced by `test_successful_handshake_and_heartbeat()`.
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_handshake.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_handshake.cpp)

2. **ACK sequence must match request sequence.**
   - Enforced by `test_sequence_mismatch_rejected()`.
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_handshake.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_handshake.cpp)

3. **Command APIs before init fail with `BridgeError::NotReady`.**
   - Enforced by `test_not_ready_error_before_init()`.
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_failures.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_failures.cpp)

4. **Transport timeout during init maps to `BridgeError::TransportFailure`.**
   - Enforced by `test_transport_failure_during_init()`.
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_failures.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_failures.cpp)

5. **Malformed ACK payload maps to protocol failure with command-decode phase metadata.**
   - Enforced by `test_malformed_ack_payload_handling()`.
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_failures.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_failures.cpp)

6. **Explicit NACK maps to protocol failure with command-response metadata domain/phase.**
   - Enforced by `test_explicit_nack_behavior_for_command_methods()`.
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_failures.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_failures.cpp)

7. **Timeout/protocol errors are retried; retry exhaustion reports timeout; success clears error state.**
   - Enforced by:
     - `test_timeout_retry_then_success()`
     - `test_timeout_retry_exhausted()`
     - `test_protocol_error_retried_and_succeeds()`
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_failures.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_failures.cpp)

8. **Unsupported command reports `BridgeError::Unsupported`.**
   - Enforced by `test_unsupported_command_maps_to_unsupported_error()`.
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_failures.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_failures.cpp)

9. **Link maintenance performs heartbeat/re-establish before command dispatch when needed.**
   - Enforced by:
     - `test_ensure_link_heartbeat_due_sends_heartbeat_then_command()`
     - `test_ensure_link_timeout_reestablishes_then_command_succeeds()`
     - `test_ensure_link_timeout_reestablish_failure_returns_false()`
   - Link: [`hexapod-server/tests/test_hardware_bridge_transport_capabilities.cpp`](../hexapod-server/tests/test_hardware_bridge_transport_capabilities.cpp)

### Ownership contract

- **`SimpleHardwareBridge` owns handshake state, capability negotiation state, retry policy, and API-level error mapping.**
- **Transport session/endpoint owns packet I/O mechanics** (send/receive boundaries and sequence carriage).
- **Bridge exposes high-level domain errors and metadata** to shield control/runtime code from packet-protocol specifics.

### Failure contract

- Transport-level unresponsiveness must not be silent; it becomes typed bridge errors.
- Protocol-level invalidity (bad seq, malformed payload, NACK) must map to protocol-classified failures with phase/domain metadata.
- When re-establishing link fails, command dispatch is aborted (no best-effort command on stale link).

### Uncovered invariants / follow-up tasks

1. **No property-based fuzz coverage for payload decode robustness.**
   - Add fuzz tests around ACK/NACK payload decoders and packet framing corruption.

2. **No long-run soak test for retry/backoff behavior over hours.**
   - Add endurance test validating no retry-counter drift/state leakage.

3. **No explicit timing-jitter contract for heartbeat cadence.**
   - Add tolerance-band tests for early/late heartbeat scheduling under scheduler jitter.

---

## 3) Scenario Engine Contract

### Invariants

1. **Invalid scenario mode names are rejected with explicit errors.**
   - Enforced by `test_invalid_mode_rejected()`.

2. **Sensor contacts array must be valid shape/length.**
   - Enforced by `test_invalid_contacts_size_rejected()`.

3. **Validation modes are contractually distinct:**
   - permissive mode accepts unknown keys/partial motion fields;
   - strict mode rejects unknown keys and motion fields without mode.
   - Enforced by:
     - `test_unknown_key_permissive_vs_strict()`
     - `test_invalid_combination_permissive_vs_strict()`

4. **Strict mode enforces gait and fault-value semantics.**
   - Enforced by:
     - `test_strict_invalid_gait_rejected()`
     - `test_strict_low_voltage_value_non_positive_rejected()`
     - `test_strict_high_current_value_non_positive_rejected()`
     - `test_strict_low_voltage_value_requires_low_voltage_enabled()`
     - `test_strict_high_current_value_requires_high_current_enabled()`

5. **Strict mode rejects contradictory sensor directives (`clear_contacts=true` with explicit contacts).**
   - Enforced by `test_strict_clear_contacts_with_contacts_rejected()`.

6. **Production scenario file for dynamic turn/priority/safety parses in strict mode and contains expected event categories.**
   - Enforced by `test_dynamic_turn_priority_safety_scenario_loads_strict()`.

- Link for all above: [`hexapod-server/tests/test_scenario_driver_validation.cpp`](../hexapod-server/tests/test_scenario_driver_validation.cpp)

### Ownership contract

- **`ScenarioDriver` owns TOML parsing and semantic validation boundaries** (permissive vs strict).
- **Scenario definition owns declarative test-time behavior**, while runtime consumers assume already-validated events.

### Failure contract

- Invalid scenario definitions fail fast at load with human-readable error messages.
- Strict mode acts as configuration safety gate; permissive mode supports exploratory iteration.

### Uncovered invariants / follow-up tasks

1. **Event ordering and overlap semantics are not explicitly contract-tested.**
   - Add tests for out-of-order timestamps, duplicate `at_ms`, and deterministic merge/precedence behavior.

2. **Runtime application semantics of parsed events are under-covered.**
   - Add end-to-end tests proving event execution timing aligns with `tick_ms`/`duration_ms` contract.

3. **No schema-version migration contract tests.**
   - Add forward/backward compatibility tests if scenario schema evolves.

---

## 4) Telemetry Pipeline Contract

### Invariants

1. **Serialized payload shape is stable (`type`, `schema_version`, `timestamp_ms`, `geometry`, `angles_deg`).**
   - Enforced by `test_exact_keys_and_schema_version()`.

2. **Leg key order is canonical and stable (`LF, LM, LR, RF, RM, RR`).**
   - Enforced by `test_leg_order_is_stable_and_canonical()`.

3. **Unit conversions are stable:**
   - geometry meters -> millimeters;
   - joint radians -> degrees.
   - Enforced by:
     - `test_geometry_unit_conversion_meters_to_mm()`
     - `test_joint_unit_conversion_radians_to_degrees()`

- Link: [`hexapod-server/tests/test_telemetry_json_serialization.cpp`](../hexapod-server/tests/test_telemetry_json_serialization.cpp)

4. **Visualiser mapping helpers are deterministic and unit-correct.**
   - Enforced by:
     - `testLegIdMappingIsDeterministic()`
     - `testJointConversionUsesDegrees()`
     - `testGeometryConversionUsesMillimeters()`

- Link: [`hexapod-server/tests/test_visualiser_telemetry.cpp`](../hexapod-server/tests/test_visualiser_telemetry.cpp)

5. **Runtime publish cadence contract:**
   - geometry published once at telemetry start;
   - control-step telemetry rate-limited by publish period for both successful and rejected steps.
   - Enforced by:
     - `runTelemetryCadenceSuccessCase()`
     - `runTelemetryCadenceRejectCase()`

- Link: [`hexapod-server/tests/test_robot_runtime_loop.cpp`](../hexapod-server/tests/test_robot_runtime_loop.cpp)

### Ownership contract

- **`telemetry_json` owns wire schema + canonical ordering + unit encoding.**
- **visualiser telemetry helpers own model-to-visualiser mapping semantics.**
- **runtime telemetry publisher owns emission cadence and geometry refresh policy.**

### Failure contract

- Telemetry generation must remain deterministic and parseable with fixed schema version.
- Invalid/rejected control steps still emit status telemetry on configured cadence (observability during fault handling).

### Uncovered invariants / follow-up tasks

1. **No backward-compatibility golden-file tests for schema changes.**
   - Add snapshot-based contract tests with representative payload fixtures.

2. **No transport-level delivery/error visibility tests for telemetry publisher.**
   - Add tests for publish failure counters, retry/drop behavior, and non-blocking guarantees.

3. **No performance contract for serialization latency/allocations under sustained loop rates.**
   - Add microbenchmarks and budget assertions (e.g., max serialization time per frame).

---

## Suggested Next Worklist (Prioritized)

1. Add explicit runtime stage-order and stale->recovery tests (safety-critical behavior).
2. Add scenario event ordering/overlap execution tests.
3. Add telemetry schema snapshot compatibility tests.
4. Add fuzz+soak tests for bridge protocol robustness.
