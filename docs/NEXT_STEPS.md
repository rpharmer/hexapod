# Next Steps (2026-03-20)

## Immediate (this sprint)

- Introduce immutable `ControlConfig` and thread it through runtime constructors.
- Add a test-enabled CMake preset and CI `ctest` lane.
- Refactor host transport into a single transaction helper.

## Near-term (next 1-2 sprints)

- Add stream sample IDs and freshness gates in control loop.
- Add strict scenario schema validation and malformed input tests.
- Add telemetry counters/histograms for loop jitter and command latency.

## Longer-term

- De-singleton firmware command handlers.
- Evaluate deterministic single-tick pipeline coordinator with async bus integration.
- Add hardware-in-the-loop nightly suite with selected scenarios.
