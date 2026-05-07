# Algorithms Glossary

## Core roles

- **Supervisor**: A policy layer that can inhibit motion or force safer behavior based on system state.
- **Safety supervisor**: `SafetySupervisor`; evaluates fault rules and maintains fault lifecycle/latching state.
- **Freshness gate**: `RuntimeFreshnessGate`; validates estimator/intent stream freshness and gates control-step admission.
- **Governor**: `CommandGovernor`; scales and reshapes requested locomotion commands according to stability/risk indicators.
- **Module**: A stage in the control pipeline (command processor, gait scheduler, stability, body controller, IK).

## Locomotion terms

- **Intent**: Desired robot behavior (`MotionIntent`) from operator/navigation layers.
- **Pipeline**: Ordered control-stage chain that converts intent + estimator + safety state into joint targets.
- **Gait state**: Scheduler/stability state for stance/swing timing and support metrics.
- **Control status**: Per-step status summary (mode, fault, loop counter, validity).

## Simulation terms

- **Broadphase**: Fast overlap culling over fat AABB proxies to produce candidate body pairs.
- **Narrowphase**: Contact generation for candidate pairs (specialized routines + GJK/EPA fallback).
- **Manifold**: Contact grouping used for warm-starting and friction/normal impulse solving.
- **Island**: Connected component of bodies/constraints solved as a unit.
- **TOI**: Time-of-impact continuous collision pass used to reduce tunneling.
- **Warm-start**: Reuse of previous-step impulse state to improve iterative convergence.

## Integration terms

- **Navigation manager**: Produces and tracks map-aware path segments and bridge lifecycle.
- **Nav-locomotion bridge**: Converts navigation progress/errors into locomotion intent updates.
- **Telemetry snapshot**: Structured runtime sample used for observability and debugging.