# locomotion_interface Contract (v1)

## Responsibilities
- Translate high-level motion command envelopes into hardware bridge writes.
- Encapsulate underlying bridge API (`IHardwareBridge::write(...)` and simulation equivalent).

## Inputs
- `motion_command` from `motion_arbiter`.

## Outputs
- Command dispatch result:
  - accepted/rejected
  - failure code/category
  - `timestamp_ms`

## Timing
- Must support control-loop cadence without blocking mission/navigation paths.
- Command transport failures must surface synchronously or via bounded-latency status channel.

## Fault Behavior
- On transport failure, raise typed fault and request safe stop/hold path through arbiter.
