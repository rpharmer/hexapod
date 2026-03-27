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
  - `ContractEnvelope.stream_id = autonomy.locomotion.command_output`

## Boundary Validation
- The locomotion command output boundary is validated with `contract_enforcer` before dispatch.
- Required metadata: `contract_version` (`v1.x`), `frame_id`, `correlation_id`, `sample_id`, and fresh `timestamp_ms`.
- Duplicate or out-of-order `sample_id` values on `autonomy.locomotion.command_output` are rejected.

## Timing
- Must support control-loop cadence without blocking mission/navigation paths.
- Command transport failures must surface synchronously or via bounded-latency status channel.

## Fault Behavior
- On transport failure, raise typed fault and request safe stop/hold path through arbiter.
