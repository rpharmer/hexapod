# Refactoring Review

See the canonical in-depth review at `docs/CODEBASE_REVIEW.md`.

## Priority refactor list

1. Protocol payload codec unification in `hexapod-common`.
2. Firmware handshake gating fix (state transitions only on successful handshake).
3. Safety fault latching state machine.
4. `tomlParser(...)` decomposition into dedicated parser units.
5. Shared protocol dimension constants to remove `18`/`6` literals.
6. Consolidated ACK/NACK transaction helper in host bridge.
7. `DoubleBuffer` synchronization model simplification.

