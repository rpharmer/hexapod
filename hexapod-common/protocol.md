# Protocol

This document defines the protocol for communication between hexapod-server and hexapod-client.

## Handshake Protocol

- Host sends 3 bytes: `HELLO (0x10)`, `PROTOCOL_VERSION (0x01)`, `CAPABILITIES (bitmask)`.
- Firmware responds with 4 bytes: `ACK (0x11)`, `PROTOCOL_VERSION`, `STATUS (0=ok)`, `DEVICE_ID`.
- If version mismatches, firmware responds with `NACK (0x12)` and an error code.

The host will retry handshake protocol (3 attempts with 100ms delay) if it fails


