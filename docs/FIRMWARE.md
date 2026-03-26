# Hexapod Serial Protocol

This document defines the wire protocol used between `hexapod-server` (host) and `hexapod-client` (firmware).

## Transport framing

All messages use a framed packet format:

```text
[STX=0x7E][LEN][SEQ_LO][SEQ_HI][CMD][PAYLOAD...][CRC16_LO][CRC16_HI][ETX=0x7F]
```

- `LEN`: byte count of `SEQ_LO + SEQ_HI + CMD + PAYLOAD`.
- `SEQ`: request/response sequence number (`uint16`, little-endian as `SEQ_LO` then `SEQ_HI`) used to correlate replies with requests.
- `CRC16`: CRC-CCITT over `LEN + SEQ_LO + SEQ_HI + CMD + PAYLOAD`.
- `STX/ETX`: start/end delimiters.

Reference implementation:

- `hexapod-common/include/framing.hpp`
- `hexapod-common/framing.cpp`

Decoder behavior on both host and firmware:

1. Append incoming bytes to a rolling RX buffer.
2. Repeatedly call `tryDecodePacket(rxBuffer, packet)`.
3. On parse errors (`LEN`, `CRC`, or `ETX` invalid), decoder drops bytes and re-syncs to the next `STX`.

## Common constants

- Protocol version: `0x04`
- ACK/NACK:
  - `ACK  = 0x11`
  - `NACK = 0x12`
- Status:
  - `STATUS_OK = 0x00`
  - `STATUS_BUSY = 0x01`
- Errors:
  - `TIMEOUT = 0xA1`
  - `VERSION_MISMATCH = 0xA2`
  - `INVALID_ARGUMENT = 0xA3`
  - `INVALID_PAYLOAD_LENGTH = 0xA4`
  - `OUT_OF_RANGE_INDEX = 0xA5`
  - `UNSUPPORTED_COMMAND = 0xA6`
  - `BUSY_NOT_READY = 0xA7`
  - `ALREADY_PAIRED = 0xA8`

## Command table

| Command | Value | Direction | Request payload | Response |
|---|---:|---|---|---|
| _Sequence handling_ | - | Bidirectional | Every request includes a 2-byte `SEQ` in the frame header. | Every response echoes the same `SEQ` value from the request. |
| `HELLO` | `0x10` | Server → Client | `version:u8, capabilities:u8` | `ACK(version:u8,status:u8,device_id:u8)` or `NACK(error:u8)` |
| `SET_ANGLE_CALIBRATIONS` | `0x01` | Server → Client | 18 × `(min:f32, max:f32)` | `ACK` or `NACK(error:u8)` |
| `SET_TARGET_ANGLE` | `0x02` | Server → Client | `servo_id:u8, angle:f32` | `ACK` or `NACK(error:u8)` |
| `SET_POWER_RELAY` | `0x03` | Server → Client | `relay_state:u8` (`0`/`1`) | `ACK` or `NACK(error:u8)` |
| `GET_ANGLE_CALIBRATIONS` | `0x04` | Server → Client | _none_ | `ACK` + 18 × `(min:f32, max:f32)` or `NACK(error:u8)` |
| `GET_CURRENT` | `0x05` | Server → Client | _none_ | `ACK(current:f32)` or `NACK(error:u8)` |
| `GET_VOLTAGE` | `0x06` | Server → Client | _none_ | `ACK(voltage:f32)` or `NACK(error:u8)` |
| `GET_SENSOR` | `0x07` | Server → Client | `sensor_id:u8` | `ACK(voltage:f32)` or `NACK(error:u8)` |
| `SET_JOINT_TARGETS` | `0x13` | Server → Client | 18 × `target_pos_rad:f32` | `ACK` or `NACK(error:u8)` |
| `GET_FULL_HARDWARE_STATE` | `0x14` | Server → Client | _none_ | `ACK` + 18 × `joint_pos_rad:f32`, 6 × `foot_contact:u8`, `voltage:f32`, `current:f32` or `NACK(error:u8)` |
| `SET_SERVOS_ENABLED` | `0x15` | Server → Client | 18 × `enable:bool` | `ACK` or `NACK(error:u8)` |
| `GET_SERVOS_ENABLED` | `0x16` | Server → Client | _none_ | `ACK` + 18 × `enable:bool` or `NACK(error:u8)` |
| `SET_SERVOS_TO_MID` | `0x17` | Server → Client | _none_ | `ACK` or `NACK(error:u8)` |
| `KILL` | `0x18` | Server → Client | _none_ | no response required (firmware exits active loop) |
| `GET_LED_INFO` | `0x19` | Server → Client | _none_ | `ACK` + LED state payload or `NACK(error:u8)` |
| `SET_LED_COLORS` | `0x1A` | Server → Client | RGB payload (`kProtocolLedColorsPayloadBytes`) | `ACK` or `NACK(error:u8)` |

> Multibyte numeric fields are serialized in little-endian byte order.

## Handshake sequence

1. Server sends framed `HELLO` request with payload `PROTOCOL_VERSION` and capability flags.
2. Client validates version compatibility and readiness.
3. Client responds with either:
   - `ACK` payload: `PROTOCOL_VERSION, STATUS_OK, DEVICE_ID`, or
   - `NACK` payload with an error code (for example `VERSION_MISMATCH`).
4. Server retries up to three times if handshake fails or times out.

## Implementation notes

- Host implementation uses `SerialCommsServer::send_packet` / `recv_packet`.
- Firmware implementation uses `SerialCommsClient::send_packet` / `recv_packet`.
- Both wrappers use `encodePacket` and `tryDecodePacket` from `hexapod-common`.

## Command ID typing rule (internal vs edge)

**Rule:** use `CommandCode` (typed enum) for all internal APIs and control flow. Raw `uint8_t` command IDs are allowed only at explicit wire/compatibility seams.
Short version: **typed internally, raw at the protocol edges**.

### Allowed raw-ID seams

- Packet framing and transport boundaries where bytes are serialized/deserialized:
  - `hexapod-common/include/framing.hpp` + `hexapod-common/framing.cpp`
  - `SerialComms::send_packet(...)` and `DecodedPacket::cmd`
  - `TransportSession::send(...)` and `wait_for_ack(...)`
- Protocol compatibility aliases in `hexapod-common/include/hexapod-common.hpp` (for gradual migration only).

### Internal typing requirements

- Convert raw decoded `packet.cmd` to `CommandCode` immediately via `try_parse_command_code(...)`.
- Route matching and command switches must be typed (`CommandCode`), not raw integers.
- Host command APIs (`CommandClient`, `BridgeCommandApi`) accept `CommandCode` parameters only.
- Prefer enum names (`CommandCode::ACK`, `ErrorCode::INVALID_PAYLOAD_LENGTH`, etc.) over raw/legacy aliases in new internal logic.
