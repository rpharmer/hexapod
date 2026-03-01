# Hexapod Serial Protocol

This document defines the wire protocol used between `hexapod-server` (host) and `hexapod-client` (firmware).

## Transport framing

All messages use a framed packet format:

```text
[STX=0x7E][LEN][SEQ][CMD][PAYLOAD...][CRC16_LO][CRC16_HI][ETX=0x7F]
```

- `LEN`: byte count of `SEQ + CMD + PAYLOAD`.
- `SEQ`: request/response sequence number used to correlate replies with commands.
- `CRC16`: CRC-CCITT over `LEN + SEQ + CMD + PAYLOAD`.
- `STX/ETX`: start/end delimiters.

Reference implementation lives in:
- `hexapod-common/include/framing.hpp`
- `hexapod-common/framing.cpp`

Both server and client should:
1. Append incoming bytes to a rolling RX buffer.
2. Repeatedly call `tryDecodePacket(rxBuffer, packet)`.
3. On parse errors (`LEN`, `CRC`, or `ETX` invalid), decoder drops bytes and re-syncs to next `STX`.

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

## Command table

| Command | Value | Direction | Request payload | Response |
|---|---:|---|---|---|
| _Sequence handling_ | - | Bidirectional | Every request includes a `SEQ` byte in the frame header. | Every response echoes the same `SEQ` byte from the request. |
| `HELLO` | `0x10` | Server → Client | `version:u8, capabilities:u8` | `ACK(version:u8,status:u8,device_id:u8)` or `NACK(error:u8)` |
| `SET_ANGLE_CALIBRATIONS` | `0x01` | Server → Client | 18 × `(min:u16, max:u16)` | `ACK` or `NACK(error:u8)` |
| `SET_TARGET_ANGLE` | `0x02` | Server → Client | `servo_id:u8, angle:u16` | `ACK` or `NACK(error:u8)` |
| `SET_POWER_RELAY` | `0x03` | Server → Client | `relay_state:u8` (`0`/`1`) | `ACK` or `NACK(error:u8)` |
| `GET_ANGLE_CALIBRATIONS` | `0x04` | Server → Client | _none_ | `ACK` + 18 × `(min:f32, max:f32)` or `NACK(error:u8)` |
| `GET_CURRENT` | `0x05` | Server → Client | _none_ | `ACK(current:f32)` or `NACK(error:u8)` |
| `GET_VOLTAGE` | `0x06` | Server → Client | _none_ | `ACK(voltage:f32)` or `NACK(error:u8)` |
| `GET_SENSOR` | `0x07` | Server → Client | `sensor_id:u8` | `ACK(voltage:f32)` or `NACK(error:u8)` |

> Multibyte numeric fields are serialized in little-endian byte order.

## Handshake sequence

1. Server sends framed `HELLO` request with payload:
   - `PROTOCOL_VERSION`
   - requested capabilities bitmask
2. Client validates version.
3. Client responds with:
   - `ACK` payload: `PROTOCOL_VERSION, STATUS_OK, DEVICE_ID`, or
   - `NACK` payload: error code (`VERSION_MISMATCH`, etc.)
4. Server retries up to 3 times if handshake fails or times out.

## Implementation notes

- Host implementation uses `SerialCommsServer::send_packet` / `recv_packet`.
- Firmware implementation uses `SerialCommsClient::send_packet` / `recv_packet`.
- Both wrappers are built on top of `encodePacket` and `tryDecodePacket` from `hexapod-common`.
