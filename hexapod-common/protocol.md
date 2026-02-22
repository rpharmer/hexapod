# Protocol

This document defines the protocol for communication between hexapod-server and hexapod-client.

## Handshake Protocol

- Host sends 3 bytes: `HELLO (0x10)`, `PROTOCOL_VERSION (0x01)`, `CAPABILITIES (bitmask)`.
- Firmware responds with 4 bytes: `ACK (0x11)`, `PROTOCOL_VERSION`, `STATUS (0=ok)`, `DEVICE_ID`.
- If version mismatches, firmware responds with `NACK (0x12)` and an error code.

The host will retry handshake protocol (3 attempts with 100ms delay) if it fails.

## Communication Protocol

| Command Byte             | Value | Explantation                                            |
|--------------------------|-------|---------------------------------------------------------|
| `HELLO`                  | `0x10`| Begin handshake protocol                                |
| `ACK`                    | `0x11`| Acknowledged                                            |
| `NACK`                   | `0x12`| Not Acknowledged                                        |
| `SET_ANGLE_CALIBRATIONS` | `0x01`| Set angle calibrations using values from server         |
| `SET_TARGET_ANGLE`       | `0x02`| Set target angle using value from server                |
| `SET_POWER_RELAY`        | `0x03`| Set power relay state                                   |
| `GET_ANGLE_CALIBRATIONS` | `0x04`| Get angle calibrations                                  |
| `GET_CURRENT`            | `0x05`| Get current                                             |
| `GET_VOLTAGE`            | `0x06`| Get voltage                                             |
| `GET_SENSOR`             | `0x07`| Get sensor reading                                      |

## Commands

### `HELLO`

- Direction: Server -> Client
- Value: `HELLO (0x01)`
- Payload: `PROTOCOL_VERSION (0x01)`, `CAPABILITIES (bitmask)`
- Response:
  - Acknowledged: `ACK (0x11)`, `PROTOCOL_VERSION`, `STATUS (0=ok)`, `DEVICE_ID`   
  Assuming versions match, client is ready and capabilities are available.
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`   
  Sent if versions are mismatched, client isn't available or capabilities can't be met.
- Notes: Server will attempt to retry handshake 3 times, leaving 100ms delay between.

### `SET_ANGLE_CALIBRATIONS`

- Direction: Server -> Client
- Value: `SET_ANGLE_CALIBRATIONS (0x01)`
- Payload: calib[0][0],calib[0][1],calib[1][0]calib[1][1],..., calib[17][1]    
calibration values for each of the 18 servos defining min (calib[][0]) and max (calib[][1]) values as `uint16`
- Response:
  - Acknowledged: `ACK (0x11)`   
  On successful transfer and calibration
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

