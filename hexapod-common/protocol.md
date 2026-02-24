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

## Packet framing and re-sync strategy (example)

When using serial links, the receiver can lose packet boundaries if bytes are dropped or added. A simple, robust framing format is:

```text
[STX=0x7E][LEN][CMD][PAYLOAD ...][CRC16_LO][CRC16_HI][ETX=0x7F]
```

- `STX`: start delimiter.
- `LEN`: number of bytes in `CMD + PAYLOAD`.
- `CRC16`: checksum calculated over `LEN + CMD + PAYLOAD`.
- `ETX`: end delimiter.

If parsing fails (bad length, bad CRC, missing ETX), discard bytes until the next `STX` and attempt parsing again.

### C++ framing example (shared by server/client)

```cpp
#include <cstdint>
#include <vector>

namespace framing {

constexpr uint8_t STX = 0x7E;
constexpr uint8_t ETX = 0x7F;

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

std::vector<uint8_t> encodePacket(uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    const uint8_t len = static_cast<uint8_t>(1 + payload.size()); // CMD + PAYLOAD

    frame.push_back(STX);
    frame.push_back(len);
    frame.push_back(cmd);
    frame.insert(frame.end(), payload.begin(), payload.end());

    const uint16_t crc = crc16_ccitt(&frame[1], static_cast<size_t>(len) + 1); // LEN + CMD + PAYLOAD
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    frame.push_back(ETX);
    return frame;
}

struct DecodedPacket {
    uint8_t cmd{};
    std::vector<uint8_t> payload;
};

// rxBuffer is a rolling buffer of bytes read from UART.
// Returns true when one valid packet is decoded and removed from rxBuffer.
bool tryDecodePacket(std::vector<uint8_t>& rxBuffer, DecodedPacket& out) {
    while (!rxBuffer.empty()) {
        // Re-sync to STX.
        if (rxBuffer[0] != STX) {
            rxBuffer.erase(rxBuffer.begin());
            continue;
        }

        // Minimum frame: STX LEN CMD CRC1 CRC2 ETX => 6 bytes
        if (rxBuffer.size() < 6)
            return false;

        const uint8_t len = rxBuffer[1];
        const size_t frameSize = static_cast<size_t>(len) + 5; // STX+LEN + (LEN bytes) + CRC2 + ETX

        if (len == 0) {
            // Invalid length; drop STX and keep searching.
            rxBuffer.erase(rxBuffer.begin());
            continue;
        }

        if (rxBuffer.size() < frameSize)
            return false; // Wait for more bytes

        if (rxBuffer[frameSize - 1] != ETX) {
            // Framing error; drop STX and retry.
            rxBuffer.erase(rxBuffer.begin());
            continue;
        }

        const uint16_t rxCrc = static_cast<uint16_t>(rxBuffer[frameSize - 3]) |
                               (static_cast<uint16_t>(rxBuffer[frameSize - 2]) << 8);
        const uint16_t calcCrc = crc16_ccitt(&rxBuffer[1], static_cast<size_t>(len) + 1);
        if (rxCrc != calcCrc) {
            // Corrupt packet; drop STX and search next one.
            rxBuffer.erase(rxBuffer.begin());
            continue;
        }

        out.cmd = rxBuffer[2];
        out.payload.assign(rxBuffer.begin() + 3, rxBuffer.begin() + 2 + len);

        // Consume the decoded frame.
        rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + frameSize);
        return true;
    }

    return false;
}

} // namespace framing
```

Usage pattern on both sides:

1. Continuously append received UART bytes into `rxBuffer`.
2. Call `tryDecodePacket(rxBuffer, pkt)` in a loop.
3. If it returns `false`, wait for more bytes.
4. If out-of-sync/corrupt data appears, decoder self-recovers by scanning to the next `STX`.

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
- Payload: `calib[0][0], calib[0][1], calib[1][0], calib[1][1],..., calib[17][1]`    
calibration values for each of the 18 servos defining min (`calib[][0]`) and max (`calib[][1]`) values as `uint16`
- Response:
  - Acknowledged: `ACK (0x11)`   
  On successful transfer and calibration
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

### `SET_TARGET_ANGLE`

- Direction: Server -> Client
- Value: `SET_TARGET_ANGLE (0x02)`
- Payload: `servoID, angle`    
servoID as `uint8` and the corresponding angle as `uint16`
- Response:
  - Acknowledged: `ACK (0x11)`   
  On success
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

### `SET_POWER_RELAY`

- Direction: Server -> Client
- Value: `SET_POWER_RELAY (0x03)`
- Payload: `relayState`    
sets the state of the power relay `uint8`
- Response:
  - Acknowledged: `ACK (0x11)`   
  On success
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

### `GET_ANGLE_CALIBRATIONS`

- Direction: Server -> Client
- Value: `GET_ANGLE_CALIBRATIONS (0x04)`
- Payload: None    
requests the current angle calibrations
- Response:
  - Acknowledged: `ACK (0x11)` `calib[0][0], calib[0][1], calib[1][0], calib[1][1],..., calib[17][1]`   
  On success; calibration values for each of the 18 servos defining min (`calib[][0]`) and max (`calib[][1]`) values as `float`
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

### `GET_CURRENT`

- Direction: Server -> Client
- Value: `GET_CURRENT (0x05)`
- Payload: None  
requests the current value
- Response:
  - Acknowledged: `ACK (0x11)`, `currentValue`   
  On success
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

### `GET_VOLTAGE`

- Direction: Server -> Client
- Value: `GET_VOLTAGE (0x06)`
- Payload: None   
requests the voltage value
- Response:
  - Acknowledged: `ACK (0x11)`, `voltageValue`   
  On success
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

### `GET_SENSOR`

- Direction: Server -> Client
- Value: `GET_SENSOR (0x07)`
- Payload: `sensorID`   
requests the value of the sensor
- Response:
  - Acknowledged: `ACK (0x11)`, `sensorValue`   
  On success
  - Not Acknowledged: `NACK (0x12)`, `ERROR_CODE`
  On error
- Notes: ???

