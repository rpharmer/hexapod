#include "framing.hpp"


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

std::vector<uint8_t> encodePacket(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    const uint8_t len = static_cast<uint8_t>(2 + payload.size()); // SEQ + CMD + PAYLOAD

    frame.push_back(STX);
    frame.push_back(len);
    frame.push_back(seq);
    frame.push_back(cmd);
    frame.insert(frame.end(), payload.begin(), payload.end());

    const uint16_t crc = crc16_ccitt(&frame[1], static_cast<size_t>(len) + 1); // LEN + SEQ + CMD + PAYLOAD
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    frame.push_back(ETX);
    return frame;
}

// rxBuffer is a rolling buffer of bytes read from UART.
// Returns true when one valid packet is decoded and removed from rxBuffer.
bool tryDecodePacket(std::vector<uint8_t>& rxBuffer, DecodedPacket& out) {
    while (!rxBuffer.empty()) {
        // Re-sync to STX.
        if (rxBuffer[0] != STX) {
            rxBuffer.erase(rxBuffer.begin());
            continue;
        }

        // Minimum frame: STX LEN SEQ CMD CRC1 CRC2 ETX => 7 bytes
        if (rxBuffer.size() < 7)
            return false;

        const uint8_t len = rxBuffer[1];
        const size_t frameSize = static_cast<size_t>(len) + 5; // STX+LEN + (LEN bytes) + CRC2 + ETX

        if (len < 2) {
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

        out.seq = rxBuffer[2];
        out.cmd = rxBuffer[3];
        out.payload.assign(rxBuffer.begin() + 4, rxBuffer.begin() + 2 + len);

        // Consume the decoded frame.
        rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + frameSize);
        return true;
    }

    return false;
}

