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

std::vector<uint8_t> encodePacket(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    const uint8_t len = static_cast<uint8_t>(3 + payload.size()); // SEQ(2) + CMD + PAYLOAD

    frame.push_back(STX);
    frame.push_back(len);
    frame.push_back(static_cast<uint8_t>(seq & 0xFF));
    frame.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
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

        // Minimum frame: STX LEN SEQ_LO SEQ_HI CMD CRC1 CRC2 ETX => 8 bytes
        if (rxBuffer.size() < 8)
            return false;

        const uint8_t len = rxBuffer[1];
        const size_t frameSize = static_cast<size_t>(len) + 5; // STX+LEN + (LEN bytes) + CRC2 + ETX

        if (len < 3) {
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

        out.seq = static_cast<uint16_t>(rxBuffer[2]) |
                  (static_cast<uint16_t>(rxBuffer[3]) << 8);
        out.cmd = rxBuffer[4];
        out.payload.assign(rxBuffer.begin() + 5, rxBuffer.begin() + 2 + len);

        // Consume the decoded frame.
        rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + frameSize);
        return true;
    }

    return false;
}

// Generic inline template to get a specific byte from any integer type
template <typename T>
inline uint8_t getByte(T value, size_t index) {
    static_assert(std::is_integral<T>::value, "T must be an integral type");

    using UnsignedT = typename std::make_unsigned<T>::type;

    // Ensure index is within valid range
    if (index >= sizeof(T)) {
        throw std::out_of_range("Byte index out of range");
    }

    // Shift right by (index * 8) bits and mask to get the desired byte
    return static_cast<uint8_t>(
        (static_cast<UnsignedT>(value) >> (index * 8)) & 0xFF
    );
}
