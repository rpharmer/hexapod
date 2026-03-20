// Header guard
#ifndef FRAMING_H
#define FRAMING_H

#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include <type_traits> // For std::is_integral, std::make_unsigned
#include <stdexcept>   // For std::out_of_range

#ifdef TARGET_PICO
#include <pico/stdlib.h>
#endif

#include <vector>

constexpr uint8_t STX = 0x7E;
constexpr uint8_t ETX = 0x7F;
constexpr std::size_t MAX_RX_BUFFER_BYTES = 1024;
constexpr std::size_t MAX_TRANSPORT_RX_BUFFER_BYTES = 1024;
constexpr std::size_t FRAME_LENGTH_FIELD_MAX = 0xFF;
constexpr std::size_t FRAME_HEADER_BYTES_WITHOUT_PAYLOAD = 3; // SEQ(2) + CMD
constexpr std::size_t FRAME_MAX_PAYLOAD_BYTES = FRAME_LENGTH_FIELD_MAX - FRAME_HEADER_BYTES_WITHOUT_PAYLOAD;

uint16_t crc16_ccitt(const uint8_t* data, size_t len);
std::vector<uint8_t> encodePacket(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload);

struct DecodedPacket {
    uint16_t seq{};
    uint8_t cmd{};
    std::vector<uint8_t> payload;
};

bool tryDecodePacket(std::vector<uint8_t>& rxBuffer, DecodedPacket& out);

template <typename T>
inline void append_scalar(std::vector<uint8_t>& buffer, const T value)
{
    const auto* begin = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), begin, begin + sizeof(T));
}

template <typename T>
inline bool read_scalar(const std::vector<uint8_t>& payload, std::size_t& offset, T& out)
{
    if (offset + sizeof(T) > payload.size())
    {
        return false;
    }

    std::memcpy(&out, payload.data() + offset, sizeof(T));
    offset += sizeof(T);
    return true;
}

//template <typename T>
//inline uint8_t getByte(T value, size_t index);

#endif
