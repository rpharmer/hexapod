// Header guard
#ifndef FRAMING_H
#define FRAMING_H

#include <stdint.h>
#include <stdio.h>

#ifdef TARGET_PICO
#include <pico/stdlib.h>
#endif

#include <vector>

constexpr uint8_t STX = 0x7E;
constexpr uint8_t ETX = 0x7F;

uint16_t crc16_ccitt(const uint8_t* data, size_t len);
std::vector<uint8_t> encodePacket(uint8_t cmd, const std::vector<uint8_t>& payload);

struct DecodedPacket {
    uint8_t cmd{};
    std::vector<uint8_t> payload;
};

bool tryDecodePacket(std::vector<uint8_t>& rxBuffer, DecodedPacket& out);

#endif