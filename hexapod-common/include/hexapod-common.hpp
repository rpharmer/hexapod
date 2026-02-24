// Header guard
#ifndef HEXAPOD_COMMON_H
#define HEXAPOD_COMMON_H

#include <stdint.h>
#include "framing.hpp"

// protocol version
const uint8_t PROTOCOL_VERSION       = 0x04;

// command codes
const uint8_t HELLO                  = 0x10;
const uint8_t ACK                    = 0x11;
const uint8_t NACK                   = 0x12;
const uint8_t SET_ANGLE_CALIBRATIONS = 0x01;
const uint8_t SET_TARGET_ANGLE       = 0x02;
const uint8_t SET_POWER_RELAY        = 0x03;
const uint8_t GET_ANGLE_CALIBRATIONS = 0x04;
const uint8_t GET_CURRENT            = 0x05;
const uint8_t GET_VOLTAGE            = 0x06;
const uint8_t GET_SENSOR             = 0x07;

// status codes
const uint8_t STATUS_OK              = 0x00;
const uint8_t STATUS_BUSY            = 0x01;

// error codes
const uint8_t TIMEOUT                = 0xA1;
const uint8_t VERSION_MISMATCH       = 0xA2;


/* Abstract base class defining which communication functions must be implemented. Server
 and Client use different libaries but derived class will hide this */
class SerialComms
{
public:
    // functions to send data
    virtual void send_char(char) = 0;    // send a char     (1 byte)
    virtual void send_u8(uint8_t) = 0;   // send a uint8_t  (1 byte)
    virtual void send_u16(uint16_t) = 0; // send a uint16_t (2 bytes)
    virtual void send_u32(uint32_t) = 0; // send a uint32_t (4 bytes)
    virtual void send_i16(int16_t) = 0;  // send a int16_t  (2 bytes)
    virtual void send_i32(int32_t) = 0;  // send a int32_t  (4 bytes)
    virtual void send_f32(float) = 0;    // send a float    (4 bytes)
    
    // functions to recieve data
    virtual int recv_char(char*) = 0;    // recieve a char     (1 byte) 
    virtual int recv_u8(uint8_t*) = 0;   // recieve a uint8_t  (1 bytes)
    virtual int recv_u16(uint16_t*) = 0; // recieve a uint16_t (2 bytes)
    virtual int recv_u32(uint32_t*) = 0; // recieve a uint32_t (4 bytes)
    virtual int recv_i16(int16_t*) = 0;  // recieve a int16_t  (2 bytes)
    virtual int recv_i32(int32_t*) = 0;  // recieve a int32_t  (4 bytes)
    virtual int recv_f32(float*) = 0;    // recieve a float    (4 bytes)
    
    
    // framed protocol helpers
    virtual void send_packet(uint8_t cmd, const std::vector<uint8_t>& payload) = 0;
    virtual bool recv_packet(DecodedPacket& packet) = 0;
    
protected:
    std::vector<uint8_t> rxBuffer;
};

#endif // #ifndef HEXAPOD_COMMON_H