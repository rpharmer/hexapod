// Header guard
#ifndef HEXAPOD_COMMON_H
#define HEXAPOD_COMMON_H

#include <stdint.h>
#include "framing.hpp"

// protocol version
const uint8_t PROTOCOL_VERSION       = 0x04;

// command codes
const uint8_t SET_ANGLE_CALIBRATIONS = 0x01;
const uint8_t SET_TARGET_ANGLE       = 0x02;
const uint8_t SET_POWER_RELAY        = 0x03;
const uint8_t GET_ANGLE_CALIBRATIONS = 0x04;
const uint8_t GET_CURRENT            = 0x05;
const uint8_t GET_VOLTAGE            = 0x06;
const uint8_t GET_SENSOR             = 0x07;
const uint8_t HEARTBEAT              = 0x08;
const uint8_t DIAGNOSTIC             = 0x09;
const uint8_t HELLO                  = 0x10;
const uint8_t ACK                    = 0x11;
const uint8_t NACK                   = 0x12;
const uint8_t SET_JOINT_TARGETS      = 0x13;
const uint8_t GET_FULL_HARDWARE_STATE= 0x14;


// status codes
const uint8_t STATUS_OK              = 0x00;
const uint8_t STATUS_BUSY            = 0x01;

// error codes
const uint8_t TIMEOUT                = 0xA1;
const uint8_t VERSION_MISMATCH       = 0xA2;
const uint8_t INVALID_ARGUMENT       = 0xA3;
const uint8_t INVALID_PAYLOAD_LENGTH = 0xA4;
const uint8_t OUT_OF_RANGE_INDEX     = 0xA5;
const uint8_t UNSUPPORTED_COMMAND    = 0xA6;
const uint8_t BUSY_NOT_READY         = 0xA7;

// enum to map joints to ints
// [R/L] is Right/ Left handside from point of view of robot
// 1st number is Leg number 1 being front legs, 2 middle legs and 3 being back legs
// 2nd number is 1 -> Coxa, 2 -> Femur, 3 -> Tibia
// order is [2nd (R>L)][1st (descending)][3rd (ascending)]
// 0 - 17 corresponds to physical pin (1 - 18)
enum class JointID : uint8_t {
  R31 = 0,
  R32 = 1,
  R33 = 2,
  L31 = 3,
  L32 = 4,
  L33 = 5,
  R21 = 6,
  R22 = 7,
  R23 = 8,
  L21 = 9,
  L22 = 10,
  L23 = 11,
  R11 = 12,
  R12 = 13,
  R13 = 14,
  L11 = 15,
  L12 = 16,
  L13 = 17
};

enum class LegID : uint8_t {
  R3 = 0,
  L3 = 1,
  R2 = 2,
  L2 = 3,
  R1 = 4,
  L1 = 5
};

enum class LegJointID : uint8_t {
  Coxa = 0,
  Femur = 1,
  Tibia = 2
};

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
    virtual void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) = 0;
    virtual bool recv_packet(DecodedPacket& packet) = 0;
    
protected:
    std::vector<uint8_t> rxBuffer;
};

#endif // #ifndef HEXAPOD_COMMON_H
