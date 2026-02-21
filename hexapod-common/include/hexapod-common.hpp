// Header guard
#ifndef HEXAPOD_COMMON_H
#define HEXAPOD_COMMON_H

#include <stdint.h>

#define  SET_ANGLE_CALIBRATIONS   0x01
#define  SET_TARGET_ANGLE         0x02
#define  SET_POWER_RELAY          0x03
#define  GET_ANGLE_CALIBRATIONS   0x04
#define  GET_CURRENT              0x05
#define  GET_VOLTAGE              0x06
#define  GET_SENSOR               0x07


#define MSB0(x) ((uint8_t)((x) & 0xff))
#define MSB1(x) ((uint8_t)(((x) >>  8) & 0xff))
#define MSB2(x) ((uint8_t)(((x) >> 16) & 0xff))
#define MSB3(x) ((uint8_t)(((x) >> 24) & 0xff))

// c3 is most significant byte, c0 is least significant byte
#define GETINT(c3,c2,c1,c0)(((c3) << 24) | ((c2) << 16) | ((c1) << 8) | (c0))


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
};

#endif // #ifndef HEXAPOD_COMMON_H