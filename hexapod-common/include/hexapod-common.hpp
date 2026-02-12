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

#endif // #ifndef HEXAPOD_COMMON_H