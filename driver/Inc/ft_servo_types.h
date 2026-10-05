#ifndef FT_SERVO_TYPES_H
#define FT_SERVO_TYPES_H

#include <stdint.h>

typedef enum {
    FT_SERVO_OK = 0,
    FT_SERVO_TIMEOUT = -1,
    FT_SERVO_IO_ERROR = -2,
    FT_SERVO_BAD_REPLY = -3,
    FT_SERVO_BAD_CHECKSUM = -4,
    FT_SERVO_BAD_ARGUMENT = -5
} FtServoResult;

#endif
