#ifndef FT_SERVO_H
#define FT_SERVO_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    FT_SERVO_OK = 0,
    FT_SERVO_TIMEOUT = -1,
    FT_SERVO_IO_ERROR = -2,
    FT_SERVO_BAD_REPLY = -3,
    FT_SERVO_BAD_CHECKSUM = -4,
    FT_SERVO_BAD_ARGUMENT = -5
} FtServoResult;

FtServoResult ft_servo_ping(uint8_t id, uint8_t *status);

#endif