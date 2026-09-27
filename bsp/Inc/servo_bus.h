#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <stdint.h>

typedef enum {
    SERVO_BUS_OK = 0,
    SERVO_BUS_TIMEOUT = -1,
    SERVO_BUS_ERROR = -2
} ServoBusResult;

/* 发送结束后自动切回接收方向 */
ServoBusResult servo_bus_send(uint8_t *data, uint16_t length);
ServoBusResult servo_bus_receive(uint8_t *data, uint16_t length);

#endif