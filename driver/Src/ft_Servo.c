#include "ft_servo.h"
#include "servo_bus.h"

FtServoResult ft_servo_ping(uint8_t id, uint8_t *status)
{
    if (id > 253U || status == NULL) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    uint8_t tx[6] = {0xFF, 0xFF, id, 0x02, 0x01, 0x00};
    uint8_t rx[6] = {0};

    tx[5] = (uint8_t)~((uint32_t)id + tx[3] + tx[4]);

    ServoBusResult bus = servo_bus_send(tx, sizeof(tx));
    if (bus != SERVO_BUS_OK) {
        return (bus == SERVO_BUS_TIMEOUT)
            ? FT_SERVO_TIMEOUT : FT_SERVO_IO_ERROR;
    }

    bus = servo_bus_receive(rx, sizeof(rx));
    if (bus != SERVO_BUS_OK) {
        return (bus == SERVO_BUS_TIMEOUT)
            ? FT_SERVO_TIMEOUT : FT_SERVO_IO_ERROR;
    }

    if (rx[0] != 0xFFU || rx[1] != 0xFFU ||
        rx[2] != id    || rx[3] != 0x02U) {
        return FT_SERVO_BAD_REPLY;
    }

    uint8_t checksum =
        (uint8_t)~((uint32_t)rx[2] + rx[3] + rx[4]);

    if (rx[5] != checksum) {
        return FT_SERVO_BAD_CHECKSUM;
    }

    *status = rx[4];
    return FT_SERVO_OK;
}