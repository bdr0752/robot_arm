#include "ft_servo.h"
#include "servo_bus.h"
#include "ft_servo_defs.h"


/*
    PING 指令的实现。
    * 返回值：
    *  FT_SERVO_OK：收到指定 ID 的有效应答
    *  FT_SERVO_TIMEOUT：超时，该 ID 未应答
    *  FT_SERVO_IO_ERROR：发送或接收失败
    *  FT_SERVO_BAD_REPLY：应答的帧头、ID 或长度错误
    *  FT_SERVO_BAD_CHECKSUM：校验和错误
    *  FT_SERVO_BAD_ARGUMENT：参数错误（ID 超过 253 或 status 为 NULL）
    *
    * 有效应答中的状态字节通过 status 返回。
*/
FtServoResult ft_servo_ping(uint8_t id, uint8_t *status)
{
    if (id > 253U || status == NULL) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    uint8_t tx[6] = {0xFF, 0xFF, id, 0x02, FT_SCS_INST_PING, 0x00};
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
/*
    读取舵机当前位置的实现。
    * 返回值：
    *  FT_SERVO_OK：收到指定 ID 的有效应答
    *  FT_SERVO_TIMEOUT：超时，该 ID 未应答
    *  FT_SERVO_IO_ERROR：发送或接收失败
    *  FT_SERVO_BAD_REPLY：应答的帧头、ID 或长度错误
    *  FT_SERVO_BAD_CHECKSUM：校验和错误
    *  FT_SERVO_BAD_ARGUMENT：参数错误（ID 超过 253 或 position/status 为 NULL）
    *
    * 有效应答中的状态字节通过 status 返回，位置通过 position 返回。
*/
FtServoResult ft_servo_read_position(
    uint8_t id, uint16_t *position, uint8_t *status
)
{
    if (id > 253U || position == NULL || status == NULL) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    /* FF FF | ID | 长度4 | READ | 起始地址 | 读取2字节 | 校验 */
    uint8_t tx[8] = {
        0xFF, 0xFF, id, 0x04,
        FT_SCS_INST_READ, FT_SMS_REG_PRESENT_POS_L, 0x02, 0x00
    };
    uint8_t rx[8] = {0};

    tx[7] = (uint8_t)~(
        (uint32_t)tx[2] + tx[3] + tx[4] + tx[5] + tx[6]
    );

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

    /* 应答：FF FF | ID | 长度4 | 状态 | 位置低字节 | 位置高字节 | 校验 */
    if (rx[0] != 0xFFU || rx[1] != 0xFFU ||
        rx[2] != id    || rx[3] != 0x04U) {
        return FT_SERVO_BAD_REPLY;
    }

    uint8_t checksum = (uint8_t)~(
        (uint32_t)rx[2] + rx[3] + rx[4] + rx[5] + rx[6]
    );
    if (rx[7] != checksum) {
        return FT_SERVO_BAD_CHECKSUM;
    }

    *status = rx[4];
    *position = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
    return FT_SERVO_OK;
}
