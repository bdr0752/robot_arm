#include "ft_Servo.h"
#include "servo_bus.h"
#include "ft_servo_defs.h"


/**
  * @brief  向指定 ID 发送 PING，并验证应答帧。
  * @param  id: 舵机 ID，范围 0～253。
  * @param  status: 成功时写入应答中的状态字节。
  * @retval FT_SERVO_OK、超时、收发错误、应答格式错误、校验错误或参数错误。
  */
FtServoResult ft_servo_ping(uint8_t id, uint8_t *status)
{
    if (id > 253U || status == NULL) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    uint8_t tx[6] = {0xFF, 0xFF, id, 0x02, FT_SCS_INST_PING, 0x00};
    uint8_t rx[6] = {0};

    tx[5] = (uint8_t)~((uint32_t)id + tx[3] + tx[4]);

    ServoBusResult bus = servo_bus_exchange(tx, sizeof(tx), rx, sizeof(rx));
    if (bus != SERVO_BUS_OK) {
        if (bus == SERVO_BUS_LENGTH) {
            return FT_SERVO_BAD_REPLY;
        }
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
/**
  * @brief  读取指定舵机的当前位置寄存器，并验证应答帧。
  * @param  id: 舵机 ID，范围 0～253。
  * @param  position: 成功时写入位置寄存器的原始数值。
  * @param  status: 非 NULL 时在成功后写入应答状态字节。
  * @retval FT_SERVO_OK、超时、收发错误、应答格式错误、校验错误或参数错误。
  */
FtServoResult ft_servo_read_position(
    uint8_t id, uint16_t *position, uint8_t *status
)
{
    return ft_servo_read_word(id, FT_SMS_REG_PRESENT_POS_L, position, status);
}
