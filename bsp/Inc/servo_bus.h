#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <stdint.h>

typedef enum {
    SERVO_BUS_OK = 0,
    SERVO_BUS_TIMEOUT = -1,
    SERVO_BUS_ERROR = -2,
    SERVO_BUS_LENGTH = -3  /* 已收到数据，但长度与 rx_length 不一致 */
} ServoBusResult;

/*
 * 完成一次 RS485 请求/应答：先启动接收 DMA，再发送 tx，随后切回接收并等待。
 * 成功时将恰好 rx_length 字节复制到 rx；rx_length 为 0 时只发送，rx 可以为 NULL。
 * 帧内容和校验和由上层舵机驱动检查。
 * 只能由一个调用者串行使用这一路舵机 UART。
 */
ServoBusResult servo_bus_exchange(uint8_t *tx, uint16_t tx_length,
                                  uint8_t *rx, uint16_t rx_length);

#endif
