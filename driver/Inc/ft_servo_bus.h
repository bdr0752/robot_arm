#ifndef FT_SERVO_BUS_H
#define FT_SERVO_BUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FT_SERVO_BUS_OK = 0,
    FT_SERVO_BUS_TIMEOUT = -1,
    FT_SERVO_BUS_ERROR = -2,
    FT_SERVO_BUS_LENGTH = -3  /* 已收到数据，但长度与 rx_length 不一致 */
} FtServoBusResult;

/*
 * 完成一次 RS485 请求/应答：保持接收 DMA，再启动发送 DMA，发送完成中断切回接收。
 * 成功时将恰好 rx_length 字节复制到 rx；rx_length 为 0 时只发送，rx 可以为 NULL。
 * 有应答时同步等待：发送完成最多 100 ms，随后等待应答最多 20 ms。
 * 无应答调用立即返回，OK 仅表示 DMA 已启动；发送忙或启动失败返回 ERROR。
 * 有应答请求在接收启动失败时返回 ERROR，不停止正在进行的接收 DMA。
 * 帧内容和校验和由上层舵机驱动检查。
 * 只能由一个调用者串行使用这一路舵机 UART。
 */
FtServoBusResult ft_servo_bus_exchange(uint8_t *tx, uint16_t tx_length,
                                  uint8_t *rx, uint16_t rx_length);

#ifdef __cplusplus
}
#endif

#endif
