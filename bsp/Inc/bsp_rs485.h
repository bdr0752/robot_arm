#ifndef BSP_RS485_H
#define BSP_RS485_H

#include "bsp_uart.h"

/* 方向引脚高电平发送、低电平接收；资源由 bsp_board 绑定。 */
typedef struct BspRs485 {
    BspUartManager *uart;
    GPIO_TypeDef *direction_port;
    uint16_t direction_pin;
} BspRs485;

/* 绑定已有 UART 与方向引脚，并设置为接收方向，不初始化外设。 */
void Bsp_Rs485_Attach(BspRs485 *bus, BspUartManager *uart,
                       GPIO_TypeDef *direction_port, uint16_t direction_pin);
/* 阻塞发送，HAL 调用返回后恢复接收方向；不返回发送状态。 */
void Bsp_Rs485_Transmit(BspRs485 *bus, uint8_t *data,
                         uint16_t length, uint32_t timeout_ms);

#endif
