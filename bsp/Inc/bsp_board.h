#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include <stdint.h>

typedef struct BspUartManager BspUartManager;

/* 在 HAL_Init() 和 SystemClock_Config() 之后调用一次，集中初始化板上外设。 */
void Bsp_Board_Init(void);

/* PB14 同时控制 RS485 的 DE 和 RE#：非零为发送，零为接收。 */
void Bsp_Board_Rs485_Set_Tx(uint8_t enabled);
/* 返回板上舵机串口的接收管理对象；调用前先执行 Bsp_Board_Init()。 */
BspUartManager *Bsp_Board_ServoUart(void);

/* 在 PA7 输出 WS2812 数据；data 按 GRB 顺序存放。 */
void bsp_board_ws2812_send_grb(const uint8_t *data, uint16_t length);

#endif
