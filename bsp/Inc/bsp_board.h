#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include <stdint.h>

typedef struct BspUartManager BspUartManager;
typedef struct BspRs485 BspRs485;

/* 在 HAL_Init() 和 SystemClock_Config() 之后调用一次，集中初始化板上外设。 */
void Bsp_Board_Init(void);

/* 返回绑定 USART3 与 PB14 方向引脚的 RS485 对象；须先初始化本板。 */
BspRs485 *Bsp_Board_ServoRs485(void);
/* 返回 USART1 调试串口管理对象；调用前先执行 Bsp_Board_Init()。 */
BspUartManager *Bsp_Board_DebugUart(void);

/* 兼容旧调用；新代码使用 bsp_ws2812.h 中的 Bsp_Ws2812_SendGrb()。 */
void bsp_board_ws2812_send_grb(const uint8_t *data, uint16_t length);

#endif
