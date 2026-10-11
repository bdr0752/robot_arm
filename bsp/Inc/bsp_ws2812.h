#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#include <stdint.h>

/* GPIO 初始化完成后调用，将本板 PA7 数据线保持为低电平。 */
void Bsp_Ws2812_Init(void);

/* 输出 GRB 字节流；调用前须初始化 GPIO 与 DWT。 */
void Bsp_Ws2812_SendGrb(const uint8_t *data, uint16_t length);

#endif
