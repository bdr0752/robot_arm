#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* 在 HAL_Init() 和 SystemClock_Config() 之后调用一次，集中初始化板上外设。 */
void Bsp_Board_Init(void);

#ifdef __cplusplus
}
#endif

#endif
