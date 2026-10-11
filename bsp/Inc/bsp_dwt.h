#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "stm32h7xx_hal.h"

/* 在系统时钟配置完成后调用一次，启用 CPU 周期计数。 */
void Bsp_Dwt_Init(void);

/**
  * @brief  获取当前 CPU 周期计数值。
  * @retval 32 位周期计数；计数器会自然回绕。
  */
__STATIC_FORCEINLINE uint32_t Bsp_Dwt_GetCycles(void)
{
    return DWT->CYCCNT;
}

/**
  * @brief  从指定起点忙等待若干 CPU 周期。
  * @param  start: 起始周期计数值。
  * @param  cycles: 等待的周期数；调用前须初始化 DWT。
  * @retval 无
  * @note   无符号减法处理一次计数回绕；中断会延长实际等待时间。
  */
__STATIC_FORCEINLINE void Bsp_Dwt_WaitCycles(uint32_t start, uint32_t cycles)
{
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

#endif
