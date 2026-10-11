#include "bsp_dwt.h"

/**
  * @brief  启用 DWT 周期计数器并将计数清零。
  * @retval 无
  * @note   系统初始化时调用一次；重复调用会重置计时起点。
  */
void Bsp_Dwt_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
