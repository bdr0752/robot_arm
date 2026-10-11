#include "bsp_dwt.hpp"

/**
 * @brief 启用 DWT 并清零周期计数，初始化时调用一次。
 * @retval 无
 */
void BSP::Dwt::Init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
