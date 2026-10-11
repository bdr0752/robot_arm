#pragma once
#include "bsp_dwt.hpp"
#include "bsp_rs485.hpp"
#include "bsp_ws2812.hpp"

namespace BSP {
    // 在 HAL_Init() 和 SystemClock_Config() 后调用一次。
    void Init();
}
