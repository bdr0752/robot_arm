#pragma once
#include "stm32h7xx_hal.h"

namespace BSP {
    class Dwt {
    public:
        static void Init();

        /**
         * @brief 读取 CPU 周期计数。
         * @retval 自然回绕的 32 位计数
         */
        __STATIC_FORCEINLINE uint32_t GetCycles() { return DWT->CYCCNT; }

        /**
         * @brief 读取系统毫秒计数。
         * @retval HAL tick
         */
        static uint32_t GetTick() { return HAL_GetTick(); }

        /**
         * @brief 从起点忙等待周期数。
         * @param start 起始计数。
         * @param cycles 周期数。
         * @retval 无
         */
        __STATIC_FORCEINLINE void WaitCycles(uint32_t start, uint32_t cycles) {
            while ((uint32_t)(DWT->CYCCNT - start) < cycles) {}
        }
    };
}
