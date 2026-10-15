#pragma once
#include "bsp_uart.hpp"

namespace BSP {
    // 本板 USART3 + PB14；PB14 同时控制 DE 和 RE#。
    class RS485 {
    public:
        static void Init();
        static bool Transmit(const uint8_t *data, uint16_t size);
    private:
        static void ReceiveDirection();
    };
}
