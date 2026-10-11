#pragma once
#include <stdint.h>

namespace BSP {
    class WS2812 {
    public:
        static void Init();
        static void SendGrb(const uint8_t *data, uint16_t size);
    };
}
