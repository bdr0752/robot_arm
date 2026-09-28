#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

#define WS2812_LED_COUNT 1U

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Ws2812Color;

void ws2812_init(void);
void ws2812_set_pixel(uint32_t index, Ws2812Color color);
void ws2812_set_all(Ws2812Color color);
void ws2812_show(void);

#endif
