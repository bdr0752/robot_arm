#include "ws2812.h"
#include "bsp_board.h"

static Ws2812Color pixels[WS2812_LED_COUNT];

void ws2812_init(void)
{
    Ws2812Color off = {0U, 0U, 0U};
    ws2812_set_all(off);
}

void ws2812_set_pixel(uint32_t index, Ws2812Color color)
{
    if (index < WS2812_LED_COUNT) {
        pixels[index] = color;
    }
}

void ws2812_set_all(Ws2812Color color)
{
    for (uint32_t i = 0U; i < WS2812_LED_COUNT; ++i) {
        pixels[i] = color;
    }
}

void ws2812_show(void)
{
    uint8_t grb[WS2812_LED_COUNT * 3U];
    for (uint32_t i = 0U; i < WS2812_LED_COUNT; ++i) {
        grb[3U * i] = pixels[i].green;
        grb[3U * i + 1U] = pixels[i].red;
        grb[3U * i + 2U] = pixels[i].blue;
    }
    bsp_board_ws2812_send_grb(grb, sizeof(grb));
}
