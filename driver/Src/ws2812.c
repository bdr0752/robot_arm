#include "ws2812.h"
#include "bsp_board.h"

static Ws2812Color pixels[WS2812_LED_COUNT];

/**
  * @brief  将 WS2812 的颜色缓存初始化为全灭。
  * @retval 无
  */
void ws2812_init(void)
{
    Ws2812Color off = {0U, 0U, 0U};
    ws2812_set_all(off);
}

/**
  * @brief  修改指定灯珠的缓存颜色，不立即输出。
  * @param  index: 灯珠索引；越界时不执行操作。
  * @param  color: RGB 颜色值。
  * @retval 无
  */
void ws2812_set_pixel(uint32_t index, Ws2812Color color)
{
    if (index < WS2812_LED_COUNT) {
        pixels[index] = color;
    }
}

/**
  * @brief  将所有灯珠的缓存颜色设为相同值，不立即输出。
  * @param  color: RGB 颜色值。
  * @retval 无
  */
void ws2812_set_all(Ws2812Color color)
{
    for (uint32_t i = 0U; i < WS2812_LED_COUNT; ++i) {
        pixels[i] = color;
    }
}

/**
  * @brief  把颜色缓存转换成 GRB 数据并发送到灯珠。
  * @retval 无
  */
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
