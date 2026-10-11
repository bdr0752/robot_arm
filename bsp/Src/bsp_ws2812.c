#include "bsp_ws2812.h"
#include "bsp_dwt.h"

#include <stddef.h>

/**
  * @brief  将本板 WS2812 数据线 PA7 置为低电平。
  * @retval 无
  * @note   引脚模式由 CubeMX 初始化，此处只设置输出电平。
  */
void Bsp_Ws2812_Init(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

/**
  * @brief  在 PA7 按 WS2812 时序输出 GRB 字节流。
  * @param  data: 按 GRB 顺序排列的数据缓冲区。
  * @param  length: 数据字节数；为零时不发送。
  * @retval 无
  * @note   发送期间关闭中断，结束后恢复进入函数前的中断状态。
  */
void Bsp_Ws2812_SendGrb(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) {
        return;
    }

    /* 沿用原有脉宽，根据当前 CPU 时钟换算周期数。 */
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    const uint32_t zero_high = (cycles_per_us * 3U) / 10U;
    const uint32_t one_high = (cycles_per_us * 7U) / 10U;
    const uint32_t bit_period = (cycles_per_us * 5U) / 4U;
    const uint32_t pin = GPIO_PIN_7;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    for (uint16_t i = 0U; i < length; ++i) {
        uint8_t value = data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            uint32_t high = (value & 0x80U) != 0U ? one_high : zero_high;
            uint32_t start = Bsp_Dwt_GetCycles();

            GPIOA->BSRR = pin;
            Bsp_Dwt_WaitCycles(start, high);
            GPIOA->BSRR = pin << 16U;
            Bsp_Dwt_WaitCycles(start, bit_period);
            value <<= 1U;
        }
    }
    GPIOA->BSRR = pin << 16U;
    __set_PRIMASK(primask);

    /* 帧结束后保持低电平 1 ms，使灯珠锁存本次颜色。 */
    Bsp_Dwt_WaitCycles(Bsp_Dwt_GetCycles(), cycles_per_us * 1000U);
}
