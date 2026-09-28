#include "bsp_board.h"
#include "bsp_uart.h"

#include "gpio.h"
#include "dma.h"
#include "usart.h"

/* 本工程的普通 .bss 位于 DTCM，DMA1 无法访问；链接脚本将该缓冲区放到 RAM_D2。 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t servo_rx_buffer[32];
static BspUartManager servo_uart;

/**
  * @brief  初始化本板 GPIO、DMA、USART3 和舵机串口管理对象。
  * @note   须在 HAL_Init() 与 SystemClock_Config() 之后调用一次。
  * @retval 无
  */
void Bsp_Board_Init(void)
{
    /* DMA 必须先于 USART3 初始化，使 UART 的 DMA 句柄能够完成绑定。 */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();

    /* 上电后先保持接收方向，WS2812 数据线保持低电平。 */
    Bsp_Board_Rs485_Set_Tx(0U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

    /* WS2812 的软件时序使用 DWT 周期计数器。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 只绑定已初始化的 huart3；UART 初始化仍由上面的 MX 函数完成。 */
    if (Bsp_Uart_Attach(&servo_uart, &huart3,
                        servo_rx_buffer, sizeof(servo_rx_buffer)) != BSP_UART_OK) {
        Error_Handler();
    }
}

/**
  * @brief  切换 RS485 收发方向。
  * @param  enabled: 非零为发送，零为接收。
  * @retval 无
  */
void Bsp_Board_Rs485_Set_Tx(uint8_t enabled)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,
                      enabled != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  获取板上舵机 UART 的接收管理对象。
  * @retval 已绑定 huart3 的管理对象指针。
  */
BspUartManager *Bsp_Board_ServoUart(void)
{
    return &servo_uart;
}

/**
  * @brief  等待 DWT 计数器经过指定的 CPU 周期数。
  * @param  start: 起始周期计数值。
  * @param  cycles: 需要等待的周期数。
  * @retval 无
  */
static void board_wait_cycles(uint32_t start, uint32_t cycles)
{
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

/**
  * @brief  在 PA7 按 WS2812 时序输出 GRB 字节流。
  * @param  data: 按 GRB 顺序排列的数据缓冲区。
  * @param  length: 数据字节数；为零时不发送。
  * @retval 无
  */
void bsp_board_ws2812_send_grb(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) {
        return;
    }

    /* 根据当前 SystemCoreClock 换算 WS2812 脉宽，单位为 CPU 周期。 */
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
            uint32_t start = DWT->CYCCNT;

            GPIOA->BSRR = pin;
            board_wait_cycles(start, high);
            GPIOA->BSRR = pin << 16U;
            board_wait_cycles(start, bit_period);
            value <<= 1U;
        }
    }
    GPIOA->BSRR = pin << 16U;
    __set_PRIMASK(primask);

    /* 帧结束后保持低电平 1 ms，使灯珠锁存本次颜色。 */
    board_wait_cycles(DWT->CYCCNT, cycles_per_us * 1000U);
}
