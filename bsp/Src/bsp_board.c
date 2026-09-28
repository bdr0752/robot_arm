#include "bsp_board.h"

#include <stddef.h>

#include "gpio.h"
#include "usart.h"

static BspBoardResult board_uart_result(HAL_StatusTypeDef result)
{
    if (result == HAL_OK) {
        return BSP_BOARD_OK;
    }
    if (result == HAL_TIMEOUT) {
        return BSP_BOARD_TIMEOUT;
    }
    return BSP_BOARD_ERROR;
}

void Bsp_Board_Init(void)
{
    bsp_board_rs485_set_tx(0U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Bsp_Board_Rs485_Set_Tx(uint8_t enabled)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,
                      enabled != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

BspBoardResult Bsp_Board_UART_Transmit(
    uint8_t *data, uint16_t length, uint32_t timeout_ms
)
{
    if (data == NULL || length == 0U) {
        return BSP_BOARD_ERROR;
    }

    return board_uart_result(
        HAL_UART_Transmit(&huart3, data, length, timeout_ms)
    );
}

BspBoardResult Bsp_Board_UART_Receive(
    uint8_t *data, uint16_t length, uint32_t timeout_ms
)
{
    if (data == NULL || length == 0U) {
        return BSP_BOARD_ERROR;
    }

    return board_uart_result(
        HAL_UART_Receive(&huart3, data, length, timeout_ms)
    );
}

static void board_wait_cycles(uint32_t start, uint32_t cycles)
{
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

void bsp_board_ws2812_send_grb(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) {
        return;
    }

    /* DWT counts core cycles; this project configures the core at 550 MHz. */
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

    /* A long low interval latches the frame, including newer WS2812 variants. */
    board_wait_cycles(DWT->CYCCNT, cycles_per_us * 1000U);
}
