#include "bsp_board.hpp"
#include "bsp_board.h"
#include "gpio.h"
#include "dma.h"

/**
 * @brief 初始化外设并启动 BSP 模块，系统时钟配置后调用一次。
 * @retval 无
 */
void BSP::Init()
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_USART1_UART_Init();

    Dwt::Init();
    WS2812::Init();
    RS485::Init();
    UART1::Init();
    UART3::Init();
}

/**
 * @brief 为 main.c 提供 BSP 初始化入口。
 * @retval 无
 */
extern "C" void Bsp_Board_Init(void)
{
    BSP::Init();
}
