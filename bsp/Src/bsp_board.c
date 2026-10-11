#include "bsp_board.h"
#include "bsp_uart.h"
#include "bsp_rs485.h"
#include "bsp_dwt.h"
#include "bsp_ws2812.h"

#include "gpio.h"
#include "dma.h"
#include "usart.h"

/* 本工程的普通 .bss 位于 DTCM，DMA1 无法访问；链接脚本将该缓冲区放到 RAM_D2。 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t servo_rx_buffer[32];
static BspUartManager servo_uart;
static BspRs485 servo_rs485;

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t debug_rx_buffer[128];
static BspUartManager debug_uart;

/**
  * @brief  初始化本板 GPIO、DMA、USART3、USART1 和串口管理对象。
  * @note   须在 HAL_Init() 与 SystemClock_Config() 之后调用一次。
  * @retval 无
  */
void Bsp_Board_Init(void)
{
    /* DMA 必须先于 UART 初始化，使 UART 的 DMA 句柄能够完成绑定。 */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_USART1_UART_Init();

    /* WS2812 数据线保持低电平。 */
    Bsp_Ws2812_Init();

    /* WS2812 的软件时序使用 DWT 周期计数器。 */
    Bsp_Dwt_Init();

    /* 只绑定已初始化的 huart3；UART 初始化仍由上面的 MX 函数完成。 */
    if (Bsp_Uart_Attach(&servo_uart, &huart3,
                        servo_rx_buffer, sizeof(servo_rx_buffer)) != BSP_UART_OK) {
        Error_Handler();
    }
    if (Bsp_Uart_Attach(&debug_uart, &huart1,
                        debug_rx_buffer, sizeof(debug_rx_buffer)) != BSP_UART_OK) {
        Error_Handler();
    }
    /* PB14 同时控制 DE 和 RE#，绑定后保持接收方向。 */
    Bsp_Rs485_Attach(&servo_rs485, &servo_uart, GPIOB, GPIO_PIN_14);
}

/**
  * @brief  获取板上舵机 RS485 对象。
  * @retval 已绑定 USART3 与 PB14 方向引脚的对象指针。
  */
BspRs485 *Bsp_Board_ServoRs485(void)
{
    return &servo_rs485;
}

/**
  * @brief  获取 USART1 调试串口的 DMA 接收管理对象。
  * @retval 已绑定 huart1 的管理对象指针，不与舵机串口共用缓冲区。
  */
BspUartManager *Bsp_Board_DebugUart(void)
{
    return &debug_uart;
}

/**
  * @brief  兼容旧的板级 WS2812 发送接口，转交独立模块输出。
  * @param  data: GRB 字节流。
  * @param  length: 数据字节数。
  * @retval 无
  */
void bsp_board_ws2812_send_grb(const uint8_t *data, uint16_t length)
{
    Bsp_Ws2812_SendGrb(data, length);
}
