#include "bsp_uart.hpp"

// DMA1 无法访问 DTCM，沿用 .dma_buffer / RAM_D2 链接配置。
__attribute__((section(".dma_buffer"),
               aligned(32))) uint8_t BSP::UART1::rxbuf[RXBUF_SIZE];
__attribute__((section(".dma_buffer"),
               aligned(32))) uint8_t BSP::UART3::rxbuf[RXBUF_SIZE];
BSP::UartCallback BSP::UART1::callback = 0;
BSP::UartCallback BSP::UART3::callback = 0;

/**
 * @brief 分发空闲与写满事件，交付后继续接收。
 * @param huart 串口句柄。
 * @param size 实际长度。
 * @retval 无
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t size) {
  if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    return;
  if (huart == &huart1)
    BSP::UART1::OnReceive(size);
  else if (huart == &huart3)
    BSP::UART3::OnReceive(size);
}

/**
 * @brief 分发 UART 错误并重新接收。
 * @param huart 串口句柄。
 * @retval 无
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1)
    BSP::UART1::OnError();
  else if (huart == &huart3)
    BSP::UART3::OnError();
}
