#include "bsp_rs485.h"

/**
  * @brief  绑定 UART 和 RS485 方向引脚，并切换到接收方向。
  * @param  bus: RS485 对象。
  * @param  uart: 已绑定的 UART 管理对象。
  * @param  direction_port: 已初始化的方向 GPIO 端口。
  * @param  direction_pin: 方向 GPIO 引脚；高电平发送、低电平接收。
  * @retval 无
  */
void Bsp_Rs485_Attach(BspRs485 *bus, BspUartManager *uart,
                       GPIO_TypeDef *direction_port, uint16_t direction_pin)
{
    bus->uart = uart;
    bus->direction_port = direction_port;
    bus->direction_pin = direction_pin;
    HAL_GPIO_WritePin(direction_port, direction_pin, GPIO_PIN_RESET);
}

/**
  * @brief  切换到发送方向，阻塞发送后恢复接收方向。
  * @param  bus: 已绑定的 RS485 对象。
  * @param  data: 待发送的有效数据缓冲区。
  * @param  length: 发送字节数。
  * @param  timeout_ms: HAL 发送等待超时时间，单位毫秒。
  * @retval 无
  * @note   使用阻塞 UART 发送，不处理设备应答，也不报告发送状态。
  */
void Bsp_Rs485_Transmit(BspRs485 *bus, uint8_t *data,
                         uint16_t length, uint32_t timeout_ms)
{
    HAL_GPIO_WritePin(bus->direction_port, bus->direction_pin, GPIO_PIN_SET);
    Bsp_Uart_Transmit(bus->uart, data, length, timeout_ms);
    HAL_GPIO_WritePin(bus->direction_port, bus->direction_pin, GPIO_PIN_RESET);
}
