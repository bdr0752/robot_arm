#include "bsp_rs485.hpp"

/**
 * @brief 将本板 RS485 设置为接收方向。
 * @retval 无
 */
void BSP::RS485::Init()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

/**
 * @brief 切换发送方向，阻塞发送后恢复接收方向。
 * @param data 有效发送地址。
 * @param size 数据字节数。
 * @param timeout_ms HAL 等待超时，单位毫秒。
 * @retval 无
 */
void BSP::RS485::Transmit(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    UART3::Transmit(data, size, timeout_ms);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
