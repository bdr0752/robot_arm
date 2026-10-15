#include "bsp_rs485.hpp"

/**
 * @brief 将本板 RS485 设置为接收方向。
 * @retval 无
 */
void BSP::RS485::Init()
{
    UART3::RegisterTxCallback(ReceiveDirection);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

/**
 * @brief 切换发送方向并启动 DMA，完成回调负责恢复接收方向。
 * @param data 有效发送地址。
 * @param size 数据字节数。
 * @retval true 为启动成功，false 为忙或启动失败。
 */
bool BSP::RS485::Transmit(const uint8_t *data, uint16_t size)
{
    if (UART3::IsBusy()) return false;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    if (UART3::Transmit(data, size)) return true;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    return false;
}

/**
 * @brief 发送完成或 HAL 终止发送后切回接收。
 * @retval 无
 */
void BSP::RS485::ReceiveDirection()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
