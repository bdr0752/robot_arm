#include "ft_servo_bus.h"
#include "bsp_dwt.hpp"
#include "bsp_rs485.hpp"
#include <string.h>

static uint8_t Rx_Buffer[32];
static volatile uint16_t Rx_Size;
static volatile bool Rx_Done;
static volatile bool Rx_Waiting;

/**
 * @brief 在中断中保存本次应答，等待与应答状态由舵机层管理。
 * @param data 本次数据；0 表示 UART 接收错误。
 * @param size 实际字节数，不超过 USART3 的 32 字节 DMA 缓冲区。
 * @retval 无
 */
static void Receive(const uint8_t *data, uint16_t size) {
  if (!Rx_Waiting || Rx_Done)
    return;
  if (data)
    memcpy(Rx_Buffer, data, size);
  Rx_Size = data ? size : 0U;
  __DMB();
  Rx_Done = true;
}

/**
 * @brief 发送飞特请求并等待指定长度的应答。
 * @param tx 有效请求数据。
 * @param tx_length 请求字节数。
 * @param rx 应答输出地址；无需应答时可为 0。
 * @param rx_length 期望应答字节数，0 表示只发送。
 * @retval 总线结果；无需应答时 OK 仅表示发送 DMA 已启动。
 * @note 只能在主循环或任务中串行调用。协议校验由 ft_servo_protocol 完成。
 */
FtServoBusResult ft_servo_bus_exchange(uint8_t *tx, uint16_t tx_length,
                                       uint8_t *rx, uint16_t rx_length) {
  if (rx_length > sizeof(Rx_Buffer))
    return FT_SERVO_BUS_LENGTH;
  if (BSP::UART3::IsBusy())
    return FT_SERVO_BUS_ERROR;
  if (rx_length == 0U) {
    return BSP::RS485::Transmit(tx, tx_length) ? FT_SERVO_BUS_OK
                                               : FT_SERVO_BUS_ERROR;
  }

  BSP::UART3::RegisterCallback(Receive);
  Rx_Done = false;
  Rx_Size = 0U;
  Rx_Waiting = true;
  // 接收由 UART 回调持续重启；请求之间不再停止正在运行的 DMA。
  if (BSP::UART3::Init() != HAL_OK) {
    Rx_Waiting = false;
    return FT_SERVO_BUS_ERROR;
  }
  if (!BSP::RS485::Transmit(tx, tx_length)) {
    Rx_Waiting = false;
    return FT_SERVO_BUS_ERROR;
  }

  uint32_t start = BSP::Dwt::GetTick();
  // 保持协议层同步接口；UART 的 DMA 启动本身不等待。
  while (BSP::UART3::IsBusy()) {
    if ((uint32_t)(BSP::Dwt::GetTick() - start) >= 100U) {
      Rx_Waiting = false;
      return FT_SERVO_BUS_TIMEOUT;
    }
  }
  start = BSP::Dwt::GetTick();
  while (!Rx_Done) {
    if ((uint32_t)(BSP::Dwt::GetTick() - start) >= 20U) {
      Rx_Waiting = false;
      return FT_SERVO_BUS_TIMEOUT;
    }
  }
  __DMB();
  Rx_Waiting = false;
  if (Rx_Size == 0U)
    return FT_SERVO_BUS_ERROR;
  if (Rx_Size != rx_length)
    return FT_SERVO_BUS_LENGTH;
  memcpy(rx, Rx_Buffer, rx_length);
  return FT_SERVO_BUS_OK;
}
