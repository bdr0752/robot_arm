#include "ft_servo_bus.h"
#include "bsp_board.h"
#include "bsp_rs485.h"

#include <stddef.h>
#include <string.h>

#define FT_SERVO_TX_TIMEOUT_MS 100U
#define FT_SERVO_RX_TIMEOUT_MS 20U

/**
  * @brief  将 BSP UART 操作结果转换为舵机总线结果。
  * @param  result: BSP UART 返回值。
  * @retval 对应的 FtServoBusResult。
  */
static FtServoBusResult bus_result(BspUartResult result)
{
    if (result == BSP_UART_OK) {
        return FT_SERVO_BUS_OK;
    }
    if (result == BSP_UART_TIMEOUT) {
        return FT_SERVO_BUS_TIMEOUT;
    }
    return FT_SERVO_BUS_ERROR;
}

/**
  * @brief  通过 RS485 发送一帧请求并接收一帧应答。
  * @param  tx: 待发送帧的缓冲区。
  * @param  tx_length: 发送字节数。
  * @param  rx: 应答输出缓冲区；rx_length 为零时可以为 NULL。
  * @param  rx_length: 期望收到的应答字节数；零表示无需应答。
  * @retval FT_SERVO_BUS_OK、接收超时、接收错误或应答长度错误。
  * @note   不报告 HAL 发送状态；无需应答时 OK 表示调用了发送接口。
  */
FtServoBusResult ft_servo_bus_exchange(uint8_t *tx, uint16_t tx_length,
                                  uint8_t *rx, uint16_t rx_length)
{
    BspRs485 *bus = Bsp_Board_ServoRs485();
    BspUartManager *uart = bus->uart;
    if (tx == NULL || tx_length == 0U ||
        (rx_length != 0U && rx == NULL) || rx_length > uart->rx_capacity) {
        return FT_SERVO_BUS_ERROR;
    }

    /* 需要应答时先准备接收，避免漏掉发送结束后的首字节。 */
    BspUartResult result = BSP_UART_OK;
    if (rx_length != 0U) {
        result = Bsp_Uart_StartReceiveToIdle(uart);
        if (result != BSP_UART_OK) {
            return bus_result(result);
        }
    }

    /* RS485 模块负责方向切换；本层只组织舵机请求应答。 */
    Bsp_Rs485_Transmit(bus, tx, tx_length, FT_SERVO_TX_TIMEOUT_MS);
    if (rx_length == 0U) {
        return FT_SERVO_BUS_OK;
    }

    uint16_t received = 0U;
    result = Bsp_Uart_WaitReceive(uart, FT_SERVO_RX_TIMEOUT_MS, &received);
    if (result != BSP_UART_OK) {
        /* 等待超时时 UART 已中止接收；接收错误仍需在这里清理。 */
        if (result != BSP_UART_TIMEOUT) {
            Bsp_Uart_AbortReceive(uart);
        }
        return bus_result(result);
    }
    /* 此层只检查字节数量；帧头、ID 和校验和由 ft_servo_protocol 处理。 */
    if (received != rx_length) {
        return FT_SERVO_BUS_LENGTH;
    }
    memcpy(rx, uart->rx_buffer, received);
    return FT_SERVO_BUS_OK;
}
