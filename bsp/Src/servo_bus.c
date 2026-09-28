#include "servo_bus.h"
#include "bsp_board.h"
#include "bsp_uart.h"

#include <stddef.h>
#include <string.h>

#define SERVO_UART_TX_TIMEOUT_MS 100U
#define SERVO_UART_RX_TIMEOUT_MS 20U

/**
  * @brief  将 BSP UART 操作结果转换为舵机总线结果。
  * @param  result: BSP UART 返回值。
  * @retval 对应的 ServoBusResult。
  */
static ServoBusResult bus_result(BspUartResult result)
{
    if (result == BSP_UART_OK) {
        return SERVO_BUS_OK;
    }
    if (result == BSP_UART_TIMEOUT) {
        return SERVO_BUS_TIMEOUT;
    }
    return SERVO_BUS_ERROR;
}

/**
  * @brief  通过 RS485 发送一帧请求并接收一帧应答。
  * @param  tx: 待发送帧的缓冲区。
  * @param  tx_length: 发送字节数。
  * @param  rx: 应答输出缓冲区；rx_length 为零时可以为 NULL。
  * @param  rx_length: 期望收到的应答字节数；零表示无需应答。
  * @retval SERVO_BUS_OK、超时、收发错误或应答长度错误。
  */
ServoBusResult servo_bus_exchange(uint8_t *tx, uint16_t tx_length,
                                  uint8_t *rx, uint16_t rx_length)
{
    BspUartManager *uart = Bsp_Board_ServoUart();
    if (tx == NULL || tx_length == 0U ||
        (rx_length != 0U && rx == NULL) || rx_length > uart->rx_capacity) {
        return SERVO_BUS_ERROR;
    }

    /* 需要应答时先准备接收，避免漏掉发送结束后的首字节。 */
    BspUartResult result = BSP_UART_OK;
    if (rx_length != 0U) {
        result = Bsp_Uart_StartReceiveToIdle(uart);
        if (result != BSP_UART_OK) {
            return bus_result(result);
        }
    }

    /* PB14 拉高进入发送；阻塞发送完成后立即拉低，等待舵机应答。 */
    Bsp_Board_Rs485_Set_Tx(1U);
    /* 同步写的最长协议帧为 259 字节，115200 bit/s 下发送约需 23 ms。 */
    result = Bsp_Uart_Transmit(uart, tx, tx_length, SERVO_UART_TX_TIMEOUT_MS);
    Bsp_Board_Rs485_Set_Tx(0U);
    if (result != BSP_UART_OK) {
        if (rx_length != 0U) {
            Bsp_Uart_AbortReceive(uart);
        }
        return bus_result(result);
    }
    if (rx_length == 0U) {
        return SERVO_BUS_OK;
    }

    uint16_t received = 0U;
    result = Bsp_Uart_WaitReceive(uart, SERVO_UART_RX_TIMEOUT_MS, &received);
    if (result != BSP_UART_OK) {
        Bsp_Uart_AbortReceive(uart);
        return bus_result(result);
    }
    /* 此层只检查字节数量；帧头、ID 和校验和由 ft_Servo 处理。 */
    if (received != rx_length) {
        return SERVO_BUS_LENGTH;
    }
    memcpy(rx, uart->rx_buffer, received);
    return SERVO_BUS_OK;
}
