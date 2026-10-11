#ifndef TEST_VOFA_BSP_UART_H
#define TEST_VOFA_BSP_UART_H

#include <stdint.h>

typedef enum {
    BSP_UART_OK = 0,
    BSP_UART_TIMEOUT = -1,
    BSP_UART_ERROR = -2,
    BSP_UART_ARGUMENT = -3
} BspUartResult;

/* 主机替身只保留 vofa.c 使用的 BSP 状态，不包含 HAL 外设。 */
typedef struct BspUartManager {
    uint8_t *rx_buffer;
    uint16_t rx_capacity;
    volatile uint16_t rx_size;
    volatile uint8_t rx_done;
    volatile uint8_t rx_error;
} BspUartManager;

BspUartResult Bsp_Uart_StartReceiveToIdle(BspUartManager *manager);
BspUartResult Bsp_Uart_WaitReceive(BspUartManager *manager,
                                  uint32_t timeout_ms, uint16_t *size);
void Bsp_Uart_AbortReceive(BspUartManager *manager);
void Bsp_Uart_Transmit(BspUartManager *manager, uint8_t *data,
                               uint16_t length, uint32_t timeout_ms);

#endif
