#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
    BSP_UART_OK = 0,
    BSP_UART_TIMEOUT = -1,
    BSP_UART_ERROR = -2,
    BSP_UART_ARGUMENT = -3
} BspUartResult;

typedef struct BspUartManager BspUartManager;
/* 接收结束时在 UART/DMA 中断上下文调用；size 是本次实际收到的字节数。 */
typedef void (*BspUartRxCallback)(BspUartManager *manager,
                                  const uint8_t *data, uint16_t size);

/* 一个对象管理一路已初始化 UART 的 DMA 接收状态。 */
struct BspUartManager {
    UART_HandleTypeDef *huart;    /* CubeMX/HAL 初始化好的 UART 句柄 */
    uint8_t *rx_buffer;           /* DMA 可访问的接收缓冲区 */
    uint16_t rx_capacity;         /* 缓冲区容量，单位：字节 */
    volatile uint16_t rx_size;    /* 本次实际收到的字节数 */
    volatile uint8_t rx_done;     /* 中断已报告接收结束或错误 */
    volatile uint8_t rx_error;    /* HAL 报告接收错误 */
    BspUartRxCallback callback;   /* 可选的用户回调 */
};

/* 绑定已有 huart 和缓冲区；不初始化 UART，也不分配缓冲区。 */
BspUartResult Bsp_Uart_Attach(BspUartManager *manager,
                             UART_HandleTypeDef *huart,
                             uint8_t *rx_buffer, uint16_t rx_capacity);
/* 回调可设为 NULL；回调内不要执行耗时或阻塞操作。 */
void Bsp_Uart_SetRxCallback(BspUartManager *manager, BspUartRxCallback callback);
/* 清除上次状态并启动 DMA 接收；收到空闲帧或填满缓冲区时结束。 */
BspUartResult Bsp_Uart_StartReceiveToIdle(BspUartManager *manager);
/* 阻塞发送；HAL_UART_Transmit 返回时最后一个字节已发送完毕。 */
BspUartResult Bsp_Uart_Transmit(BspUartManager *manager,
                                uint8_t *data, uint16_t length,
                                uint32_t timeout_ms);
/* 等待中断设置 rx_done；超时会停止接收，成功时通过 size 返回实际长度。 */
BspUartResult Bsp_Uart_WaitReceive(BspUartManager *manager,
                                   uint32_t timeout_ms, uint16_t *size);
/* 停止当前接收，用于发送失败或接收出错后的清理。 */
void Bsp_Uart_AbortReceive(BspUartManager *manager);





#endif
