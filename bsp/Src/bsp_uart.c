#include "bsp_uart.h"

#include <stddef.h>

#define BSP_UART_MAX_MANAGERS 4U

/* HAL 回调只提供 huart，用此表找回对应的接收管理对象。 */
static BspUartManager *managers[BSP_UART_MAX_MANAGERS];

/**
  * @brief  根据 HAL UART 句柄查找对应的接收管理对象。
  * @param  huart: 触发回调的 UART 句柄。
  * @retval 管理对象指针；未绑定时返回 NULL。
  */
static BspUartManager *find_manager(UART_HandleTypeDef *huart)
{
    for (uint32_t i = 0U; i < BSP_UART_MAX_MANAGERS; ++i) {
        if (managers[i] != NULL && managers[i]->huart == huart) {
            return managers[i];
        }
    }
    return NULL;
}

/**
  * @brief  将已初始化的 UART 和 DMA 接收缓冲区绑定到管理对象。
  * @param  manager: 待绑定的管理对象。
  * @param  huart: 已完成 HAL 初始化的 UART 句柄。
  * @param  rx_buffer: DMA 可访问的接收缓冲区。
  * @param  rx_capacity: 接收缓冲区容量，单位为字节。
  * @retval BSP_UART_OK、参数错误或绑定失败。
  */
BspUartResult Bsp_Uart_Attach(BspUartManager *manager,
                             UART_HandleTypeDef *huart,
                             uint8_t *rx_buffer, uint16_t rx_capacity)
{
    if (manager == NULL || huart == NULL || rx_buffer == NULL || rx_capacity == 0U) {
        return BSP_UART_ARGUMENT;
    }
    /* 同一路 UART 只能绑定一个管理对象，避免两个缓冲区争用接收 DMA。 */
    if (find_manager(huart) != NULL) {
        return BSP_UART_ERROR;
    }

    for (uint32_t i = 0U; i < BSP_UART_MAX_MANAGERS; ++i) {
        if (managers[i] == NULL) {
            manager->huart = huart;
            manager->rx_buffer = rx_buffer;
            manager->rx_capacity = rx_capacity;
            manager->rx_size = 0U;
            manager->rx_done = 0U;
            manager->rx_error = 0U;
            manager->callback = NULL;
            managers[i] = manager;
            return BSP_UART_OK;
        }
    }
    return BSP_UART_ERROR;
}

/**
  * @brief  设置收到空闲帧或缓冲区写满时调用的用户回调。
  * @param  manager: 已绑定的管理对象。
  * @param  callback: 回调函数；NULL 表示不调用用户回调。
  * @retval 无
  */
void Bsp_Uart_SetRxCallback(BspUartManager *manager, BspUartRxCallback callback)
{
    if (manager != NULL) {
        manager->callback = callback;
    }
}

/**
  * @brief  清除旧状态并启动 UART DMA 接收至空闲事件。
  * @param  manager: 已绑定的管理对象。
  * @retval BSP_UART_OK、参数错误或 HAL 启动失败。
  */
BspUartResult Bsp_Uart_StartReceiveToIdle(BspUartManager *manager)
{
    if (manager == NULL || manager->huart == NULL ||
        manager->huart->hdmarx == NULL || manager->rx_buffer == NULL ||
        manager->rx_capacity == 0U) {
        return BSP_UART_ARGUMENT;
    }

    /* 每次接收都是一次新的请求/应答，先清除上次的结果。 */
    manager->rx_size = 0U;
    manager->rx_done = 0U;
    manager->rx_error = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(manager->huart, manager->rx_buffer,
                                     manager->rx_capacity) != HAL_OK) {
        HAL_UART_AbortReceive(manager->huart);
        return BSP_UART_ERROR;
    }
    /* 半满事件不是完整应答；只保留空闲和缓冲区写满事件。 */
    __HAL_DMA_DISABLE_IT(manager->huart->hdmarx, DMA_IT_HT);
    return BSP_UART_OK;
}

/**
  * @brief  使用已绑定的 UART 阻塞发送指定数据。
  * @param  manager: 已绑定的管理对象。
  * @param  data: 待发送数据缓冲区。
  * @param  length: 发送字节数。
  * @param  timeout_ms: HAL 发送超时时间，单位为毫秒。
  * @retval BSP_UART_OK、超时、参数错误或 HAL 发送失败。
  */
BspUartResult Bsp_Uart_Transmit(BspUartManager *manager,
                                uint8_t *data, uint16_t length,
                                uint32_t timeout_ms)
{
    if (manager == NULL || manager->huart == NULL || data == NULL || length == 0U) {
        return BSP_UART_ARGUMENT;
    }
    HAL_StatusTypeDef result = HAL_UART_Transmit(manager->huart, data,
                                                 length, timeout_ms);
    if (result == HAL_OK) {
        return BSP_UART_OK;
    }
    return result == HAL_TIMEOUT ? BSP_UART_TIMEOUT : BSP_UART_ERROR;
}

/**
  * @brief  等待 DMA 接收事件或 UART 错误回调。
  * @param  manager: 已绑定且已启动接收的管理对象。
  * @param  timeout_ms: 最长等待时间，单位为毫秒；超时会中止接收。
  * @param  size: 成功时写入本次实际收到的字节数。
  * @retval BSP_UART_OK、超时、参数错误或接收错误。
  */
BspUartResult Bsp_Uart_WaitReceive(BspUartManager *manager,
                                   uint32_t timeout_ms, uint16_t *size)
{
    if (manager == NULL || size == NULL) {
        return BSP_UART_ARGUMENT;
    }

    /* 主循环等待中断回调写入 rx_done；用 HAL tick 限制等待时间。 */
    uint32_t start = HAL_GetTick();
    while (manager->rx_done == 0U) {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
            Bsp_Uart_AbortReceive(manager);
            return BSP_UART_TIMEOUT;
        }
    }
    /* 先看到完成标志，再读取回调写入的长度和错误状态。 */
    __DMB();
    if (manager->rx_error != 0U) {
        return BSP_UART_ERROR;
    }
    *size = manager->rx_size;
    return BSP_UART_OK;
}

/**
  * @brief  中止当前 UART 接收并清除完成标志。
  * @param  manager: 管理对象；NULL 时不执行操作。
  * @retval 无
  */
void Bsp_Uart_AbortReceive(BspUartManager *manager)
{
    if (manager != NULL && manager->huart != NULL) {
        HAL_UART_AbortReceive(manager->huart);
        manager->rx_done = 0U;
    }
}

/**
  * @brief  处理 HAL 的接收至空闲事件并记录实际接收长度。
  * @param  huart: 触发事件的 UART 句柄。
  * @param  size: 本次 DMA 接收的实际字节数。
  * @retval 无
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    /* UART 空闲或 DMA 缓冲区写满时，由 HAL 在中断中调用。 */
    BspUartManager *manager = find_manager(huart);
    if (manager == NULL) {
        return;
    }

    /* size 是实际收到的字节数，不是缓冲区容量。 */
    if (size > manager->rx_capacity) {
        manager->rx_error = 1U;
    } else {
        manager->rx_size = size;
        if (manager->callback != NULL) {
            manager->callback(manager, manager->rx_buffer, size);
        }
    }
    /* 最后置完成标志，让等待方读到本次长度和错误状态。 */
    __DMB();
    manager->rx_done = 1U;
}

/**
  * @brief  处理 HAL UART 错误事件并通知等待接收的代码。
  * @param  huart: 出错的 UART 句柄。
  * @retval 无
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 串口出错时也唤醒等待方，由上层执行接收清理和结果映射。 */
    BspUartManager *manager = find_manager(huart);
    if (manager != NULL) {
        manager->rx_error = 1U;
        manager->rx_done = 1U;
    }
}
