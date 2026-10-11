#include "vofa.h"
#include "bsp_board.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static uint8_t Rx_Buffer[128];
static BspUartManager Uart = {
    .rx_buffer = Rx_Buffer,
    .rx_capacity = sizeof(Rx_Buffer)
};
static char Tx_Line[256];
static unsigned Tx_Count;

/**
  * @brief  提供测试 USART1 管理对象。
  * @retval 管理对象地址。
  */
BspUartManager *Bsp_Board_DebugUart(void)
{
    return &Uart;
}

/**
  * @brief  模拟启动一次 DMA 接收。
  * @param  manager: 管理对象。
  * @retval BSP_UART_OK。
  */
BspUartResult Bsp_Uart_StartReceiveToIdle(BspUartManager *manager)
{
    manager->rx_done = 0U;
    manager->rx_error = 0U;
    manager->rx_size = 0U;
    return BSP_UART_OK;
}

/**
  * @brief  模拟读取已完成接收的长度和状态。
  * @param  manager: 管理对象。
  * @param  timeout_ms: 超时值，App 仅使用零超时。
  * @param  size: 接收长度输出。
  * @retval BSP_UART_OK 或接收错误。
  */
BspUartResult Bsp_Uart_WaitReceive(BspUartManager *manager,
                                  uint32_t timeout_ms, uint16_t *size)
{
    assert(manager->rx_done != 0U && timeout_ms == 0U);
    if (manager->rx_error != 0U) {
        return BSP_UART_ERROR;
    }
    *size = manager->rx_size;
    return BSP_UART_OK;
}

/**
  * @brief  模拟停止接收。
  * @param  manager: 管理对象。
  * @retval 无。
  */
void Bsp_Uart_AbortReceive(BspUartManager *manager)
{
    manager->rx_done = 0U;
}

/**
  * @brief  保存待发送文本，用于检查 FireWater 格式。
  * @param  manager: 管理对象。
  * @param  data: 发送内容。
  * @param  length: 内容长度。
  * @param  timeout_ms: 超时值。
  * @retval 无。
  */
void Bsp_Uart_Transmit(BspUartManager *manager, uint8_t *data,
                               uint16_t length, uint32_t timeout_ms)
{
    assert(manager == &Uart && length < sizeof(Tx_Line) && timeout_ms > 0U);
    assert(memchr(data, '\0', length) == NULL);
    memcpy(Tx_Line, data, length);
    Tx_Line[length] = '\0';
    ++Tx_Count;
}

/**
  * @brief  注入一个 DMA 完成的数据块。
  * @param  Text: 本次接收文本。
  * @retval 无。
  */
static void Feed(const char *Text)
{
    size_t Length = strlen(Text);
    assert(Uart.rx_done == 0U && Length <= Uart.rx_capacity);
    memcpy(Rx_Buffer, Text, Length);
    Uart.rx_size = (uint16_t)Length;
    Uart.rx_done = 1U;
}

/**
  * @brief  验证发送格式、拆包、多行、行边界和串口结果映射。
  * @retval 0 表示全部检查通过。
  */
int main(void)
{
    const float Channels[] = {1.25f, -2.0f, 0.0f};
    char Line[VOFA_RX_LINE_SIZE];
    assert(Vofa_Send(Channels, 3U) == VOFA_ARGUMENT);
    assert(Vofa_Init() == VOFA_OK);
    assert(Vofa_Send(Channels, 3U) == VOFA_OK);
    assert(strcmp(Tx_Line, "1.25,-2,0\n") == 0);
    unsigned Previous_Tx_Count = Tx_Count;
    const float Invalid_Channel[] = {NAN};
    assert(Vofa_Send(Invalid_Channel, 1U) == VOFA_ARGUMENT);
    assert(Vofa_Send(Channels, 0U) == VOFA_ARGUMENT);
    assert(Tx_Count == Previous_Tx_Count);

    Feed("kp:1");
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_EMPTY);
    Feed(".5\r\nki:2\n");
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_OK);
    assert(strcmp(Line, "kp:1.5") == 0);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_OK);
    assert(strcmp(Line, "ki:2") == 0);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_EMPTY);

    Feed("\n\r\nlong:123\nnext\n");
    assert(Vofa_ReadLine(Line, 4U) == VOFA_BUFFER_FULL);
    assert(Line[0] == '\0');
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_OK);
    assert(strcmp(Line, "next") == 0);

    char Long_Line[128];
    memset(Long_Line, 'x', sizeof(Long_Line) - 1U);
    Long_Line[sizeof(Long_Line) - 1U] = '\0';
    Feed(Long_Line);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_EMPTY);
    Feed("x\nvalid\n");
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_BUFFER_FULL);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_OK);
    assert(strcmp(Line, "valid") == 0);
    return 0;
}
