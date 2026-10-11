#include "vofa.h"
#include "bsp_uart.hpp"
#include <assert.h>
#include <math.h>
#include <string.h>

static DMA_HandleTypeDef Rx_Dma;
UART_HandleTypeDef huart1 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE};
UART_HandleTypeDef huart3 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE};
uint32_t Test_Primask;
static char Tx_Line[256];
static unsigned Tx_Count;
static unsigned Rx_Start_Count;

/**
 * @brief 模拟 HAL 启动接收，保留实际 BSP 使用的 DMA 地址。
 * @param Uart 串口。
 * @param Data 缓冲区。
 * @param Size 容量。
 * @retval 0
 */
extern "C" int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *Uart, uint8_t *Data, uint16_t Size)
{
    Uart->rx_buffer = Data;
    Uart->rx_capacity = Size;
    ++Rx_Start_Count;
    return 0;
}

/**
 * @brief 模拟停止接收。
 * @param Uart 串口。
 * @retval 0
 */
extern "C" int HAL_UART_AbortReceive(UART_HandleTypeDef *Uart)
{
    Uart->rx_buffer = 0;
    return 0;
}

/**
 * @brief 保存 FireWater 文本。
 * @param Uart 串口。
 * @param Data 数据。
 * @param Size 长度。
 * @param Timeout 超时。
 * @retval 0
 */
extern "C" int HAL_UART_Transmit(UART_HandleTypeDef *Uart, uint8_t *Data, uint16_t Size, uint32_t Timeout)
{
    assert(Uart == &huart1 && Size < sizeof(Tx_Line) && Timeout > 0U);
    assert(memchr(Data, '\0', Size) == 0);
    memcpy(Tx_Line, Data, Size);
    Tx_Line[Size] = '\0';
    ++Tx_Count;
    return 0;
}

/**
 * @brief 提供模拟事件类型。
 * @param Uart 串口。
 * @retval 事件类型
 */
extern "C" uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *Uart)
{
    return Uart->event;
}

/**
 * @brief 经实际 BSP HAL 回调交付数据。
 * @param Text 数据块。
 * @retval 无
 */
static void Feed(const char *Text)
{
    size_t Length = strlen(Text);
    assert(huart1.rx_buffer && Length <= huart1.rx_capacity);
    memcpy(huart1.rx_buffer, Text, Length);
    huart1.event = Length == huart1.rx_capacity ? HAL_UART_RXEVENT_TC : HAL_UART_RXEVENT_IDLE;
    unsigned Previous_Start_Count = Rx_Start_Count;
    HAL_UARTEx_RxEventCallback(&huart1, (uint16_t)Length);
    assert(Rx_Start_Count == Previous_Start_Count + 1U);
}

/**
 * @brief 验证发送格式、拆包、多行、行边界和串口结果映射。
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
    // DMA 写满事件必须交付数据，不能只重启接收。
    char Full_Block[129];
    for (unsigned i = 0U; i < 128U; i += 2U) {
        Full_Block[i] = 'a';
        Full_Block[i + 1U] = '\n';
    }
    Full_Block[128] = '\0';
    Feed(Full_Block);
    for (unsigned i = 0U; i < 64U; ++i) {
        assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_OK);
        assert(strcmp(Line, "a") == 0);
    }
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_EMPTY);

    // 队列溢出后清空并等待换行重新同步。
    Feed(Full_Block);
    Feed(Full_Block);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_BUFFER_FULL);
    Feed("\nrecovered\n");
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_BUFFER_FULL);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_OK);
    assert(strcmp(Line, "recovered") == 0);

    HAL_UART_ErrorCallback(&huart1);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_UART_ERROR);
    Feed("\nnext\n");
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_BUFFER_FULL);
    assert(Vofa_ReadLine(Line, sizeof(Line)) == VOFA_OK);
    assert(strcmp(Line, "next") == 0);
    assert(Test_Primask == 0U);
    return 0;
}
