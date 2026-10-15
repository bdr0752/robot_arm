#include "vofa.h"
#include "bsp_uart.hpp"
#include <assert.h>
#include <math.h>
#include <string.h>

static DMA_HandleTypeDef Rx_Dma;
UART_HandleTypeDef huart1 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE, HAL_UART_STATE_READY, HAL_UART_STATE_READY};
UART_HandleTypeDef huart3 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE, HAL_UART_STATE_READY, HAL_UART_STATE_READY};
uint32_t Test_Primask;
static char Tx_Line[256];
static unsigned Tx_Count;
static uint8_t *Pending_Data;
static uint16_t Pending_Size;
static bool Start_Failure;
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
    Uart->RxState = HAL_UART_STATE_BUSY_RX;
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
    Uart->RxState = HAL_UART_STATE_READY;
    return 0;
}

/**
 * @brief 保留 DMA 地址，延后读取数据以验证缓冲区寿命。
 * @param Uart 串口。
 * @param Data 数据。
 * @param Size 长度。
 * @retval HAL 启动结果
 */
extern "C" int HAL_UART_Transmit_DMA(UART_HandleTypeDef *Uart, uint8_t *Data, uint16_t Size)
{
    assert(Uart == &huart1 && Size < sizeof(Tx_Line));
    if (Start_Failure) return -1;
    assert(Uart->gState == HAL_UART_STATE_READY);
    Pending_Data = Data;
    Pending_Size = Size;
    Uart->gState = HAL_UART_STATE_BUSY_TX;
    ++Tx_Count;
    return 0;
}

/** @brief 模拟最后一个停止位发送完成。@retval 无 */
static void CompleteTransmit()
{
    assert(Pending_Data && BSP::UART1::IsBusy());
    assert(memchr(Pending_Data, '\0', Pending_Size) == 0);
    memcpy(Tx_Line, Pending_Data, Pending_Size);
    Tx_Line[Pending_Size] = '\0';
    Pending_Data = 0;
    huart1.gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&huart1);
    assert(!BSP::UART1::IsBusy());
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
    huart1.RxState = HAL_UART_STATE_READY;
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
    assert(BSP::UART1::IsBusy());
    const float Other[] = {99.0f};
    assert(Vofa_Send(Other, 1U) == VOFA_UART_ERROR);
    CompleteTransmit();
    assert(strcmp(Tx_Line, "1.25,-2,0\n") == 0);
    uint8_t Source[] = {'o', 'k', '\n'};
    assert(BSP::UART1::Transmit(Source, sizeof(Source)));
    memset(Source, 'x', sizeof(Source));
    CompleteTransmit();
    assert(strcmp(Tx_Line, "ok\n") == 0);
    Start_Failure = true;
    assert(Vofa_Send(Channels, 3U) == VOFA_UART_ERROR);
    assert(!BSP::UART1::IsBusy());
    Start_Failure = false;
    uint8_t Too_Long[257] = {0};
    assert(!BSP::UART1::Transmit(Too_Long, sizeof(Too_Long)));
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
