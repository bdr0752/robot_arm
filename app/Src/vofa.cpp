#include "vofa.h"
#include "bsp_uart.hpp"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define VOFA_TX_BUFFER_SIZE 256U
#define VOFA_TX_TIMEOUT_MS 100U
#define VOFA_RX_BUFFER_SIZE 256U

static bool Initialized;
static char Rx_Buffer[VOFA_RX_BUFFER_SIZE];
static volatile uint16_t Rx_Head;
static volatile uint16_t Rx_Tail;
static volatile bool Rx_Overflow;
static volatile bool Rx_Error;
static char Rx_Line[VOFA_RX_LINE_SIZE];
static uint16_t Rx_Length;
static bool Rx_Discard;

/**
 * @brief 在中断中复制数据，文本解析留在主循环。
 * @param Data 接收数据，0 表示错误。
 * @param Size 字节数。
 * @retval 无
 */
static void Vofa_Receive(const uint8_t *Data, uint16_t Size)
{
    if (!Data) { Rx_Error = true; return; }
    if (Rx_Overflow || Rx_Error) return;
    for (uint16_t i = 0U; i < Size; ++i) {
        uint16_t Next = (Rx_Head + 1U) % VOFA_RX_BUFFER_SIZE;
        if (Next == Rx_Tail) { Rx_Overflow = true; return; }
        Rx_Buffer[Rx_Head] = (char)Data[i];
        __DMB();
        Rx_Head = Next;
    }
}

/**
 * @brief 注册 USART1 回调并清空接收状态，在 BSP 初始化后调用。
 * @retval VOFA_OK，不报告 HAL 启动状态
 */
Enum_Vofa_Result Vofa_Init(void)
{
    BSP::UART1::Stop();
    Rx_Head = Rx_Tail = 0U;
    Rx_Overflow = Rx_Error = false;
    Rx_Length = 0U;
    Rx_Discard = false;
    BSP::UART1::RegisterCallback(Vofa_Receive);
    BSP::UART1::Init();
    Initialized = true;
    return VOFA_OK;
}

/**
 * @brief 将通道数组编码为 FireWater 采样帧并阻塞发送。
 * @param Channels: 浮点通道数组，数值须有限。
 * @param Count: 通道数量，范围 1～VOFA_MAX_CHANNELS。
 * @retval VOFA_OK、参数错误或缓冲区不足；不报告 UART 发送状态。
 * @note 格式为 ch0,ch1,...,chN\n；不重定向 printf，不发送字符串结尾 NUL。
 */
Enum_Vofa_Result Vofa_Send(const float *Channels, uint8_t Count)
{
    if (!Initialized || Channels == NULL || Count == 0U ||
        Count > VOFA_MAX_CHANNELS) {
        return VOFA_ARGUMENT;
    }

    char Tx_Buffer[VOFA_TX_BUFFER_SIZE];
    uint16_t Tx_Length = 0U;
    for (uint8_t i = 0U; i < Count; ++i) {
        if (!isfinite(Channels[i])) {
            return VOFA_ARGUMENT;
        }
        int Length = snprintf(&Tx_Buffer[Tx_Length], sizeof(Tx_Buffer) - Tx_Length,
                              "%s%.6g", i == 0U ? "" : ",", (double)Channels[i]);
        if (Length < 0 || (unsigned)Length >= sizeof(Tx_Buffer) - Tx_Length) {
            return VOFA_BUFFER_FULL;
        }
        Tx_Length += (uint16_t)Length;
    }
    if (Tx_Length >= sizeof(Tx_Buffer) - 1U) {
        return VOFA_BUFFER_FULL;
    }
    Tx_Buffer[Tx_Length++] = '\n';

    BSP::UART1::Transmit((const uint8_t *)Tx_Buffer,
                      Tx_Length, VOFA_TX_TIMEOUT_MS);
    return VOFA_OK;
}

/**
 * @brief 从回调写入的队列取一个字节。
 * @param Byte 输出地址。
 * @retval 数据、空队列、溢出或接收错误
 */
static Enum_Vofa_Result Vofa_ReadByte(char *Byte)
{
    // 只保护队列索引和标志；不在关中断期间解析文本。
    uint32_t Primask = __get_PRIMASK();
    __disable_irq();
    if (Rx_Error || Rx_Overflow) {
        Enum_Vofa_Result Result = Rx_Error ? VOFA_UART_ERROR : VOFA_BUFFER_FULL;
        Rx_Tail = Rx_Head;
        Rx_Error = Rx_Overflow = false;
        __set_PRIMASK(Primask);
        Rx_Length = 0U;
        Rx_Discard = true;
        return Result;
    }
    if (Rx_Tail == Rx_Head) {
        __set_PRIMASK(Primask);
        return VOFA_EMPTY;
    }
    *Byte = Rx_Buffer[Rx_Tail];
    Rx_Tail = (Rx_Tail + 1U) % VOFA_RX_BUFFER_SIZE;
    __set_PRIMASK(Primask);
    return VOFA_OK;
}

/**
 * @brief 非阻塞提取一行上位机文本，保留跨 DMA 数据块的部分行。
 * @param Line: 输出缓冲区，成功时以 NUL 结尾，不包含 CR/LF。
 * @param Capacity: 缓冲区大小，至少为 2 字节。
 * @retval VOFA_OK、无完整行、参数错误、行过长或串口错误。
 * @note 接受 LF、CRLF 或 LFCR；空行忽略。过长行丢弃至下个 LF。 输出容量不足时丢弃该行并返回 VOFA_BUFFER_FULL，不返回截断命令。
 */
Enum_Vofa_Result Vofa_ReadLine(char *Line, uint16_t Capacity)
{
    if (!Initialized || Line == NULL || Capacity < 2U) {
        return VOFA_ARGUMENT;
    }
    Line[0] = '\0';

    char Byte;
    Enum_Vofa_Result Result;
    while ((Result = Vofa_ReadByte(&Byte)) == VOFA_OK) {
        if (Byte == '\r') {
            continue;
        }
        if (Byte == '\n') {
            if (Rx_Discard || Rx_Length >= Capacity) {
                Rx_Length = 0U;
                Rx_Discard = false;
                return VOFA_BUFFER_FULL;
            }
            if (Rx_Length == 0U) {
                continue;
            }
            memcpy(Line, Rx_Line, Rx_Length);
            Line[Rx_Length] = '\0';
            Rx_Length = 0U;
            return VOFA_OK;
        }
        if (Rx_Discard) {
            continue;
        }
        if (Byte == '\0' || Rx_Length == sizeof(Rx_Line) - 1U) {
            Rx_Length = 0U;
            Rx_Discard = true;
        } else {
            Rx_Line[Rx_Length++] = Byte;
        }
    }
    return Result;
}
