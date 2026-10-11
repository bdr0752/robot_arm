#include "vofa.h"

#include "bsp_board.h"
#include "bsp_uart.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define VOFA_TX_BUFFER_SIZE 256U
#define VOFA_TX_TIMEOUT_MS 100U

/* USART1 独立接收状态；DMA 数据仅在完成后读取，未消费的数据块不重启 DMA。 */
static BspUartManager *Vofa_Uart;
static uint16_t Rx_Index;
static char Rx_Line[VOFA_RX_LINE_SIZE];
static uint16_t Rx_Length;
static bool Rx_Discard;

/**
  * @brief  将 BSP 串口结果转换为 VOFA 返回值。
  * @param  Result: BSP 操作结果。
  * @retval 对应的 VOFA 结果。
  */
static Enum_Vofa_Result Vofa_UartResult(BspUartResult Result)
{
    if (Result == BSP_UART_OK) {
        return VOFA_OK;
    }
    if (Result == BSP_UART_TIMEOUT) {
        return VOFA_TIMEOUT;
    }
    return VOFA_UART_ERROR;
}

/**
  * @brief  绑定 BSP 的 USART1 对象并启动 DMA 空闲接收。
  * @retval VOFA_OK 或 UART 启动错误。
  * @note   在 Bsp_Board_Init() 之后调用；再次调用会清除未处理数据。
  */
Enum_Vofa_Result Vofa_Init(void)
{
    BspUartManager *Uart = Bsp_Board_DebugUart();
    if (Vofa_Uart != NULL) {
        Bsp_Uart_AbortReceive(Vofa_Uart);
    }
    Vofa_Uart = NULL;
    Rx_Index = 0U;
    Rx_Length = 0U;
    Rx_Discard = false;

    Enum_Vofa_Result Result = Vofa_UartResult(Bsp_Uart_StartReceiveToIdle(Uart));
    if (Result == VOFA_OK) {
        Vofa_Uart = Uart;
    }
    return Result;
}

/**
  * @brief  将通道数组编码为 FireWater 采样帧并阻塞发送。
  * @param  Channels: 浮点通道数组，数值须有限。
  * @param  Count: 通道数量，范围 1～VOFA_MAX_CHANNELS。
  * @retval VOFA_OK、参数错误或缓冲区不足；不报告 UART 发送状态。
  * @note   格式为 ch0,ch1,...,chN\n；不重定向 printf，不发送字符串结尾 NUL。
  */
Enum_Vofa_Result Vofa_Send(const float *Channels, uint8_t Count)
{
    if (Vofa_Uart == NULL || Channels == NULL || Count == 0U ||
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

    Bsp_Uart_Transmit(Vofa_Uart, (uint8_t *)Tx_Buffer,
                      Tx_Length, VOFA_TX_TIMEOUT_MS);
    return VOFA_OK;
}

/**
  * @brief  从已经完成的 DMA 数据块取出一个字节，消费完后重启接收。
  * @param  Byte: 输出字节地址。
  * @retval VOFA_OK、无数据或串口错误。
  */
static Enum_Vofa_Result Vofa_ReadByte(char *Byte)
{
    if (Vofa_Uart->rx_done == 0U) {
        return VOFA_EMPTY;
    }
    /* 完成标志已置位，零超时不会等待；复用 BSP 的内存屏障与错误检查。 */
    uint16_t Rx_Size;
    BspUartResult Result = Bsp_Uart_WaitReceive(Vofa_Uart, 0U, &Rx_Size);
    if (Result != BSP_UART_OK) {
        Bsp_Uart_AbortReceive(Vofa_Uart);
        Bsp_Uart_StartReceiveToIdle(Vofa_Uart);
        Rx_Index = 0U;
        Rx_Length = 0U;
        Rx_Discard = true;
        return VOFA_UART_ERROR;
    }
    if (Rx_Index >= Rx_Size) {
        Rx_Index = 0U;
        Result = Bsp_Uart_StartReceiveToIdle(Vofa_Uart);
        if (Result != BSP_UART_OK) {
            return Vofa_UartResult(Result);
        }
        return VOFA_EMPTY;
    }

    *Byte = (char)Vofa_Uart->rx_buffer[Rx_Index++];
    if (Rx_Index == Rx_Size) {
        Rx_Index = 0U;
        return Vofa_UartResult(Bsp_Uart_StartReceiveToIdle(Vofa_Uart));
    }
    return VOFA_OK;
}

/**
  * @brief  非阻塞提取一行上位机文本，保留跨 DMA 数据块的部分行。
  * @param  Line: 输出缓冲区，成功时以 NUL 结尾，不包含 CR/LF。
  * @param  Capacity: 缓冲区大小，至少为 2 字节。
  * @retval VOFA_OK、无完整行、参数错误、行过长或串口错误。
  * @note   接受 LF、CRLF 或 LFCR；空行忽略。过长行丢弃至下个 LF。
  *         输出容量不足时丢弃该行并返回 VOFA_BUFFER_FULL，不返回截断命令。
  */
Enum_Vofa_Result Vofa_ReadLine(char *Line, uint16_t Capacity)
{
    if (Vofa_Uart == NULL || Line == NULL || Capacity < 2U) {
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
