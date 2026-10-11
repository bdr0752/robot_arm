#ifndef APP_VOFA_H
#define APP_VOFA_H

#include <stdint.h>

#define VOFA_MAX_CHANNELS 8U
#define VOFA_RX_LINE_SIZE 128U

typedef enum {
    VOFA_OK = 0,
    VOFA_EMPTY = 1,
    VOFA_ARGUMENT = -1,
    VOFA_BUFFER_FULL = -2,
    VOFA_UART_ERROR = -3,
    VOFA_TIMEOUT = -4
} Enum_Vofa_Result;

/* Bsp_Board_Init() 之后调用；本模块独占 USART1 接收管理对象。 */
Enum_Vofa_Result Vofa_Init(void);
/* 阻塞发送 1～8 个通道，ASCII CSV、换行结尾，保留 6 位有效数字。
 * VOFA_OK 表示编码并调用发送接口，不代表 UART 发送状态。 */
Enum_Vofa_Result Vofa_Send(const float *Channels, uint8_t Count);
/* 非阻塞读取一行，不含 CR/LF；无完整行返回 VOFA_EMPTY。
 * 部分行跨 DMA 数据块保留；主循环持续读取，直至返回 VOFA_EMPTY。
 * DMA 为普通模式，处理数据块到重新启动接收之间存在空窗，不保证连续流无丢字节。
 * 只接收文本，不解释上位机命令；不在中断中调用。 */
Enum_Vofa_Result Vofa_ReadLine(char *Line, uint16_t Capacity);

#endif
