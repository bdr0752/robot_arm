#ifndef APP_VOFA_H
#define APP_VOFA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/* Bsp_Board_Init() 之后调用；注册 USART1 的唯一接收回调。 */
Enum_Vofa_Result Vofa_Init(void);
/* 非阻塞 DMA 发送 1～8 个通道，ASCII CSV、换行结尾，保留 6 位有效数字。
 * VOFA_OK 表示 DMA 已启动，不代表发送完成；忙或启动失败返回 VOFA_UART_ERROR。
 * 函数返回后 Channels 可以修改或释放。 */
Enum_Vofa_Result Vofa_Send(const float *Channels, uint8_t Count);
/* 非阻塞读取一行，不含 CR/LF；无完整行返回 VOFA_EMPTY。
 * 部分行跨 DMA 数据块保留；主循环持续读取，直至返回 VOFA_EMPTY。
 * 回调复制数据后 BSP 重启 DMA；主循环须及时消费，接收队列可容纳 255 字节。
 * 只接收文本，不解释上位机命令；不在中断中调用。 */
Enum_Vofa_Result Vofa_ReadLine(char *Line, uint16_t Capacity);

#ifdef __cplusplus
}
#endif

#endif
