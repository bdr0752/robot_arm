# USART1 与 VOFA FireWater

## 分层与接口

BSP 在 GPIO、DMA 初始化后调用 `MX_USART1_UART_Init()`，随后启动 `BSP::UART1` 的 128 字节 DMA 接收。App 注册接收回调，不直接调用 HAL 或访问 UART 内部状态。现有配置为 USART1、PA9 TX、PA10 RX、115200、8N1；与 USART3 舵机总线独立。

- `Vofa_Init()`：在 `Bsp_Board_Init()` 之后调用，注册接收回调并清空队列；返回值不报告 HAL 启动状态。
- `Vofa_Send(Channels, Count)`：非阻塞 DMA 发送 1～8 个浮点通道，6 位有效数字，以逗号分隔、LF 结尾，不发送 NUL。`VOFA_OK` 表示 DMA 已启动；发送忙或启动失败返回 `VOFA_UART_ERROR`，由调用者稍后重试。
- `Vofa_ReadLine(Line, Capacity)`：非阻塞提取一行文本，自动拼接跨 DMA 数据块的部分行，剔除 CR/LF；成功返回 `VOFA_OK`，无完整行返回 `VOFA_EMPTY`。

接收只提供文本，不预设命令名称、不修改 PID 或舵机参数。上位机自定义命令以 LF 结束，例如 `kp:1.5\n`；这是调用示例，不是 FireWater 官方规定的控制命令。

## 使用示例

下列片段由调用者按需加入应用；当前未接入 `main.c` 的调试收发。

```c
#include "vofa.h"

/* Bsp_Board_Init() 之后调用一次，检查返回值。 */
Enum_Vofa_Result Result = Vofa_Init();

/* 按实际调试周期发送，例如目标、反馈、PWM。 */
float Channels[] = {Target, Now, Out};
Result = Vofa_Send(Channels, 3U);

/* 主循环及时排空已经收到的完整行。 */
char Line[VOFA_RX_LINE_SIZE];
while ((Result = Vofa_ReadLine(Line, sizeof(Line))) == VOFA_OK) {
    /* 在此处理 Line；库不解释命令。 */
}
/* VOFA_EMPTY 为正常无数据；其他返回值由调用者处理。 */
```

VOFA+ 选择 FireWater 引擎及相同串口参数。[官方 FireWater 文档](https://www.vofa.plus/docs/learning/dataengines/firewater/)说明数据使用 CSV 风格，换行结束，前缀可省略。本库只实现采样文本，不实现图片传输。

## 运行限制与验证

DMA 使用普通模式；回调将数据复制到 256 字节环形队列（可用 255 字节），BSP 随后重启 DMA。主循环及时调用 `Vofa_ReadLine()` 消费；队列满时返回 `VOFA_BUFFER_FULL` 并等待换行重新同步。回调与重启之间仍存在空窗，不保证持续高速接收。一次 DMA 块可包含多行；每行最多 127 字节，超长或输出容量不足返回 `VOFA_BUFFER_FULL`，不交付截断命令。

模块使用单个 USART1 对象；不要同时让其他模块启动这一路接收，不在中断中调用收发，不并发调用。发送复制到 UART1 内部 256 字节 DMA 缓冲区，函数返回后输入数组可以修改或释放。没有发送队列，采样频率需按报文长度与 115200 波特率安排；上一次发送未完成时本次调用返回 `VOFA_UART_ERROR`。

GCC 的 DMA 缓冲区使用现有 `.dma_buffer` / RAM_D2 链接配置；若使用 Keil 或启用 DCache，另行核对实际缓冲区地址及缓存一致性。CMake 已为 newlib-nano 开启 `_printf_float`；Keil 已加入源文件和头文件路径，但未运行 Keil 构建。

主机协议测试命令（仓库根目录）：

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -Itests/stubs/bsp -Ibsp/Inc -Iapp/Inc tests/test_vofa.cpp app/Src/vofa.cpp bsp/Src/bsp_uart.cpp -o build/test_vofa.exe
./build/test_vofa.exe
```

硬件 USART1 收发和 VOFA+ 波形显示仍需实物验证。
