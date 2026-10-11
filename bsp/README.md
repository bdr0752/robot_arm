# BSP 模块说明

## 结构

手写 BSP 使用 C++ 类与 `BSP` 命名空间，参考 RMPP 的静态外设类与回调结构。
CubeMX、HAL、舵机协议和 PID 继续使用 C。

| 文件 | 职责 |
| --- | --- |
| `bsp_board.hpp/.cpp` | `BSP::Init()` 集中初始化硬件和 BSP 模块 |
| `bsp_uart.hpp/.cpp` | `UART1`、`UART3` 的 DMA 接收、回调分发与发送 |
| `bsp_rs485.hpp/.cpp` | USART3 + PB14 的方向控制与发送 |
| `bsp_dwt.hpp/.cpp` | CPU 周期计数、周期等待与系统毫秒计数 |
| `bsp_ws2812.hpp/.cpp` | PA7 的灯珠波形输出 |
| `bsp_board.h` | 供 `main.c` 调用的单个初始化入口 |

## 使用

在 `HAL_Init()` 和时钟配置后调用一次 `Bsp_Board_Init()`，C++ 使用 `BSP::Init()`。
GPIO 和 DMA 先初始化，再初始化 USART3/USART1，最后启动 BSP 模块。
UART 的 `Init()` 只启动接收，不配置波特率、引脚和时钟。

```cpp
#include "bsp_uart.hpp"

void Receive(const uint8_t *data, uint16_t size);

BSP::UART1::RegisterCallback(Receive);
BSP::UART1::Transmit(data, size);
BSP::RS485::Transmit(data, size);
```

每路串口仅注册一个回调；重复注册替换原回调。回调运行在中断中，数据只在
回调期间有效，使用者须在回调中处理或复制；`data == 0` 表示接收错误。
普通 DMA 的空闲、写满事件均交付数据并重启接收，半满事件忽略。
使用者不要在回调内发送、等待或自行重启 DMA。

## 职责与约定

UART 不再包含管理对象注册表、公开接收状态、阻塞等待及结果转换。
发送直接调用阻塞 HAL，不返回发送状态。沿用阻塞方式是为了支持现有栈缓冲区
和 RS485 发送后的方向切换；若改成中断发送，须同时处理缓冲区寿命和完成回调。
回调用函数指针，每路只有一个使用者，不引入动态分配、回调列表或运行时间统计。

`ft_servo_protocol → driver/ft_servo_bus.cpp → BSP::RS485 / UART3`。
舵机总线复制应答并负责 20 ms 超时等待与长度检查，协议层负责帧校验。
VOFA 使用 UART1 回调复制数据，主循环解析文本，不访问 DMA 缓冲区或接收标志。

## 构建与验证

CMake 使用 C11/C++11；Keil 已更新 `.cpp/.hpp` 列表，保持现有 ARMCC5 配置，
本次类和模板写法不依赖 C++11 专有语法。Keil 编译仍待验证。
DMA 缓冲区沿用 32 字节对齐及 `.dma_buffer` / RAM_D2 配置；Keil 的实际放置
和启用 DCache 后的一致性须另行核对。

DMA 回调处理与重启之间仍有空窗，不能保证连续高速接收无丢失。
WS2812 沿用软件时序、发送期间关闭中断及 1 ms 锁存等待，需要硬件复测。
主机测试和编译通过不等于串口或运动已在硬件验证。

主机端总线测试直接编译实际 UART、RS485 与舵机总线实现，用 HAL 替身验证
方向顺序、应答、长度错误、超时与接收错误：

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -Itests/stubs/bsp -Ibsp/Inc -Idriver/Inc tests/test_ft_servo_bus.cpp driver/Src/ft_servo_bus.cpp bsp/Src/bsp_uart.cpp bsp/Src/bsp_rs485.cpp -o build/test_ft_servo_bus.exe
./build/test_ft_servo_bus.exe
```
