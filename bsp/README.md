# BSP 模块说明

## 职责划分

| 模块 | 职责 |
| --- | --- |
| `bsp_board` | 集中初始化 GPIO、DMA、UART，绑定串口与 RS485 资源 |
| `bsp_uart` | 使用已初始化的 `huart`，管理发送、DMA 空闲接收及 HAL 回调 |
| `bsp_dwt` | 初始化 CPU 周期计数器，提供周期读取与忙等待 |
| `bsp_ws2812` | 本板 PA7 的 WS2812 波形输出，不管理颜色缓存 |
| `bsp_rs485` | 绑定 UART 与方向引脚，发送前切换到 TX，发送调用返回后恢复 RX |

`Inc` 存放头文件，`Src` 存放实现文件。HAL 和寄存器操作由 BSP 实现。

## 初始化与调用

在 `HAL_Init()`、`SystemClock_Config()` 后调用一次 `Bsp_Board_Init()`。
该函数先初始化 GPIO 与 DMA，再初始化串口、设置初始电平、启用 DWT
并绑定 UART 管理对象。UART 接收仍由使用者启动。

舵机调用关系：`ft_servo_protocol → driver/ft_servo_bus → bsp_rs485 → bsp_uart / GPIO`。
`ft_servo_bus` 负责准备接收、等待舵机应答、接收超时和长度检查；
`bsp_rs485` 只负责通用方向切换与发送，不识别设备协议。
接收由总线使用者通过其绑定的 UART 管理对象启动；本板绑定 USART3 与 PB14。
灯珠调用关系：`driver/ws2812 → bsp_ws2812 → bsp_dwt / GPIO`。
`driver/ws2812` 负责颜色缓存与 GRB 转换，`bsp_ws2812` 负责实际波形输出。
原 `bsp_board_ws2812_send_grb()` 保留为兼容入口，新代码使用
`Bsp_Ws2812_SendGrb()`。

## 当前接收约定

本次拆分保留 UART 的单次 DMA 接收、阻塞发送和舵机请求应答方式。
UART 管理对象仍公开状态字段，VOFA 仍依赖这些字段；接收接口封装另行调整。

UART 在 `Bsp_Uart_Attach()` 时检查句柄、RX DMA、缓冲区及容量。
后续收发要求使用已绑定对象和有效参数，不重复检查内部配置。
发送直接调用 HAL，不转换或返回发送状态。无应答的总线调用返回成功仅表示
已调用发送接口。协议校验、接收长度和接收超时处理仍由相应模块执行。

## 时序与移植

WS2812 沿用原有脉宽和 1 ms 锁存等待，发送期间关闭中断后恢复原状态。
DWT 的周期读取、等待采用内联函数，避免在每一位输出中增加普通函数调用。
软件输出脉宽仍受编译结果影响，拆分后须在硬件上复测灯珠与串口通信。
移植时核对 GPIO、DWT 可用性、系统时钟及 DMA 缓冲区内存位置。
