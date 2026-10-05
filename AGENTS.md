# Repository Guidelines

## 协作要求

始终使用简体中文，表达直接、严谨。先核对现有文件、配置和官方资料；区分已确认事实、建议、假设与待验证项。不猜测器件型号、寄存器含义、引脚、尺寸或单位。不要使用恭维或夸张措辞。用户要求自行操作时，只提供步骤，不修改文件。

## 项目结构与模块

这是 STM32H723VGTx 的 CubeMX/CMake 固件工程。`robot_arm.ioc` 保存外设配置；`Core` 放应用入口与生成代码；`bsp` 中 `bsp_board` 集中初始化外设，`bsp_uart` 绑定已初始化的 UART 并管理 DMA 接收，`servo_bus` 管理 RS485 收发；`driver` 中 `ft_servo_protocol` 处理协议及寄存器原始值，`ft_Servo` 处理角度换算，另有 WS2812 驱动；`tests` 放主机端测试。新增 `.c` 时更新根目录 `CMakeLists.txt`。

## 构建、测试与开发命令

需要 CMake、Ninja 和 `arm-none-eabi-gcc` 位于 `PATH`。在本目录运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

产物位于 `build/Debug/`。发布配置改用 `Release` 预设。本工程未提供烧录命令；烧录前确认目标板、接口与固件文件。

## 代码风格与命名

使用 C11。CubeMX 生成文件中的手写代码放在 `/* USER CODE BEGIN ... */` 区域，避免重新生成时丢失。沿用模块前缀，例如 `ft_servo_read_position()`、`Bsp_Uart_Attach()`；自写函数在定义前使用 `@brief`、`@param`、`@retval` 说明作用与行为。寄存器地址、单位和串口参数须依据实际器件资料。

## 测试要求

目前没有测试框架、覆盖率要求或 `ctest` 测试项。`tests/test_ft_servo_protocol.c` 检查组帧与校验；`tests/test_ft_servo_motion.c` 检查角度换算，两者可用主机 GCC 分别运行。提交前至少完成 Debug 构建并检查警告。编译和主机测试不代表硬件已验证；运动测试不得默认在上电时自动执行，须先确认型号、行程、负载与停止办法，并记录通信、超时和异常结果。

## 提交与合并请求

当前目录没有 Git 仓库或可核对的提交历史，因此不存在已确认的提交消息约定。若纳入版本控制，建议用简短的动词式主题说明改动，例如 `Add servo UART adapter`。合并请求应写明改动目的、关联问题、构建结果，以及硬件测试所用的芯片、固件配置和观测结果；涉及接口或时序时附日志或截图。
