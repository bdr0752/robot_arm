# Repository Guidelines

## 协作要求

始终使用简体中文，表达直接、严谨。先核对现有文件、配置和官方资料；区分已确认事实、建议、假设与待验证项。不猜测器件型号、寄存器含义、引脚、尺寸或单位。不要使用恭维或夸张措辞。用户要求自行操作时，只提供步骤，不修改文件。

## 项目结构与模块

这是 STM32H723VGTx 的 CubeMX/CMake 固件工程，也提供 `MDK-ARM/robot_arm.uvprojx`。`robot_arm.ioc` 保存外设配置；`Core` 放应用入口与生成代码；`bsp` 中 `bsp_board` 集中初始化外设，`bsp_uart` 绑定已初始化的 UART 并管理 DMA 空闲接收，`servo_bus` 管理 RS485 收发；`driver` 中 `ft_servo_protocol` 处理协议及寄存器原始值，`ft_Servo` 提供模式切换封装与角度换算，另有 WS2812 驱动；`tests` 放主机端测试；`TASKS.md` 记录验证进度。新增 `.c` 时同步核对 CMake 与 Keil 的源文件列表、头文件路径。

## 构建、测试与开发命令

需要 CMake、Ninja 和 `arm-none-eabi-gcc` 位于 `PATH`。在本目录运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

产物位于 `build/Debug/`。发布配置改用 `Release` 预设。VS Code 的 `DAPLink: Flash ELF` 与 `DAPLink: Build and Flash` 任务通过 OpenOCD 烧写 Flash，并非 RAM 下载；执行前确认目标板、接口与固件文件。Keil 工程可用于构建和调试，下载配置须另行核对。

## 代码风格与命名

CMake 使用 C11；修改跨工具链代码时核对 Keil 配置。CubeMX 生成文件中的手写代码放在 `/* USER CODE BEGIN ... */` 区域。沿用模块前缀，例如 `ft_servo_read_position()`、`Bsp_Uart_Attach()`；自写函数在定义前使用 `@brief`、`@param`、`@retval` 说明作用与行为。寄存器地址、单位和串口参数须依据实际器件资料。测试示例保持简短，不擅自引入复杂测试管理框架。

## 测试要求

目前没有测试框架、覆盖率要求或 `ctest` 测试项。`tests/test_ft_servo_protocol.c` 检查组帧与校验；`tests/test_ft_servo_motion.c` 检查角度换算；新增封装的覆盖情况须单独核对。固件修改提交前至少完成 Debug 构建并检查警告。编译和主机测试不代表硬件已验证。单播调用同时检查函数返回值和舵机状态字节；广播成功仅表示发送完成。运动测试先确认型号、机械行程、负载与停止办法，避免新增上电自动运动。

## 提交与合并请求

现有 Git 历史包含 `feat:单舵机位置模式与速度模式封装` 等提交，也有无前缀主题，尚无统一格式。建议采用 `feat:`、`fix:`、`docs:` 加简短改动说明。合并请求应写明目的、关联问题、构建结果，以及硬件测试所用的芯片、固件配置和观测结果；涉及接口或时序时附日志或截图，未验证项目明确标注。
