# Repository Guidelines

## 项目结构与模块

这是 STM32H723VGTx 的 CubeMX/CMake 固件工程。`robot_arm.ioc` 保存芯片与外设配置；`Core/Src`、`Core/Inc` 放应用入口、中断和 HAL 配置；`bsp/Inc`、`bsp/Src` 放板级接口及实现，`driver/Inc`、`driver/Src` 放舵机和 WS2812 驱动；`Drivers` 是 STM32 HAL 与 CMSIS 代码；`cmake/stm32cubemx` 列出生成的源码。新增模块时，将 `.c` 加入根目录 `CMakeLists.txt` 的 `target_sources()`，并按需加入头文件路径。

## 构建、测试与开发命令

需要 CMake、Ninja 和 `arm-none-eabi-gcc` 位于 `PATH`。在本目录运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

产物位于 `build/Debug/`。发布配置改用 `Release` 预设。本工程未提供烧录命令；烧录前确认目标板、接口与固件文件。

## 代码风格与命名

使用 C11，与现有 CubeMX 生成代码保持一致。应用代码放在 `/* USER CODE BEGIN ... */` 与对应 `END` 之间，避免重新生成时丢失。函数和变量沿用现有模块风格；新模块使用清晰的领域前缀，例如 `servo_read_position()`。寄存器地址、单位和串口参数须依据实际器件资料，不要猜测。

## 测试要求

目前没有测试框架、测试目录或覆盖率要求，也没有 `ctest` 测试项。提交前至少完成 Debug 构建并检查编译警告；涉及外设或电机控制时，在对应硬件上验证通信、超时和异常处理，并记录测试条件与结果。

## 提交与合并请求

当前目录没有 Git 仓库或可核对的提交历史，因此不存在已确认的提交消息约定。若纳入版本控制，建议用简短的动词式主题说明改动，例如 `Add servo UART adapter`。合并请求应写明改动目的、关联问题、构建结果，以及硬件测试所用的芯片、固件配置和观测结果；涉及接口或时序时附日志或截图。
