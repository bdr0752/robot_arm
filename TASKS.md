# 舵机与 WS2812 驱动进度

更新日期：2026-09-28。构建通过不代表已完成本轮硬件验证。

## 已完成：代码与配置

- [x] `robot_arm.ioc` 配置 USART3（PD8/PD9）、收发 DMA、RS485 方向脚 PB14，以及 WS2812 数据脚 PA7。
- [x] CubeMX 生成的 GPIO、DMA、USART3 初始化仍由 `main.c` 调用；`bsp_board_init()` 在其后设置接收方向、拉低 PA7 并启用 DWT 计数器。
- [x] `bsp_board` 封装本板 UART、RS485 方向脚和 PA7 波形输出；`servo_bus` 不直接引用 HAL。
- [x] `ft_Servo` 提供 PING 和读取当前位置；`main.c` 的 ID 扫描已改为调用驱动。
- [x] `ws2812` 提供单灯的设置颜色和发送接口，按 GRB 顺序输出。
- [x] `cmake --preset Debug`、`cmake --build --preset Debug` 构建通过。

## 待验证与后续功能

- [ ] 在目标板验证舵机扫描、ID `0x06` 的 PING 和读取位置，并记录结果。
- [ ] 在目标板验证 WS2812 的 PA7 输出、电平和波形时序；当前代码只做了编译验证。
- [ ] 实现并验证写目标位置、读取当前电流、通信错误恢复。
- [ ] 若 CubeMX 重新生成代码，检查 PA7 初始化与 `robot_arm.ioc` 是否保持一致。
