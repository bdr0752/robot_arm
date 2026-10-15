# PID、速度环与角度环

## 文件与调用

- `Inc/PID.h`、`Src/PID.c`：独立 PID 算法，不依赖 HAL 或舵机通信。
- `Inc/servo_speed_loop.h`、`Src/servo_speed_loop.c`：读取速度、调用 PID、下发 PWM。
- `Inc/servo_angle_loop.h`、`Src/servo_angle_loop.c`：读取位置、换算角度，外环输出目标速度，复用现有速度环。
- `PID_Init()` 保存参数，`PID_Reset()` 清除计算历史，`PID_Adjust()` 执行一次计算。
- `ServoSpeedLoop_Init()` 绑定 ID 与 PID 参数；上层设置 PWM 模式、扭矩后，周期调用 `ServoSpeedLoop_Update()`。

速度环使用带符号的速度寄存器原始值，不换算 RPM。调用间隔以秒传入 PID；使用 DMA 收发并同步等待应答，不在中断中调用，同一总线串行访问。未实现自动停机或模式切换。

## 命名与计算顺序

沿用参考例程的命名格式：`K_P`、`K_I`、`K_D` 为增益，`Target`、`Now` 为目标与反馈，`Pre_Now` 为上次反馈，`D_T` 为采样间隔。C 结构体为 `Struct_PID_Config` 与 `Struct_PID`。`I_Out_Min/Max` 限制积分项输出，`D_Filter_Time` 为微分低通时间常数。

计算依次为比例项、积分项、微分项、线性前馈、抗饱和与总输出限幅、历史更新。`K_F * Target` 为前馈，`F_OUT` 保存本轮前馈；饱和判断包含前馈。饱和时禁止积分继续向饱和方向增加；微分针对测量值，避免目标阶跃冲击。首个样本的微分输出为零。

`D_Filter_Enable = true` 时，根据 `D_Filter_Time` 对微分进行低通滤波；为 `false` 时直接使用原始微分，忽略滤波时间常数。启用时，时间常数须为有限非负数；设为 0 同样使用原始微分。未指定 `D_Filter_Enable` 的零初始化配置默认关闭滤波。修改后可调用 `PID_Reset()` 清除旧历史。

可监视 `pid.Target`、`Now`、`Error`、`P_Out`、`I_Out`、`D_Out` 和 `Out`。速度环依次读取反馈、直接计算 PID、发送 PWM，不复制或整份回写 PID 对象。PID 计算后立即更新历史，PWM 写入失败时不回退；`loop.pwm` 表示计算输出，发送结果仍需检查返回值和状态字节。读取失败时不计算 PID。

原来的 `PidConfig`、`PidController` 和 `Pid_Init/Reset/Update` 已替换为以上名称，调用代码和测试同步更新。PID 参数和周期由实际通信时间、负载及反馈确定，没有已验证的默认参数。

## 角度串级环

控制链为：目标角度（度）→ 角度 PID → 目标速度（寄存器原始值）→ 速度 PID 和速度前馈 → PWM。
舵机保持 PWM 开环模式，不切换到舵机内置的位置模式。

`ServoAngleLoop` 绑定已初始化的 `ServoSpeedLoop`，不重设内环参数或清除内环历史。
`FtServoMotionConfig` 提供实测角度、位置端点；不猜测零点或机械行程。
外环 `Out_Min/Max` 是目标速度上下限，范围须包含零并位于 -32767～32767 内；
`I_Out_Min/Max` 是外环积分输出的速度限幅。外环 `K_F` 必须为零，速度前馈仍由内环提供。

```c
#include "servo_angle_loop.h"

ServoAngleLoop angle_loop;
/* loop 为现有速度环指针；两个配置由调用者按标定与限速填写。 */
bool initialized = ServoAngleLoop_Init(
    &angle_loop, loop, &motion_config, &angle_pid_config);

/* 初始化成功、PWM 模式与扭矩确认后，在控制周期内调用。 */
FtServoResult result = ServoAngleLoop_Update(
    &angle_loop, target_angle, D_T / 1000.0f, &pid_status);
/* 同时检查 result == FT_SERVO_OK 和 pid_status == 0。 */
```

角度环调用会更新速度环，本周期不要再单独调用 `ServoSpeedLoop_Update()`。
角度目标限于标定范围；仅支持位置为非负、端点不相同且不跨回绕点的区间，不实现多圈、最短路径或机械限位保护。
位置增加方向须与解码后的速度正方向一致，须在低输出下实测确认。
反向标定时自动反转外环目标速度；内环继续使用当前已验证的 PWM 整体取反。

可观察 `angle_loop.target_angle`、`measured_angle`、`target_speed`、`pid.Error` 与内环 `pwm`。
首次调试外环先使用 P 控制、I/D/F 为零，并根据机构设置较小的速度和 PWM 限幅；没有已验证的默认增益。
外环抗饱和只针对其速度限幅，不跟踪内环 PWM 饱和。停止测试后才能重置两个 PID 历史。
一次更新包含位置读取、速度读取和 PWM 写入，实际周期须测量；通信失败不自动清除旧 PWM。
当前没有把角度环接入 `main.c` 的上电运动。

## 主机检查

在仓库根目录分别构建运行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IController/Inc tests/test_pid.c Controller/Src/PID.c -o build/test_pid.exe -lm
./build/test_pid.exe
gcc -std=c11 -Wall -Wextra -Werror -IController/Inc -Idriver/Inc tests/test_servo_speed_loop.c Controller/Src/PID.c Controller/Src/servo_speed_loop.c -o build/test_servo_speed_loop.exe -lm
./build/test_servo_speed_loop.exe
gcc -std=c11 -Wall -Wextra -Werror -IController/Inc -Idriver/Inc tests/test_servo_angle_loop.c Controller/Src/PID.c Controller/Src/servo_speed_loop.c Controller/Src/servo_angle_loop.c -o build/test_servo_angle_loop.exe -lm
./build/test_servo_angle_loop.exe
```

测试只验证算法和通信替身下的反馈处理，不代表实物闭环已稳定。
