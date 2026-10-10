# PID 与舵机速度环

## 文件与调用

- `Inc/PID.h`、`Src/PID.c`：独立 PID 算法，不依赖 HAL 或舵机通信。
- `Inc/servo_speed_loop.h`、`Src/servo_speed_loop.c`：读取速度、调用 PID、下发 PWM。
- `PID_Init()` 保存参数，`PID_Reset()` 清除计算历史，`PID_Adjust()` 执行一次计算。
- `ServoSpeedLoop_Init()` 绑定 ID 与 PID 参数；上层设置 PWM 模式、扭矩后，周期调用 `ServoSpeedLoop_Update()`。

速度环使用带符号的速度寄存器原始值，不换算 RPM。调用间隔以秒传入 PID；使用阻塞通信，不在中断中调用，同一总线串行访问。未实现自动停机或模式切换。

## 命名与计算顺序

沿用参考例程的命名格式：`K_P`、`K_I`、`K_D` 为增益，`Target`、`Now` 为目标与反馈，`Pre_Now` 为上次反馈，`D_T` 为采样间隔。C 结构体为 `Struct_PID_Config` 与 `Struct_PID`。`I_Out_Min/Max` 限制积分项输出，`D_Filter_Time` 为微分低通时间常数。

计算依次为比例项、积分项、微分项、抗饱和与总输出限幅、历史更新。饱和时禁止积分继续向饱和方向增加；微分针对测量值，避免目标阶跃冲击。首个样本的微分输出为零。没有引入前馈、死区或额外控制模式。

`D_Filter_Enable = true` 时，根据 `D_Filter_Time` 对微分进行低通滤波；为 `false` 时直接使用原始微分，忽略滤波时间常数。启用时，时间常数须为有限非负数；设为 0 同样使用原始微分。未指定 `D_Filter_Enable` 的零初始化配置默认关闭滤波。修改后可调用 `PID_Reset()` 清除旧历史。

可监视 `pid.Target`、`Now`、`Error`、`P_Out`、`I_Out`、`D_Out` 和 `Out`。速度环对象中的 PID 历史只在 PWM 写入成功且状态为零后提交。

原来的 `PidConfig`、`PidController` 和 `Pid_Init/Reset/Update` 已替换为以上名称，调用代码和测试同步更新。PID 参数和周期由实际通信时间、负载及反馈确定，没有已验证的默认参数。

## 主机检查

在仓库根目录分别构建运行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IController/Inc tests/test_pid.c Controller/Src/PID.c -o build/test_pid.exe -lm
./build/test_pid.exe
gcc -std=c11 -Wall -Wextra -Werror -IController/Inc -Idriver/Inc tests/test_servo_speed_loop.c Controller/Src/PID.c Controller/Src/servo_speed_loop.c -o build/test_servo_speed_loop.exe -lm
./build/test_servo_speed_loop.exe
```

测试只验证算法和通信替身下的反馈处理，不代表实物闭环已稳定。
