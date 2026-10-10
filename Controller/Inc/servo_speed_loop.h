#ifndef SERVO_SPEED_LOOP_H
#define SERVO_SPEED_LOOP_H

#include "PID.h"
#include "ft_servo_types.h"

/* 目标和反馈均使用 SMS/STS 带符号速度原始值，不进行 RPM 换算。 */
typedef struct {
    uint8_t id;
    Struct_PID pid;
    float target_speed;
    int16_t measured_speed;
    int16_t pwm;
} ServoSpeedLoop;

bool ServoSpeedLoop_Init(ServoSpeedLoop *loop, uint8_t id,
                         const Struct_PID_Config *config);
/* 调用前配置 PWM 模式和扭矩；周期调用，D_T 为实际反馈间隔。
 * 内部使用阻塞串口，不能在中断中调用，同一总线须串行访问。
 * 返回 FT_SERVO_OK 且 *status == 0 才表示本次读写成功。
 * 不包含自动停机、故障锁存或模式切换。 */
FtServoResult ServoSpeedLoop_Update(ServoSpeedLoop *loop, float target_speed,
                                     float D_T, uint8_t *status);

#endif
