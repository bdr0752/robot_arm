#ifndef SERVO_ANGLE_LOOP_H
#define SERVO_ANGLE_LOOP_H

#include "servo_speed_loop.h"
#include "ft_Servo.h"

/* 单个不跨位置回绕点的标定区间；外环角度为度，输出为速度原始值。 */
typedef struct {
    Struct_PID pid;
    ServoSpeedLoop *speed_loop;
    float target_position;
    float measured_position;
    float target_speed;
} ServoAngleLoop;

/* speed_loop 须已初始化且与 motion 使用同一 ID。config 为外环配置，
 * Out_Min/Max 限制角度坐标正方向下的目标速度，K_F 必须为零。
 * 初始化不访问硬件，也不清除已调好的速度环参数和历史。 */
bool ServoAngleLoop_Init(ServoAngleLoop *loop, ServoSpeedLoop *speed_loop,
                         const Struct_PID_Config *config);

/* 调用前配置 PWM 模式和扭矩；本接口替代本周期的速度环单独更新。
 * target_angle 须位于标定角度范围；D_T 为实际反馈周期，单位秒。
 * 返回 OK 且 *status == 0 才表示本轮通信成功；失败不自动停止旧 PWM。
 * 不实现回绕、多圈或机械行程保护；不得在中断中调用。 */
FtServoResult ServoAngleLoop_Update(ServoAngleLoop *loop, float target_angle,
                                     float D_T, uint8_t *status);

#endif
