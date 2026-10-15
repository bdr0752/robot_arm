#include "servo_angle_loop.h"
#include "PID.h"
#include "ft_servo_types.h"

#include <cstdint>
#include <math.h>
#include <stddef.h>

/**
 * @brief 绑定角度标定与已初始化的速度环，初始化外环 PID。
 * @param loop 角度环对象。
 * @param speed_loop 本台舵机已调好的速度环对象。
 * @param motion 不跨回绕点的角度与位置端点标定。
 * @param config 外环 PID，输出为目标速度原始值，前馈为零。
 * @retval true 为成功，false 为配置无效。
 */
bool ServoAngleLoop_Init(ServoAngleLoop *loop, ServoSpeedLoop *speed_loop,
                         const Struct_PID_Config *config)
{
    if (loop == NULL || speed_loop == NULL|| config == NULL ||
        config->K_F != 0.0f ||
        config->Out_Min < -32767.0f || config->Out_Max > 32767.0f ||
        config->Out_Min > 0.0f || config->Out_Max < 0.0f ||
        !PID_Init(&loop->pid, config)) {
        return false;
    }

    loop->speed_loop = speed_loop;
    loop->target_position = 0.0f;
    loop->measured_position = 0.0f;
    loop->target_speed = 0.0f;
    return true;
}

/**
 * @brief 读取角度，外环计算目标速度，再由内环计算并下发 PWM。
 * @param loop 已初始化的角度环对象。
 * @param target_angle 标定范围内的目标角度，单位度。
 * @param D_T 实际反馈间隔，单位秒，必须大于零。
 * @param status 非 NULL，接收最近一次舵机应答状态。
 * @retval 驱动结果；成功还需检查 status 为零。
 * @note 速度反馈的正方向须与位置原始值增加方向一致，须实测确认。
 *       两个环的 PID 历史直接更新，通信失败时不回退。
 */
FtServoResult ServoAngleLoop_Update(ServoAngleLoop *loop, float target_position,
                                     float D_T, uint8_t *status)
{
    if (loop == NULL || status == NULL || loop->speed_loop == NULL ||
        loop->speed_loop->id != loop->motion.id || loop->pid.Config.K_F != 0.0f ||
        !isfinite(target_position) || !isfinite(D_T) || D_T <= 0.0f) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    uint16_t measured_position;
    float target_speed;
    float measured_speed;
    
    *status = 0xFFU;
    FtServoResult result = ft_servo_read_position(loop->motion.id, &measured_position, status);
    if (result != FT_SERVO_OK || *status != 0U) return result;
    if (measured_position > INT16_MAX) return FT_SERVO_BAD_REPLY;

    PID_Adjust(&loop->pid, target_position, measured_position, D_T, &target_speed);
    FtServoResult speed_result = ServoSpeedLoop_Update(loop->speed_loop, target_speed, D_T, status);
    loop->target_position = target_position;
    loop->measured_position = measured_position;
    loop->target_speed = target_speed;
    return speed_result;
}
