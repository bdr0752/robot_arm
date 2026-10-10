#include "servo_speed_loop.h"

#include "ft_servo_protocol.h"

#include <math.h>
#include <stddef.h>

/**
  * @brief  绑定舵机 ID 并初始化速度 PID，不访问硬件。
  * @param  loop: 速度环对象，每台舵机使用独立对象。
  * @param  id: 单播 ID，范围 0～253。
  * @param  config: PID 配置；输出须包含零且位于 -1000～1000 内。
  * @retval true 为初始化成功，false 为参数无效。
  */
bool ServoSpeedLoop_Init(ServoSpeedLoop *loop, uint8_t id,
                         const Struct_PID_Config *config)
{
    if (loop == NULL || config == NULL || id > 253U ||
        config->Out_Min < -1000.0f || config->Out_Max > 1000.0f ||
        config->Out_Min > 0.0f || config->Out_Max < 0.0f ||
        !PID_Init(&loop->pid, config)) {
        return false;
    }
    loop->id = id;
    loop->target_speed = 0.0f;
    loop->measured_speed = 0;
    loop->pwm = 0;
    return true;
}

/**
  * @brief  读取速度，计算 PID，再写入 PWM 开环输出。
  * @param  loop: 已成功初始化的对象，舵机须预先处于 PWM 模式且启用扭矩。
  * @param  target_speed: 带符号速度原始值，范围 -32767～32767。
  * @param  D_T: 两次有效反馈间隔，单位秒，必须大于零。
  * @param  status: 必须非 NULL，接收读或写应答状态；通信失败可能保持 0xFF。
  * @retval 基础驱动结果；还需检查 status，舵机报告错误时不继续控制。
  * @note   SMS/STS 速度反馈用 BIT15 表示方向，不是二进制补码。
  *         pwm 保存最近一次成功下发的值；不自动停机或关闭扭矩。
  */
FtServoResult ServoSpeedLoop_Update(ServoSpeedLoop *loop, float target_speed,
                                     float D_T, uint8_t *status)
{
    if (loop == NULL || status == NULL || loop->id > 253U ||
        !isfinite(target_speed) || target_speed < -32767.0f ||
        target_speed > 32767.0f || !isfinite(D_T) || D_T <= 0.0f) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    uint16_t raw_speed;
    *status = 0xFFU;
    FtServoResult result = ft_servo_read_speed(loop->id, &raw_speed, status);
    if (result != FT_SERVO_OK || *status != 0U) {
        return result;
    }
    int16_t measured_speed = (int16_t)(raw_speed & 0x7FFFU);
    if ((raw_speed & 0x8000U) != 0U) {
        measured_speed = (int16_t)-measured_speed;
    }

    /* 写成功后才提交 PID 历史，避免失败发送被当成已执行控制。 */
    Struct_PID pid_next = loop->pid;
    float pwm_out;
    if (!PID_Adjust(&pid_next, target_speed, (float)measured_speed,
                    D_T, &pwm_out)) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    int16_t pwm = (int16_t)pwm_out;
    *status = 0xFFU;
    result = ft_servo_write_pwm(loop->id, pwm, status);
    if (result == FT_SERVO_OK && *status == 0U) {
        loop->pid = pid_next;
        loop->target_speed = target_speed;
        loop->measured_speed = measured_speed;
        loop->pwm = pwm;
    }
    return result;
}
