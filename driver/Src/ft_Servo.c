#include "ft_Servo.h"
#include "ft_servo_protocol.h"

#include <float.h>
#include <stddef.h>

/**
  * @brief  检查角度与位置端点配置是否可用于线性换算。
  * @param  config: 舵机 ID 和两个标定端点。
  * @retval 1 表示有效，0 表示无效。
  */
static uint8_t motion_config_valid(const FtServoMotionConfig *config)
{
    return config != NULL && config->id <= 253U &&
           config->min_angle_deg >= -FLT_MAX &&
           config->max_angle_deg <= FLT_MAX &&
           config->min_angle_deg < config->max_angle_deg &&
           config->position_at_min_angle <= INT16_MAX &&
           config->position_at_max_angle <= INT16_MAX &&
           config->position_at_min_angle != config->position_at_max_angle;
}

/**
  * @brief  根据标定端点将目标角度换算为目标位置并发送。
  * @param  config: 舵机 ID 和角度、位置端点。
  * @param  angle_deg: 目标角度，须在配置的角度区间内。
  * @param  speed: 舵机目标速度寄存器原始值。
  * @param  acceleration: 舵机加速度寄存器原始值。
  * @param  status: 非 NULL 时写入舵机应答状态字节。
  * @retval FT_SERVO_OK、参数错误或底层通信错误。
  */
FtServoResult ft_servo_motion_goto_angle(const FtServoMotionConfig *config,
                                         float angle_deg, uint16_t speed,
                                         uint8_t acceleration, uint8_t *status)
{
    if (!motion_config_valid(config) ||
        !(angle_deg >= config->min_angle_deg && angle_deg <= config->max_angle_deg)) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    float ratio = (angle_deg - config->min_angle_deg) /
                  (config->max_angle_deg - config->min_angle_deg);
    float raw = (float)config->position_at_min_angle + ratio *
                ((float)config->position_at_max_angle -
                 (float)config->position_at_min_angle);
    /* 端点限制已保证结果非负；加 0.5 后截断得到最近的整数位置。 */
    uint16_t position = (uint16_t)(raw + 0.5f);
    return ft_servo_write_position(config->id, (int16_t)position,
                                   speed, acceleration, status);
}

/**
  * @brief  读取原始位置并按标定端点换算为角度。
  * @param  config: 舵机 ID 和角度、位置端点。
  * @param  angle_deg: 成功时写入当前角度；越过标定端点时会线性外推。
  * @param  status: 非 NULL 时写入舵机应答状态字节。
  * @retval FT_SERVO_OK、参数错误或底层通信错误。
  */
FtServoResult ft_servo_motion_read_angle(const FtServoMotionConfig *config,
                                         float *angle_deg, uint8_t *status)
{
    if (!motion_config_valid(config) || angle_deg == NULL) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    uint16_t position;
    FtServoResult result = ft_servo_read_position(config->id, &position, status);
    if (result != FT_SERVO_OK) {
        return result;
    }
    float ratio = ((float)position - (float)config->position_at_min_angle) /
                  ((float)config->position_at_max_angle -
                   (float)config->position_at_min_angle);
    *angle_deg = config->min_angle_deg +
                 ratio * (config->max_angle_deg - config->min_angle_deg);
    return FT_SERVO_OK;
}

/**
  * @brief  以当前反馈位置为起点，相对转动指定角度。
  * @param  config: 舵机 ID 和角度、位置端点。
  * @param  delta_deg: 相对角度，正负方向由标定端点决定。
  * @param  speed: 舵机目标速度寄存器原始值。
  * @param  acceleration: 舵机加速度寄存器原始值。
  * @param  status: 非 NULL 时写入最后一次写位置的应答状态字节。
  * @retval FT_SERVO_OK、目标越界、读取失败或写入失败。
  */
FtServoResult ft_servo_motion_move_by_angle(const FtServoMotionConfig *config,
                                            float delta_deg, uint16_t speed,
                                            uint8_t acceleration, uint8_t *status)
{
    float current_angle;
    FtServoResult result = ft_servo_motion_read_angle(config, &current_angle, NULL);
    if (result != FT_SERVO_OK) {
        return result;
    }
    return ft_servo_motion_goto_angle(config, current_angle + delta_deg,
                                      speed, acceleration, status);
}
