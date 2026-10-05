#include "ft_Servo.h"
#include "ft_servo_types.h"

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

/**
 * @brief  临时切换至恒速模式，设置目标速度并开启扭矩。
 * @param  id: 单播舵机 ID。
 * @param  speed: 目标速度原始值。
 * @param  acceleration: 加速度原始值。
 * @param  speed_status: 速度写入的应答状态，不能为 NULL。
 * @retval 1：各项检查通过；0：操作失败。
 * @note   模式切换时关闭扭矩；保持锁标志为 1，不保存模式修改。
 */
uint8_t Ft_SetSpeed(uint8_t id, int16_t speed,
                    uint8_t acceleration)
{
    FtServoResult result;

    uint8_t mode_status = 0xFFU;
    uint8_t torque_status = 0xFFU;
    uint8_t lock_status = 0xFFU;
	uint8_t speed_status = 0xFFU;
    uint8_t mode = 0xFFU;
    uint8_t lock = 0xFFU;
    uint8_t torque = 0xFFU;

    if (id > 253U || speed_status == NULL) {
        return 0U;
    }

    

    /* 读取当前模式。 */
    result = ft_servo_read_mode(id, &mode, &mode_status);
    if (result != FT_SERVO_OK || mode_status != 0U) {
        return 0U;
    }

    /* 读取当前扭矩开关状态。 */
    result = ft_servo_read_byte(
        id, FT_SMS_REG_TORQUE_ENABLE, &torque, &torque_status);
    if (result != FT_SERVO_OK || torque_status != 0U ||
        torque > 1U) {
        return 0U;
    }

    if (mode != FT_SMS_MODE_WHEEL) {
        /* 确认锁定，避免本次模式修改被掉电保存。 */
        result = ft_servo_read_byte(
            id, FT_SMS_REG_LOCK, &lock, &lock_status);
        if (result != FT_SERVO_OK || lock_status != 0U) {
            return 0U;
        }

        if (lock != 1U) {
            result = ft_servo_set_lock(id, 1U, &lock_status);
            if (result != FT_SERVO_OK || lock_status != 0U) {
                return 0U;
            }

            result = ft_servo_read_byte(
                id, FT_SMS_REG_LOCK, &lock, &lock_status);
            if (result != FT_SERVO_OK || lock_status != 0U ||
                lock != 1U) {
                return 0U;
            }
        }

        /* 仅在切换模式时关闭扭矩。 */
        if (torque == 1U) {
            result = ft_servo_set_torque(id, 0U, &torque_status);
            if (result != FT_SERVO_OK || torque_status != 0U) {
                return 0U;
            }

            result = ft_servo_read_byte(
                id, FT_SMS_REG_TORQUE_ENABLE,
                &torque, &torque_status);
            if (result != FT_SERVO_OK || torque_status != 0U ||
                torque != 0U) {
                return 0U;
            }
        }

        /* 写模式后，立即检查写入结果。 */
        result = ft_servo_set_mode(
            id, FT_SMS_MODE_WHEEL, &mode_status);
        if (result != FT_SERVO_OK || mode_status != 0U) {
            return 0U;
        }

        /* 读回确认模式。 */
        result = ft_servo_read_mode(id, &mode, &mode_status);
        if (result != FT_SERVO_OK || mode_status != 0U ||
            mode != FT_SMS_MODE_WHEEL) {
            return 0U;
        }
    }

    /* 先写目标速度。 */
    result = ft_servo_write_speed(
        id, speed, acceleration, &speed_status);
    if (result != FT_SERVO_OK || speed_status != 0U) {
        return 0U;
    }

    /* 已开启时保持运行；关闭时再开启。 */
    if (torque == 0U) {
        result = ft_servo_set_torque(id, 1U, &torque_status);
        if (result != FT_SERVO_OK || torque_status != 0U) {
            return 0U;
        }

        result = ft_servo_read_byte(
            id, FT_SMS_REG_TORQUE_ENABLE, &torque, &torque_status);
        if (result != FT_SERVO_OK || torque_status != 0U ||
            torque != 1U) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  临时切换至位置模式，写入目标位置并开启扭矩。
 * @param  id: 单播舵机 ID，范围 0～253。
 * @param  position: 目标位置原始值，不是角度。
 * @param  speed: 运动速度原始值，不能为负数。
 * @param  acceleration: 加速度原始值。
 * @retval 1：操作及检查通过；0：操作失败。
 * @note   模式切换时关闭扭矩；锁标志保持为 1，不保存模式修改。
 *         本函数不等待舵机到位，也不检查机械行程。
 */
uint8_t Ft_SetPosition(uint8_t id, int16_t position,
                      int16_t speed, uint8_t acceleration)
{
    FtServoResult result;

    uint8_t mode_status = 0xFFU;
    uint8_t torque_status = 0xFFU;
    uint8_t lock_status = 0xFFU;
    uint8_t position_status = 0xFFU;

    uint8_t mode = 0xFFU;
    uint8_t lock = 0xFFU;
    uint8_t torque = 0xFFU;

    /* 检查参数；-32768 无法使用当前驱动的符号格式编码。 */
    if (id > 253U || speed < 0 || position == INT16_MIN) {
        return 0U;
    }

    /* 读取当前模式。 */
    result = ft_servo_read_mode(id, &mode, &mode_status);
    if (result != FT_SERVO_OK || mode_status != 0U) {
        return 0U;
    }

    /* 读取当前扭矩开关状态。 */
    result = ft_servo_read_byte(
        id, FT_SMS_REG_TORQUE_ENABLE, &torque, &torque_status);
    if (result != FT_SERVO_OK || torque_status != 0U ||
        torque > 1U) {
        return 0U;
    }

    if (mode != FT_SMS_MODE_POSITION) {
        /* 确认锁定，避免本次模式修改被掉电保存。 */
        result = ft_servo_read_byte(
            id, FT_SMS_REG_LOCK, &lock, &lock_status);
        if (result != FT_SERVO_OK || lock_status != 0U) {
            return 0U;
        }

        if (lock != 1U) {
            result = ft_servo_set_lock(id, 1U, &lock_status);
            if (result != FT_SERVO_OK || lock_status != 0U) {
                return 0U;
            }

            /* 读回确认锁标志。 */
            result = ft_servo_read_byte(
                id, FT_SMS_REG_LOCK, &lock, &lock_status);
            if (result != FT_SERVO_OK || lock_status != 0U ||
                lock != 1U) {
                return 0U;
            }
        }

        /* 仅在切换模式时关闭扭矩。 */
        if (torque == 1U) {
            result = ft_servo_set_torque(id, 0U, &torque_status);
            if (result != FT_SERVO_OK || torque_status != 0U) {
                return 0U;
            }

            /* 读回确认扭矩关闭。 */
            result = ft_servo_read_byte(
                id, FT_SMS_REG_TORQUE_ENABLE,
                &torque, &torque_status);
            if (result != FT_SERVO_OK || torque_status != 0U ||
                torque != 0U) {
                return 0U;
            }
        }

        /* 切换为位置模式。 */
        result = ft_servo_set_mode(
            id, FT_SMS_MODE_POSITION, &mode_status);
        if (result != FT_SERVO_OK || mode_status != 0U) {
            return 0U;
        }

        /* 读回确认模式。 */
        result = ft_servo_read_mode(id, &mode, &mode_status);
        if (result != FT_SERVO_OK || mode_status != 0U ||
            mode != FT_SMS_MODE_POSITION) {
            return 0U;
        }
    }

    /* 先写入目标位置、速度和加速度。 */
    result = ft_servo_write_position(
        id, position, (uint16_t)speed,
        acceleration, &position_status);
    if (result != FT_SERVO_OK || position_status != 0U) {
        return 0U;
    }

    /* 扭矩已开启时保持运行；关闭时再开启。 */
    if (torque == 0U) {
        result = ft_servo_set_torque(id, 1U, &torque_status);
        if (result != FT_SERVO_OK || torque_status != 0U) {
            return 0U;
        }

        /* 读回确认扭矩开启。 */
        result = ft_servo_read_byte(
            id, FT_SMS_REG_TORQUE_ENABLE,
            &torque, &torque_status);
        if (result != FT_SERVO_OK || torque_status != 0U ||
            torque != 1U) {
            return 0U;
        }
    }

    return 1U;
}
