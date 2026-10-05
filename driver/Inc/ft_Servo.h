#ifndef FT_SERVO_H
#define FT_SERVO_H

#include "ft_servo_types.h"
#include "ft_servo_protocol.h"
/*
 * 按实际舵机标定两个端点。位置值是舵机寄存器原始值，不是角度。
 * 两个位置端点可以递增或递减，以适配安装方向。
 * 此层不猜测机械零位、行程或位置模式的寄存器值。
 */
typedef struct {
    uint8_t id;
    float min_angle_deg;
    float max_angle_deg;
    uint16_t position_at_min_angle;
    uint16_t position_at_max_angle;
} FtServoMotionConfig;

/* 运动前需由调用者确认舵机已进入位置模式并启用扭矩。 */
FtServoResult ft_servo_motion_goto_angle(const FtServoMotionConfig *config,
                                         float angle_deg, uint16_t speed,
                                         uint8_t acceleration, uint8_t *status);
FtServoResult ft_servo_motion_read_angle(const FtServoMotionConfig *config,
                                         float *angle_deg, uint8_t *status);
FtServoResult ft_servo_motion_move_by_angle(const FtServoMotionConfig *config,
                                            float delta_deg, uint16_t speed,
                                            uint8_t acceleration, uint8_t *status);
uint8_t Ft_SetSpeed(uint8_t id, int16_t speed,
                    uint8_t acceleration);
											
uint8_t Ft_SetPosition(uint8_t id, int16_t position,
                      int16_t speed, uint8_t acceleration);
#endif
