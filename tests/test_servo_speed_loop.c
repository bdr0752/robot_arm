#include "servo_speed_loop.h"
#include "ft_servo_protocol.h"

#include <assert.h>

static uint16_t feedback_speed;
static int16_t commanded_pwm;

/**
  * @brief  提供带 BIT15 方向位的速度反馈，代替实际串口。
  * @param  id: 测试舵机 ID。
  * @param  speed: 原始速度输出。
  * @param  status: 应答状态输出。
  * @retval FT_SERVO_OK。
  */
FtServoResult ft_servo_read_speed(uint8_t id, uint16_t *speed, uint8_t *status)
{
    assert(id == 6U);
    *speed = feedback_speed;
    *status = 0U;
    return FT_SERVO_OK;
}

/**
  * @brief  记录速度环下发的 PWM，代替实际串口。
  * @param  id: 测试舵机 ID。
  * @param  pwm: PWM 原始值。
  * @param  status: 应答状态输出。
  * @retval FT_SERVO_OK。
  */
FtServoResult ft_servo_write_pwm(uint8_t id, int16_t pwm, uint8_t *status)
{
    assert(id == 6U);
    commanded_pwm = pwm;
    *status = 0U;
    return FT_SERVO_OK;
}

/**
  * @brief  验证正反转速度解码及 PWM 输出限幅。
  * @param  无。
  * @retval 0 表示所有检查通过。
  */
int main(void)
{
    const Struct_PID_Config config = {
        .K_P = 1.0f,
        .K_I = 0.0f,
        .K_D = 0.0f,
        .Out_Min = -200.0f,
        .Out_Max = 200.0f,
        .I_Out_Min = -100.0f,
        .I_Out_Max = 100.0f,
        .D_Filter_Time = 0.0f
    };
    ServoSpeedLoop loop;
    uint8_t status;
    assert(ServoSpeedLoop_Init(&loop, 6U, &config));

    feedback_speed = 100U;
    assert(ServoSpeedLoop_Update(&loop, 150.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(status == 0U && loop.measured_speed == 100 && commanded_pwm == 50);

    feedback_speed = 0x8064U;
    assert(ServoSpeedLoop_Update(&loop, -150.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(status == 0U && loop.measured_speed == -100 && commanded_pwm == -50);

    assert(ServoSpeedLoop_Update(&loop, 1000.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(commanded_pwm == 200 && loop.pwm == 200);
    return 0;
}
