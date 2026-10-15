#include "servo_angle_loop.h"

#include <assert.h>
#include <math.h>

static uint16_t feedback_position = 1500U;
static uint16_t feedback_speed;
static int16_t commanded_pwm;
static unsigned write_count;
static FtServoResult position_result = FT_SERVO_OK;
static uint8_t position_status;

/** @brief 提供测试位置反馈。@param id 舵机 ID。@param position 位置输出。@param status 状态输出。@retval 指定读取结果 */
FtServoResult ft_servo_read_position(uint8_t id, uint16_t *position, uint8_t *status)
{
    assert(id == 6U);
    *position = feedback_position;
    *status = position_status;
    return position_result;
}

/** @brief 提供测试速度反馈。@param id 舵机 ID。@param speed 速度输出。@param status 状态输出。@retval OK */
FtServoResult ft_servo_read_speed(uint8_t id, uint16_t *speed, uint8_t *status)
{
    assert(id == 6U);
    *speed = feedback_speed;
    *status = 0U;
    return FT_SERVO_OK;
}

/** @brief 捕获串级控制的最终输出。@param id 舵机 ID。@param pwm 输出。@param status 状态输出。@retval OK */
FtServoResult ft_servo_write_pwm(uint8_t id, int16_t pwm, uint8_t *status)
{
    assert(id == 6U);
    commanded_pwm = pwm;
    ++write_count;
    *status = 0U;
    return FT_SERVO_OK;
}

/** @brief 验证角度换算、串级输出、限速、反向标定和读失败时不发送。@retval 0 为通过 */
int main(void)
{
    Struct_PID_Config speed_config = {
        .K_P = 1.0f, .K_F = 0.5f,
        .Out_Min = -200.0f, .Out_Max = 200.0f,
        .I_Out_Min = -100.0f, .I_Out_Max = 100.0f
    };
    Struct_PID_Config angle_config = {
        .K_P = 2.0f,
        .Out_Min = -100.0f, .Out_Max = 100.0f,
        .I_Out_Min = -20.0f, .I_Out_Max = 20.0f
    };
    FtServoMotionConfig motion = {6U, 0.0f, 100.0f, 1000U, 2000U};
    ServoSpeedLoop speed_loop;
    ServoAngleLoop angle_loop;
    uint8_t status;
    assert(ServoSpeedLoop_Init(&speed_loop, 6U, &speed_config));
    assert(ServoAngleLoop_Init(&angle_loop, &speed_loop, &motion, &angle_config));
    assert(speed_loop.pid.Config.K_F == 0.5f);

    feedback_speed = 20U;
    assert(ServoAngleLoop_Update(&angle_loop, 80.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(status == 0U && fabsf(angle_loop.measured_angle - 50.0f) < 0.001f);
    assert(angle_loop.target_speed == 60.0f && speed_loop.target_speed == 60.0f);
    assert(commanded_pwm == -70); // 内环误差输出 40，速度前馈 30。

    assert(ServoAngleLoop_Update(&angle_loop, 100.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(angle_loop.target_speed == 100.0f);
    assert(ServoAngleLoop_Update(&angle_loop, 50.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(angle_loop.target_speed == 0.0f && commanded_pwm == 20);

    unsigned previous_write_count = write_count;
    assert(ServoAngleLoop_Update(&angle_loop, 101.0f, 0.02f, &status) == FT_SERVO_BAD_ARGUMENT);
    position_result = FT_SERVO_TIMEOUT;
    assert(ServoAngleLoop_Update(&angle_loop, 80.0f, 0.02f, &status) == FT_SERVO_TIMEOUT);
    position_result = FT_SERVO_OK;
    position_status = 1U;
    assert(ServoAngleLoop_Update(&angle_loop, 80.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(status == 1U && write_count == previous_write_count);
    position_status = 0U;

    motion.position_at_min_angle = 2000U;
    motion.position_at_max_angle = 1000U;
    feedback_speed = 0U;
    assert(ServoAngleLoop_Init(&angle_loop, &speed_loop, &motion, &angle_config));
    assert(ServoAngleLoop_Update(&angle_loop, 80.0f, 0.02f, &status) == FT_SERVO_OK);
    assert(angle_loop.target_speed == -60.0f && commanded_pwm == 90);
    angle_config.K_F = 1.0f;
    assert(!ServoAngleLoop_Init(&angle_loop, &speed_loop, &motion, &angle_config));
    return 0;
}
