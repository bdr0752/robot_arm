#include "ft_Servo.h"

#include <assert.h>
#include <stddef.h>

static uint16_t feedback_position;
static uint8_t written_id;
static int16_t written_position;
static uint16_t written_speed;
static uint8_t written_acceleration;
static unsigned write_count;

FtServoResult ft_servo_read_position(uint8_t id, uint16_t *position, uint8_t *status)
{
    assert(id == 6U);
    *position = feedback_position;
    if (status != NULL) {
        *status = 0U;
    }
    return FT_SERVO_OK;
}

FtServoResult ft_servo_write_position(uint8_t id, int16_t position,
                                      uint16_t speed, uint8_t acceleration,
                                      uint8_t *status)
{
    written_id = id;
    written_position = position;
    written_speed = speed;
    written_acceleration = acceleration;
    ++write_count;
    if (status != NULL) {
        *status = 0U;
    }
    return FT_SERVO_OK;
}

int main(void)
{
    FtServoMotionConfig config = {6U, 0.0f, 180.0f, 1000U, 3000U};
    uint8_t status = 0xFFU;
    assert(ft_servo_motion_goto_angle(&config, 90.0f, 20U, 10U, &status)
           == FT_SERVO_OK);
    assert(write_count == 1U && written_id == 6U && written_position == 2000);
    assert(written_speed == 20U && written_acceleration == 10U && status == 0U);

    feedback_position = 2000U;
    float angle = -1.0f;
    assert(ft_servo_motion_read_angle(&config, &angle, NULL) == FT_SERVO_OK);
    assert(angle == 90.0f);
    assert(ft_servo_motion_move_by_angle(&config, 45.0f, 30U, 5U, NULL)
           == FT_SERVO_OK);
    assert(write_count == 2U && written_position == 2500);

    config.position_at_min_angle = 3000U;
    config.position_at_max_angle = 1000U;
    assert(ft_servo_motion_goto_angle(&config, 45.0f, 20U, 10U, NULL)
           == FT_SERVO_OK);
    assert(write_count == 3U && written_position == 2500);

    assert(ft_servo_motion_goto_angle(&config, 181.0f, 20U, 10U, NULL)
           == FT_SERVO_BAD_ARGUMENT);
    config.position_at_max_angle = config.position_at_min_angle;
    assert(ft_servo_motion_goto_angle(&config, 90.0f, 20U, 10U, NULL)
           == FT_SERVO_BAD_ARGUMENT);
    assert(write_count == 3U);
    return 0;
}
