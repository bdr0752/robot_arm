#include "ft_Servo.h"
#include "ft_servo_defs.h"
#include "servo_bus.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

static const uint8_t *expected_tx;
static uint16_t expected_tx_length;
static const uint8_t *reply;
static uint16_t reply_length;
static unsigned call_count;

ServoBusResult servo_bus_exchange(uint8_t *tx, uint16_t tx_length,
                                  uint8_t *rx, uint16_t rx_length)
{
    assert(tx_length == expected_tx_length);
    assert(memcmp(tx, expected_tx, tx_length) == 0);
    assert(rx_length == reply_length);
    if (rx_length != 0U) {
        assert(rx != NULL);
        memcpy(rx, reply, rx_length);
    }
    ++call_count;
    return SERVO_BUS_OK;
}

static void expect_exchange(const uint8_t *tx, uint16_t tx_length,
                            const uint8_t *rx, uint16_t rx_length)
{
    expected_tx = tx;
    expected_tx_length = tx_length;
    reply = rx;
    reply_length = rx_length;
    call_count = 0U;
}

int main(void)
{
    uint8_t status = 0xFFU;
    const uint8_t ack[] = {0xFF, 0xFF, 0x06, 0x02, 0x00, 0xF7};
    const uint8_t position_write[] = {
        0xFF, 0xFF, 0x06, 0x0A, 0x03, 0x29,
        0x0A, 0x00, 0x08, 0x00, 0x00, 0x14, 0x00, 0x9D
    };
    expect_exchange(position_write, sizeof(position_write), ack, sizeof(ack));
    assert(ft_servo_write_position(0x06, 2048, 20U, 10U, &status) == FT_SERVO_OK);
    assert(call_count == 1U && status == 0U);

    const uint8_t reverse_speed[] = {
        0xFF, 0xFF, 0x06, 0x0A, 0x03, 0x29,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x64, 0x80, 0xDC
    };
    expect_exchange(reverse_speed, sizeof(reverse_speed), ack, sizeof(ack));
    assert(ft_servo_write_speed(0x06, -100, 3U, &status) == FT_SERVO_OK);
    assert(call_count == 1U);

    const uint8_t read_request[] = {0xFF, 0xFF, 0x06, 0x04, 0x02, 0x38, 0x02, 0xB9};
    const uint8_t read_reply[] = {0xFF, 0xFF, 0x06, 0x04, 0x00, 0x34, 0x12, 0xAF};
    uint16_t value = 0U;
    expect_exchange(read_request, sizeof(read_request), read_reply, sizeof(read_reply));
    assert(ft_servo_read_word(0x06, FT_SMS_REG_PRESENT_POS_L, &value, &status)
           == FT_SERVO_OK);
    assert(call_count == 1U && value == 0x1234U && status == 0U);

    uint8_t corrupt_reply[sizeof(read_reply)];
    memcpy(corrupt_reply, read_reply, sizeof(read_reply));
    corrupt_reply[7] ^= 1U;
    expect_exchange(read_request, sizeof(read_request), corrupt_reply, sizeof(corrupt_reply));
    value = 0xAAAAU;
    assert(ft_servo_read_word(0x06, FT_SMS_REG_PRESENT_POS_L, &value, &status)
           == FT_SERVO_BAD_CHECKSUM);
    assert(value == 0xAAAAU);

    const uint8_t broadcast_torque[] = {
        0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x28, 0x01, 0xD1
    };
    expect_exchange(broadcast_torque, sizeof(broadcast_torque), NULL, 0U);
    assert(ft_servo_set_torque(FT_SCS_BROADCAST_ID, 1U, NULL) == FT_SERVO_OK);
    assert(call_count == 1U);

    const uint8_t action[] = {0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA};
    expect_exchange(action, sizeof(action), NULL, 0U);
    assert(ft_servo_action(FT_SCS_BROADCAST_ID, NULL) == FT_SERVO_OK);
    assert(call_count == 1U);

    const FtServoPositionCommand commands[] = {
        {0x06, 2048, 20U, 10U},
        {0x07, 1024, 10U, 1U}
    };
    const uint8_t sync_position[] = {
        0xFF, 0xFF, 0xFE, 0x14, 0x83, 0x29, 0x07,
        0x06, 0x0A, 0x00, 0x08, 0x00, 0x00, 0x14, 0x00,
        0x07, 0x01, 0x00, 0x04, 0x00, 0x00, 0x0A, 0x00, 0xF8
    };
    expect_exchange(sync_position, sizeof(sync_position), NULL, 0U);
    assert(ft_servo_sync_write_position(commands, 2U) == FT_SERVO_OK);
    assert(call_count == 1U);

    call_count = 0U;
    assert(ft_servo_read(0xFE, 0x38, (uint8_t *)&value, 2U, &status)
           == FT_SERVO_BAD_ARGUMENT);
    assert(ft_servo_write_position(0x06, INT16_MIN, 20U, 10U, &status)
           == FT_SERVO_BAD_ARGUMENT);
    assert(ft_servo_set_torque(0x06, 2U, &status) == FT_SERVO_BAD_ARGUMENT);
    assert(call_count == 0U);
    return 0;
}
