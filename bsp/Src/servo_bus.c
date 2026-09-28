#include "servo_bus.h"
#include "bsp_board.h"

#include <stddef.h>

#define SERVO_UART_TIMEOUT_MS 20U

static ServoBusResult bus_result(BspBoardResult result)
{
    if (result == BSP_BOARD_OK) {
        return SERVO_BUS_OK;
    }
    if (result == BSP_BOARD_TIMEOUT) {
        return SERVO_BUS_TIMEOUT;
    }
    return SERVO_BUS_ERROR;
}

ServoBusResult servo_bus_send(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) {
        return SERVO_BUS_ERROR;
    }

    /* Select the board's transmit direction. */
    bsp_board_rs485_set_tx(1U);
    BspBoardResult result =
        bsp_board_servo_uart_send(data, length, SERVO_UART_TIMEOUT_MS);

    /* Always restore the board's receive direction. */
    bsp_board_rs485_set_tx(0U);
    return bus_result(result);
}

ServoBusResult servo_bus_receive(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) {
        return SERVO_BUS_ERROR;
    }

    return bus_result(
        bsp_board_servo_uart_receive(data, length, SERVO_UART_TIMEOUT_MS)
    );
}
