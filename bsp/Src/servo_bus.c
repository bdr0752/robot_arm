#include "servo_bus.h"
#include "gpio.h"
#include "usart.h"

#define SERVO_UART_TIMEOUT_MS 20U

static ServoBusResult bus_result(HAL_StatusTypeDef result)
{
    if (result == HAL_OK) {
        return SERVO_BUS_OK;
    }
    if (result == HAL_TIMEOUT) {
        return SERVO_BUS_TIMEOUT;
    }
    return SERVO_BUS_ERROR;
}

ServoBusResult servo_bus_send(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) {
        return SERVO_BUS_ERROR;
    }

    /* PB14 同时连接 DE 与 RE#：高电平发送 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_StatusTypeDef result =
        HAL_UART_Transmit(&huart3, data, length, SERVO_UART_TIMEOUT_MS);

    /* 无论发送结果如何，都切回接收方向 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    return bus_result(result);
}

ServoBusResult servo_bus_receive(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) {
        return SERVO_BUS_ERROR;
    }

    return bus_result(
        HAL_UART_Receive(&huart3, data, length, SERVO_UART_TIMEOUT_MS)
    );
}