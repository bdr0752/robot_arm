#include "ft_servo_bus.h"
#include "bsp_rs485.hpp"
#include "bsp_dwt.hpp"
#include <assert.h>
#include <string.h>

static DMA_HandleTypeDef Rx_Dma;
UART_HandleTypeDef huart1 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE};
UART_HandleTypeDef huart3 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE};
uint32_t Test_Primask;
DWT_Type Test_Dwt;
GPIO_TypeDef Test_GpioB;
static GPIO_PinState Direction;
static uint32_t Tick;
static const uint8_t Reply[] = {0xFF, 0xFF, 6, 2, 0, 0xF7};
static uint16_t Reply_Size;
static bool Receive_Error;
static bool Tx_Done;
static unsigned Tx_Count;

/** @brief 记录接收 DMA 的地址与容量。@param Uart 串口。@param Data 缓冲区。@param Size 容量。@retval 0 */
extern "C" int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *Uart, uint8_t *Data, uint16_t Size)
{
    Uart->rx_buffer = Data;
    Uart->rx_capacity = Size;
    return 0;
}

/** @brief 模拟停止 DMA。@param Uart 串口。@retval 0 */
extern "C" int HAL_UART_AbortReceive(UART_HandleTypeDef *Uart)
{
    Uart->rx_buffer = 0;
    return 0;
}

/** @brief 校验发送期间的方向及参数。@param Uart 串口。@param Data 数据。@param Size 长度。@param Timeout 超时。@retval 0 */
extern "C" int HAL_UART_Transmit(UART_HandleTypeDef *Uart, uint8_t *Data, uint16_t Size, uint32_t Timeout)
{
    assert(Uart == &huart3 && Direction == GPIO_PIN_SET);
    assert(Data && Size && Timeout == 100U);
    Tx_Done = true;
    ++Tx_Count;
    return 0;
}

/** @brief 模拟方向引脚，并在返回接收方向后注入应答。@param Port 端口。@param Pin 引脚。@param State 电平。@retval 无 */
extern "C" void HAL_GPIO_WritePin(GPIO_TypeDef *Port, uint16_t Pin, GPIO_PinState State)
{
    assert(Port == GPIOB && Pin == GPIO_PIN_14);
    Direction = State;
    if (State != GPIO_PIN_RESET || !Tx_Done) return;
    Tx_Done = false;
    if (Receive_Error) {
        HAL_UART_ErrorCallback(&huart3);
    } else if (Reply_Size) {
        assert(huart3.rx_buffer && Reply_Size <= huart3.rx_capacity);
        memcpy(huart3.rx_buffer, Reply, Reply_Size);
        HAL_UARTEx_RxEventCallback(&huart3, Reply_Size);
    }
}

/** @brief 提供模拟 DMA 事件。@param Uart 串口。@retval 事件类型 */
extern "C" uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *Uart) { return Uart->event; }

/** @brief 推进模拟时钟，让无应答情况超时。@retval 毫秒 */
extern "C" uint32_t HAL_GetTick(void) { return Tick++; }

/** @brief 验证方向顺序、应答复制、长度错误、超时与 UART 错误。@retval 0 表示通过 */
int main()
{
    const uint8_t Request[] = {0xFF, 0xFF, 6, 2, 1, 0xF6};
    uint8_t Rx[sizeof(Reply)];
    BSP::RS485::Init();
    BSP::UART3::Init();
    Reply_Size = sizeof(Reply);
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_OK);
    assert(Direction == GPIO_PIN_RESET && memcmp(Rx, Reply, sizeof(Rx)) == 0);
    Reply_Size = 3U;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_LENGTH);
    Reply_Size = 0U;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_TIMEOUT);
    Receive_Error = true;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_ERROR);
    Receive_Error = false;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), 0, 0) == FT_SERVO_BUS_OK);
    assert(Direction == GPIO_PIN_RESET && Tx_Count == 5U);
    return 0;
}
