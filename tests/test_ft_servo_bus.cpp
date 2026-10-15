#include "ft_servo_bus.h"
#include "bsp_rs485.hpp"
#include "bsp_dwt.hpp"
#include <assert.h>
#include <string.h>

static DMA_HandleTypeDef Rx_Dma;
UART_HandleTypeDef huart1 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE, HAL_UART_STATE_READY, HAL_UART_STATE_READY};
UART_HandleTypeDef huart3 = {&Rx_Dma, 0, 0, HAL_UART_RXEVENT_IDLE, HAL_UART_STATE_READY, HAL_UART_STATE_READY};
uint32_t Test_Primask;
DWT_Type Test_Dwt;
GPIO_TypeDef Test_GpioB;
static GPIO_PinState Direction;
static uint32_t Tick;
static const uint8_t Reply[] = {0xFF, 0xFF, 6, 2, 0, 0xF7};
static uint16_t Reply_Size;
static bool Receive_Error;
static bool Tx_Done;
static bool Start_Failure;
static unsigned Tx_Count;
static unsigned Rx_Stop_Count;
static bool Rx_Start_Failure;

/** @brief 记录接收 DMA 的地址与容量。@param Uart 串口。@param Data 缓冲区。@param Size 容量。@retval 0 */
extern "C" int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *Uart, uint8_t *Data, uint16_t Size)
{
    if (Rx_Start_Failure) return HAL_ERROR;
    Uart->rx_buffer = Data;
    Uart->rx_capacity = Size;
    Uart->RxState = HAL_UART_STATE_BUSY_RX;
    return 0;
}

/** @brief 模拟停止 DMA。@param Uart 串口。@retval 0 */
extern "C" int HAL_UART_AbortReceive(UART_HandleTypeDef *Uart)
{
    ++Rx_Stop_Count;
    Uart->rx_buffer = 0;
    Uart->RxState = HAL_UART_STATE_READY;
    return 0;
}

/** @brief 启动模拟 DMA，不立即完成发送。@param Uart 串口。@param Data 数据。@param Size 长度。@retval HAL 启动结果 */
extern "C" int HAL_UART_Transmit_DMA(UART_HandleTypeDef *Uart, uint8_t *Data, uint16_t Size)
{
    assert(Uart == &huart3 && Direction == GPIO_PIN_SET);
    assert(Data && Size);
    if (Start_Failure) return -1;
    Uart->gState = HAL_UART_STATE_BUSY_TX;
    Tx_Done = true;
    ++Tx_Count;
    return 0;
}

/** @brief 模拟 UART TC，并由真实 BSP 回调恢复接收方向。@retval 无 */
static void CompleteTransmit()
{
    assert(Direction == GPIO_PIN_SET && BSP::UART3::IsBusy());
    huart3.gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&huart3);
    assert(Direction == GPIO_PIN_RESET && !BSP::UART3::IsBusy());
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
        huart3.RxState = HAL_UART_STATE_READY;
        HAL_UARTEx_RxEventCallback(&huart3, Reply_Size);
    }
}

/** @brief 提供模拟 DMA 事件。@param Uart 串口。@retval 事件类型 */
extern "C" uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *Uart) { return Uart->event; }

/** @brief 推进模拟时钟，让无应答情况超时。@retval 毫秒 */
extern "C" uint32_t HAL_GetTick(void) { if (Tx_Done) CompleteTransmit(); return Tick++; }

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
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_OK);
    // 回调已重启接收；连续请求不应再次停止接收 DMA。
    assert(Rx_Stop_Count == 0U);
    Reply_Size = 3U;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_LENGTH);
    Reply_Size = 0U;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_TIMEOUT);
    Receive_Error = true;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_ERROR);
    Receive_Error = false;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), 0, 0) == FT_SERVO_BUS_OK);
    assert(Direction == GPIO_PIN_SET && BSP::UART3::IsBusy() && Tx_Count == 6U);
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), 0, 0) == FT_SERVO_BUS_ERROR);
    assert(Direction == GPIO_PIN_SET && Tx_Count == 6U);
    CompleteTransmit();
    Start_Failure = true;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), 0, 0) == FT_SERVO_BUS_ERROR);
    assert(Direction == GPIO_PIN_RESET && !BSP::UART3::IsBusy());
    Start_Failure = false;
    assert(BSP::RS485::Transmit(Request, sizeof(Request)));
    // RX 错误不能提前切换仍在发送的总线。
    HAL_UART_ErrorCallback(&huart3);
    assert(Direction == GPIO_PIN_SET && BSP::UART3::IsBusy());
    // 模拟 HAL 结束 DMA TX 后报告错误。
    Tx_Done = false;
    huart3.gState = HAL_UART_STATE_READY;
    HAL_UART_ErrorCallback(&huart3);
    assert(Direction == GPIO_PIN_RESET && !BSP::UART3::IsBusy());
    BSP::UART3::Stop();
    Rx_Start_Failure = true;
    unsigned Previous_Tx_Count = Tx_Count;
    assert(ft_servo_bus_exchange(const_cast<uint8_t *>(Request), sizeof(Request), Rx, sizeof(Rx)) == FT_SERVO_BUS_ERROR);
    assert(Tx_Count == Previous_Tx_Count && Direction == GPIO_PIN_RESET);
    Rx_Start_Failure = false;
    assert(BSP::UART3::Init() == HAL_OK);
    return 0;
}
