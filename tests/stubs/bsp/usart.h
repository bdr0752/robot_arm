#pragma once
#include <stdint.h>

// UART 与 GPIO 一起用于 RS485 测试，真实工程由 HAL 头文件声明。
typedef struct { unsigned unused; } GPIO_TypeDef;
extern GPIO_TypeDef Test_GpioB;
#define GPIOB (&Test_GpioB)
#define GPIO_PIN_14 (1U << 14)
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;

typedef struct { unsigned unused; } DMA_HandleTypeDef;
typedef int HAL_StatusTypeDef;
typedef struct {
    DMA_HandleTypeDef *hdmarx;
    uint8_t *rx_buffer;
    uint16_t rx_capacity;
    uint32_t event;
    uint32_t gState;
    uint32_t RxState;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart1, huart3;
extern uint32_t Test_Primask;

#define HAL_UART_RXEVENT_HT 0U
#define HAL_UART_RXEVENT_TC 1U
#define HAL_UART_RXEVENT_IDLE 2U
#define DMA_IT_HT 1U
#define HAL_OK 0
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define HAL_UART_STATE_BUSY_RX 2U
#define HAL_ERROR 1
#define __HAL_DMA_DISABLE_IT(handle, interrupt) ((void)(handle), (void)(interrupt))

static inline void __DMB(void) {}
static inline uint32_t __get_PRIMASK(void) { return Test_Primask; }
static inline void __disable_irq(void) { Test_Primask = 1U; }
static inline void __set_PRIMASK(uint32_t mask) { Test_Primask = mask; }

extern "C" {
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
}
