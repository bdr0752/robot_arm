#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include <stdint.h>

typedef enum {
    BSP_BOARD_OK = 0,
    BSP_BOARD_TIMEOUT = -1,
    BSP_BOARD_ERROR = -2
} BspBoardResult;

/* Call after CubeMX GPIO, DMA and USART3 initialization. */
void Bsp_Board_Init(void);

/* Nonzero selects transmit; zero selects receive. */
void Bsp_Board_Rs485_Set_Tx(uint8_t enabled);

BspBoardResult Bsp_Board_UART_Transmit(
    uint8_t *data, uint16_t length, uint32_t timeout_ms
);
BspBoardResult Bsp_Board_UART_Receive(
    uint8_t *data, uint16_t length, uint32_t timeout_ms
);

/* Transmit GRB bytes on the board's WS2812 data pin. */
void bsp_board_ws2812_send_grb(const uint8_t *data, uint16_t length);

#endif
