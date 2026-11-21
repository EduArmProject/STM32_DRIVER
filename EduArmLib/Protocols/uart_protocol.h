#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Configuration on STM32cubemx

"Connectivity" -> USART
- Mode :  Single Wire (Half-Duplex) 
Parameters Settings:
- Baud Rate : 115200 Bits/s
- Word Length : 8 Bits (Including Parity)
- Parity : None
- Stop Bits : 1
*/

typedef struct {
    UART_HandleTypeDef* huart; 
} UART_Device_t;



// Functions for UART Half-Duplex
HAL_StatusTypeDef UART_HD_Transmit(UART_Device_t *device, uint8_t *tx_data, uint16_t Size);
HAL_StatusTypeDef UART_HD_Receive(UART_Device_t *device, uint8_t *rx_data, uint16_t Size, uint32_t Timeout);

#endif 