/**
  ******************************************************************************
  * @file    uart_protocol.h
  * @author  Samuel_D / Corentin_K
  * @brief   This file contains all the functions prototypes for uart protocols
  ******************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ******************************************************************************
  */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
/* Configuration on STM32cubemx

"Connectivity" -> USART
- Mode :  Single Wire (Half-Duplex) 
Parameters Settings:
- Baud Rate : 115200 Bits/s
- Word Length : 8 Bits (Including Parity)
- Parity : None
- Stop Bits : 1
*/

/** @brief UART device handle. */
typedef struct {
    UART_HandleTypeDef* huart; 
} UART_Device_t;



// Functions for UART Half-Duplex
/**
  * @brief  Transmits data using half-duplex UART.
  * @param  device UART device instance.
  * @param  tx_data Transmit buffer.
  * @param  Size Number of bytes to transmit.
  * @retval HAL status.
  */
HAL_StatusTypeDef UART_HD_Transmit(UART_Device_t *device, uint8_t *tx_data, uint16_t Size);

/**
  * @brief  Receives data using half-duplex UART.
  * @param  device UART device instance.
  * @param  rx_data Receive buffer.
  * @param  Size Number of bytes to receive.
  * @param  Timeout Receive timeout.
  * @retval HAL status.
  */
HAL_StatusTypeDef UART_HD_Receive(UART_Device_t *device, uint8_t *rx_data, uint16_t Size, uint32_t Timeout);

#endif 