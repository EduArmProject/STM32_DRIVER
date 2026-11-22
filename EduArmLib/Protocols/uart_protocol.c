/**
  ******************************************************************************
  * @file    uart_protocol.c
  * @author  Samuel_D / Corentin_K
  * @brief   This file contains all the functions for uart protocols
  ******************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ******************************************************************************
  */


#include "uart_protocol.h"
#include "main.h"

/**
  * @brief  Send data over half-duplex UART.
  * @param  device UART device instance.
  * @param  tx_data Sent buffer.
  * @param  Size Number of bytes.
  * @retval HAL status.
  */
HAL_StatusTypeDef UART_HD_Transmit(UART_Device_t *device, uint8_t *tx_data, uint16_t Size) {
    HAL_HalfDuplex_EnableTransmitter(device->huart);
    HAL_StatusTypeDef status = HAL_UART_Transmit(device->huart, tx_data, Size, 10);
    HAL_HalfDuplex_EnableReceiver(device->huart);
    return status;
}

/**
  * @brief  Receives data over half-duplex UART.
  * @param  device UART device instance.
  * @param  rx_data Reception buffer.
  * @param  Size Number of bytes.
  * @param  Timeout Reception timeout.
  * @retval HAL status.
  */
HAL_StatusTypeDef UART_HD_Receive(UART_Device_t *device, uint8_t *rx_data, uint16_t Size, uint32_t Timeout) {
    HAL_StatusTypeDef status = HAL_UART_Receive(device->huart, rx_data, Size, Timeout);
    return status;
}
