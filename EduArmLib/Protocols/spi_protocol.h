/**
  ******************************************************************************
  * @file    spi_protocol.h
  * @author  Corentin_k_
  * @brief   Header file of SPI protocol abstraction layer.
  *          This file provides definitions, structures, and prototypes for
  *          communicating with SPI devices using software Chip Select (CS):
  *           + SPI device structure definition
  *           + 16-bit synchronous transmission/reception functions
  ******************************************************************************
  */

#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>
#include "main.h"

/* STM32CubeMX SPI1 Configuration Summary:
 * - Mode: Full-Duplex Master
 * - Data Size: 16 Bits
 * - First Bit: MSB First
 * - Prescaler: 32 (Target ~5 Mbit/s)
 * - Clock Polarity (CPOL): Low (CPOL = 0)
 * - Clock Phase (CPHA): 2 Edge (CPHA = 1) -> Mode 1 for AS5047P
 * - CRC Calculation: Disabled
 * - NSS Signal Type: Software
 */

/**
  * @brief SPI Device handle structure
  */
typedef struct {
    SPI_HandleTypeDef* hspi;     /*!< Pointer to HAL SPI handle */
    GPIO_TypeDef*      cs_port;  /*!< GPIO port for Chip Select (CS) pin */
    uint16_t           cs_pin;   /*!< GPIO pin for Chip Select (CS) */
} SPI_Device_t;

/**
  * @brief  Transmit and receive one 16-bit word over SPI with software CS assertion.
  * @param  device  Pointer to target SPI device configuration structure
  * @param  tx_data 16-bit word to transmit to the slave device
  * @retval 16-bit word received simultaneously from the slave device
  */
uint16_t SPI_Transmit_16b(SPI_Device_t *device, uint16_t tx_data);

#endif /* SPI_PROTOCOL_H */
