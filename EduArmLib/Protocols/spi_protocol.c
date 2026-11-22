/**
  ******************************************************************************
  * @file    spi_protocol.c
  * @author  Corentin_k_
  * @brief   SPI protocol abstraction module driver.
  *          This file provides firmware functions to manage 16-bit SPI
  *          transfers with software Chip Select (CS) assertion:
  *           + Automatic CS line pulling (active LOW)
  *           + Full-duplex synchronous 16-bit data exchange
  *           + Error handling and timeout control
  ******************************************************************************
  */

#include "spi_protocol.h"
#include "main.h"

/**
  * @brief  Transmit and receive one 16-bit word over SPI with software CS assertion.
  * @note   CS is automatically asserted (LOW) before transfer and released (HIGH) after.
  * @param  device  Pointer to target SPI device configuration structure
  * @param  tx_data 16-bit word to transmit to the slave device
  * @retval 16-bit word received simultaneously from the slave device
  */
uint16_t SPI_Transmit_16b(SPI_Device_t *device, uint16_t tx_data)
{
    uint16_t rx_data = 0U;

    /* Assert slave chip select (CS active LOW) */
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);

    /* Send and receive 1 packet of 16 bits (DataSize = 16 bits in CubeMX) */
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        device->hspi,
        (uint8_t *)&tx_data,
        (uint8_t *)&rx_data,
        1U,  /* 1 word of 16 bits */
        10U  /* Timeout in milliseconds */
    );

    /* De-assert slave chip select (CS inactive HIGH) */
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        Error_Handler();
    }

    return rx_data;
}
