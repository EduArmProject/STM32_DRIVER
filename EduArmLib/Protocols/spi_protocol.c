/**
  ******************************************************************************
  * @file    spi_protocol.c
  * @author  Corentin_k_
  * @brief   SPI protocol abstraction module driver.
  *          This file provides firmware functions to manage 16-bit SPI
  *          transfers with software Chip Select (CS) assertion:
  *           + Automatic CS line pulling (active LOW)
  *           + Hardware-accurate nanosecond delays for AS5047P timing specs
  *           + Full-duplex synchronous 16-bit data exchange
  *           + Non-blocking error status tracking
  ******************************************************************************
  */

#include "spi_protocol.h"
#include <stddef.h>
#include "main.h"

/* Last HAL status from SPI transfer (inspect with debugger or in drivers) */
volatile HAL_StatusTypeDef spi_last_status = HAL_OK;

/**
  * @brief  Wait for at least @p ns nanoseconds using Cortex-M4 DWT cycle counter.
  * @note   Compiler-optimization-independent (works identically in -O0 and -Os).
  *         Falls back cleanly via loop bound if CYCCNT cannot increment.
  * @param  ns Minimum delay duration in nanoseconds
  * @retval None
  */
static void SPI_Delay_Ns(uint32_t ns)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        DWT->CYCCNT = 0U;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t mhz = SystemCoreClock / 1000000U;
    if (mhz == 0U)
    {
        mhz = 160U;
    }

    uint32_t cycles = (mhz * ns + 999U) / 1000U;
    if (cycles == 0U)
    {
        cycles = 1U;
    }

    uint32_t start     = DWT->CYCCNT;
    uint32_t max_loops = (cycles * 4U) + 100U;

    while (((DWT->CYCCNT - start) < cycles) && (max_loops > 0U))
    {
        max_loops--;
        __NOP();
    }
}

/**
  * @brief  Transmit and receive one 16-bit word over SPI with software CS assertion.
  * @note   Enforces AS5047P datasheet timing requirements (Figure 11):
  *           - t_L   >= 350 ns (CS falling edge to first SCK edge)  -> 500 ns
  *           - t_H   >= 0.5*t_CLK (last SCK edge to CS rising edge) -> 150 ns
  *           - t_CSn >= 350 ns (CS HIGH between two transmissions)  -> 600 ns
  * @param  device  Pointer to target SPI device configuration structure
  * @param  tx_data 16-bit word to transmit to the slave device
  * @retval 16-bit word received simultaneously from the slave device
  */
uint16_t SPI_Transmit_16b(SPI_Device_t *device, uint16_t tx_data)
{
    uint16_t rx_data = 0U;

    if ((device == NULL) || (device->hspi == NULL))
    {
        spi_last_status = HAL_ERROR;
        return 0U;
    }

    /* Assert slave chip select (CS active LOW) */
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);

    /* Guarantee t_L >= 350 ns before first SCK edge */
    SPI_Delay_Ns(500U);

    /* Send and receive 1 packet of 16 bits (DataSize = 16 bits in CubeMX) */
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        device->hspi,
        (uint8_t *)&tx_data,
        (uint8_t *)&rx_data,
        1U,  /* 1 word of 16 bits */
        10U  /* Timeout in milliseconds */
    );

    /* Guarantee t_H >= 150 ns after last SCK edge before releasing CS */
    SPI_Delay_Ns(150U);

    /* De-assert slave chip select (CS inactive HIGH) */
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

    /* Guarantee t_CSn >= 350 ns before next SPI transfer can start */
    SPI_Delay_Ns(600U);

    spi_last_status = status;
    return rx_data;
}
