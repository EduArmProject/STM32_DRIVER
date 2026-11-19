#include "spi_protocol.h"
#include "main.h"

uint16_t SPI_Transmit_16b(SPI_Device_t *device, uint16_t tx_data) {
    uint16_t rx_data = 0;

    // Activate slave (CS active low)
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);

    // Send and receive 1 packet of 16 bits
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        device->hspi,
        (uint8_t*)&tx_data,
        (uint8_t*)&rx_data,
        1,   // 1 packet of 16 bits (DataSize = 16 bits in CubeMX)
        10   // timeout 10 ms
    );

    // Deactivate slave
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        Error_Handler();
    }

    return rx_data;
}
