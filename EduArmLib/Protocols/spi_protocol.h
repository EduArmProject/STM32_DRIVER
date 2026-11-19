#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H


#include "stm32g4xx_hal.h"

/* Configuration on STM32cubemx 

SPI Parameter Settings
Data Size 16 Bits
First Bit : MSB First
Prescaker for BaudRate : 32
Automaticly calculate the baud rate and display 5 MBits/s
Clock Polarity (CPOL) : Low (CPOL=0)
Clock Phase (CPHA) : 2 Edge (CPHA=1)
CRC Calculation : Disabled
NSS Signal Type : Software

NVSIC Setting check the add colomn 
*/


typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t      cs_pin;
} SPI_Device_t;


uint16_t SPI_Transmit_16b(SPI_Device_t *device, uint16_t tx_data);


#endif