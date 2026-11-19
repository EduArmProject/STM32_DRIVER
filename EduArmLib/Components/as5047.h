#ifndef AS5047P_H
#define AS5047P_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "spi_protocol.h"


// link of the datasheet : https://www.snapeda.com/parts/AS5047P-ATSM/ams/datasheet/

/* Configuration on STM32cubemx 
Setup the CS Pin
type GPIO Output
level : High
GPIO mode : Output Push Pull
GPIO Pull-up/ Pull-down : No puul-up and no pull-down
Maximum outpu speed : High
User Label : SPI_CS
*/ 

/* "It communicates at clock rates up to 10 MHz.
The AS5047P SPI uses mode=1 (CPOL=0, CPHA=1) to exchange
data. As shown in Figure 11, a data transfer starts with the
falling edge of CSn (SCL is low). The AS5047P samples MOSI data
on the falling edge of SCL. SPI commands are executed at the
end of the frame (rising edge of CSn). The bit order is MSB first.
Data is protected by parity"
*/ 

// --- Volatile Registers (Figure 18) ---
#define AS5047P_REG_NOP       0x0000 // No operation
#define AS5047P_REG_ERRFL     0x0001 // Error register
#define AS5047P_REG_PROG      0x0003 // Programming register
#define AS5047P_REG_DIAAGC    0x3FFC // Diagnostic and AGC
#define AS5047P_REG_MAG       0x3FFD // CORDIC magnitude
#define AS5047P_REG_ANGLEUNC  0x3FFE // Measured angle without dynamic angle error compensation
#define AS5047P_REG_ANGLECOM  0x3FFF // Measured angle with dynamic angle error compensation

// SPI command Precalculated
// Structure: Bit 15: Parity (Even), Bit 14: R/W (Read=1), Bits 13:0: Address
#define AS5047P_CMD_READ_ANGLE 0xFFFF // Parity(1) + Read(1) + 0x3FFF
#define AS5047P_CMD_READ_DIAG  0xFFFC // Parity(1) + Read(1) + 0x3FFC
#define AS5047P_CMD_READ_NOP   0xC000 // Parity(1) + Read(1) + 0x0000
#define AS5047P_CMD_CLEAR_ERR  0x4001 // Parity(0) + Read(1) + 0x0001


typedef struct {
    SPI_Device_t spi_interface; 
    float        current_angle;
    bool         error_flag;
    uint8_t      agc_value;   // 0 to 255 (Automatic Gain Control)
    bool         mag_too_high;
    bool         mag_too_low;
} AS5047_t;



void AS5047P_Init(AS5047_t *encoder, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);


bool AS5047P_Diagnostic(AS5047_t *encoder);
uint16_t AS5047P_Read_Magnitude(AS5047_t *encoder);
uint16_t AS5047P_Clear_Error(AS5047_t *encoder);


bool AS5047P_Read_Angle(AS5047_t *encoder);
void     AS5047P_Update_MultiTurn(AS5047_t *encoder);

// Homing
void     AS5047P_Set_Zero(AS5047_t *encoder); 
void     AS5047P_Set_Zero_Offset(AS5047_t *encoder, uint16_t raw_offset);


#endif