/**
  ******************************************************************************
  * @file    as5047.h
  * @author  Corentin_k_
  * @brief   Header file of AS5047P magnetic rotary position sensor driver.
  *          This file provides registers, structures, and function prototypes
  *          to interface with the AMS AS5047P 14-bit on-axis magnetic sensor:
  *           + Sensor initialization and hardware binding
  *           + Angle measurement and pre-scaled degree conversion
  *           + Multi-turn tracking and robot arm gear ratio handling
  *           + Magnetic diagnostics (AGC, MAGH, MAGL) and error recovery
  ******************************************************************************
  */

#ifndef AS5047_H
#define AS5047_H

#include <stdint.h>
#include <stdbool.h>
#include "spi_protocol.h"

// Reference datasheet: https://www.snapeda.com/parts/AS5047P-ATSM/ams/datasheet/
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

/* --- Volatile Registers (Datasheet Figure 18) --- */
#define AS5047P_REG_NOP        0x0000U /*!< No operation */
#define AS5047P_REG_ERRFL      0x0001U /*!< Error register */
#define AS5047P_REG_PROG       0x0003U /*!< Programming register */
#define AS5047P_REG_DIAAGC     0x3FFCU /*!< Diagnostic and AGC */
#define AS5047P_REG_MAG        0x3FFDU /*!< CORDIC magnitude */
#define AS5047P_REG_ANGLEUNC   0x3FFEU /*!< Measured angle without dynamic angle error compensation */
#define AS5047P_REG_ANGLECOM   0x3FFFU /*!< Measured angle with dynamic angle error compensation */

/* --- SPI Commands Precalculated (Bit 15: Parity Even, Bit 14: Read=1) --- */
#define AS5047P_CMD_READ_ANGLE    0xFFFFU /*!< Parity(1) + Read(1) + 0x3FFF (ANGLECOM) */
#define AS5047P_CMD_READ_ANGLEUNC 0x7FFEU /*!< Parity(0) + Read(1) + 0x3FFE (ANGLEUNC) */
#define AS5047P_CMD_READ_DIAG     0xFFFCU /*!< Parity(1) + Read(1) + 0x3FFC (DIAAGC) */
#define AS5047P_CMD_READ_MAG      0x7FFDU /*!< Parity(0) + Read(1) + 0x3FFD (MAG) */
#define AS5047P_CMD_READ_NOP      0xC000U /*!< Parity(1) + Read(1) + 0x0000 (NOP) */
#define AS5047P_CMD_CLEAR_ERR     0x4001U /*!< Parity(0) + Read(1) + 0x0001 (ERRFL) */


/**
  * @brief Precalculated factor to convert raw 14-bit count to degrees (360.0f / 16384.0f)
  */
#define AS5047_RAW_TO_DEG   0.02197265625f

/**
  * @brief AS5047P Sensor Handle Structure
  */
typedef struct {
    SPI_Device_t spi_interface;     /*!< Underlying SPI device interface */

    /* --- Motor Shaft Measurements (before reduction) --- */
    uint16_t  raw_angle;            /*!< Raw 14-bit sensor measurement (0 - 16383) */
    int16_t   zero_raw_angle;       /*!< Raw reference point recorded at homing */
    float     angle_deg;            /*!< Relative motor shaft angle (0.0° - 360.0°) */

    /* --- Multi-turn Tracking --- */
    int32_t   revolution_count;     /*!< Total completed motor revolutions (+ / -) */
    int32_t   last_raw_angle;       /*!< Previous raw angle for rollover detection */
    float     total_angle_deg;      /*!< Continuous cumulative motor shaft angle */

    /* --- Gearbox and Arm Joint Output --- */
    float     gear_ratio;           /*!< Reduction ratio (e.g., 30.0f for 1:30 reduction) */
    float     inv_gear_ratio;       /*!< Precalculated inverse (1.0f / gear_ratio) to avoid divisions */
    float     arm_angle_deg;        /*!< Actual joint angle of the robotic arm after gearbox */

    /* --- Diagnostic and State --- */
    uint16_t  magnitude;            /*!< CORDIC magnitude (magnetic field strength) */
    uint8_t   agc_value;            /*!< Automatic Gain Control value (0 - 255) */
    bool      mag_too_high;         /*!< True if magnet is too close (MAGH bit) */
    bool      mag_too_low;          /*!< True if magnet is too far (MAGL bit) */
    bool      cordic_overflow;      /*!< True if CORDIC overflow occurred (COF bit) */
    bool      offset_comp_ok;       /*!< True if internal offset compensation is ready (LF bit) */
    bool      error_flag;           /*!< True if SPI frame transmission error detected */
    uint16_t  error;                /*!< Detailed error register content (ERRFL bits 2:0) */
    uint16_t  last_rx_frame;        /*!< Raw 16-bit SPI word from last register read */
    bool      multiturn_initialized; /*!< True after the first valid angle read */
} AS5047_t;

/**
  * @brief  Initialize AS5047P encoder structure and configure default values.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @param  hspi    Pointer to HAL SPI handle
  * @param  cs_port GPIO port of Chip Select pin
  * @param  cs_pin  GPIO pin of Chip Select
  * @retval None
  */
void AS5047_Init(AS5047_t *encoder, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
  * @brief  Perform diagnostic check on magnet positioning and AGC value.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval true if magnetic field is within optimal operating range, false otherwise
  */
bool AS5047_Diagnostic(AS5047_t *encoder);

/**
  * @brief  Read CORDIC magnitude of magnetic field strength from sensor.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Read_Magnitude(AS5047_t *encoder);

/**
  * @brief  Fetch and clear error register (ERRFL) from AS5047P.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Fetch_Error(AS5047_t *encoder);

/**
  * @brief  Read instant absolute single-turn angle (0° to 360°) with homing offset applied.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval true if measurement was successful, false if error flag was raised
  */
bool AS5047_Read_Angle(AS5047_t *encoder);

/**
  * @brief  Update multi-turn continuous angle tracking and compute arm joint angle.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Update_MultiTurn(AS5047_t *encoder);

/**
  * @brief  Configure the reduction ratio between motor shaft and arm joint.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @param  ratio   Reduction gear ratio (e.g., 30.0f for a 1:30 gearbox)
  * @retval None
  */
void AS5047_Set_Gear_Ratio(AS5047_t *encoder, float ratio);

/**
  * @brief  Set current position as mechanical zero reference point (homing).
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Set_Zero(AS5047_t *encoder);

#endif /* AS5047_H */
