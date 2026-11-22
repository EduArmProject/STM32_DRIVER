/**
  ******************************************************************************
  * @file    as5047.c
  * @author  Corentin_k_
  * @brief   AS5047P magnetic rotary position sensor module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the AS5047P 14-bit on-axis magnetic sensor:
  *           + Sensor initialization and SPI device binding
  *           + Angle measurement and pre-scaled degree conversion
  *           + Multi-turn tracking and robot arm gear ratio handling
  *           + Magnetic diagnostics (AGC, MAGH, MAGL) and error recovery
  ******************************************************************************
  */

#include "as5047.h"

/**
  * @brief  Internal function to read a 14-bit register from AS5047P via SPI.
  * @note   Implements 2-stage SPI pipeline (command followed by NOP) and error checking.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @param  reg_cmd Precalculated 16-bit command word (parity + read bit + address)
  * @param  data    Pointer to output variable where 14-bit data will be stored
  * @retval true if communication succeeded, false if error flag (bit 14) was set
  */
static bool AS5047_Read_Register(AS5047_t *encoder, uint16_t reg_cmd, uint16_t *data)
{
    /* 1. Send read command */
    SPI_Transmit_16b(&encoder->spi_interface, reg_cmd);

    /* 2. Send NOP command to retrieve previous response */
    uint16_t response = SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_NOP);

    /* 3. Check error flag (Bit 14: EF) */
    if (response & (1U << 14))
    {
        encoder->error_flag = true;
        /* Read error register to clear error flag */
        SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_CLEAR_ERR);
        return false;
    }

    encoder->error_flag = false;

    /* 4. Mask and extract 14-bit payload (Bits 13:0) */
    if (data != NULL)
    {
        *data = response & 0x3FFFU;
    }

    return true;
}

/**
  * @brief  Initialize AS5047P encoder structure and configure default values.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @param  hspi    Pointer to HAL SPI handle
  * @param  cs_port GPIO port of Chip Select pin
  * @param  cs_pin  GPIO pin of Chip Select
  * @retval None
  */
void AS5047_Init(AS5047_t *encoder, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    /* Bind SPI bus interface and Chip Select pin */
    encoder->spi_interface.hspi    = hspi;
    encoder->spi_interface.cs_port = cs_port;
    encoder->spi_interface.cs_pin  = cs_pin;

    /* Initialize motor shaft angle variables */
    encoder->raw_angle         = 0U;
    encoder->zero_raw_angle    = 0;
    encoder->angle_deg         = 0.0f;

    /* Initialize multi-turn tracking */
    encoder->revolution_count  = 0;
    encoder->last_raw_angle    = 0;
    encoder->total_angle_deg   = 0.0f;

    /* Initialize gearbox reduction (default 1:30) and precalculate inverse */
    encoder->gear_ratio        = 30.0f;
    encoder->inv_gear_ratio    = 1.0f / 30.0f;
    encoder->arm_angle_deg     = 0.0f;

    /* Initialize diagnostic fields */
    encoder->magnitude         = 0U;
    encoder->agc_value         = 0U;
    encoder->mag_too_high      = false;
    encoder->mag_too_low       = false;
    encoder->error_flag        = false;
    encoder->error             = 0U;
    encoder->multiturn_initialized = false;

    /* Set Chip Select pin HIGH (inactive state) */
    HAL_GPIO_WritePin(
        encoder->spi_interface.cs_port,
        encoder->spi_interface.cs_pin,
        GPIO_PIN_SET
    );
}

/**
  * @brief  Perform diagnostic check on magnet positioning and AGC value.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval true if magnetic field is within optimal operating range, false otherwise
  */
bool AS5047_Diagnostic(AS5047_t *encoder)
{
    uint16_t diag_data = 0U;
    if (!AS5047_Read_Register(encoder, AS5047P_CMD_READ_DIAG, &diag_data))
    {
        return false;
    }

    /* Bit 11: MAGL (Magnetic field too low) */
    encoder->mag_too_low  = (diag_data & (1U << 11)) ? true : false;

    /* Bit 10: MAGH (Magnetic field too high) */
    encoder->mag_too_high = (diag_data & (1U << 10)) ? true : false;

    /* Bits 7:0 : AGC value (Automatic Gain Control) */
    encoder->agc_value    = (uint8_t)(diag_data & 0x00FFU);

    /* Return false if any magnetic field anomaly is detected */
    if (encoder->mag_too_low || encoder->mag_too_high)
    {
        return false;
    }
    return true;
}

/**
  * @brief  Read CORDIC magnitude of magnetic field strength from sensor.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Read_Magnitude(AS5047_t *encoder)
{
    uint16_t mag = 0U;
    if (AS5047_Read_Register(encoder, AS5047P_CMD_READ_MAG, &mag))
    {
        encoder->magnitude = mag;
    }
}

/**
  * @brief  Fetch and clear error register (ERRFL) from AS5047P.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Fetch_Error(AS5047_t *encoder)
{
    uint16_t err = 0U;
    if (AS5047_Read_Register(encoder, AS5047P_CMD_CLEAR_ERR, &err))
    {
        encoder->error = err;
    }
}

/**
  * @brief  Read instant absolute single-turn angle (0° to 360°) with homing offset applied.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval true if measurement was successful, false if error flag was raised
  */
bool AS5047_Read_Angle(AS5047_t *encoder)
{
    uint16_t raw = 0U;
    if (!AS5047_Read_Register(encoder, AS5047P_CMD_READ_ANGLE, &raw))
    {
        return false;
    }

    encoder->raw_angle = raw;

    /* Calculate relative motor angle from mechanical zero reference point */
    int32_t diff = (int32_t)raw - (int32_t)encoder->zero_raw_angle;
    if (diff < 0)
    {
        diff += 16384;
    }

    /* Direct multiplication by precalculated constant instead of division */
    encoder->angle_deg = (float)diff * AS5047_RAW_TO_DEG;
    return true;
}

/**
  * @brief  Update multi-turn continuous angle tracking and compute arm joint angle.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Update_MultiTurn(AS5047_t *encoder)
{
    if (!AS5047_Read_Angle(encoder))
    {
        return;
    }

    if (!encoder->multiturn_initialized)
    {
        encoder->last_raw_angle = encoder->raw_angle;
        encoder->revolution_count = 0;
        encoder->multiturn_initialized = true;
    }

    /* Detect full-turn rollover between consecutive measurements (half-turn threshold: 8192) */
    int32_t delta = (int32_t)encoder->raw_angle - encoder->last_raw_angle;

    if (delta < -8192)
    {
        /* Positive clockwise rotation overflow: 16383 -> 0 */
        encoder->revolution_count++;
    }
    else if (delta > 8192)
    {
        /* Negative counter-clockwise rotation underflow: 0 -> 16383 */
        encoder->revolution_count--;
    }

    encoder->last_raw_angle = encoder->raw_angle;

    /* Calculate continuous cumulative motor shaft angle */
    encoder->total_angle_deg = ((float)encoder->revolution_count * 360.0f) + encoder->angle_deg;

    /* Calculate actual robotic arm joint angle using precalculated inverse gear ratio */
    encoder->arm_angle_deg = encoder->total_angle_deg * encoder->inv_gear_ratio;
}

/**
  * @brief  Configure the reduction ratio between motor shaft and arm joint.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @param  ratio   Reduction gear ratio (e.g., 30.0f for a 1:30 gearbox)
  * @retval None
  */
void AS5047_Set_Gear_Ratio(AS5047_t *encoder, float ratio)
{
    if (ratio > 0.0f)
    {
        encoder->gear_ratio     = ratio;
        encoder->inv_gear_ratio = 1.0f / ratio; /* Single precalculation */
        encoder->arm_angle_deg  = encoder->total_angle_deg * encoder->inv_gear_ratio;
    }
}

/**
  * @brief  Set current position as mechanical zero reference point (homing).
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval None
  */
void AS5047_Set_Zero(AS5047_t *encoder)
{
    encoder->zero_raw_angle   = encoder->raw_angle;
    encoder->revolution_count = 0;
    encoder->last_raw_angle   = encoder->raw_angle;
    encoder->multiturn_initialized = true;

    encoder->angle_deg        = 0.0f;
    encoder->total_angle_deg  = 0.0f;
    encoder->arm_angle_deg    = 0.0f;
}
