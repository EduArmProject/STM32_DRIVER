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
#include <stddef.h>

/**
  * @brief  Verify even parity over a 16-bit AS5047P SPI frame.
  * @param  frame 16-bit received SPI word
  * @retval true if total number of '1' bits is even, false otherwise
  */
static bool AS5047_Is_Even_Parity(uint16_t frame)
{
    frame ^= (frame >> 8);
    frame ^= (frame >> 4);
    frame ^= (frame >> 2);
    frame ^= (frame >> 1);
    return ((frame & 0x0001U) == 0U);
}

/**
  * @brief  Internal function to read a 14-bit register from AS5047P via SPI.
  * @note   Implements 2-stage SPI pipeline (command followed by NOP), parity
  *         check, HAL status check, and full 2-stage error recovery via ERRFL.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @param  reg_cmd Precalculated 16-bit command word (parity + read bit + address)
  * @param  data    Pointer to output variable where 14-bit data will be stored
  * @retval true if communication succeeded, false on HAL/SPI/parity/EF error
  */
static bool AS5047_Read_Register(AS5047_t *encoder, uint16_t reg_cmd, uint16_t *data)
{
    /* 1. Send read command */
    SPI_Transmit_16b(&encoder->spi_interface, reg_cmd);
    if (spi_last_status != HAL_OK)
    {
        encoder->error_flag = true;
        return false;
    }

    /* 2. Send NOP command to retrieve previous response */
    uint16_t response = SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_NOP);
    encoder->last_rx_frame = response;

    if (spi_last_status != HAL_OK)
    {
        encoder->error_flag = true;
        return false;
    }

    /* 3. Detect disconnected / floating MISO (all ones) */
    if (response == 0xFFFFU)
    {
        encoder->error_flag = true;
        encoder->error      = 0xFFFFU;
        return false;
    }

    /* 4. Check error flag (Bit 14: EF) and even parity (Bit 15) */
    if (((response & (1U << 14)) != 0U) || !AS5047_Is_Even_Parity(response))
    {
        encoder->error_flag = true;

        /* Read ERRFL (0x0001) followed by NOP to retrieve error flags & flush pipeline */
        SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_CLEAR_ERR);
        uint16_t err_resp = SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_NOP);
        encoder->error = err_resp & 0x0007U; /* Bit 0: FRERR, Bit 1: INVCOMM, Bit 2: PARERR */
        return false;
    }

    encoder->error_flag = false;

    /* 5. Mask and extract 14-bit payload (Bits 13:0) */
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
    encoder->cordic_overflow   = false;
    encoder->offset_comp_ok    = false;
    encoder->error_flag        = false;
    encoder->error             = 0U;
    encoder->last_rx_frame     = 0U;
    encoder->multiturn_initialized = false;

    /* Set Chip Select pin HIGH (inactive state) */
    HAL_GPIO_WritePin(
        encoder->spi_interface.cs_port,
        encoder->spi_interface.cs_pin,
        GPIO_PIN_SET
    );

    /* Wait for AS5047P internal power-up sequence to complete.
     * The sensor needs time to initialize its oscillator, load OTP,
     * and complete offset compensation before accepting SPI commands. */
    HAL_Delay(10U);

    /* Flush the SPI pipeline and clear any power-on error flags */
    SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_NOP);
    SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_CLEAR_ERR);
    SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_NOP);
}

/**
  * @brief  Perform diagnostic check on magnet positioning and AGC value.
  * @note   If SPI communication fails (or MISO reads 0x0000), encoder->error_flag
  *         is set true. If SPI succeeded but magnetic field is out of optimal
  *         range (MAGL/MAGH/COF), encoder->error_flag stays false and the
  *         corresponding diagnostic flags indicate the magnetic condition.
  * @param  encoder Pointer to AS5047_t encoder handle structure
  * @retval true if SPI OK and magnetic field is within optimal range, false otherwise
  */
bool AS5047_Diagnostic(AS5047_t *encoder)
{
    uint16_t diag_data = 0U;
    if (!AS5047_Read_Register(encoder, AS5047P_CMD_READ_DIAG, &diag_data))
    {
        return false;
    }

    /* An active AS5047P has LF=1 (0x0100) or non-zero AGC/MAG flags;
     * 0x0000 indicates MISO stuck LOW or sensor unpowered. */
    if (diag_data == 0x0000U)
    {
        encoder->error_flag = true;
        return false;
    }

    /* Bit 11: MAGL (Magnetic field too low) */
    encoder->mag_too_low     = ((diag_data & (1U << 11)) != 0U);

    /* Bit 10: MAGH (Magnetic field too high) */
    encoder->mag_too_high    = ((diag_data & (1U << 10)) != 0U);

    /* Bit 9: COF (CORDIC overflow) */
    encoder->cordic_overflow = ((diag_data & (1U << 9))  != 0U);

    /* Bit 8: LF (Internal offset compensation completed) */
    encoder->offset_comp_ok  = ((diag_data & (1U << 8))  != 0U);

    /* Bits 7:0 : AGC value (Automatic Gain Control) */
    encoder->agc_value       = (uint8_t)(diag_data & 0x00FFU);

    /* Return false if any magnetic field anomaly or CORDIC overflow is detected */
    if (encoder->mag_too_low || encoder->mag_too_high ||
        encoder->cordic_overflow || !encoder->offset_comp_ok)
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
        encoder->error = err & 0x0007U;
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
        encoder->last_raw_angle        = (int32_t)encoder->raw_angle;
        encoder->revolution_count      = 0;
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

    encoder->last_raw_angle = (int32_t)encoder->raw_angle;

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
    encoder->zero_raw_angle        = (int16_t)encoder->raw_angle;
    encoder->revolution_count      = 0;
    encoder->last_raw_angle        = (int32_t)encoder->raw_angle;
    encoder->multiturn_initialized = true;

    encoder->angle_deg             = 0.0f;
    encoder->total_angle_deg       = 0.0f;
    encoder->arm_angle_deg         = 0.0f;
}
