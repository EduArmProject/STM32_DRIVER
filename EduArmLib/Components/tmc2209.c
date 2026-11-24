/**
  ******************************************************************************
  * @file    tmc2209.c
  * @author  Corenin_K / Samuel_D
  * @brief   This file contains all the functions for TMC2209
  ******************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                     ##### How to use TMC2209 #####
  ==============================================================================
    [..]
    It uses basic functions to interact with the TMC2209
    [..]
    The TMC2209 contains two functions categories:
         (+) Common TMC2209 functions to control motor movement (STEP/ DIR)
         (+) Common TMC2209 UART to configure and read registers and settings

  @endverbatim
  ******************************************************************************

  */

#include "tmc2209.h"


/**
  * @brief  Calculates the TMC2209 CRC8 checksum.
  * @param  data Data buffer.
  * @param  len Buffer length.
  * @retval CRC8 checksum.
  */
static uint8_t TMC2209_CRC8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07; // XOR operation
            } else {
                crc <<= 1;
            }
            byte >>= 1;
        }
    }
    return crc;
}

/**
  * @brief  Writes a register in the TMC2209.
  * Build and send a write datagram to the TMC2209 via UART (half-duplex).
  * Datagram format (8 bytes):
  *   [0] Sync byte  = 0x05
  *   [1] Node addr
  *   [2] Reg | 0x80 (write flag)
  *   [3..6] Data MSB first
  *   [7] CRC8 over bytes [0..6]
  * @param  driver TMC2209 driver instance.
  * @param  reg Register address.
  * @param  value Register value.
  * @retval None

  */
void TMC2209_WriteRegister(TMC2209_t *driver, uint8_t reg, uint32_t value) {
    uint8_t datagram[8];
    datagram[0] = 0x05;                       // Sync byte
    datagram[1] = driver->node_address;        // Node address (0 to 3)
    datagram[2] = reg | 0x80;                 // Register address + write bit
    datagram[3] = (value >> 24) & 0xFF;       // MSB first
    datagram[4] = (value >> 16) & 0xFF;
    datagram[5] = (value >>  8) & 0xFF;
    datagram[6] = (value >>  0) & 0xFF;       // LSB
    datagram[7] = TMC2209_CRC8(datagram, 7);  // CRC over first 7 bytes

    UART_HD_Transmit(&driver->uart_interface, datagram, 8);
}

/**
  * @brief  Requests a register read from the TMC2209.
  * @param  driver TMC2209 driver instance.
  * @param  reg Register address.
  * @retval None
  */
void TMC2209_ReadRegister_Request(TMC2209_t *driver, uint8_t reg) {
    uint8_t datagram[4];
    datagram[0] = 0x05;
    datagram[1] = driver->node_address;
    datagram[2] = reg & 0x7F;                 // Register address + read bit
    datagram[3] = TMC2209_CRC8(datagram, 3);

    HAL_StatusTypeDef status = UART_HD_Transmit(&driver->uart_interface, datagram, sizeof(datagram));
    if (status != HAL_OK) {
        return;
    }
}

/**
  * @brief  Receives a TMC2209 register reply.
  * @param  driver TMC2209 driver instance.
  * @param  value Buffer receiving the register value.
  * @retval None
  */
void TMC2209_ReadRegister_Reply(TMC2209_t *driver, uint8_t value[4]) {
    uint8_t reply[8];
    HAL_StatusTypeDef status = UART_HD_Receive(&driver->uart_interface, reply, sizeof(reply), 10);
    if (status != HAL_OK) {
    	return;
    }

    if (reply[7] != TMC2209_CRC8(reply, 7)) {
    	return;
    }

    value[0] = reply[3];
    value[1] = reply[4];
    value[2] = reply[5];
    value[3] = reply[6];
}


/**
  * @brief  Reads a register from the TMC2209.
  * @param  driver TMC2209 driver instance.
  * @param  reg Register address.
  * @param  value Buffer receiving the register value.
  * @retval None
  */
void TMC2209_ReadRegister(TMC2209_t *driver, uint8_t reg, uint8_t value[4]) {

    TMC2209_ReadRegister_Request(driver, reg);

    TMC2209_ReadRegister_Reply(driver, value);
}

/**
  * @brief  Checks the TMC2209 UART connection.
  * @param  driver TMC2209 driver instance.
  * @retval true Connection is available.
  * @retval false Connection failed.
  */
bool TMC2209_Check_UART(TMC2209_t *driver) {
    uint8_t value[4];
    TMC2209_ReadRegister(driver, TMC2209_REG_ION, value);

    if (value[0] == 0x21) // Version must be equal to 0x21, so we can check uart
        return true;
    return false;
}

/**
  * @brief  Configures the TMC2209 driver.
  * @param  driver TMC2209 driver instance.
  * @param  use_vref_analog_scale Selects the analog current scale.
  * @param  activate_silent_mode Enables silent mode.
  * @param  inverse_motor_direction Inverts motor direction.
  * @param  run_current Run current setting.
  * @param  hold_current Hold current setting.
  * @retval true Configuration succeeded.
  * @retval false Configuration failed.
  */
bool TMC2209_Configure(
    TMC2209_t *driver,
    bool use_vref_analog_scale,
    bool activate_silent_mode,
    bool inverse_motor_direction,
    uint8_t run_current,  // 0...31 (0=1/32 ... 31=32/32)
    uint8_t hold_current  // 0...31 (0=1/32 ... 31=32/32)
) {
    // --- Global configuration (GCONF) ---
    uint32_t gconf_value = 0;

    // Bit 0: 1 = Use VREF as current reference
    gconf_value |= use_vref_analog_scale ? (1 << 0) : 0;

    // Bit 2: 0 = StealthChop (silent), 1 = SpreadCycle (high dynamics)
    gconf_value |= activate_silent_mode ? 0 : (1 << 2);

    // Bit 3: 1 = Inverse motor direction
    gconf_value |= inverse_motor_direction ? (1 << 3) : 0;

    // Bit 6: 1 = PDN_UART disabled (required when using UART interface)
    gconf_value |= (1 << 6);

    // Bit 7: 1 = Microstep resolution from MRES register (ignores MS1/MS2 pins)
    gconf_value |= (1 << 7);

    // Bit 8: 1 = Software pulse generator optimization enabled
    gconf_value |= (1 << 8);

    TMC2209_WriteRegister(driver, TMC2209_REG_GCONF, gconf_value);
    HAL_Delay(2);


    // --- Chopper and driver configuration (CHOPCONF) ---
    // Reset default = 0x10000053
    uint32_t chopconf_value = 0x10000053;

    // Bit 28: 1 = Interpolate to 256 microsteps for smooth operation
    chopconf_value |= (1 << 28);

    // Bits 27..24: MRES = 4 → 1/16 microstep
    uint8_t mres = 4;
    chopconf_value &= ~((uint32_t)0xF << 24);
    chopconf_value |= ((uint32_t)mres  << 24);

    TMC2209_WriteRegister(driver, TMC2209_REG_CHOPCONF, chopconf_value);
    HAL_Delay(2);


    // --- Driver current control (IHOLD_IRUN) ---
    uint32_t current_value = 0;

    // Bits 4..0: IHOLD — standstill current (0=1/32 … 31=32/32)
    current_value |= (hold_current & 0x1F) << 0;

    // Bits 12..8: IRUN — run current (0=1/32 … 31=32/32)
    // Recommended IRUN >= 16 for best microstep performance
    current_value |= (run_current & 0x1F) << 8;

    // Bits 19..16: IHOLDDELAY — ramp-down delay steps after standstill
    current_value |= (7 << 16); // mid value

    TMC2209_WriteRegister(driver, TMC2209_REG_IHOLD_IRUN, current_value);
    HAL_Delay(2);


    // --- Power-down delay (TPOWERDOWN) ---
    TMC2209_WriteRegister(driver, TMC2209_REG_TPOWERDOWN, 20);
    HAL_Delay(2);

    driver->run_current  = run_current;
    driver->hold_current = hold_current;

    return true;
}

/**
  * @brief  Initializes a TMC2209 driver.
  * @param  driver TMC2209 driver instance.
  * @param  huart UART handle.
  * @param  address Driver UART address.
  * @param  enn_port Enable GPIO port.
  * @param  enn_pin Enable GPIO pin.
  * @param  dir_port Direction GPIO port.
  * @param  dir_pin Direction GPIO pin.
  * @param  step_timer Step timer handle.
  * @param  step_channel Step timer channel.
  * @retval None
  */
void TMC2209_Init(
    TMC2209_t *driver,
    UART_HandleTypeDef *huart,
    uint8_t address,
    GPIO_TypeDef *enn_port,
    uint16_t enn_pin,
    GPIO_TypeDef *dir_port,
    uint16_t dir_pin,
    TIM_HandleTypeDef *step_timer,
    uint32_t step_channel
) {
    driver->uart_interface.huart = huart;
    driver->node_address         = address;

    // EN and Direction GPIOs
    driver->enn_port = enn_port;
    driver->enn_pin  = enn_pin;
    driver->dir_port = dir_port;
    driver->dir_pin  = dir_pin;

    // STEP via PWM timer
    driver->step_timer     = step_timer;
    driver->step_channel   = step_channel;
    driver->step_remaining = 0;

    driver->is_enabled       = false;
    driver->error_flag       = false;
    driver->overtemp_warning = false;

    // Keep motor disabled at startup (EN active low, pulled high = disabled)
    HAL_GPIO_WritePin(driver->enn_port, driver->enn_pin, GPIO_PIN_SET);

    bool status = TMC2209_Configure(driver, false, true, false, 16, 8);
    if (!status)
        return;
        
    tmc2209_driver = driver;
}

/**
  * @brief  Enables the TMC2209 motor driver.
  * @param  driver TMC2209 driver instance.
  * @retval None
  */
void TMC2209_Enable(TMC2209_t *driver) {
    HAL_GPIO_WritePin(driver->enn_port, driver->enn_pin, GPIO_PIN_RESET);
    driver->is_enabled = true;
}

/**
  * @brief  Disables the TMC2209 motor driver.
  * @param  driver TMC2209 driver instance.
  * @retval None
  */
void TMC2209_Disable(TMC2209_t *driver) {
    if (driver->step_timer != NULL) {
        HAL_TIM_PWM_Stop(driver->step_timer, driver->step_channel);
    }
    HAL_GPIO_WritePin(driver->enn_port, driver->enn_pin, GPIO_PIN_SET);
    driver->is_enabled = false;
}

/**
  * @brief  Stops motor step generation.
  * @param  driver TMC2209 driver instance.
  * @retval None
  */
void TMC2209_Stop(TMC2209_t *driver) {
    HAL_TIM_PWM_Stop(driver->step_timer, driver->step_channel);
}
/**
  * @brief  Sets the motor direction.
  * @param  driver TMC2209 driver instance.
  * @param  dir Direction: 1 forward, -1 backward.
  * @retval None
  */
void TMC2209_Set_Direction(TMC2209_t *driver, int8_t dir) {
    if (dir == 1) {
        HAL_GPIO_WritePin(driver->dir_port, driver->dir_pin, GPIO_PIN_SET);
    } else if (dir == -1) {
        HAL_GPIO_WritePin(driver->dir_port, driver->dir_pin, GPIO_PIN_RESET);
    } else {
        return;
    }
}

/**
  * @brief  Starts a step movement.
  * @param  driver TMC2209 driver instance.
  * @param  steps Number of steps.
  * @retval None
  */
void TMC2209_Set_Step(TMC2209_t *driver, uint32_t steps) {

    if (driver->is_enabled == false)
        TMC2209_Enable(driver);

    if (driver->step_remaining > 0)
        return;

    driver->step_remaining = steps;

    // Interuption to every pulse completed
    HAL_TIM_PWM_Start_IT(driver->step_timer, driver->step_channel);
}

/**
  * @brief  Handles a completed PWM pulse.
  * @param  htim Timer handle.
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

    if (tmc2209_driver == NULL)
        return;

    TMC2209_t *driver = tmc2209_driver;

    driver->step_remaining--;

    if (driver->step_remaining == 0) {
        HAL_TIM_PWM_Stop_IT(htim, driver->step_channel);
    }
}

/**
  * @brief  Sets the motor step frequency.
  * @param  driver TMC2209 driver instance.
  * @param  freq_hz Step frequency.
  * @retval None
  */
void TMC2209_Set_Speed(TMC2209_t *driver, uint32_t freq_hz) {
    uint32_t counter_register_value;

    if (freq_hz < TMC2209_STEP_FREQ_MIN) {
        freq_hz = TMC2209_STEP_FREQ_MIN;
    }
    if (freq_hz > TMC2209_STEP_FREQ_MAX) {
        freq_hz = TMC2209_STEP_FREQ_MAX;
    }

    counter_register_value = (TMC2209_TIMER_TICK_HZ / freq_hz) - 1;

    __HAL_TIM_SET_AUTORELOAD(driver->step_timer, counter_register_value);
    __HAL_TIM_SET_COMPARE(driver->step_timer, driver->step_channel, (counter_register_value + 1) / 2);

    if ((driver->step_timer->Instance->CR1 & TIM_CR1_CEN) == 0) {
        driver->step_timer->Instance->EGR = TIM_EGR_UG;
    }

    driver->step_freq = freq_hz;
}

/**
  * @brief  Checks whether the motor is moving.
  * @param  driver TMC2209 driver instance.
  * @retval true Motor is moving.
  * @retval false Motor is stopped.
  */
bool TMC2209_Is_Mooving(TMC2209_t *driver) {
    if (driver->step_remaining > 0)
        return true;
    return false;
}

