/**
  ******************************************************************************
  * @file    tmc2209.h
  * @author  Corenin_K / Samuel_D
  * @brief   This file contains all the functions prototypes for TMC2209
  ******************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ******************************************************************************
  */

#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "uart_protocol.h" // Importe l'interface UART

/*
GPIO EN

GPIO output level      : High
GPIO mode              : Output Push Pull
GPIO Pull-up/Pull-down : No pull-up and no pull-down
Maximum output speed   : Low

Add a 10k pull-up resistor on the PCB to 3.3V

GPIO DIR

GPIO output level      : Low
GPIO mode              : Output Push Pull
GPIO Pull-up/Pull-down : No pull-up and no pull-down
Maximum output speed   : Low


GPIO STEP
[Timers -> TIM1 -> Mode]
Clock Source           : Internal Clock
Channel2               : PWM Generation CH2

[Timers -> TIM1 -> Parameter Settings -> Counter Settings]
Prescaler (PSC - 16 bits value)                  : 160 - 1    // Divide 160MHz to 1MHz
Counter Mode                                     : Up
Counter Period (AutoReload Register - 16 bits... : 500 - 1    // Target 2kHz (1 000 000 / 2000)
Internal Clock Division (CKD)                    : No Division
Repetition Counter (RCR - 16 bits value)         : 0
auto-reload preload                              : Enable     // CRITICAL: For smooth speed changes

[Timers -> TIM1 -> Parameter Settings -> PWM Generation Channel 2]
Mode                                             : PWM mode 1
Pulse (16 bits value)                            : 250        // 50% Duty cycle (Half of period)
Output compare preload                           : Enable
Fast Mode                                        : Disable
CH Polarity                                      : High

*/


// Register map
#define TMC2209_REG_GCONF      0x00 // Global configuration
#define TMC2209_REG_IOIN       0x06 // Reads the state of all input pins and the version
#define TMC2209_REG_IHOLD_IRUN 0x10 // Driver current control (IHOLD / IRUN / IHOLDDELAY)
#define TMC2209_REG_TPOWERDOWN 0x11 // Powerdown delay (reset default = 20)
#define TMC2209_REG_CHOPCONF   0x6C // Chopper and driver configuration

#define GEAR_REDUCER 30
#define TMC2209_TIMER_TICK_HZ   1000000
#define TMC2209_STEP_FREQ_MIN   16  
#define TMC2209_STEP_FREQ_MAX   30000

/* Enum values are also the repeated blue LED pulse counts. */
typedef enum {
    TMC2209_ERROR_NONE = 0,
    TMC2209_ERROR_TX_GCONF = 1,
    TMC2209_ERROR_TX_CHOPCONF = 2,
    TMC2209_ERROR_TX_IHOLD_IRUN = 3,
    TMC2209_ERROR_TX_TPOWERDOWN = 4,
    TMC2209_ERROR_TX_READ_GCONF = 5,
    TMC2209_ERROR_RX_READ_GCONF = 6,
    TMC2209_ERROR_GCONF_MISMATCH = 7,
    TMC2209_ERROR_TX_READ_CHOPCONF = 8,
    TMC2209_ERROR_RX_READ_CHOPCONF = 9,
    TMC2209_ERROR_CHOPCONF_MISMATCH = 10,
    TMC2209_ERROR_TX_READ_IOIN = 11,
    TMC2209_ERROR_RX_READ_IOIN = 12,
    TMC2209_ERROR_IOIN_VERSION = 13,
    TMC2209_ERROR_TX_READ_REGISTER = 14,
    TMC2209_ERROR_RX_READ_REGISTER = 15
} TMC2209_ErrorCode_t;


/**
  * @brief TMC2209 driver instance and hardware configuration.
  */
typedef struct {
    UART_Device_t      uart_interface; /**< UART communication interface. */
    uint8_t            node_address;   /**< TMC2209 UART node address. */
    GPIO_TypeDef*      enn_port;       /**< Enable GPIO port. */
    uint16_t           enn_pin;        /**< Enable GPIO pin. */
    GPIO_TypeDef*      dir_port;       /**< Direction GPIO port. */
    uint16_t           dir_pin;        /**< Direction GPIO pin. */
    TIM_HandleTypeDef* step_timer;     /**< Timer used to generate steps. */
    uint32_t           step_channel;   /**< Timer PWM channel used for steps. */
    uint32_t           step_freq;      /**< Step frequency in Hz. */
    volatile uint32_t  step_remaining; /**< Remaining steps; updated by interrupts. */
    bool               is_enabled;     /**< Driver enable state. */
    uint8_t            run_current;    /**< Run current setting (0 to 31). */
    uint8_t            hold_current;   /**< Hold current setting (0 to 31). */
    uint32_t           debug_ioin_value;     /**< Last IOIN value read over UART. */
    uint32_t           debug_gconf_value;    /**< GCONF value read back after configuration. */
    uint32_t           debug_chopconf_value; /**< CHOPCONF value read back after configuration. */
    bool               debug_ioin_valid;     /**< Last IOIN frame passed header and CRC checks. */
    bool               debug_gconf_valid;    /**< GCONF readback matched the requested value. */
    bool               debug_chopconf_valid; /**< CHOPCONF readback matched the requested value. */
    TMC2209_ErrorCode_t debug_error_code;    /**< Last TMC2209 initialization or UART error. */
    bool               error_flag;     /**< General driver error flag. */
    bool               overtemp_warning; /**< Overtemperature warning flag. */
} TMC2209_t;

static TMC2209_t *tmc2209_driver = NULL;

/**
  * @brief  Initializes a TMC2209 driver instance.
  * @param  driver TMC2209 driver instance.
  * @param  huart UART handle used for TMC2209 communication.
  * @param  enn_port Enable GPIO port.
  * @param  enn_pin Enable GPIO pin.
  * @param  dir_port Direction GPIO port.
  * @param  dir_pin Direction GPIO pin.
  * @param  step_timer Timer handle used to generate motor steps.
  * @param  step_channel Timer PWM channel used to generate motor steps.
  * @retval true Initialization and configuration succeeded.
  * @retval false Invalid address or UART transmission failed.
  */
bool TMC2209_Init(TMC2209_t *driver, UART_HandleTypeDef *huart,
                  GPIO_TypeDef *enn_port, uint16_t enn_pin,
                  GPIO_TypeDef *dir_port, uint16_t dir_pin,
                  TIM_HandleTypeDef *step_timer, uint32_t step_channel);

/**
  * @brief  Configures the TMC2209 registers and motor currents.
  * @param  driver TMC2209 driver instance.
  * @param  use_vref_analog_scale Enables analog current scaling when true.
  * @param  activate_silent_mode Enables silent mode when true.
  * @param  inverse_motor_direction Inverts the motor direction when true.
  * @param  run_current Run current setting from 0 to 31.
  * @param  hold_current Hold current setting from 0 to 31.
  * @retval true Configuration completed.
  * @retval false Configuration failed.
  */
bool TMC2209_Configure(
    TMC2209_t *driver,
    bool use_vref_analog_scale,
    bool activate_silent_mode,
    bool inverse_motor_direction,
    uint8_t run_current,
    uint8_t hold_current
);

/**
  * @brief  Checks the TMC2209 UART connection.
  * @param  driver TMC2209 driver instance.
  * @retval true UART communication is available.
  * @retval false UART communication failed.
  */
bool TMC2209_Check_UART(TMC2209_t *driver);

/**
  * @brief  Checks whether the motor is moving.
  * @param  driver TMC2209 driver instance.
  * @retval true At least one step remains.
  * @retval false The motor is stopped.
  */
bool TMC2209_Is_Mooving(TMC2209_t *driver);

/**
  * @brief  Writes a 32-bit value to a TMC2209 register.
  * @param  driver TMC2209 driver instance.
  * @param  reg Register address.
  * @param  value Register value.
  * @retval true Transmission succeeded.
  * @retval false Transmission failed.
  */
bool TMC2209_WriteRegister(TMC2209_t *driver, uint8_t reg, uint32_t value);

/**
  * @brief  Sends a register read request over UART.
  * @param  driver TMC2209 driver instance.
  * @param  reg Register address.
  * @retval true Transmission succeeded.
  * @retval false Transmission failed.
  */
bool TMC2209_ReadRegister_Request(TMC2209_t *driver, uint8_t reg);

/**
  * @brief  Receives a register reply and extracts its 32-bit value.
  * @param  driver TMC2209 driver instance.
  * @param  reg Register address expected in the reply.
  * @param  value Buffer receiving four bytes of register data.
  * @retval true Reply header and CRC are valid.
  * @retval false Reception failed or reply is invalid.
  */
bool TMC2209_ReadRegister_Reply(TMC2209_t *driver, uint8_t reg, uint8_t value[4]);

/**
  * @brief  Requests and receives a TMC2209 register value.
  * @param  driver TMC2209 driver instance.
  * @param  reg Register address.
  * @param  value Buffer receiving four bytes of register data.
  * @retval true Read succeeded.
  * @retval false Request or reply failed.
  */
bool TMC2209_ReadRegister(TMC2209_t *driver, uint8_t reg, uint8_t value[4]);

/**
  * @brief  Enables the motor driver.
  * @param  driver TMC2209 driver instance.
  * @retval None.
  */
void TMC2209_Enable(TMC2209_t *driver);

/**
  * @brief  Disables the motor driver and stops PWM step generation.
  * @param  driver TMC2209 driver instance.
  * @retval None.
  */
void TMC2209_Disable(TMC2209_t *driver);

/**
  * @brief  Stops motor step generation.
  * @param  driver TMC2209 driver instance.
  * @retval None.
  */
void TMC2209_Stop(TMC2209_t *driver);

/**
  * @brief  Sets the motor direction.
  * @param  driver TMC2209 driver instance.
  * @param  dir Direction: 1 for forward, -1 for backward, 0 for no change.
  * @retval None.
  */
void TMC2209_Set_Direction(TMC2209_t *driver, int8_t dir);

/**
  * @brief  Sets the motor step frequency.
  * @param  driver TMC2209 driver instance.
  * @param  freq_hz Step frequency in Hz.
  * @retval None.
  */
void TMC2209_Set_Speed(TMC2209_t *driver, uint32_t freq_hz);

/**
  * @brief  Starts a movement for a given number of steps.
  * @param  driver TMC2209 driver instance.
  * @param  steps Number of steps to generate.
  * @retval None.
  */
void TMC2209_Set_Step(TMC2209_t *driver, uint32_t steps);

#endif // TMC2209_H
