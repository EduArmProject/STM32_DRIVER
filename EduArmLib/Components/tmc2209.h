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
#define TMC2209_REG_ION        0x06 // Reads the state of all input pins available, we can check if the uart works
#define TMC2209_REG_IHOLD_IRUN 0x10 // Driver current control (IHOLD / IRUN / IHOLDDELAY)
#define TMC2209_REG_TPOWERDOWN 0x11 // Powerdown delay (reset default = 20)
#define TMC2209_REG_CHOPCONF   0x6C // Chopper and driver configuration

#define GEAR_REDUCER 1


typedef struct {
    UART_Device_t      uart_interface;
    uint8_t            node_address; 
    GPIO_TypeDef*      enn_port;
    uint16_t           enn_pin;
    GPIO_TypeDef*      dir_port;
    uint16_t           dir_pin;
    TIM_HandleTypeDef* step_timer;
    uint32_t           step_channel;
    volatile uint32_t  step_remaining; // can change at any time with the interupt, so the variable is volatile
    bool               is_enabled;
    uint8_t            run_current;  
    uint8_t            hold_current;
    bool               error_flag;
    bool               overtemp_warning;
} TMC2209_t;

static TMC2209_t *tmc2209_driver = NULL;


void TMC2209_Init(TMC2209_t *driver, UART_HandleTypeDef *huart, uint8_t address, 
                  GPIO_TypeDef *enn_port, uint16_t enn_pin,
                  GPIO_TypeDef *dir_port, uint16_t dir_pin,
                  TIM_HandleTypeDef *step_timer, uint32_t step_channel);

bool TMC2209_Configure(
    TMC2209_t *driver,
    bool use_vref_analog_scale,
    bool activate_silent_mode,
    bool inverse_motor_direction,
    uint8_t run_current,  // 0...31 (0=1/32 ... 31=32/32)
    uint8_t hold_current  // 0...31 (0=1/32 ... 31=32/32)
);

bool TMC2209_Check_UART(TMC2209_t *driver);
bool TMC2209_Is_Mooving(TMC2209_t *driver);

// R&W with uart com
void TMC2209_WriteRegister(TMC2209_t *driver, uint8_t reg, uint32_t value);
void TMC2209_ReadRegister_Request(TMC2209_t *driver, uint8_t reg);
void TMC2209_ReadRegister_Reply(TMC2209_t *driver, uint8_t value[4]);
void TMC2209_ReadRegister(TMC2209_t *driver, uint8_t reg, uint8_t value[4]);

void TMC2209_Enable(TMC2209_t *driver);
void TMC2209_Disable(TMC2209_t *driver);
void TMC2209_Stop(TMC2209_t *driver);

void TMC2209_Set_Direction(TMC2209_t *driver, int8_t dir);
void TMC2209_Set_Step(TMC2209_t *driver, uint32_t steps);

#endif // TMC2209_H
