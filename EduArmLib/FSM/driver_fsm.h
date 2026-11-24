#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

/**
  * @file    motor_can.h
  * @brief   Public interface for EduArm motor CAN commands and system states.
  */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Opcodes for each CAN command */
/** @brief CAN command opcodes. */
typedef enum {
    CMD_STOP  = 0x00,  // Stop the motor
    CMD_STEPS = 0x01,  // Move x steps
    CMD_SPEED = 0x02   // Set the speed
} CommandOpcode_t;

/* Motor command received over CAN (8 bytes, packed) */
/** @brief Motor command payload received over CAN. */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;     /**< Command type. */
    uint8_t  direction;  /**< 0 = clockwise, 1 = counter-clockwise. */
    uint32_t steps;      /**< Number of steps. */
    uint16_t speed;      /**< Motor speed. */
} MotorCommand_t;

/* System state machine states */
/** @brief System state machine states. */
typedef enum {
    DRIVER_FIRST_INIT,
    DRIVER_INIT_COMM,
    DRIVER_INIT_ENCODER,
    DRIVER_INIT_MOTOR,
    DRIVER_INIT_PID,
    DRIVER_READY,
    DRIVER_MOVING,
    DRIVER_HOMING,
    DRIVER_ERROR,
    DRIVER_SHUTDOWN
} DriverState_t;

/* Global variables exposed to other modules */
/** @brief DIP switch motor address. */
extern uint32_t               motor_add;
/** @brief Most recently received motor command. */
extern MotorCommand_t         command_received;
/** @brief Current FSM state. */
extern SystemState_t          current_state;
/** @brief FDCAN transmit header. */
extern FDCAN_TxHeaderTypeDef  TxHeader;
/** @brief FDCAN transmit payload. */
extern uint8_t                TxData[8];
/** @brief FDCAN receive header. */
extern FDCAN_RxHeaderTypeDef  RxHeader;
/** @brief FDCAN receive payload. */
extern uint8_t                RxData[64];

/* Public functions */
/**
  * @brief  Reads the DIP switches until a valid motor address is available.
  * @retval Motor address selected by the DIP switches.
  */
uint8_t Fetch_Motor_Address(void);

/**
  * @brief  Changes the system state and updates the LEDs.
  * @param  new_state New system state.
  * @retval None.
  */
void    Set_System_State(SystemState_t new_state);

/**
  * @brief  Updates the status LEDs for the specified system state.
  * @param  state System state used to determine LED output.
  * @retval None.
  */
void    Update_System_LEDs(SystemState_t state);

#endif /* MOTOR_CAN_H */
