#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Opcodes for each CAN command */
typedef enum {
    CMD_STOP  = 0x00,  // Stop the motor
    CMD_STEPS = 0x01,  // Move x steps
    CMD_SPEED = 0x02   // Set the speed
} CommandOpcode_t;

/* Motor command received over CAN (8 bytes, packed) */
typedef struct __attribute__((packed)) {
    uint8_t  opcode;     // 1 byte : command type
    uint8_t  direction;  // 1 byte : 0 = clockwise, 1 = counter-clockwise
    uint32_t steps;      // 4 bytes: number of steps
    uint16_t speed;      // 2 bytes: speed
} MotorCommand_t;

/* System state machine states */
typedef enum {
    STATE_FIRST_INIT,
    STATE_INIT,      // Initializing all drivers
    STATE_READY,     // Waiting for a motor command
    STATE_MOVING,
    STATE_ERROR,
    STATE_DISABLED,
    STATE_HOMMING,
    STATE_SHUTDOWN
} SystemState_t;

/* Global variables exposed to other modules */
extern uint32_t               motor_add;
extern MotorCommand_t         command_received;
extern SystemState_t          current_state;
extern FDCAN_TxHeaderTypeDef  TxHeader;
extern uint8_t                TxData[8];
extern FDCAN_RxHeaderTypeDef  RxHeader;
extern uint8_t                RxData[64];

/* Public functions */
uint8_t Read_DIP_Switches(void);
uint8_t Fetch_Motor_Address(void);
void    Set_System_State(SystemState_t new_state);
void    Update_System_LEDs(SystemState_t state);

#endif /* MOTOR_CAN_H */
