#include <string.h>
#include "motor_can.h"
#include "fdcan.h"
#include "canfd_protocol.h"
#include "board_io.h"
/* Global variable definitions */
/** @brief Motor address. */
uint32_t              motor_add      = 0;
/** @brief Error detection flag. */
bool                  error_detected = false;
/** @brief Current system state. */
SystemState_t         current_state  = STATE_INIT;
/** @brief Most recently received motor command. */
MotorCommand_t        command_received;
/** @brief FDCAN receive header. */
FDCAN_RxHeaderTypeDef RxHeader;
/** @brief FDCAN receive payload. */
uint8_t               RxData[64];
/** @brief FDCAN transmit header. */
FDCAN_TxHeaderTypeDef TxHeader;
/** @brief FDCAN transmit payload. */
uint8_t               TxData[8];


/**
  * @brief  Reads the DIP switch address.
  * @retval Motor address.
  */
uint8_t Fetch_Motor_Address(void) {
    uint8_t address;
    while ((address = Read_DIP_Switches()) == 0) {
        HAL_Delay(10);
    }
    return address;
}


/* Update status LEDs according to the current system state */
/**
  * @brief  Updates the status LEDs according to the current system state.
  * @param  state System state used to determine LED output.
  * @retval None.
  */
void Update_System_LEDs(SystemState_t state) {
    switch (state) {
        case STATE_INIT:
            // Blinking handled by HAL_SYSTICK_Callback led at ?? Mhz

        case STATE_READY:
            // No error : green LED stays ON
            //HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            break;

        case STATE_ERROR:
            // Error : green LED OFF
            //HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
            break;

        // Led blue blink during transmission at diffferent frequence depend of receiving or transmit

        default:
            break;
    }
}

static uint32_t init_led_timer = 0;

/* SysTick callback : blink the green LED every 250 ms during STATE_INIT */
/**
  * @brief  Handles the SysTick callback and blinks the green LED during initialization.
  * @retval None.
  */
void HAL_SYSTICK_Callback(void) {
    if (current_state == STATE_INIT) {
        init_led_timer++;
        if (init_led_timer >= 250) {
            init_led_timer = 0;
            //HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }
    }
}

/**
  * @brief  Changes the current system state.
  * @param  new_state New system state.
  * @retval None.
  */
void Set_System_State(SystemState_t new_state) {
    if (current_state != new_state) {
        current_state = new_state;
        Update_System_LEDs(current_state);
    }
}


int Driver_TMC(void){
switch (driver_state) {
        case DRIVER_FIRST_INIT:
            motor_add = Fetch_Motor_Address();
            driver_state = DRIVER_INIT_COMM;
            break;

        case DRIVER_INIT_COMM:
            FDCAN_Init();
            driver_state = DRIVER_INIT_ENCODER;
            break;

        case DRIVER_INIT_ENCODER:
            AS5047_Init(&encoder, ...);
            driver_state = DRIVER_INIT_MOTOR;
            break;

        case DRIVER_INIT_MOTOR:
            TMC2209_Init(&motor, ...);
            driver_state = DRIVER_INIT_PID;
            break;

        case DRIVER_INIT_PID:
            PID_Vitesse_Init(&pid, kp, ki, kd);
            driver_state = DRIVER_READY;
            break;

        case DRIVER_READY:
            // Send message to the host can at add 0
            Handle_CAN_Message();
            break;

        case DRIVER_MOVING:
            AS5047_Update_MultiTurn(&encoder);
            Update_Motor_PID();
            break;

        case DRIVER_HOMING:
            Perform_Homing();
            break;

        case DRIVER_ERROR:
            Handle_Driver_Error();
            break;

        case DRIVER_SHUTDOWN:
            Prepare_Shutdown();
            break;
    }
}
/**
  * @brief  Runs the motor state machine.
  * @retval Execution status.
  */
int TMC_FSM(void)
{

    // START FSM
    switch(current_state) {

        case STATE_FIRST_INIT:

            Set_First_INIT(); // store in other file that the student can't see 
            break;
        case STATE_INIT:
            
            Set_System_State(STATE_INIT);
            Update_System_LEDs(current_state);

            // ---- START INIT ----
            motor_add = Fetch_Motor_Address();

            FDCAN_Init();
            // ---- END INIT ----


            Set_System_State(STATE_READY);
            Update_System_LEDs(current_state);
            break;

        case STATE_READY:
            break;

        case STATE_SHUTDOWN :
            // clear all param function ..
            // can store data in the spi memory if needed
            break;


        // END FSM

    }
}

int AS504_fsM(void){

}

int CAN_FSM(void){

}


