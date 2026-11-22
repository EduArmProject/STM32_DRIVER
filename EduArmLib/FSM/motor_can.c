#include <string.h>
#include "motor_can.h"
#include "fdcan.h"
#include "canfd_protocol.h"

/* Global variable definitions */
uint32_t              motor_add      = 0;
bool                  error_detected = false;
SystemState_t         current_state  = STATE_INIT;
MotorCommand_t        command_received;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t               RxData[64];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t               TxData[8];


/* Read DIP switches to get the motor CAN address */
uint8_t Read_DIP_Switches(void) {
    uint8_t dip1 = HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin);
    uint8_t dip2 = HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin);
    uint8_t dip3 = HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin);

    uint8_t address = (!dip1 << 0) |
                      (!dip2 << 1) |
                      (!dip3 << 2);

    return address;
}

uint8_t Fetch_Motor_Address(void) {
    uint8_t address;
    while ((address = Read_DIP_Switches()) == 0) {
        HAL_Delay(10);
    }
    return address;
}


/* Update status LEDs according to the current system state */
void Update_System_LEDs(SystemState_t state) {
    switch (state) {
        case STATE_INIT:
            break; // Blinking handled by HAL_SYSTICK_Callback

        case STATE_READY:
            // No error : green LED stays ON
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            break;

        case STATE_ERROR:
            // Error : green LED OFF
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}

static uint32_t init_led_timer = 0;

/* SysTick callback : blink the green LED every 250 ms during STATE_INIT */
void HAL_SYSTICK_Callback(void) {
    if (current_state == STATE_INIT) {
        init_led_timer++;
        if (init_led_timer >= 250) {
            init_led_timer = 0;
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }
    }
}

void Set_System_State(SystemState_t new_state) {
    if (current_state != new_state) {
        current_state = new_state;
        Update_System_LEDs(current_state);
    }
}




int main_motor(void)
{

    while (1)
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

                    FDCAN_Config();
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

        }

        // END FSM

    }
}




