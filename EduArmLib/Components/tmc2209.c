#include "tmc2209.h"


/* CRC8 calculation as specified in the TMC2209 datasheet (section 4.2)
 * CRC = (CRC << 1) OR (CRC.7 XOR CRC.1 XOR CRC.0 XOR [new incoming bit])
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

/* Build and send a write datagram to the TMC2209 via UART (half-duplex).
 * Datagram format (8 bytes):
 *   [0] Sync byte  = 0x05
 *   [1] Node addr
 *   [2] Reg | 0x80 (write flag)
 *   [3..6] Data MSB first
 *   [7] CRC8 over bytes [0..6]
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

/* Send a request to read TCM2209 register */
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

/* Received the reply for the request sent by TMC2209_ReadRegister_Request */
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

/* Check the value of a register using tow functions :
 *  -TMC2209_ReadRegister_Request
 *  -TMC2209_ReadRegister_Reply
 * Save the value of the register in value[4]non
 */
void TMC2209_ReadRegister(TMC2209_t *driver, uint8_t reg, uint8_t value[4]) {

    TMC2209_ReadRegister_Request(driver, reg);

    TMC2209_ReadRegister_Reply(driver, value);
}

bool TMC2209_Check_UART(TMC2209_t *driver) {
    uint8_t value[4];
    TMC2209_ReadRegister(driver, TMC2209_REG_ION, value);

    if (value[0] == 0x21) // Version must be equal to 0x21, so we can check uart
        return true;
    return false;
}

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
    driver->step_timer   = step_timer;
    driver->step_channel = step_channel;

    driver->is_enabled       = false;
    driver->error_flag       = false;
    driver->overtemp_warning = false;

    // Keep motor disabled at startup (EN active low, pulled high = disabled)
    HAL_GPIO_WritePin(driver->enn_port, driver->enn_pin, GPIO_PIN_SET);

    TMC2209_Configure(driver, false, true, false, 16, 8);

    tmc2209_driver = driver;
}


void TMC2209_Enable(TMC2209_t *driver) {
    HAL_GPIO_WritePin(driver->enn_port, driver->enn_pin, GPIO_PIN_RESET);
    driver->is_enabled = true;
}

void TMC2209_Disable(TMC2209_t *driver) {
    if (driver->step_timer != NULL) {
        HAL_TIM_PWM_Stop(driver->step_timer, driver->step_channel);
    }
    HAL_GPIO_WritePin(driver->enn_port, driver->enn_pin, GPIO_PIN_SET);
    driver->is_enabled = false;
}

void TMC2209_Stop(TMC2209_t *driver) {
    HAL_TIM_PWM_Stop(driver->step_timer, driver->step_channel);
}

/* Set direction for the motor
- 1 forward
- -1 backward
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

void TMC2209_Set_Step(TMC2209_t *driver, uint32_t steps) {

    if (steps == NULL)
        return;

    if (driver->is_enabled == false)
        TMC2209_Enable(driver);

    if (driver->step_remaining > 0)
        return;

    driver->step_remaining = steps;

    // Interuption to every pulse completed
    HAL_TIM_PWM_Start_IT(driver->step_timer, driver->step_channel);
}

/* Callback in non blocking modes - Interrupt */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    TMC2209_t *driver = tmc2209_driver;

    driver->step_remaining--;

    if (driver->step_remaining == 0) {
        HAL_TIM_PWM_Stop_IT(htim, driver->step_channel);
    }
}

void TMC2209_Set_Speed(TMC2209_t *driver, uint32_t frequence_impulsion) {
    
}

bool TMC2209_Is_Mooving(TMC2209_t *driver) {
    if (driver->step_remaining > 0)
        return true;
    return false;
}

