/**
  ******************************************************************************
  * @file    board_io.c
  * @author  Samuel_D / Coretin_k 
  * @brief   This file contains all the basic functions LED / DIP...
  ******************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ******************************************************************************
  */

#include "board_io.h"

/**
    * @brief  Turns on the selected LED.
    * @param  gpio_port GPIO port of the LED.
    * @param  gpio_pin GPIO pin of the LED.
    * @retval None.
    */
void LED_On(GPIO_TypeDef *gpio_port, uint16_t gpio_pin) {
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
}

/**
    * @brief  Turns off the selected LED.
    * @param  gpio_port GPIO port of the LED.
    * @param  gpio_pin GPIO pin of the LED.
    * @retval None.
    */
void LED_Off(GPIO_TypeDef *gpio_port, uint16_t gpio_pin) {
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
}

/**
    * @brief  Blink the selected LED for a defined duration.
    * @param  gpio_port GPIO port of the LED.
    * @param  gpio_pin GPIO pin of the LED.
    * @param  duration_ms Duration in milliseconds before restoring the initial state.
    * @retval None.
    */
void LED_Blink(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint32_t duration_ms) {
    for (uint8_t i = 0; i < duration_ms; i++) {
        HAL_GPIO_TogglePin(gpio_port, gpio_pin);
        HAL_Delay(LED_BLINK_INTERVAL);

        HAL_GPIO_TogglePin(gpio_port, gpio_pin);
        HAL_Delay(LED_BLINK_INTERVAL);
    }

}

/**
    * @brief  Reads the three DIP switches and returns their active-low address.
    * @retval DIP switch address.
    */
uint8_t Read_DIP_Switches(void) {
    uint8_t dip1 = HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin);
    uint8_t dip2 = HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin);
    uint8_t dip3 = HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin);

    uint8_t address = (!dip1 << 0) |
                      (!dip2 << 1) |
                      (!dip3 << 2);

    return address;
}
