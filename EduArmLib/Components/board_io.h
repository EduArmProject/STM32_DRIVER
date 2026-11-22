/**
  ****************************************************************************
  * @file    board_io.h
  * @author  Samuel_D / Corenin_K
  * @brief   Public interface for board LEDs and DIP switches.
  ****************************************************************************
  * @attention
  *
  * Open-source EduArm
  *
  ****************************************************************************
  */

#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "main.h"

#define LED_BLINK_INTERVAL 300

/**
	* @brief  Turns on the selected LED.
	* @param  gpio_port GPIO port of the LED.
	* @param  gpio_pin GPIO pin of the LED.
	* @retval None.
	*/
void LED_On(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

/**
	* @brief  Turns off the selected LED.
	* @param  gpio_port GPIO port of the LED.
	* @param  gpio_pin GPIO pin of the LED.
	* @retval None.
	*/
void LED_Off(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

/**
	* @brief  Toggles the selected LED for a defined duration.
	* @param  gpio_port GPIO port of the LED.
	* @param  gpio_pin GPIO pin of the LED.
	* @param  duration_ms Duration in milliseconds before restoring the initial state.
	* @retval None.
	*/
void LED_Toggle(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint32_t duration_ms);

/**
	* @brief  Reads the three DIP switches and returns their active-low address.
	* @retval DIP switch address.
	*/
uint8_t Read_DIP_Switches(void);

#endif /* BOARD_IO_H */