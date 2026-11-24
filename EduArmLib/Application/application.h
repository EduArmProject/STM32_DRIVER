#ifndef APPLICATION_H
#define APPLICATION_H

#include "main.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_tim.h"

typedef enum {
  APP_INIT_OK = 0,
  APP_INIT_ADDRESS_ERROR,
  APP_INIT_CAN_ERROR,
  APP_INIT_ENCODER_ERROR,
  APP_INIT_MOTOR_ERROR,
  APP_INIT_PID_ERROR
} AppInitStatus_t;

AppInitStatus_t Application_Start(SPI_HandleTypeDef *hspi,
                                  UART_HandleTypeDef *huart,
                                  TIM_HandleTypeDef *step_timer);

void Application_Loop(void);

#endif
