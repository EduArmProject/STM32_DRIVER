#include "application.h"

#include "as5047.h"
#include "board_io.h"
#include "canfd_protocol.h"
#include "pid.h"
#include "tmc2209.h"

uint32_t motor_add;

static AS5047_t encoder;
static TMC2209_t motor;
static PID pid;

static void Test_Move_20_Degrees(void)
{
  const uint32_t full_steps_per_motor_revolution = 200U;
  const uint32_t microsteps_per_full_step = 16U;
  const float arm_degrees = 20.0f;
  const float gear_reduction = 30.0f;

  uint32_t steps = (uint32_t)(
      (arm_degrees * gear_reduction / 360.0f) *
      full_steps_per_motor_revolution *
      microsteps_per_full_step);

  TMC2209_Set_Direction(&motor, 1);
  TMC2209_Set_Speed(&motor, 1000U);
  TMC2209_Set_Step(&motor, steps);

  while (TMC2209_Is_Mooving(&motor))
  {
    AS5047_Update_MultiTurn(&encoder);
  }

  TMC2209_Disable(&motor);
}

AppInitStatus_t Application_Start(SPI_HandleTypeDef *hspi,
                                  UART_HandleTypeDef *huart,
                                  TIM_HandleTypeDef *step_timer)
{
  uint32_t timeout = HAL_GetTick() + 1000U;

  motor_add = Read_DIP_Switches();
  while ((motor_add == 0U) && ((int32_t)(timeout - HAL_GetTick()) > 0))
  {
    HAL_Delay(10U);
    motor_add = Read_DIP_Switches();
  }

  if (motor_add == 0U)
  {
    return APP_INIT_ADDRESS_ERROR;
  }

  FDCAN_Init();

  AS5047_Init(&encoder, hspi, SPI_CS_GPIO_Port, SPI_CS_Pin);
  if (!AS5047_Diagnostic(&encoder))
  {
    return APP_INIT_ENCODER_ERROR;
  }

  TMC2209_Init(&motor,
               huart,
               (uint8_t)motor_add,
               MOTOR_EN_GPIO_Port,
               MOTOR_EN_Pin,
               MOTOR_DIR_GPIO_Port,
               MOTOR_DIR_Pin,
               step_timer,
               TIM_CHANNEL_2);

  if (!TMC2209_Check_UART(&motor))
  {
    return APP_INIT_MOTOR_ERROR;
  }

  PID_Vitesse_Init(&pid, 1.0f, 0.0f, 0.0f);

  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  Test_Move_20_Degrees();

  return APP_INIT_OK;
}

void Application_Loop(void)
{
  AS5047_Update_MultiTurn(&encoder);
}
