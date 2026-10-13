/**
  * @file    motor.c
  * @brief   Driver TB6612FNG cho 1 dong co DC (dong co 1, kenh A cua U4)
  */
#include "motor.h"
#include "tim.h"
#include "gpio.h"

/* Doi chan / them motor thu 2 chi can sua o day */
#define MOTOR_PWM_TIM      htim12
#define MOTOR_PWM_CH       TIM_CHANNEL_1
#define MOTOR_IN1_PORT     M1_IN1_GPIO_Port
#define MOTOR_IN1_PIN      M1_IN1_Pin
#define MOTOR_IN2_PORT     M1_IN2_GPIO_Port
#define MOTOR_IN2_PIN      M1_IN2_Pin

static int16_t motor_duty = 0;

static void motor_set_dir(GPIO_PinState in1, GPIO_PinState in2)
{
  HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, in1);
  HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, in2);
}

static void motor_set_pwm(uint16_t duty)
{
  __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CH, duty);
}

void Motor_Init(void)
{
  motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_RESET);
  motor_set_pwm(0);
  HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_PWM_CH);
  /* TB6612FNG chi lai dong co khi STBY = High (chan nay chung cho ca 2 module) */
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
  motor_duty = 0;
}

void Motor_SetSpeed(int16_t duty)
{
  if (duty > MOTOR_DUTY_MAX)  duty = MOTOR_DUTY_MAX;
  if (duty < -MOTOR_DUTY_MAX) duty = -MOTOR_DUTY_MAX;

  if (duty > 0)
  {
    motor_set_dir(GPIO_PIN_SET, GPIO_PIN_RESET);
    motor_set_pwm((uint16_t)duty);
  }
  else if (duty < 0)
  {
    motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_SET);
    motor_set_pwm((uint16_t)(-duty));
  }
  else
  {
    motor_set_pwm(0);
    motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_RESET);
  }
  motor_duty = duty;
}

void Motor_Brake(void)
{
  motor_set_dir(GPIO_PIN_SET, GPIO_PIN_SET);
  motor_set_pwm(MOTOR_DUTY_MAX);
  motor_duty = 0;
}

void Motor_Coast(void)
{
  motor_set_pwm(0);
  motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_RESET);
  motor_duty = 0;
}

int16_t Motor_GetSpeed(void)
{
  return motor_duty;
}

uint32_t Motor_GetPwmFreqHz(void)
{
  /* TIM12 nam tren APB1: khi APB1 prescaler != 1 thi timer clock = PCLK1 * 2 */
  uint32_t pclk1   = HAL_RCC_GetPCLK1Freq();
  uint32_t tim_clk = ((RCC->CFGR & RCC_CFGR_PPRE1) == 0U) ? pclk1 : pclk1 * 2U;
  uint32_t psc     = MOTOR_PWM_TIM.Instance->PSC + 1U;
  uint32_t arr     = MOTOR_PWM_TIM.Instance->ARR + 1U;

  return tim_clk / (psc * arr);
}
