/**
  * @file    motor.c
  * @brief   Driver L298N cho 1 dong co DC
  */
#include "motor.h"
#include "tim.h"
#include "gpio.h"

/* Doi chan / them motor thu 2 chi can sua o day */
#define MOTOR_PWM_TIM      htim1
#define MOTOR_PWM_CH       TIM_CHANNEL_1
#define MOTOR_IN1_PORT     GPIOE
#define MOTOR_IN1_PIN      GPIO_PIN_7
#define MOTOR_IN2_PORT     GPIOE
#define MOTOR_IN2_PIN      GPIO_PIN_8

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
  /* TIM1 la advanced timer: HAL_TIM_PWM_Start tu bat MOE */
  HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_PWM_CH);
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
  /* TIM1 nam tren APB2: khi APB2 prescaler != 1 thi timer clock = PCLK2 * 2 */
  uint32_t pclk2   = HAL_RCC_GetPCLK2Freq();
  uint32_t tim_clk = ((RCC->CFGR & RCC_CFGR_PPRE2) == 0U) ? pclk2 : pclk2 * 2U;
  uint32_t psc     = MOTOR_PWM_TIM.Instance->PSC + 1U;
  uint32_t arr     = MOTOR_PWM_TIM.Instance->ARR + 1U;

  return tim_clk / (psc * arr);
}
