/**
  * @file    motor.h
  * @brief   Driver TB6612FNG cho 1 dong co DC (kenh A: PWMA/AIN1/AIN2)
  *
  * Phan cung (cau hinh trong STM32.ioc):
  *   PWMA -> PB14 TIM12_CH1, CCR = 0..100 = duty %
  *   AIN1 -> PB12 GPIO Output (M1_IN1)
  *   AIN2 -> PE8  GPIO Output (M1_IN2)
  *   STBY -> PD9  GPIO Output (MOTOR_STBY), High = driver hoat dong
  */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_DUTY_MAX  100   /* = TIM12 Counter Period + 1 */

void    Motor_Init(void);
void    Motor_SetSpeed(int16_t duty);  /* -100..+100: dau = chieu, tri = duty % */
void    Motor_Brake(void);             /* IN1 = IN2 = High: phanh ngan mach */
void    Motor_Coast(void);             /* IN1 = IN2 = Low: tha troi */
int16_t  Motor_GetSpeed(void);
uint32_t Motor_GetPwmFreqHz(void);  /* tinh tu PSC/ARR that, khong hard-code */

#endif /* MOTOR_H */
