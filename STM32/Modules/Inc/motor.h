/**
  * @file    motor.h
  * @brief   Driver L298N cho 1 dong co DC (kenh A: ENA/IN1/IN2)
  *
  * Phan cung (cau hinh trong STM32.ioc):
  *   ENA -> PE9  TIM1_CH1, PWM 20 kHz, CCR = 0..100 = duty %
  *   IN1 -> PE7  GPIO Output
  *   IN2 -> PE8  GPIO Output
  */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_DUTY_MAX  100   /* = TIM1 Counter Period + 1 */

void    Motor_Init(void);
void    Motor_SetSpeed(int16_t duty);  /* -100..+100: dau = chieu, tri = duty % */
void    Motor_Brake(void);             /* IN1 = IN2 = High: phanh ngan mach */
void    Motor_Coast(void);             /* ENA = 0: tha troi */
int16_t  Motor_GetSpeed(void);
uint32_t Motor_GetPwmFreqHz(void);  /* tinh tu PSC/ARR that, khong hard-code */

#endif /* MOTOR_H */
