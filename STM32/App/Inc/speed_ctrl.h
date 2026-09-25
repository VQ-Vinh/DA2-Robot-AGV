/**
  * @file    speed_ctrl.h
  * @brief   Dieu khien toc do vong kin: feedforward tu bang do duoc + PID bu sai lech
  */
#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include <stdint.h>

/* Gioi han dat toc do: duoi 96 RPM dong co khong thang noi ma sat tinh (da do) */
#define SPD_RPM_MIN   100
#define SPD_RPM_MAX   270

void    SpeedCtrl_Init(void);
void    SpeedCtrl_SetTarget(int16_t rpm);   /* 0 = tha troi; dau = chieu quay */
void    SpeedCtrl_Update(uint32_t dt_ms);   /* goi moi chu ky, sau Encoder_Update */
int16_t SpeedCtrl_GetTarget(void);
int16_t SpeedCtrl_GetDuty(void);            /* duty dang xuat ra, -100..100 */
void    SpeedCtrl_SetGains(float kp, float ki, float kd);
void    SpeedCtrl_GetGains(float *kp, float *ki, float *kd);

#endif /* SPEED_CTRL_H */
