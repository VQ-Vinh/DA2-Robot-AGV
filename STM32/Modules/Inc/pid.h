/**
  * @file    pid.h
  * @brief   Bo dieu khien PID roi rac, dung chung cho moi vong dieu khien
  *
  * Dac diem:
  *   - Dao ham tinh tren gia tri DO (derivative on measurement), khong tinh tren
  *     sai lech, nen doi setpoint dot ngot khong gay xung dao ham.
  *   - Chong bao hoa tich phan bang back-calculation: phan vuot qua gioi han
  *     duoc tru nguoc lai vao khau tich phan.
  *
  * Day la module tinh toan thuan tuy, khong dung HAL, khong dung phan cung.
  */
#ifndef PID_H
#define PID_H

typedef struct
{
  float kp;         /* don vi: dau_ra / don_vi_sai_lech        */
  float ki;         /* don vi: dau_ra / (don_vi_sai_lech * s)  */
  float kd;         /* don vi: dau_ra * s / don_vi_sai_lech    */
  float out_min;
  float out_max;

  float integral;   /* trang thai noi bo */
  float prev_meas;
  int   first_run;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
void  PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
float PID_Update(PID_t *pid, float setpoint, float meas, float dt_s);
void  PID_Reset(PID_t *pid);

#endif /* PID_H */
