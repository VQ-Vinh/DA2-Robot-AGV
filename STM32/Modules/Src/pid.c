/**
  * @file    pid.c
  * @brief   Bo dieu khien PID roi rac
  */
#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->out_min = out_min;
  pid->out_max = out_max;
  PID_Reset(pid);
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PID_Reset(PID_t *pid)
{
  pid->integral  = 0.0f;
  pid->prev_meas = 0.0f;
  pid->first_run = 1;
}

float PID_Update(PID_t *pid, float setpoint, float meas, float dt_s)
{
  if (dt_s <= 0.0f) return 0.0f;

  float err = setpoint - meas;

  /* Lan dau chua co gia tri truoc do -> coi nhu dao ham bang 0 */
  if (pid->first_run)
  {
    pid->prev_meas = meas;
    pid->first_run = 0;
  }

  float p_term = pid->kp * err;
  float d_term = -pid->kd * (meas - pid->prev_meas) / dt_s;
  pid->prev_meas = meas;

  pid->integral += pid->ki * err * dt_s;

  float out = p_term + pid->integral + d_term;

  /* Back-calculation: bao hoa bao nhieu thi tru nguoc vao tich phan bay nhieu */
  if (out > pid->out_max)
  {
    pid->integral -= (out - pid->out_max);
    out = pid->out_max;
  }
  else if (out < pid->out_min)
  {
    pid->integral -= (out - pid->out_min);
    out = pid->out_min;
  }

  return out;
}
