/**
  * @file    speed_ctrl.c
  * @brief   Dieu khien toc do vong kin cho dong co GA25-370 qua L298N
  *
  * Cau truc: duty = feedforward(toc_do_dat) + PID(sai_lech)
  *
  * Feedforward lay tu bang do thuc te tren dong co that (PWM 1 kHz, khong tai,
  * nguon 12V). Nho no ma PID chi phai bu phan sai lech nho, thay vi tu do tim
  * tu 0 moi lan doi lenh - dap ung nhanh hon va it vot lo hon han.
  *
  * Bang do duoc (xem docs, duty %  ->  RPM truc ra):
  *   30 -> 96.5   35 -> 128.3   40 -> 153.6   45 -> 174.9
  *   50 -> 191.6  60 -> 210.3   100 -> 278.6
  * Duoi 30 % dong co khong quay (vung chet do ma sat tinh).
  */
#include "speed_ctrl.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

/*
 * He so PID. Don vi: % duty tren moi RPM sai lech.
 *
 * Do tu dong bang bai thu 0 -> 120 -> 240 -> 120 RPM, khong tai, PWM 1 kHz:
 *
 *   Kp/Ki/Kd          vot lo 0->120   hut 240->120   on dinh TB   sai so TB
 *   0.12/0.50/0.01        11.3%           5.4%         2.60s        1.6
 *   0.20/0.50/0.01         9.1%           3.2%         2.35s        1.3
 *   0.20/0.90/0.01        14.7%          15.4%         1.63s        0.1   <- dang dung
 *   0.20/0.90/0.03        18.0%          15.4%         1.57s        0.2
 *   0.30/0.90/0.01        10.2%          10.9%         2.02s        0.4
 *
 * Chon 0.20/0.90/0.01: sai so xac lap gan nhu bang 0 va on dinh nhanh nhat.
 * Doi lai vot lo 15% trong khoang nua giay, chap nhan duoc voi AGV.
 * Muon chay em hon thi dung 0.20/0.50/0.01, vot lo con 9% nhung cham hon 0.7s.
 *
 * Kd gan nhu khong tac dung vi feedforward da lo phan lon; tang len 0.03 chi
 * lam vot lo xau di.
 */
#define SPD_KP   0.20f
#define SPD_KI   0.90f   /* % duty / (RPM * giay) */
#define SPD_KD   0.01f   /* % duty * giay / RPM   */

/* Bang feedforward, RPM tang dan */
static const float ff_rpm[]  = {  96.5f, 128.3f, 153.6f, 174.9f, 191.6f, 210.3f, 278.6f };
static const float ff_duty[] = {  30.0f,  35.0f,  40.0f,  45.0f,  50.0f,  60.0f, 100.0f };
#define FF_POINTS  (sizeof(ff_rpm) / sizeof(ff_rpm[0]))

static PID_t   pid;
static int16_t target_rpm = 0;
static int16_t out_duty   = 0;

/* Noi suy tuyen tinh tung doan; ngoai bang thi giu gia tri dau/cuoi */
static float feedforward(float rpm)
{
  if (rpm <= ff_rpm[0])              return ff_duty[0];
  if (rpm >= ff_rpm[FF_POINTS - 1])  return ff_duty[FF_POINTS - 1];

  for (unsigned i = 1; i < FF_POINTS; i++)
  {
    if (rpm <= ff_rpm[i])
    {
      float t = (rpm - ff_rpm[i - 1]) / (ff_rpm[i] - ff_rpm[i - 1]);
      return ff_duty[i - 1] + t * (ff_duty[i] - ff_duty[i - 1]);
    }
  }
  return ff_duty[FF_POINTS - 1];
}

void SpeedCtrl_Init(void)
{
  /* PID chi bu sai lech con lai nen gioi han hep hon toan dai duty */
  PID_Init(&pid, SPD_KP, SPD_KI, SPD_KD, -40.0f, 40.0f);
  target_rpm = 0;
  out_duty   = 0;
}

void SpeedCtrl_SetTarget(int16_t rpm)
{
  if (rpm > 0)
  {
    if (rpm < SPD_RPM_MIN) rpm = SPD_RPM_MIN;
    if (rpm > SPD_RPM_MAX) rpm = SPD_RPM_MAX;
  }
  else if (rpm < 0)
  {
    if (rpm > -SPD_RPM_MIN) rpm = -SPD_RPM_MIN;
    if (rpm < -SPD_RPM_MAX) rpm = -SPD_RPM_MAX;
  }

  /* Doi chieu hoac dung han: xoa trang thai cu de khong mang tich phan sang */
  if ((rpm == 0) || ((target_rpm < 0) != (rpm < 0)))
    PID_Reset(&pid);

  target_rpm = rpm;
}

void SpeedCtrl_Update(uint32_t dt_ms)
{
  if (target_rpm == 0)
  {
    Motor_Coast();
    out_duty = 0;
    PID_Reset(&pid);
    return;
  }

  float dt      = (float)dt_ms / 1000.0f;
  float meas    = (float)Encoder_GetRpmX10() / 10.0f;
  float sp      = (float)target_rpm;

  /* Tinh toan theo tri tuyet doi roi gan lai dau, de bang feedforward
     dung chung cho ca hai chieu quay */
  float sign    = (sp < 0.0f) ? -1.0f : 1.0f;
  float sp_abs  = sp * sign;
  float meas_abs = meas * sign;

  float ff  = feedforward(sp_abs);
  float fb  = PID_Update(&pid, sp_abs, meas_abs, dt);
  float duty = ff + fb;

  if (duty > 100.0f) duty = 100.0f;
  if (duty < 0.0f)   duty = 0.0f;

  out_duty = (int16_t)(duty * sign);
  Motor_SetSpeed(out_duty);
}

void SpeedCtrl_SetGains(float kp, float ki, float kd)
{
  PID_SetTunings(&pid, kp, ki, kd);
}

void SpeedCtrl_GetGains(float *kp, float *ki, float *kd)
{
  if (kp) *kp = pid.kp;
  if (ki) *ki = pid.ki;
  if (kd) *kd = pid.kd;
}

int16_t SpeedCtrl_GetTarget(void) { return target_rpm; }
int16_t SpeedCtrl_GetDuty(void)   { return out_duty; }
