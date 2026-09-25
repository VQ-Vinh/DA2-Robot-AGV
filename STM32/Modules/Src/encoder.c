/**
  * @file    encoder.c
  * @brief   Doc encoder GA25-370 bang TIM3 Encoder Mode
  */
#include "encoder.h"
#include "tim.h"

#define ENC_TIM  htim3

static uint16_t enc_last_cnt = 0;
static int32_t  enc_total    = 0;
static int32_t  enc_rpm_x10  = 0;

void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&ENC_TIM, TIM_CHANNEL_ALL);
  Encoder_Reset();
}

void Encoder_Update(uint32_t dt_ms)
{
  uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM);
  /* Ep ve int16_t: dung ca khi counter tran 65535 <-> 0 va ca hai chieu quay */
  int16_t delta = (int16_t)(cnt - enc_last_cnt);
  enc_last_cnt = cnt;
  enc_total += delta;

  if (dt_ms == 0) return;
  /* rpm = delta / 937.2 / (dt_ms / 60000); nhan 10 de giu 1 chu so thap phan */
  enc_rpm_x10 = (int32_t)(((int64_t)delta * 60000 * 10 * 10) /
                          ((int64_t)ENC_CPR_X10 * dt_ms));
}

int32_t Encoder_GetCount(void)
{
  return enc_total;
}

int32_t Encoder_GetRpmX10(void)
{
  return enc_rpm_x10;
}

void Encoder_Reset(void)
{
  enc_last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM);
  enc_total    = 0;
  enc_rpm_x10  = 0;
}
