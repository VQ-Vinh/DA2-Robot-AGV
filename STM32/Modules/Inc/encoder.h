/**
  * @file    encoder.h
  * @brief   Doc encoder GA25-370 bang TIM3 Encoder Mode (TI1 & TI2, dem x4)
  *
  * Phan cung (cau hinh trong STM32.ioc):
  *   Encoder A -> PB4  TIM3_CH1
  *   Encoder B -> PB5  TIM3_CH2
  */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/*
 * Hang so quy doi, DO TRUC TIEP tren dong co that (khong lay theo nhan hang).
 *
 *   Quay dia encoder o duoi dong co 10 vong  -> 440 xung   => 44 xung/vong truc motor
 *   Quay banh xe (truc ra) 20 vong           -> 17960 xung => 898 xung/vong truc ra
 *
 * Suy ra ti so hop so that = 898 / 44 = 20.41:1, khong phai 21.3:1 nhu nhan ghi.
 * Doi sang dong co khac phai do lai: ENC_CPR_X10 = (xung dem duoc) * 10 / (so vong quay).
 */
#define ENC_CPR_X10         8980   /* 898.0 xung / vong truc ra */

/* Chi de tham khao, khong dung trong tinh toan */
#define ENC_PPR_X4          44     /* 11 xung/kenh/vong truc motor * 4 (che do TI12) */
#define ENC_GEAR_RATIO_X10  204    /* hop so 20.4:1 (do duoc) */

void    Encoder_Init(void);
void    Encoder_Update(uint32_t dt_ms);  /* goi dinh ky, dt_ms = chu ky goi */
int32_t Encoder_GetCount(void);          /* tong xung tich luy, da xu ly tran 16 bit */
int32_t Encoder_GetRpmX10(void);         /* toc do truc ra, RPM * 10 */
void    Encoder_Reset(void);

#endif /* ENCODER_H */
