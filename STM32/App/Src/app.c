/**
  * @file    app.c
  * @brief   Demo vong ho GA25-370 + L298N tren FreeRTOS:
  *          controlTask doi duty theo kich ban va doc encoder moi 50 ms,
  *          gui mau qua queue cho logTask in ra UART bang DMA.
  *
  * printf (UART polling) chi dung trong App_Init, truoc khi scheduler chay.
  * Sau do chi logTask ghi UART, qua UartTx_Send (DMA).
  */
#include "app.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"
#include "motor.h"
#include "encoder.h"
#include "speed_ctrl.h"
#include "uart_tx.h"
#include "cli.h"

#define SAMPLE_PERIOD_MS  50    /* chu ky doc encoder */
#define STEP_PERIOD_MS    3000  /* thoi gian giu moi muc toc do dat */
#define STEP_SAMPLES      (STEP_PERIOD_MS / SAMPLE_PERIOD_MS)
#define LOG_QUEUE_LEN     8
#define LOG_LINE_MAX      64    /* byte, du cho dong dai nhat ~39 ky tu */
#define LOG_TX_TIMEOUT_MS 20    /* 64 byte o 115200 mat ~5.6 ms */

/* Kich ban thu vong kin: tang dan, giam, dung, roi dao chieu */
static const int16_t demo_rpm[] = { 0, 120, 180, 240, 180, 120, 0, -150, -220, 0 };
#define DEMO_STEPS  (sizeof(demo_rpm) / sizeof(demo_rpm[0]))

typedef struct
{
  int16_t sp;       /* toc do dat, RPM */
  int16_t duty;     /* duty bo dieu khien xuat ra, % */
  int32_t rpm_x10;  /* toc do do duoc, RPM * 10 */
} Sample_t;

static osMessageQueueId_t log_queue;

void App_Init(void)
{
  Motor_Init();
  Encoder_Init();
  SpeedCtrl_Init();
  printf("\r\n=== GA25-370 + L298N: open-loop test (FreeRTOS) ===\r\n");
  printf("PWM %lu Hz, CPR truc ra = %d.%d\r\n", (unsigned long)Motor_GetPwmFreqHz(), ENC_CPR_X10 / 10, ENC_CPR_X10 % 10);
}

void App_RtosInit(void)
{
  log_queue = osMessageQueueNew(LOG_QUEUE_LEN, sizeof(Sample_t), NULL);
  UartTx_Init();
  Cli_Init();
}

void App_ControlTask(void *argument)
{
  (void)argument;
  uint32_t step    = 0;
  uint32_t samples = 0;
  CliMode_t prev_mode = CLI_MODE_SPEED;
  uint32_t next    = osKernelGetTickCount();

  SpeedCtrl_SetTarget(0);   /* khoi dong o trang thai dung yen */

  for (;;)
  {
    /* DelayUntil: giu chu ky 50 ms co dinh, khong troi theo thoi gian xu ly */
    next += SAMPLE_PERIOD_MS * osKernelGetTickFreq() / 1000U;
    osDelayUntil(next);

    Encoder_Update(SAMPLE_PERIOD_MS);
    Cli_Poll();

    CliMode_t mode = Cli_GetMode();

    /* Vua quay lai che do demo: ap dung ngay muc hien tai cua kich ban */
    if (mode == CLI_MODE_DEMO && prev_mode != CLI_MODE_DEMO)
    {
      samples = 0;
      SpeedCtrl_SetTarget(demo_rpm[step]);
    }
    prev_mode = mode;

    /* Che do duty: nguoi dung dieu khien truc tiep, PID khong duoc chen vao */
    if (mode != CLI_MODE_DUTY)
      SpeedCtrl_Update(SAMPLE_PERIOD_MS);

    /* Lay mau sau khi da cap nhat: sp, rpm va duty cung thuoc mot chu ky.
       Duty lay tu Motor de dung cho ca ba che do. */
    Sample_t s = {
      .sp      = SpeedCtrl_GetTarget(),
      .duty    = Motor_GetSpeed(),
      .rpm_x10 = Encoder_GetRpmX10(),
    };
    /* timeout 0: queue day (UART cham) thi bo mau, khong duoc lam tre vong dieu khien */
    if (Cli_IsLogOn())
      osMessageQueuePut(log_queue, &s, 0, 0);

    /* Doi lenh dat o cuoi chu ky, de mau vua gui van thuoc lenh cu */
    if (mode == CLI_MODE_DEMO && ++samples >= STEP_SAMPLES)
    {
      samples = 0;
      step = (step + 1) % DEMO_STEPS;
      SpeedCtrl_SetTarget(demo_rpm[step]);
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);  /* heartbeat */
  }
}

void App_LogTask(void *argument)
{
  (void)argument;
  Sample_t s;
  static char line[LOG_LINE_MAX];  /* DMA doc tu day, phai song toi khi gui xong */

  for (;;)
  {
    if (osMessageQueueGet(log_queue, &s, NULL, osWaitForever) != osOK)
      continue;

    int n = snprintf(line, sizeof(line), "sp=%4d  rpm=%s%ld.%ld  duty=%4d\r\n",
                     s.sp,
                     (s.rpm_x10 < 0) ? "-" : "",
                     labs(s.rpm_x10) / 10, labs(s.rpm_x10) % 10,
                     s.duty);
    if (n <= 0) continue;
    if (n >= (int)sizeof(line)) n = sizeof(line) - 1;  /* bi cat bot */

    /* Block trong luc DMA gui (~3.3 ms), CPU ranh cho task khac */
    UartTx_Send((const uint8_t *)line, (uint16_t)n, LOG_TX_TIMEOUT_MS);
  }
}
