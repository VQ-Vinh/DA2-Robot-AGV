/**
  * @file    app.h
  * @brief   Tang ung dung chay tren FreeRTOS
  *
  *   main.c      : App_Init()      - khoi tao module, truoc khi scheduler chay
  *   freertos.c  : App_RtosInit()  - tao queue, trong MX_FREERTOS_Init()
  *                 App_ControlTask / App_LogTask - than cua controlTask / logTask
  */
#ifndef APP_H
#define APP_H

void App_Init(void);
void App_RtosInit(void);
void App_ControlTask(void *argument);  /* chu ky 50 ms: encoder + doi duty */
void App_LogTask(void *argument);      /* nhan mau tu queue va in UART */

#endif /* APP_H */
