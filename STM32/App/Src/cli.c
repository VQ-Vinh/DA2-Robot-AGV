/**
  * @file    cli.c
  * @brief   Nhan lenh tu PC qua UART de dat duty, dat toc do va chinh he so PID
  */
#include "cli.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart_rx.h"
#include "uart_tx.h"
#include "motor.h"
#include "speed_ctrl.h"

#define CLI_REPLY_MAX     96
#define CLI_TX_TIMEOUT_MS 50

/* Mac dinh: dung yen, cho lenh. Kich ban demo chi chay khi go 'demo'. */
static CliMode_t mode   = CLI_MODE_SPEED;
static int       log_on = 1;

/* In so thuc dang x.xxx ma khong can printf float (do khong bat -u _printf_float).
 * Tra ve phan nguyen va phan le rieng de dua vao %d.%03d. */
static void split_float(float v, int *int_part, int *frac_part)
{
  int milli  = (int)(v * 1000.0f + (v < 0 ? -0.5f : 0.5f));
  if (milli < 0) milli = -milli;
  *int_part  = (v < 0 ? -1 : 1) * (milli / 1000);
  *frac_part = milli % 1000;
}

static void reply(const char *text)
{
  UartTx_Send((const uint8_t *)text, (uint16_t)strlen(text), CLI_TX_TIMEOUT_MS);
}

static void reply_state(void)
{
  static char buf[CLI_REPLY_MAX];
  float kp, ki, kd;
  int ip, fp, ii, fi, id, fd;

  SpeedCtrl_GetGains(&kp, &ki, &kd);
  split_float(kp, &ip, &fp);
  split_float(ki, &ii, &fi);
  split_float(kd, &id, &fd);

  const char *mode_name = (mode == CLI_MODE_DEMO)  ? "demo"
                        : (mode == CLI_MODE_SPEED) ? "speed"
                                                   : "duty";

  int n = snprintf(buf, sizeof(buf),
                   "# mode=%s sp=%d duty=%d kp=%d.%03d ki=%d.%03d kd=%d.%03d log=%d\r\n",
                   mode_name, SpeedCtrl_GetTarget(), Motor_GetSpeed(),
                   ip, fp, ii, fi, id, fd, log_on);
  if (n > 0) UartTx_Send((const uint8_t *)buf, (uint16_t)n, CLI_TX_TIMEOUT_MS);
}

/* Tach lenh va tham so. Tra ve con tro toi tham so, hoac NULL neu khong co. */
static char *split_arg(char *line)
{
  char *p = line;
  while (*p && *p != ' ' && *p != '\t') p++;
  if (*p == 0) return NULL;
  *p++ = 0;
  while (*p == ' ' || *p == '\t') p++;
  return (*p == 0) ? NULL : p;
}

static void set_gain(const char *which, const char *arg)
{
  if (arg == NULL) { reply("# thieu tham so\r\n"); return; }

  float kp, ki, kd;
  SpeedCtrl_GetGains(&kp, &ki, &kd);
  float v = strtof(arg, NULL);

  if      (which[1] == 'p') kp = v;
  else if (which[1] == 'i') ki = v;
  else                      kd = v;

  SpeedCtrl_SetGains(kp, ki, kd);
  reply_state();
}

static void handle(char *line)
{
  char *arg = split_arg(line);

  /* Lenh viet hoa hay thuong deu duoc */
  for (char *p = line; *p; p++)
    if (*p >= 'A' && *p <= 'Z') *p += 32;

  if (strcmp(line, "s") == 0)
  {
    if (arg == NULL) { reply("# thieu toc do\r\n"); return; }
    mode = CLI_MODE_SPEED;
    SpeedCtrl_SetTarget((int16_t)atoi(arg));
    reply_state();
  }
  else if (strcmp(line, "d") == 0)
  {
    if (arg == NULL) { reply("# thieu duty\r\n"); return; }
    mode = CLI_MODE_DUTY;
    SpeedCtrl_SetTarget(0);          /* xoa trang thai PID, khong de no dieu khien */
    Motor_SetSpeed((int16_t)atoi(arg));
    reply_state();
  }
  else if (strcmp(line, "kp") == 0 || strcmp(line, "ki") == 0 || strcmp(line, "kd") == 0)
  {
    set_gain(line, arg);
  }
  else if (strcmp(line, "demo") == 0)
  {
    mode = CLI_MODE_DEMO;
    reply_state();
  }
  else if (strcmp(line, "stop") == 0)
  {
    mode = CLI_MODE_SPEED;
    SpeedCtrl_SetTarget(0);
    Motor_Coast();
    reply_state();
  }
  else if (strcmp(line, "log") == 0)
  {
    if (arg != NULL) log_on = (atoi(arg) != 0);
    reply_state();
  }
  else if (strcmp(line, "?") == 0 || strcmp(line, "help") == 0)
  {
    reply("# s <rpm> | d <duty> | kp <v> | ki <v> | kd <v> | demo | stop | log 0|1 | ?\r\n");
    reply_state();
  }
  else
  {
    reply("# lenh la, go ? de xem danh sach\r\n");
  }
}

void Cli_Init(void)
{
  UartRx_Init();
  mode   = CLI_MODE_SPEED;
  log_on = 1;
}

void Cli_Poll(void)
{
  static char line[UART_RX_LINE_MAX];

  /* timeout 0: chi lay dong da san sang, khong lam tre vong dieu khien */
  while (UartRx_GetLine(line, 0))
    handle(line);
}

CliMode_t Cli_GetMode(void) { return mode; }
int       Cli_IsLogOn(void) { return log_on; }
