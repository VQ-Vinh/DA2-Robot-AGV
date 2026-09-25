/**
  * @file    uart_rx.c
  * @brief   Nhan lenh tu PC qua USART2 bang ngat, cat thanh tung dong
  */
#include "uart_rx.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>

#define RX_UART     huart2
#define RX_QUEUE_N  4    /* so dong dem duoc truoc khi bi bo */

typedef struct
{
  char text[UART_RX_LINE_MAX];
} RxLine_t;

static osMessageQueueId_t rx_queue;
static uint8_t            rx_byte;             /* ngat ghi vao day tung byte */
static char               rx_buf[UART_RX_LINE_MAX];
static uint16_t           rx_len;

void UartRx_Init(void)
{
  rx_queue = osMessageQueueNew(RX_QUEUE_N, sizeof(RxLine_t), NULL);
  rx_len   = 0;
  HAL_UART_Receive_IT(&RX_UART, &rx_byte, 1);
}

int UartRx_GetLine(char *out, uint32_t timeout_ms)
{
  RxLine_t line;
  uint32_t ticks = timeout_ms * osKernelGetTickFreq() / 1000U;

  if (osMessageQueueGet(rx_queue, &line, NULL, ticks) != osOK)
    return 0;

  memcpy(out, line.text, UART_RX_LINE_MAX);
  out[UART_RX_LINE_MAX - 1] = 0;
  return 1;
}

/* HAL goi trong USART2_IRQHandler moi khi nhan du 1 byte */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &RX_UART)
    return;

  char c = (char)rx_byte;

  if (c == '\n')
  {
    if (rx_len > 0)
    {
      RxLine_t line;
      rx_buf[rx_len] = 0;
      memcpy(line.text, rx_buf, rx_len + 1);
      /* Trong ngat: timeout phai la 0. Queue day thi bo dong nay. */
      osMessageQueuePut(rx_queue, &line, 0, 0);
      rx_len = 0;
    }
  }
  else if (c != '\r')
  {
    /* Dong qua dai: bo phan thua, giu lai phan dau */
    if (rx_len < UART_RX_LINE_MAX - 1)
      rx_buf[rx_len++] = c;
  }

  HAL_UART_Receive_IT(&RX_UART, &rx_byte, 1);
}
