/**
  * @file    retarget.c
  * @brief   Chuyen huong printf ra USART2
  */
#include "retarget.h"
#include "usart.h"

#define RETARGET_UART  huart2

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&RETARGET_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
