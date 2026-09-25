/**
  * @file    uart_rx.h
  * @brief   Nhan lenh tu PC qua USART2, cat thanh tung dong
  *
  * Phan cung (cau hinh trong STM32.ioc):
  *   USART2_RX -> PA3, nhan bang NGAT (USART2_IRQn uu tien 5, da bat san cho DMA TX)
  *   Khong dung DMA cho chieu nhan: lenh rat ngan va thua nho.
  *
  * Ngat nhan tung byte roi ghep vao bo dem. Gap xuong dong thi day ca dong
  * vao queue cho task doc ra. Ky tu CR bi bo qua nen terminal nao cung dung duoc.
  */
#ifndef UART_RX_H
#define UART_RX_H

#include <stdint.h>

#define UART_RX_LINE_MAX  48   /* du cho lenh dai nhat, vi du "kp 0.125" */

void UartRx_Init(void);  /* tao queue va bat ngat nhan; goi sau osKernelInitialize() */

/* Lay mot dong da nhan duoc. out phai co it nhat UART_RX_LINE_MAX byte.
 * Tra ve 1 neu co dong moi, 0 neu khong. timeout_ms = 0 thi khong cho. */
int UartRx_GetLine(char *out, uint32_t timeout_ms);

#endif /* UART_RX_H */
