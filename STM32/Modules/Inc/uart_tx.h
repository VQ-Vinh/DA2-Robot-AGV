/**
  * @file    uart_tx.h
  * @brief   Gui du lieu ra USART2 bang DMA, task goi bi block cho toi khi gui xong
  *
  * Phan cung (cau hinh trong STM32.ioc):
  *   USART2_TX -> DMA1 Stream 6, Normal, Byte
  *   USART2_IRQn va DMA1_Stream6_IRQn: uu tien 5 (duoc goi API FreeRTOS)
  *
  * CPU khong phai cho tung byte nhu HAL_UART_Transmit: task gui duoc block,
  * cac task khac chay trong luc DMA day du lieu ra chan TX.
  */
#ifndef UART_TX_H
#define UART_TX_H

#include <stdint.h>

void UartTx_Init(void);  /* tao semaphore, goi sau osKernelInitialize() */

/* Chi goi tu task, sau khi scheduler da chay. buf phai con song toi khi ham tra ve.
 * Tra ve 0 neu gui xong, -1 neu UART dang ban hoac qua timeout_ms. */
int UartTx_Send(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

#endif /* UART_TX_H */
