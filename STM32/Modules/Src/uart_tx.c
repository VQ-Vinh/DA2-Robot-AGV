/**
  * @file    uart_tx.c
  * @brief   Gui du lieu ra USART2 bang DMA
  */
#include "uart_tx.h"
#include "usart.h"
#include "cmsis_os.h"

#define TX_UART  huart2

static osSemaphoreId_t tx_done;
static osMutexId_t     tx_lock;   /* nhieu task cung gui: logTask va CLI */

void UartTx_Init(void)
{
  tx_done = osSemaphoreNew(1, 0, NULL);
  tx_lock = osMutexNew(NULL);
}

int UartTx_Send(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  if (len == 0) return 0;

  uint32_t ticks = timeout_ms * osKernelGetTickFreq() / 1000U;

  if (osMutexAcquire(tx_lock, ticks) != osOK)
    return -1;

  /* Xoa token cu: lan gui truoc bi timeout nhung callback van den tre */
  osSemaphoreAcquire(tx_done, 0);

  int rc = 0;
  if (HAL_UART_Transmit_DMA(&TX_UART, (uint8_t *)buf, len) != HAL_OK)
  {
    rc = -1;
  }
  else if (osSemaphoreAcquire(tx_done, ticks) != osOK)
  {
    HAL_UART_AbortTransmit(&TX_UART);
    rc = -1;
  }

  osMutexRelease(tx_lock);
  return rc;
}

/* HAL goi trong USART2_IRQHandler khi byte cuoi da ra khoi chan TX */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &TX_UART)
    osSemaphoreRelease(tx_done);
}
