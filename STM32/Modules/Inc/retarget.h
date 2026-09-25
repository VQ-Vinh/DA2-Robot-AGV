/**
  * @file    retarget.h
  * @brief   Chuyen huong printf ra USART2 (PA2 TX / PA3 RX, 115200 8N1)
  *
  * Khong co ham nao can goi: syscalls.c (_write) goi __io_putchar,
  * chi can link retarget.c vao project la printf chay ra UART.
  */
#ifndef RETARGET_H
#define RETARGET_H

int __io_putchar(int ch);

#endif /* RETARGET_H */
