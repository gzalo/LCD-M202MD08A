#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_NO_DATA (-1)

void uartInit(void);
int16_t uartGetByte(void);

#endif
