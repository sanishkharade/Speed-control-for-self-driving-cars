#ifndef _UART_H_
#define _UART_H_

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

void UART_Init(void);
void UART_Transmit(char s);
char UART_Receive(void);

#endif /*_UART_H_*/
