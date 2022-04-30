#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h> 	

#include "uart.h"

const char *uart_file = "/dev/ttyS0";
int uart_fd;
int main ()
{

	
	
	int uart_fd;
	
	uart_fd = open(uart_file, O_RDWR);
	if(uart_fd < 0)
	{
			printf("ERROR: OPEN failed");
	}
	
	char s;
	char r;
	while(1)
	{
			printf("Enter character\n");
			scanf("%c%*c", &s);
			printf("Character sent = %d\n", s);
			write(uart_fd, &s, 1);
			usleep(10);
			read(uart_fd, &r, 1);
			
			printf("Character received = %c\n", r); 
	}
	

}
//~ void UART_Init(void)
//~ {
	//~ uart_fd = open(uart_file, O_RDWR);
	//~ if(uart_fd < 0)
	//~ {
			//~ printf("ERROR: OPEN failed");
	//~ }
//~ }
//~ void UART_Transmit(char s)
//~ {
	//~ write(uart_fd, &s, 1);
//~ }
//~ char UART_Receive(void)
//~ {
	//~ char r;
	//~ read(uart_fd, &r, 1);
	//~ return r;
//~ }
