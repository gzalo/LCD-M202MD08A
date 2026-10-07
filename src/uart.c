#include "uart.h"
#include <avr/io.h>
#include <stdio.h>

#define USART_BAUDRATE 115200
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)	

static void USART0Init(void){
	// Set baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable transmission and reception
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	
	UCSR0A |= (1<<U2X0);
}
static int USART0SendByte(char u8Data, FILE *stream){
	if(u8Data == '\n'){
		USART0SendByte('\r', stream);
	}
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	UDR0 = u8Data;
	return 0;
}
int16_t uartGetByte(void){
	if (!(UCSR0A & _BV(RXC0))) 
		return UART_NO_DATA;
	return UDR0;
}
static FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);


void uartInit(void){
	USART0Init();
	stdout = &usart0_str;
}
