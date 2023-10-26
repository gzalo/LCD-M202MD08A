
#define USART_BAUDRATE 115200
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)

FILE *UART0Init(void){
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE>>8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    // Set frame format to 8 data bits, no parity, 1 stop bit
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
    //enable transmission and reception
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);

    UCSR0A |= (1<<U2X0);
    return &FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);
}

int USART0SendByte(char u8Data, FILE *stream){
    if(u8Data == '\n'){
        USART0SendByte('\r', stream);
    }
    while(!(UCSR0A&(1<<UDRE0))){
        // wait while previous byte is completed
    }
    // Transmit data
    UDR0 = u8Data;
    return 0;
}

uint8_t UART0GetByte(void){
    if (!(UCSR0A & _BV(RXC0)))
        return 0;
    return UDR0;
}
