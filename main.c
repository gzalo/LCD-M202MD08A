// Futaba M202MD08A LCD simplified driver code for Arduino Pro Mini (atmega328p)
// Used in IBM 41K6814 20x2 dual VFD Point of Sale displays.
// Supports both displays and some basic commands
//
// Released under Public Domain (code provided "as is", without warranty of any kind)
// 2017 - Gzalo (Gonzalo Ávila Alterach)
//
// Thanks to
// http://www.elektroda.pl/rtvforum/topic558072-90.html
// http://we.easyelectronics.ru/lcd_gfx/vyvodim-informaciyu-na-vfd-ibm-41k6814.html

#define F_CPU 16000000UL
#define __DELAY_ROUND_CLOSEST__ 
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#define LED_BIT PB5
#define VFD_TX PD2
#define VFD_RX PD3

const uint8_t vfdAddr[2] = {0x24, 0x25};
uint8_t sentFrames[2] = {0,0}, recvFrames[2] = {0,0}; 

#define RECV_BUFF_LEN (32)
#define SEND_BUFF_LEN (32)

uint8_t recvBff[RECV_BUFF_LEN];
uint8_t recvLen = 0;
uint8_t sendBff[SEND_BUFF_LEN];

#define CMD_FLAGCHAR (0x7E|0x100)
#define CMD_EOPCHAR (0x5A|0x100)
#define CMD_NSA (0x63)
#define CMD_ROL (0x0F)
#define CMD_RR (0x01)
#define CMD_SNRM (0x83)
#define CMD_RR_MASK (0x1F)
#define CMD_I_NEG_MASK (0x11)

#define US_PER_BIT (5.33)
//Era 4.95 (4.98 anda bien)
#define US_PER_BIT_TX (4.95) 
#define US_PER_BIT_TX_LAST (5.2)

#define US_PER_BIT_RX (4.8)
#define US_PER_BIT_RX_HALF (1)

#define ERR_MAX_LEN (1)
#define ERR_CHECKSUM (2)
#define ERR_ADDR (3)
#define ERR_RESPONSE (4)

#define RX_RING_MASK (RECV_BUFF_LEN - 1)
#define ACK_TIMEOUT_LOOPS (10)
#define ACK_MAX_RETRIES (3)
#define USART_NO_DATA (-1)

#include "crc.h"

void gpioInit(){
	DDRD |=  _BV(VFD_TX);
	DDRD &= ~_BV(VFD_RX);
	DDRB |= _BV(LED_BIT);
	
	PORTD |= _BV(VFD_TX); //TX Starts high
	PORTD |= _BV(VFD_RX); //RX extra pullup
	
	EICRA = (1 << ISC11); //INT1 trigger on falling edge
	EIMSK = (1 << INT1);  // Turns on INT1
	
}

volatile uint8_t rxBff[RECV_BUFF_LEN] = {0};
volatile uint8_t rxAddr[RECV_BUFF_LEN] = {0};
volatile uint8_t rxPos = 0, rxRead = 0;
volatile uint8_t rxOverflow = 0;

inline void pulse(){
	//PORTD &= ~_BV(VFD_TX);
	//PORTD |= _BV(VFD_TX);
}

ISR (INT1_vect){
    _delay_us(US_PER_BIT_RX_HALF); //Wait half clock period

	pulse();
		
	uint8_t ret = 0;
	uint8_t i;
	for(i=0;i<8;i++){ //8 data bits + 1 addr bit
		_delay_us(US_PER_BIT_RX); //Wait clock period until half of first data bit
		ret >>= 1;
		pulse();		
		if(PIND & _BV(VFD_RX)){
			ret |= 0x80;
		}else{
			ret &= 0x7F;
		}
	}
	_delay_us(US_PER_BIT_RX); //Go to addr bit
	
	uint8_t addrBit = 0;
	if(PIND & _BV(VFD_RX))
		addrBit = 1;
	pulse();
	
	_delay_us(US_PER_BIT_RX); //Go to stop bit
	
	if(PIND & _BV(VFD_RX)){
		uint8_t nextPos = (rxPos+1) & RX_RING_MASK;
		if(nextPos != rxRead){
			rxBff[rxPos] = ret;
			rxAddr[rxPos] = addrBit;
			rxPos = nextPos;
		}else{
			rxOverflow = 1;
		}
	}	
	
	EIFR = (1<<INTF1); //Clear any other falling edge interrupt
}

void timerInit(){
	//Normal mode
	TCCR1A = 0;
	//Prescaler = 8
	TCCR1B |= (1 << CS11);
}
#define USART_BAUDRATE 115200
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)	

void USART0Init(void){
	// Set baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable transmission and reception
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	
	UCSR0A |= (1<<U2X0);
}
int USART0SendByte(char u8Data, FILE *stream){
	if(u8Data == '\n'){
		USART0SendByte('\r', stream);
	}
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	UDR0 = u8Data;
	return 0;
}
int16_t USART0GetByte(void){
	if (!(UCSR0A & _BV(RXC0))) 
		return USART_NO_DATA;
	return UDR0;
}
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);

void vfdSendByte(uint8_t byte, uint8_t addr){
	//187500 bps
		
	PORTD &= ~_BV(VFD_TX);
	//1: start bit

	uint8_t i;
	for(i=0;i<8;i++){ //8 data bits
		_delay_us(US_PER_BIT_TX);
		if(byte & 1){
			PORTD |= _BV(VFD_TX);
		}else{
			PORTD &= ~_BV(VFD_TX);
		}
		byte >>=1;
	}
	_delay_us(US_PER_BIT_TX_LAST);
	
	if(addr){ //Addr bit
		PORTD |= _BV(VFD_TX);
	}else{
		PORTD &= ~_BV(VFD_TX);
	}
	_delay_us(US_PER_BIT_TX_LAST);
	
	PORTD |= _BV(VFD_TX);; //11, 12: Stop bits
	_delay_us(2*US_PER_BIT_TX);
}

void vfdPoll(uint8_t addr){	
	if(addr >= 2)
		return;
		
	_delay_us(12*US_PER_BIT);
	vfdSendByte(vfdAddr[addr] | 0x80, 1);
	_delay_us(12*US_PER_BIT);
	vfdSendByte(vfdAddr[addr] | 0x80, 1);
}

//Sends data with address, checksum, framing
//Len should be just the size of message (without adding 4)
uint8_t vfdSendData(uint8_t addr, uint8_t *msg, uint8_t len){
	if(addr >= 2)
		return ERR_ADDR;
	if(len > SEND_BUFF_LEN - 4)
		return ERR_MAX_LEN;
	
	sendBff[0] = vfdAddr[addr];        //Dst address
	
	uint8_t i; //Copy message to buffer
	for(i=1;i<=len;i++)
		sendBff[i] = msg[i-1];
  
	uint8_t chksum[2];
	crc16_x25(sendBff, len+1, chksum); //function calculating CRC-CCITT polynomial 0x1021 (includes addr)
	
	sendBff[len+1] = chksum[0];   //Checksum 1
	sendBff[len+2] = chksum[1];   //Checksum 2
	sendBff[len+3] = CMD_FLAGCHAR&0xFF; //End of frame 
	
	printf("Sending: ");
		
	//Send
	for(i=0;i<=len+3;i++){
		if(i == 0 || i == len+3){
			printf("%x! ", sendBff[i]);
			vfdSendByte(sendBff[i],1);
			_delay_us(400);
		}else{
			printf("%x ", sendBff[i]);
			vfdSendByte(sendBff[i],0);
		}
		_delay_us(200); //1 extra stop byte
	}
	return 0;
}
 
void vfdSNRM(uint8_t addr){
	uint8_t buffer[1];
	buffer[0] = CMD_SNRM;
	vfdSendData(addr, buffer, 1);
	vfdPoll(addr);
}

//Caller should have an extra byte to the beginning, to add info. command
uint8_t vfdSendInfo(uint8_t addr, uint8_t *bff, uint8_t len){
	if(addr >= 2)
		return ERR_ADDR;
	
	bff[0] = (recvFrames[addr]<<5) | (sentFrames[addr]<<1);
	//printf("SentInfo R=%d S=%d ", recvFrames[addr], sentFrames[addr]);
	
	uint8_t ret = vfdSendData(addr, bff, len);
	if(ret != 0)
		return ret;
	sentFrames[addr] = (sentFrames[addr]+1)&7;

	_delay_ms(1);
	
	vfdPoll(addr);	

	return 0;
}
uint8_t vfdSendRR(uint8_t addr){
	if(addr >= 2)
		return ERR_ADDR;
	
	uint8_t bff[1] = {(recvFrames[addr]<<5) | CMD_RR};
	//printf("SentRR R=%d ", sentFrames[addr]);
	
	uint8_t ret = vfdSendData(addr, bff, 1);
	if(ret != 0)
		return ret;

	_delay_ms(1);
	
	vfdPoll(addr);	
	_delay_ms(1);

	return 0;
}

uint8_t vfdTrig(uint8_t addr, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4){
	uint8_t tmp[6] = {0x00};
	
	//Command, P1 a P4
	tmp[1] = 0x21;
	tmp[2] = p1;
	tmp[3] = p2;
	tmp[4] = p3;
	tmp[5] = p4;
	
	return vfdSendInfo(addr, tmp, 6);
}
uint8_t vfdInfoReq(uint8_t addr){
	uint8_t tmp[4] = {0x0};

	tmp[1] = 0x00;
	tmp[2] = 0x00;
	tmp[3] = 0x01;
	
	return vfdSendInfo(addr, tmp, 4);
}

uint8_t vfdPrintLine(uint8_t addr, char * txt, char line){
	uint8_t tmp[24] = {0x00};
	
	//Addr, Command, Clocation, 20 data bytes, Sum of bytes
	
	if(line == 1)
		tmp[1] = 0x81;
	else if(line == 2)
		tmp[1] = 0x82;
	else
		return ERR_ADDR;
	
	tmp[2] = 0; //Cursor position (ignored in this model)
	tmp[22] = 0; //8 bit sum of 20 data bytes
		
	uint8_t i = 0;
	for(i=0;i<20;i++){
		tmp[i+3] = txt[i]; 
		tmp[23] += txt[i]; //Add to sum byte
	}
	/*_delay_us(1500);
	vfdSendByte(0xCA, 1);
	_delay_us(191);
	vfdSendByte(0xCA, 1);	
	_delay_us(3000);*/
	
	return vfdSendInfo(addr, tmp, 24);
}

char lineas[2][21] = {"12345678901231231222", "ABCASDASDASDASDASDAS"};
uint8_t linea = 0;

uint8_t cmdBuff[32];
uint8_t cmdIdx = 0;
uint8_t cmdInFrame = 0;

uint8_t rol = 0, sync = 0, ack = 1;
uint8_t ackWait = 0, ackRetries = 0;
uint8_t pendingFrame = 0, pendingLine = 0;
char pendingText[21];

uint8_t rxGetByte(uint8_t *byte, uint8_t *addr){
	if(rxRead == rxPos)
		return 0;

	*byte = rxBff[rxRead];
	*addr = rxAddr[rxRead];
	rxRead = (rxRead + 1) & RX_RING_MASK;
	return 1;
}

void protocolReset(void){
	rol = 0;
	sync = 0;
	ack = 1;
	ackWait = 0;
	ackRetries = 0;
	linea = 0;
	cmdInFrame = 0;
	cmdIdx = 0;
	sentFrames[0] = 0;
	recvFrames[0] = 0;
}

void handleFrame(void){
	if(cmdIdx < 4){
		printf("Short frame\n");
		return;
	}

	uint8_t chksum[2];
	crc16_x25(cmdBuff, cmdIdx-2, chksum); //Calc checksum (excluding checksum bytes, of course)

	printf("DataRX: ");
	uint8_t i;
	for(i=0;i<cmdIdx;i++)
		printf("%x ", cmdBuff[i]);
	printf("\n");

	if(cmdBuff[cmdIdx-2] != chksum[0] || cmdBuff[cmdIdx-1] != chksum[1]){
		printf("Wrong CRC: %x %x vs Recv %x %x\n", chksum[0], chksum[1], cmdBuff[cmdIdx-2], cmdBuff[cmdIdx-1]);
		return;
	}

	uint8_t control = cmdBuff[1];
	uint8_t remoteAck = (control>>5)&7;

	if(control == CMD_ROL){
		printf("ROL OK\n");
		protocolReset();
		vfdSNRM(0);
		return;
	}
	if(control == CMD_NSA){
		protocolReset();
		rol = 1;
		return;
	}

	uint8_t isRR = (control&CMD_RR_MASK) == CMD_RR;
	uint8_t isInfo = (control&CMD_I_NEG_MASK) == 0;
	if((isRR || isInfo) && !ack && remoteAck == sentFrames[0]){
		ack = 1;
		ackWait = 0;
		ackRetries = 0;
	}

	if(isRR){
		printf("RR R=%d ", remoteAck);
		if(remoteAck != sentFrames[0])
			printf("Mismatch S/R\n");
	}else if(isInfo){
		uint8_t remoteSend = (control>>1)&7;
		printf("Info R=%d S=%d ", remoteAck, remoteSend);

		if(remoteAck != sentFrames[0])
			printf("Mismatch S/R\n");

		if(remoteSend == recvFrames[0]){
			recvFrames[0] = (recvFrames[0]+1)&7;
		}else{
			printf("Unexpected I-frame sequence\n");
		}

		vfdSendRR(0);

		if(cmdIdx > 2 && cmdBuff[2] == 0x10)
			sync = 1;
	}
}

void processRxByte(uint8_t byte, uint8_t addrBit){
	if(addrBit && byte == (CMD_EOPCHAR&0xFF)){
		cmdInFrame = 0;
		cmdIdx = 0;
		return;
	}

	if(addrBit && byte == vfdAddr[0]){
		cmdBuff[0] = byte;
		cmdIdx = 1;
		cmdInFrame = 1;
		return;
	}

	if(!cmdInFrame)
		return;

	if(addrBit && byte == (CMD_FLAGCHAR&0xFF)){
		handleFrame();
		cmdInFrame = 0;
		cmdIdx = 0;
		return;
	}

	if(cmdIdx >= sizeof(cmdBuff)){
		printf("RX frame too long\n");
		cmdInFrame = 0;
		cmdIdx = 0;
		return;
	}

	cmdBuff[cmdIdx++] = byte;
}

void processRxBuffer(void){
	if(rxOverflow){
		rxOverflow = 0;
		cmdInFrame = 0;
		cmdIdx = 0;
		printf("RX overflow\n");
	}

	uint8_t byte, addrBit;
	while(rxGetByte(&byte, &addrBit))
		processRxByte(byte, addrBit);
}

int main(void){
	_delay_ms(100);
	
	gpioInit();
	timerInit();
	USART0Init();
	stdout=&usart0_str;
	
	sei();
	
	uint8_t curX = 0, curY = 0;
	while(1){
		int16_t v;
		while((v = USART0GetByte()) != USART_NO_DATA){
			lineas[curY][curX] = (char)v;
			curX++;
			if(curX == 20) curX = 0, curY++;
			if(curY == 2) curY = 0;
		}

		processRxBuffer();
		
		if(!rol){
			printf("InitPoll\n");
			vfdPoll(0);
			_delay_ms(10);
		}else if(!sync){
			vfdInfoReq(0);
			_delay_ms(10);
		}else if(ack){
			pendingFrame = sentFrames[0];
			pendingLine = linea;
			memcpy(pendingText, lineas[linea], 20);
			pendingText[20] = 0;

			if(vfdPrintLine(0, pendingText, pendingLine+1) == 0){
				ack = 0;
				ackWait = 0;
				ackRetries = 0;
				linea++;
				if(linea == 2)
					linea = 0;
				_delay_ms(20);
				vfdPoll(0);
			}else{
				protocolReset();
			}
		}else{
			if(++ackWait >= ACK_TIMEOUT_LOOPS){
				ackWait = 0;
				if(ackRetries < ACK_MAX_RETRIES){
					ackRetries++;
					printf("Ack timeout, retry\n");
					sentFrames[0] = pendingFrame;
					if(vfdPrintLine(0, pendingText, pendingLine+1) == 0)
						vfdPoll(0);
					else
						protocolReset();
				}else{
					printf("Ack timeout, resync\n");
					protocolReset();
				}
			}else{
				vfdPoll(0);
			}
		}

		processRxBuffer();
		
		/* Test timing
		vfdSendByte(0x55,1);
		_delay_us(10);
		vfdSendByte(0xAA,0);*/

		_delay_ms(10);		
		//_delay_ms(100);			
	}
	return 0;
}
