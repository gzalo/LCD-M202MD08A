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

#define US_PER_BIT (5.33)

#define ERR_MAX_LEN (1)
#define ERR_CHECKSUM (2)
#define ERR_ADDR (3)
#define ERR_RESPONSE (4)

#include "crc.h"

void gpioInit(){
	DDRD |=  _BV(VFD_TX);
	DDRD &= ~_BV(VFD_RX);
	DDRB |= _BV(LED_BIT);
}

#define USART_BAUDRATE 56000
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART0Init(void){
	// Set baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable transmission and reception
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
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
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);

void vfdSendByte(uint8_t byte, uint8_t addr){
	
	//187500 bps
	//1 start bit
	//8 data bits
	//9th addr bit
	//11, 12 stop bits
}

uint16_t vfdRecvByte(){
	return 0;
}

uint8_t vfdRecvData(){
	uint8_t end=0;
	uint8_t len = 0;
	
	while(!end){

		uint16_t recv = vfdRecvByte();
		
 		recvBff[len] = recv & 0xFF; //Discard address bit

		if(recv == CMD_FLAGCHAR || recv == CMD_EOPCHAR) //Flag or end of poll character
			break;
		
		len++;

		if(len>=RECV_BUFF_LEN){
			recvLen = len;
			return ERR_MAX_LEN;
		}
	}

	recvLen = len;
	uint8_t chksum[2];
	crc16_x25(recvBff, len-3, chksum); //Calc checksum (excluding 3 last bytes)
	
	if(recvBff[len-3] != chksum[0] || recvBff[len-2] != chksum[1])
		return ERR_CHECKSUM; //Wrong checksum
	
	return 0; 
}

uint8_t vfdPollAndRecieve(uint8_t addr){	
	if(addr >= 2)
		return ERR_ADDR;
		
	vfdSendByte(vfdAddr[addr] | 0x80, 1);
	_delay_us(12*US_PER_BIT);
	vfdSendByte(vfdAddr[addr] | 0x80, 1);
	_delay_us(12*US_PER_BIT);

	return vfdRecvData();
}

//Sends data with address, checksum, framing
//Len should be just the size of message (without adding 4)
uint8_t vfdSendData(uint8_t addr, uint8_t *msg, uint8_t len){
	if(addr >= 2)
		return ERR_ADDR;
	
	sendBff[0] = vfdAddr[addr];        //Dst address
	
	uint8_t i; //Copy message to buffer
	for(i=1;i<=len;i++)
		sendBff[i] = msg[i-1];
  
	uint8_t chksum[2];
	crc16_x25(sendBff, len+1, chksum); //function calculating CRC-CCITT polynomial 0x1021 (includes addr)
	
	sendBff[len] = chksum[0];   //Checksum 1
	sendBff[len+1] = chksum[1];   //Checksum 2
	sendBff[len+2] = CMD_FLAGCHAR&0xFF; //End of frame 
	
	//Send
	for(i=0;i<len;i++){
		if(i == 0 || i == len+2)
			vfdSendByte(sendBff[i],1);
		else
			vfdSendByte(sendBff[i],0);
		_delay_us(US_PER_BIT); //1 extra stop byte
	}
	return 0;
}
 
 uint8_t vfdSNRM(uint8_t addr){
	uint8_t buffer[1];
	buffer[0] = CMD_SNRM;
	vfdSendData(addr, buffer, 1);
	
	uint8_t ret = vfdPollAndRecieve(addr);
	
	if(ret != 0)
		return ret;
	else if(recvBff[1] == CMD_NSA)
		return 0;
	else
		return ERR_RESPONSE;
}

uint8_t vfdInit(uint8_t addr){
	uint8_t ret = vfdPollAndRecieve(addr);
	
	if(ret != 0)
		return ret;
	
	if(recvBff[1] != CMD_ROL)
		return ERR_RESPONSE;
		
	return vfdSNRM(addr);
}

//Caller should have an extra byte to the beginning, to add info. command
uint8_t vfdSendInfo(uint8_t addr, uint8_t *bff, uint8_t len){
	if(addr >= 2)
		return ERR_ADDR;
	
	bff[0] = (sentFrames[addr]<<1) | (recvFrames[addr]<<5);
	
	uint8_t ret = vfdSendData(addr, bff, len);
	if(ret != 0)
		return ret;
	
	ret = vfdPollAndRecieve(addr);
	if(ret != 0)
		return ret;
	
	if((recvBff[1] & CMD_RR_MASK) == CMD_RR){ //RR command
		sentFrames[addr]++;
		if(sentFrames[addr] == 8)
			sentFrames[addr] = 0;	
		
		if(sentFrames[addr] != (recvBff[1]>>5)){
			//Acked wrong packet
			return ERR_RESPONSE;
		}else{
			return 0;
		}
	}else{
		return ERR_RESPONSE;
	}	
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

uint8_t vfdPrintLine(uint8_t addr, char * txt, char line){
	uint8_t tmp[23] = {0x00};
	
	//Command, Clocation, 20 data bytes, Sum of bytes
	
	if(line == 1)
		tmp[0] = 0x81;
	else if(line == 2)
		tmp[0] = 0x82;
	else
		return ERR_ADDR;
	
	tmp[1] = 0; //Cursor position (ignored in this model)
	tmp[22] = 0; //8 bit sum of 20 data bytes
		
	uint8_t i = 0;
	for(i=0;i<20;i++){
		tmp[i+2] = txt[i]; 
		tmp[22] += txt[i]; //Add to sum byte
	}
	
	return vfdSendInfo(addr, tmp, 23);
}

int main(void){
	_delay_ms(500);
	
	gpioInit();
	USART0Init();
	
	stdout=&usart0_str;
	
	while(1){
		
	
		_delay_ms(100);
		
	}
	return 0;
}