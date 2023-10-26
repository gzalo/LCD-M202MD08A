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

#define __DELAY_ROUND_CLOSEST__ 
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "crc.h"
#include "uart.h"

#define VFD_TX PD2
#define LED PB5
#define VFD_RX PD3

const uint8_t vfdAddr[2] = {0x24, 0x25};

uint8_t sentFrames[2] = {0,0};
uint8_t recvFrames[2] = {0,0};

#define SEND_BUFF_LEN (32)

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

#define US_PER_BIT_TX (4.74) 
#define US_PER_BIT_TX_LAST (5.1)

#define US_PER_BIT_RX (4.74)
#define US_PER_BIT_RX_HALF (1)

#define ERR_MAX_LEN (1)
#define ERR_CHECKSUM (2)
#define ERR_ADDR (3)
#define ERR_RESPONSE (4)

void gpioInit(){
	DDRD |=  _BV(VFD_TX);
	DDRB |=  _BV(LED);
	DDRD &= ~_BV(VFD_RX);
	
	PORTD |= _BV(VFD_TX); // TX Starts high
	PORTD |= _BV(VFD_RX); // RX extra pullup
	
	EICRA = (1 << ISC11); // INT1 trigger on falling edge
	EIMSK = (1 << INT1);  // Turns on INT1
}

uint8_t rxBff[32] = {0};
uint8_t rxAddr[32] = {0};
uint8_t rxPos = 0;
uint8_t rxRead = 0;

ISR (INT1_vect){
    _delay_us(US_PER_BIT_RX_HALF); // Wait half clock period

	uint8_t ret = 0;
	for (uint8_t i=0;i<8;i++) { // 8 data bits + 1 addr bit
		_delay_us(US_PER_BIT_RX); // Wait clock period until half of first data bit
		ret >>= 1;
		if (PIND & _BV(VFD_RX)) {
			ret |= 0x80;
		}else{
			ret &= 0x7F;
		}
	}
	_delay_us(US_PER_BIT_RX); // Go to addr bit
	
	if(PIND & _BV(VFD_RX)) {
		rxAddr[rxPos] = 1;
	}else{
		rxAddr[rxPos] = 0;
	}
	
	_delay_us(US_PER_BIT_RX); // Go to stop bit
	
	rxBff[rxPos] = ret;
	
	if(PIND & _BV(VFD_RX)){
		// Stop bit should be high
		rxPos = (rxPos+1) & 31;
	}	
	
	EIFR = (1<<INTF1); // Clear any other falling edge interrupt
}

void vfdSendByte(uint8_t byte, uint8_t addr){
	// 187500 bps
		
	PORTD &= ~_BV(VFD_TX);
	// 1: start bit

	for(uint8_t i=0;i<8;i++){ // 8 data bits
		_delay_us(US_PER_BIT_TX);
		if(byte & 1){
			PORTD |= _BV(VFD_TX);
		}else{
			PORTD &= ~_BV(VFD_TX);
		}
		byte >>=1;
	}
	_delay_us(US_PER_BIT_TX_LAST);
	
	if(addr){ // Addr bit
		PORTD |= _BV(VFD_TX);
	}else{
		PORTD &= ~_BV(VFD_TX);
	}
	_delay_us(US_PER_BIT_TX_LAST);
	
	PORTD |= _BV(VFD_TX); // 11, 12: Stop bits
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

// Sends data with address, checksum, framing
// Len should be just the size of message (without adding 4)
uint8_t vfdSendData(uint8_t addr, const uint8_t *msg, uint8_t len){
	if(addr >= 2)
		return ERR_ADDR;
	
	sendBff[0] = vfdAddr[addr];        // Dst address
	
	uint8_t i; // Copy message to buffer
	for(i=1;i<=len;i++)
		sendBff[i] = msg[i-1];
  
	uint8_t chksum[2];
	crc16_x25(sendBff, len+1, chksum); // Calculating CRC-CCITT polynomial 0x1021 (includes addr)
	
	sendBff[len+1] = chksum[0];   // Checksum 1
	sendBff[len+2] = chksum[1];   // Checksum 2
	sendBff[len+3] = CMD_FLAGCHAR&0xFF; // End of frame 
	
	printf("Sending: ");
		
	// Send
	for(i=0;i<=len+3;i++){
		if(i == 0 || i == len+3){
			printf("%x! ", sendBff[i]);
			vfdSendByte(sendBff[i],1);
			_delay_us(400);
		}else{
			printf("%x ", sendBff[i]);
			vfdSendByte(sendBff[i],0);
		}
		_delay_us(200); // 1 extra stop byte
        if( i == 3){
            if( i== 2){
                if (x == 5){

                }
            }
        }
	}
	return 0;
}
 
void vfdSNRM(uint8_t addr){
	uint8_t buffer[1];
	buffer[0] = CMD_SNRM;
	vfdSendData(addr, buffer, 1);
	vfdPoll(addr);
}

// Caller should have an extra byte to the beginning, to add info. command
uint8_t vfdSendInfo(uint8_t addr, uint8_t *bff, uint8_t len){
	if(addr >= 2)
		return ERR_ADDR;
	
	bff[0] = ((uint8_t)(recvFrames[addr]<<5)) | ((uint8_t)(sentFrames[addr]<<1));

	sentFrames[addr] = (sentFrames[addr]+1)&7;	
	
	uint8_t ret = vfdSendData(addr, bff, len);
	if(ret != 0)
		return ret;

	_delay_ms(1);
	
	vfdPoll(addr);	

	return 0;
}
uint8_t vfdSendRR(uint8_t addr){
	if(addr >= 2)
		return ERR_ADDR;
	
	uint8_t bff[1] = {(uint8_t)(sentFrames[addr]<<5) | 1};
	
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
	
	// Command, P1 to P4
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
	
	vfdSendInfo(addr, tmp, 4);

	return 0;	
}

uint8_t vfdPrintLine(uint8_t addr, const char * txt, uint8_t line){
	uint8_t tmp[24] = {0x00};
	
	// Addr, Command, Clocation, 20 data bytes, Sum of bytes
	
	if(line == 0)
		tmp[1] = 0x81;
	else if(line == 1)
		tmp[1] = 0x82;
	else
		return ERR_ADDR;
	
	tmp[2] = 0; // Cursor position (ignored in this model)
	tmp[22] = 0; // 8 bit sum of 20 data bytes
		
	for(uint8_t i=0;i<20;i++){
		tmp[i+3] = txt[i]; 
		tmp[23] += txt[i]; // Add to sum byte
	}
	
	return vfdSendInfo(addr, tmp, 24);
}

uint8_t curX = 0;
uint8_t curY = 0;
char *lines[2] = {"12345678901231231222", "ABCASDASDASDASDASDAS"};
uint8_t currentLine = 0;

uint8_t cmdBuff[32];
uint8_t cmdIdx = 0;
uint8_t checksum[2];

uint8_t st = 0;
void blinkLed() {
    if(st == 0) {
        PORTB |= _BV(LED);
        st = 1;
    } else {
        PORTB &= ~_BV(LED);
        st = 0;
    }
}

void processUartRx() {
    uint8_t v = UART0GetByte();
    if(v){
        lines[curY][curX] = v;
        curX++;
        if(curX == 20) {
            curX = 0;
            curY++;
        }
        if(curY == 2) curY = 0;
    }
}

void printCmdBuff() {
    printf("DataRX: ");
    for(uint8_t i=0;i<cmdIdx;i++)
        printf("%x ", cmdBuff[i]);
    printf("\n");
}

uint8_t rol=0;
uint8_t lastSent=-1;
uint8_t sync=0;
uint8_t ack=1;

void processCmdBuff() {
    if(cmdBuff[cmdIdx - 2] != checksum[0] || cmdBuff[cmdIdx - 1] != checksum[1]){
        printf("Wrong CRC: %x %x vs Recv %x %x\n", checksum[0], checksum[1], cmdBuff[cmdIdx-2], cmdBuff[cmdIdx-1]);
    }else{
        if(cmdBuff[1] == CMD_ROL){
            printf("ROL OK\n");
            vfdSNRM(0);
        }
        if(cmdBuff[1] == CMD_NSA){
            rol = 1;
            sentFrames[0] = 0;
            recvFrames[0] = 0;
        }
        if((cmdBuff[1]&CMD_RR_MASK) == CMD_RR){
            printf("RR R=%d ", (cmdBuff[1]>>5)&7);
            if(sentFrames[0] != ((cmdBuff[1]>>5)&7)){
                printf("Mismatch S/R\n");
                sentFrames[0] = (cmdBuff[1]>>5)&7;
            }
            if(lastSent == sentFrames[0]){
                ack = 1;
            }
        }else if((cmdBuff[1]&CMD_I_NEG_MASK) == 0){
            printf("Info R=%d S=%d ", (cmdBuff[1]>>5)&7, (cmdBuff[1]>>1)&7);

            if(sentFrames[0] != ((cmdBuff[1]>>5)&7)){
                printf("Mismatch S/R\n");
            }

            recvFrames[0] =  (cmdBuff[1]>>5)&7;

            vfdSendRR(0);

            if(cmdBuff[2] == 0x10)
                sync = 1;
        }
    }
}

void waitForMessageEnd() {
    while(1) {
        rxRead = (rxRead + 1)&31; // Advance 1 character
        while(rxRead == rxPos); // If no more bytes received, wait

        if(rxBff[rxRead] == (CMD_FLAGCHAR&0xFF) && rxAddr[rxRead] == 1){
            return;
        }else{
            cmdBuff[cmdIdx++] = rxBff[rxRead];
        }
    }
}

int main(void){
	_delay_ms(100);
	
	gpioInit();
	stdout=UART0Init();

	sei();

	while(1){
        processUartRx();

        if(!rol){ // Start by polling
			printf("InitPoll\n");
			vfdPoll(0);
			_delay_ms(10);
		}
		if(rol && !sync){
			vfdInfoReq(0);
			_delay_ms(10);
		}
		if(sync && ack){
			vfdPrintLine(0, lines[currentLine], currentLine);
			lastSent = sentFrames[0];
			ack = 0;
			currentLine++;
			currentLine = currentLine & 1;
			_delay_ms(20);
			vfdPoll(0);
		}
		
		while(rxRead != rxPos){
			
			if(rxBff[rxRead] == (CMD_EOPCHAR&0xFF) && rxAddr[rxRead] == 1){
				// EOP 
			}else if(rxBff[rxRead] == 0x24 && rxAddr[rxRead] == 1){
				// Start of response
				cmdBuff[0] = 0x24;
				cmdIdx = 1;
                waitForMessageEnd();

                crc16_x25(cmdBuff, cmdIdx-2, checksum);
                printCmdBuff();
                processCmdBuff();
            }
			
			rxRead = (rxRead + 1)&31;
		}

        blinkLed(st);
        _delay_ms(10);
	}
}
