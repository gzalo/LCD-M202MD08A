// Futaba M202MD08A LCD simplified driver for Arduino Pro Mini (ATmega328P).
// Used in IBM 41K6814 20x2 dual VFD Point of Sale displays.
// Released under Public Domain (code provided "as is", without warranty).
// 2017 - Gzalo (Gonzalo Ávila Alterach)
// Thanks to:
// http://www.elektroda.pl/rtvforum/topic558072-90.html
// http://we.easyelectronics.ru/lcd_gfx/vyvodim-informaciyu-na-vfd-ibm-41k6814.html

#define __DELAY_ROUND_CLOSEST__
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "vfd.h"

static char lines[2][21] = {
    "12345678901231231222",
    "ABCASDASDASDASDASDAS"
};

int main(void){
    _delay_ms(100);
    vfdInit();
    uartInit();
    sei();

    uint8_t curX = 0, curY = 0;
    while(1){
        int16_t byte;
        while((byte = uartGetByte()) != UART_NO_DATA){
            lines[curY][curX] = (char)byte;
            if(++curX == 20){
                curX = 0;
                curY = (curY + 1) & 1;
            }
        }

        vfdProcessRx();
        vfdService(lines);
        vfdProcessRx();
        _delay_ms(10);
    }
}
