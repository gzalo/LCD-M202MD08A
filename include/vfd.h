#ifndef VFD_H
#define VFD_H

#include <stdint.h>

void vfdInit(void);
void vfdProcessRx(void);
void vfdService(const char lines[2][21]);

void vfdPoll(uint8_t addr);
void vfdSNRM(uint8_t addr);
uint8_t vfdTrig(uint8_t addr, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4);
uint8_t vfdInfoReq(uint8_t addr);
uint8_t vfdPrintLine(uint8_t addr, char *txt, char line);

#endif
