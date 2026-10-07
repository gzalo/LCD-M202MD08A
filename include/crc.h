#ifndef CRC_H
#define CRC_H

#include <stdint.h>

void crc16_x25(uint8_t *data, uint8_t len, uint8_t *crc_out);

#endif
