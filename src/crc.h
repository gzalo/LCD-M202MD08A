
uint16_t crc_x25_update (uint16_t crc, uint8_t data){
	uint8_t i;

	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++){
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}

	return crc;
}

unsigned char crc_reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
	
void crc16_x25(uint8_t *data, uint8_t len, uint8_t *crc_out){
	uint16_t crc = 0xFFFF;
	
	for(uint16_t i=0;i<len;i++)
		crc = crc_x25_update(crc, crc_reverse(data[i]));
	
	crc ^= 0xFFFF;
	
	crc_out[0] = crc_reverse(crc>>8);
	crc_out[1] = crc_reverse(crc&0xFF);
}
