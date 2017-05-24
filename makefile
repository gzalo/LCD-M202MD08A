all:
	avr-gcc -Wall -O2 -mmcu=atmega328p -c main.c -o main.o
	avr-gcc -Wall -O2 -mmcu=atmega328p -o main.elf main.o

	avr-objcopy -O ihex -R .eeprom main.elf main.hex

run:
	E:\prgm\avrdude-usbasp-pdi-bin\avrdude.exe -p atmega328p -P com3 -c arduino -b 57600 -U flash:w:main.hex
