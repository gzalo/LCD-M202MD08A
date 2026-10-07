CC = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS = -Wall -O2 -mmcu=atmega328p -DF_CPU=16000000UL -Iinclude
SOURCES = src/main.c src/uart.c src/vfd.c src/crc.c
OBJECTS = $(SOURCES:.c=.o)

all: main.hex

main.elf: $(OBJECTS)
	$(CC) -mmcu=atmega328p -o $@ $(OBJECTS)

main.hex: main.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

src/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

run: main.hex
	E:\prgm\avrdude-usbasp-pdi-bin\avrdude.exe -p atmega328p -P com11 -c arduino -b 57600 -U flash:w:main.hex
