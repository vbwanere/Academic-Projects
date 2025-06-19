name = output
device = /dev/ttyACM0
src_dir = src
inc_dir = inc
src = $(src_dir)/main.c $(src_dir)/uart.c

flags = -std=c11 -Os
baud = 9600

printf_min = -Wl,-u,vfprintf -lprintf_min #only int/string support
printf_float = -Wl,-u,vfprintf -lprintf_flt -lm #with float/double support

link_printf = #default
#link_printf = $(printf_min)
#link_printf = $(printf_float)

# Include path for header files
CFLAGS = -I$(inc_dir)

all: build program clean

build: $(src)
	avr-gcc $(flags) -DBAUD=$(baud) -DF_CPU=16000000UL -mmcu=atmega328p \
	$(CFLAGS) $(link_printf) -o $(name).elf $^  # Updated to include $^ to compile both main.c and uart.c
	avr-objcopy -O ihex -R .eeprom $(name).elf $(name).ihex
	rm $(name).elf

list: $(src)
	avr-gcc -c -g -Wa,-a,-ad $(flags) -DBAUD=$(baud) -DF_CPU=16000000UL \
	-mmcu=atmega328p $(CFLAGS) $(src) > $(name).lst
	rm $(name).elf

program: build
	avrdude -F -V -c arduino -p ATMEGA328P -P $(device) -b 115200 \
	-U flash:w:$(name).ihex

clean:
	rm -f $(name).ihex $(name).lst
