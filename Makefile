PROGRAM    = atmega_etherboot

ISPTOOL	   = usbasp
ISPPORT	   = usb

MCU_TARGET = atmega328p
LDSECTION  = --section-start=.text=0x7800
AVR_FREQ = 20000000L

ISPFUSES    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) -U lock:w:0x3f:m -U efuse:w:0x$(EFUSE):m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m
ISPFLASH    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) -U flash:w:$(PROGRAM).hex -U lock:w:0x0f:m

HFUSE = D8
LFUSE = FF
EFUSE = 07

OBJ        = $(PROGRAM).o
OPTIMIZE   = -Os

CC         = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ)
override LDFLAGS       = -Wl,$(LDSECTION)

#override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
all: $(PROGRAM).hex W5100.h
	@echo "---------------"
	@avr-size $(PROGRAM).hex
	@echo ""
	@avr-size $(PROGRAM).hex | tail -1 | cut -c1-20 | (read i j&& echo $$((1024 * 2 - (i + j))) "bytes left")
	@echo "---------------"

burn: flash, fuse

fuse: build
	$(ISPFUSES)

flash: build
	$(ISPFLASH)

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
