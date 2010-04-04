# ATetherBOOT Severin Smith, 4.4.2010
# Adapted from Arduino Makefile (ATmegaBOOT) by E.Lins, 18.7.2005

# Designed for atmega328p in conjuction with WIZNET W5100.

# Instructions
#
# To make bootloader .hex file:
# make
#
# To burn bootloader .hex file:
# make flash
#
# To burn fuses:
# make fuse

# program name should not be changed...
PROGRAM    = ATetherBOOT

# enter the parameters for the avrdude isp tool
ISPTOOL	   = usbasp
ISPPORT	   = usb
ISPSPEED   =
BITCLOCK   = 5

# chip config
MCU_TARGET = atmega328p

AVR_FREQ = 20000000L

HFUSE = D8
LFUSE = FF
EFUSE = 07

LDSECTION  = --section-start=.text=0x7800

# the efuse should really be 0xf8; since, however, only the lower
# three bits of that byte are used on the atmega168, avrdude gets
# confused if you specify 1's for the higher bits, see:
# http://tinker.it/now/2007/02/24/the-tale-of-avrdude-atmega168-and-extended-bits-fuses/
#
# similarly, the lock bits should be 0xff instead of 0x3f (to
# unlock the bootloader section) and 0xcf instead of 0x0f (to
# lock it), but since the high two bits of the lock byte are
# unused, avrdude would get confused.

ISPFUSES    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) -B $(BITCLOCK) \
-e -u -U lock:w:0x3f:m -U efuse:w:0x$(EFUSE):m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m
ISPFLASH    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) -B $(BITCLOCK) \
-U flash:w:$(PROGRAM).hex -U lock:w:0x0f:m

STK500 = "C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe"
STK500-1 = $(STK500) -e -d$(MCU_TARGET) -pf -vf -if$(PROGRAM).hex \
-lFF -LFF -f$(HFUSE)$(LFUSE) -EF8 -ms -q -cUSB -I200kHz -s -wt
STK500-2 = $(STK500) -d$(MCU_TARGET) -ms -q -lCF -LCF -cUSB -I200kHz -s -wt


OBJ        = $(PROGRAM).o
OPTIMIZE   = -Os -funsigned-char -fno-split-wide-types -fno-inline-small-functions -mcall-prologues -ffunction-sections -fdata-sections

DEFS       = 
LIBS       =

CC         = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
override LDFLAGS       = -Wl,$(LDSECTION) -Wl,--relax -nostartfiles -Wl,-gc-sections
#override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=3'
all: $(PROGRAM).hex
	@echo -----------
	@avr-size $(PROGRAM).hex
	@echo -----------

flash: all
	$(ISPFLASH)

fuse: all
	$(ISPFUSES)

flash-stk500: all
	$(STK500-1)
	$(STK500-2)

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

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
