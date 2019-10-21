.PHONY: all clean

all: zx82.hex zx82.s19 zx82.bin

%.hex: %.bin
	objcopy -I binary -O ihex $< $@

%.s19: %.bin
	objcopy -I binary -O srec $< $@

%.bin: %.z80
	uz80as $< $@

clean:
	rm -f *.bin *.hex *.lst
