.PHONY: all clean

all: zx82.hex zx82.bin

%.hex: %.bin
	objcopy -I binary -O ihex $< $@

%.bin: %.z80
	uz80as $< $@

clean:
	rm -f *.bin *.hex *.lst
