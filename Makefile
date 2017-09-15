.PHONY: all clean

all:
	make -C rom
	make -C simsrc -f Makefile.cygwin

clean:
	make -C simsrc -f Makefile.cygwin allclean
	make -C rom clean
	rm -f *.log
