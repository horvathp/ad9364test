SYSROOT = ${HOME}/sysroot_pluto
PKG_CONFIG_PATH=${SYSROOT}/usr/lib/pkgconfig
CC=arm-linux-gnueabihf-gcc
CFLAGS += --sysroot=${SYSROOT} -Wall -Wextra -std=gnu99
LIBS += --sysroot=${SYSROOT} -liio

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
	
ad9364test: ad9364test.o ofdm_filters.o cpm_filters.o
	$(CC) -o ad9364test ad9364test.o cpm_filters.o ofdm_filters.o $(LIBS)

.PHONY: clean

clean:
	rm -f *.o
	 