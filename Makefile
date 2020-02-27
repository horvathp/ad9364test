SYSROOT = ${HOME}/sysroot_pluto
PKG_CONFIG_PATH=${SYSROOT}/usr/lib/pkgconfig
CC=arm-linux-gnueabihf-gcc
CFLAGS += --sysroot=${SYSROOT} -Wall -Wextra -std=gnu99
LIBS += --sysroot=${SYSROOT} -liio -lm -lpthread -lfftw3

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
	
ad9364test: ad9364test.o ofdm_filters.o cpm_filters.o lte_filters.o
	$(CC) -o ad9364test ad9364test.o cpm_filters.o ofdm_filters.o lte_filters.o $(LIBS)

cal_ad9361: cal_ad9361.o
	$(CC) -o cal_ad9361 cal_ad9361.o $(LIBS)

pluto_power: pluto_power.o
	$(CC) -o pluto_power pluto_power.o $(LIBS)

pluto_power_check: pluto_power_check.o
	$(CC) -o pluto_power_check pluto_power_check.o $(LIBS)

all: pluto_power pluto_power_check cal_ad9361 ad9364test

.PHONY: clean

clean:
	rm -f *.o
	 
