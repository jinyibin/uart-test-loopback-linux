CROSS_COMPILE:= armv7l-timesys-linux-gnueabi-
ARCH:= arm
CC:= $(CROSS_COMPILE)gcc
LD:= $(CROSS_COMPILE)ld
objects = uarttest.o  serial.o  

uarttest-recv : $(objects)
	$(CC) -o  uarttest-loopback $(objects) -static -lpthread

clean:
	rm uarttest-loopback $(objects)
