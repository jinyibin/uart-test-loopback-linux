#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#include "datatype.h"
#include "serial.h"

static int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
	B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,};
static int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200, 300,
	115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,};

static void set_speed(int fd, int speed)
{
	int i;
	int status;
	struct termios Opt;
	tcgetattr(fd, &Opt);
	for (i= 0; i < sizeof(speed_arr) / sizeof(int); i++) {
		if (speed == name_arr[i]) {
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if (status != 0) {
				perror("tcsetattr fd1");
				return;
			}
			tcflush(fd,TCIOFLUSH);
		}
	}
}

static int set_parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
	if (tcgetattr(fd, &options) != 0) {
		perror("SetupSerial 1");
		return -1;
	}
	options.c_cflag &= ~CSIZE;
	switch (databits) {
		case 7:
			options.c_cflag |= CS7;
			break;
		case 8:
			options.c_cflag |= CS8;
			break;
		default:
			fprintf(stderr,"Unsupported data size\n");
			return -1;
	}
	switch (parity) {
		case 'n':
		case 'N':
			options.c_cflag &= ~PARENB;
			options.c_iflag &= ~INPCK;
			break;
		case 'o':
		case 'O':
			options.c_cflag |= (PARODD | PARENB);
			options.c_iflag |= INPCK;
			break;
  		case 'e':
		case 'E':
			options.c_cflag |= PARENB;
			options.c_cflag &= ~PARODD;
			options.c_iflag |= INPCK;
			break;
		case 'S':
		case 's':
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			break;
		default:
			fprintf(stderr,"Unsupported parity\n");
			return -1;
	}
	switch (stopbits) {
		case 1:
			options.c_cflag &= ~CSTOPB;
			break;
		case 2:
			options.c_cflag |= CSTOPB;
			break;
		default:
			fprintf(stderr,"Unsupported stop bits\n");
			return -1;
	}
	if (parity != 'n')
		options.c_iflag |= INPCK;
	
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 1;
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE |ECHONL | IEXTEN | ISIG);
	options.c_oflag  &= ~OPOST;
        options.c_iflag  &= ~(IXON | IXOFF);
        options.c_iflag  &= ~(BRKINT | ICRNL | INLCR  | IXANY |IGNCR);
        options.c_oflag  &= ~(ONLCR|OCRNL);
        //tcflush(fd,TCIFLUSH);
	if (tcsetattr(fd,TCSANOW,&options) != 0) {
		perror("SetupSerial 3");
		return -1;
	}

        //tcflush(fd,TCIFLUSH);
	return 0;
}

int serial_open(char *port_name, int speed)
{
	int fd = -1;

	fd = open(port_name, O_RDWR|O_NOCTTY|O_NDELAY);
	if (fd < 0) {
		print_err("Failed to open %s\n", port_name);
		return -1;
	}
	set_speed(fd, speed);
	if (set_parity(fd, 8, 1,'N') < 0) {
		print_err("Set Parity Error\n");
		return -1;
	}
	return fd;
}

void serial_close(int fd)
{
	if(fd > 0)
		close(fd);
}

int serial_write(int fd, unsigned char *buf, int buf_size)
{
	return write(fd, buf, buf_size);
}
