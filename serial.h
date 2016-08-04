#ifndef SERIAL_H
#define SERIAL_H

int serial_open(char *port_name, int speed);
void serial_close(int fd);
int serial_write(int fd, unsigned char *buf, int buf_size);

#endif

