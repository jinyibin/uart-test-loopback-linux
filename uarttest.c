#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include "serial.h"
#include <pthread.h>

#define FRAME_LEN   104

#define GPS_SENSOR_COM "/dev/ttyO1"




static int gps_fd = -1;
static pthread_t recv_pid;

void mysleep(int ms)  
{  
    struct timeval delay;  
    delay.tv_sec = 0;  
    delay.tv_usec = ms * 1000; // 20 ms  
    select(0, NULL, NULL, NULL, &delay);  
}

void mydelay(int ms_count){
       struct timeval tpStart,tpEnd;
       float timeUse;
       gettimeofday(&tpStart, NULL);
       do {
           gettimeofday(&tpEnd, NULL);
           timeUse = 1000 * (tpEnd.tv_sec - tpStart.tv_sec) + 0.001 * (tpEnd.tv_usec - tpStart.tv_usec);
       } while(timeUse < ms_count);
}
void main(int argc,char *argv[]){
 
	#define BUF_MAX_SIZE 1024
	int i = 0;
	int nread = 0;
	int maxfd = 0;
        int baudrate = 115200;
	fd_set rfds;
        char  *dir;
	unsigned char buf[BUF_MAX_SIZE];

	struct timeval tv;

       if(argc<2){
           printf("uart loopback test bench:baudrate 8N1\n");
           printf("usage: uarttest [baudrate] [/dev/ttyLP2]\n");
           printf(" [baudrate]: 115200\n");
        }
         baudrate = atol(argv[1]);
         dir  = argv[2];

	gps_fd = serial_open(dir, baudrate);

	if (gps_fd < 0 ) {
		printf("serial %s open failed, gps_fd = %d\n", dir,gps_fd);
		return ;
	}

	tv.tv_sec=0;
        tv.tv_usec=400000; //40 ms



	while (1) {
	           FD_ZERO(&rfds);
	           FD_SET(gps_fd, &rfds);
                   memset(buf,0,BUF_MAX_SIZE);
		   if (select(1 + gps_fd, &rfds, NULL, NULL, NULL) > 0) {
				if (FD_ISSET(gps_fd, &rfds)) {
                                        nread = read(gps_fd, buf, BUF_MAX_SIZE);
					//while((nread = read(gps_fd, buf, BUF_MAX_SIZE))>0){
					    //printf("gps sensor read %d bytes\n", nread);
                                            write(gps_fd,buf,nread);
                                       // }
				}	

		} else {
			printf("Device read timeout\n");// serial port received timeout.
		}
	}


        serial_close(gps_fd);
        return ;

}
