#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "datatype.h"
#include "control.h"
#include "crc.h"
#include "serial.h"
#include "status.h"
#include "sensor.h"


#define GPS_SENSOR_COM "/dev/ttyLP2"
#define CONTROL_COM "/dev/ttyLP3"
#define HIGHT_SENOR_COM "/dev/ttyS3"


#define GPS_SENSOR 0
#define CONTROL_DEVICE 1
#define HIGH_SENSOR 2


static int gps_fd = -1;
//static int high_fd = -1;
static int control_fd = -1;
static int running = 0;

#define MAX(a,b) (a>b?a:b)

static pthread_t recv_pid;
static void *sensor_data_collect();

int sensor_open()
{
	gps_fd = serial_open(GPS_SENSOR_COM, 115200);
//	high_fd = serial_open(HIGHT_SENOR_COM, 38400);

         control_fd = serial_open(CONTROL_COM, 115200);
//	if (gps_fd < 0 || high_fd < 0 || control_fd < 0) {
	if (gps_fd < 0 || control_fd < 0) {
//		print_err("sensor open failed, gps_fd = %d, high_fd = %d, control_fd = %d\n", gps_fd, high_fd, control_fd);
		print_err("sensor open failed, gps_fd = %d, control_fd = %d\n", gps_fd, control_fd);
		return -1;
	}
	running = 1;
	pthread_create(&recv_pid, NULL, sensor_data_collect, NULL);
	return 0;
}

void sensor_close()
{
	void* result = NULL;
	running = 0;
	pthread_join(recv_pid,&result);
	serial_close(gps_fd);
//	serial_close(high_fd);
	serial_close(control_fd);
}

int control_data_parse(unsigned char *buf, int buf_size)
{
	uint16 crc_val = (buf[buf_size - 3] << 8) | buf[buf_size - 2];
	int waypoint_num = 0;
	uint32 frame_size = buf[4] << 24 | buf[5] <<16 | buf[6] << 8 | buf[7];
	unsigned char frame_type = buf[8];

	print_debug("%s: frame_type is 0x%x, size is %d\n", __FUNCTION__, frame_type, buf_size);
	if (buf[buf_size-1] != 0x4e) {
		print_err("stop byte is wrong 0x%x\n", buf[buf_size-1]);
		return -1;
	}
	if (frame_size != buf_size) {
		print_err("frame size %d is wrong, actual size is %d\n", frame_size, buf_size);
		return -1;
	}
#if CRC_CHECKSUM_ENABLE
	if (crc_val != crc_checksum16(buf, buf_size - 3)) {
		print_err("%s: crc checksum error\n");
		return -1;
	}
#endif
	switch (frame_type) {
		case 0x33:
			update_control_parameter_remote1(buf);
			break;
		case 0x3E:
			update_control_parameter_remote2(buf);
			break;
		case 0xD6:
			update_control_data(buf);
			break;
		case 0x69:
			print_debug("link test count %d received\n", buf[5]);
			set_system_status(SYS_PREPARE_SETTING);
			link_testing_send();
			break;
///////////////////////////////////////////////////////////////////////////////////////
		case 0x0C:
			if(buf[9] == 0xD7) {
				steering_test();
				set_system_status(SYS_PREPARE_TAKEOFF);
				set_flying_status(AIRCRAFT_READY);
			}
			else
				print_err("streering test data wrong? 0x%x\n", buf[9]);
			break;
		case 0x50:
			if(buf[9] == 0xD1) {
				if(get_flying_status() == AIRCRAFT_READY) {
					set_flying_status(AIRCRAFT_TAKEOFF);
				}
				else {
					print_err("aircarft takeoff is not ready %d\n", get_flying_status());
				}
			}
			else
				print_err("aircarft takeoff data wrong? 0x%x\n", buf[9]);
			break;
		case 0x8E:
			if(buf[9] == 0xD3)
				set_flying_status(AIRCRAFT_REMOTE1);
			else
				print_err("aircarft remote mode 1 data wrong? 0x%x\n", buf[9]);
			break;
		case 0xB2:
			if(buf[9] == 0xD4)
				set_flying_status(AIRCRAFT_REMOTE2);
			else
				print_err("aircarft remote mode 2 data wrong? 0x%x\n", buf[9]);
			break;
		case 0x88:
			if(buf[9] == 0xD6)
				set_flying_status(AIRCRAFT_HOVERING);
			else
				print_err("aircarft hovering data wrong? 0x%x\n", buf[9]);
			break;
		case 0xCA:
			if(buf[9] == 0xD5)
				set_flying_status(AIRCRAFT_FLYING);
			else
				print_err("aircarft flying data wrong? 0x%x\n", buf[9]);
			break;
		case 0x5F:
			if(buf[9] == 0xD2)
				set_flying_status(AIRCRAFT_RETURN);
			else
				print_err("aircarft return data wrong? 0x%x\n", buf[9]);
			break;
		case 0x86:
			if(buf[9] == 0xD9)
				data_export();
			else
				print_err("data_export data wrong? 0x%x\n", buf[9]);
			break;
		case 0x80:
			if(buf[9] == 0xD8) {
				control_cmd_confirm();
			}
			else
				print_err("aircarft flying data wrong? 0x%x\n", buf[9]);
			break;
///////////////////////////////////////////////////////////////////////////////////////
		case 0x38:
			set_aircaft_preparing_status(buf);
			aircraft_preparing_response();
			break;
		case 0x8A:
			if (buf_size == 36) {
				if (buf[9] == 0x01)
					waypoint_insert(buf + 12, buf[10] << 8| buf[11]);
				else if (buf[9] == 0x02)
					waypoint_modify(buf + 12, buf[10] << 8| buf[11]);
				else if (buf[9] == 0x04)
					waypoint_delete(buf + 12, buf[10] << 8| buf[11]);
			} else {
				waypoint_num = buf[9] << 8 | buf[10];
				if (buf_size != (waypoint_num * 25 + 14)) {
					print_err("waypoint setting is wrong\n");
					return -1;
				}
				waypoint_init(buf + 11, waypoint_num);
				waypoint_return(buf, buf_size);
			}
			break;
		case 0xFF:
			firmware_upgrade(buf+13,(uint32) *(buf+9));
			break;
		default:
			print_err("unsupport control data received\n");
			break;
	}
	return 0;
}

int serial_data_parse(int type,unsigned char *buf, int buf_size)
{
	if (type == GPS_SENSOR) {
		if (buf_size != 104 ||
			buf[0] != 0xff ||
			buf[1] != 0x02 ||
			buf[3] != 0x00 ||
			buf[4] != 0x60 ||
			buf[103] != 0x03) { //stop byte
			print_err("GPS sensor data error\n");
			return -1;
		}
                printf("%x %x %x %x %x %x \n",buf[0],buf[1],buf[3],buf[4],buf[103]);
		set_flying_attitude(buf);
		if (get_system_status() == SYS_INIT)
			set_system_status(SYS_SENSOR_READY);
//	} else if (type == HIGH_SENSOR) {
	} else if (type == CONTROL_DEVICE) {
		control_data_parse(buf,buf_size);
	}
	return 0;
}

static void *sensor_data_collect()
{
	#define BUF_MAX_SIZE 1024
	int i = 0;
	int nread = 0;
	int maxfd = 0;
	fd_set rfds;
	unsigned char buf[BUF_MAX_SIZE];

	struct timeval tv;

	tv.tv_sec=0;
    tv.tv_usec=400000; //40 ms

	FD_ZERO(&rfds);
	FD_SET(gps_fd, &rfds);
//	FD_SET(high_fd, &rfds);
	FD_SET(control_fd, &rfds);

	maxfd = MAX(gps_fd, control_fd);
	while (running) {
		if (select(1 + maxfd, &rfds, NULL, NULL, &tv) > 0) {
				if (FD_ISSET(gps_fd, &rfds)) {
					nread = read(gps_fd, buf, BUF_MAX_SIZE);
					print_debug("gps sensor read %d bytes\n", nread);
                                        for(i=0;i<nread;i++){
                                           print_debug("%x\n",buf[i]);
                                        }
                                        i=0;
					serial_data_parse(GPS_SENSOR, buf, nread);
//				}	else if(FD_ISSET(high_fd, &rfds)) {
//					nread = read(high_fd, buf, BUFSIZE);
//					print_debug("High sensor read %d bytes\n", nread);
//					serial_data_parse(HIGH_SENSOR, buf, nread);
				} else if (FD_ISSET(control_fd, &rfds)) {
					nread = read(control_fd, buf, BUF_MAX_SIZE);
					print_debug("Control device read %d bytes\n", nread);
					serial_data_parse(CONTROL_DEVICE, buf, nread);
				}
		} else {
			//print_err("Device read timeout\n");// serial port received timeout.
		}
	}
}

int control_cmd_send(uint8 *buf,uint32 buf_size)
{
	return serial_write(control_fd, buf, buf_size);
}

