/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.c
 * Author: root
 *
 * Created on January 8, 2018, 8:02 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <stdint.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <math.h>

#define _PI  3.14159265358979323846 /* pi */

#define G 1
#define R 2
#define B 4

#define timer_max 65535UL
#define timer_offset 24250UL
#define timer_range timer_max-timer_offset
#define timer_slope_deg 114.680556l // timer counts per degree
#define timer_slope_rad 6570.711826l // timer counts per radian

double timer_deg_count = timer_range;

typedef struct S_type {
	uint16_t strobe[16];
	double pos[16], deg, rad;
	int n, fd;
} S_type;

#pragma pack(push,1) // pack it compatiable with the PIC18 and xc8

typedef struct L_prefix {
	uint8_t cmd;
	uint8_t pos;
} L_prefix;

typedef struct L_seq {
	uint8_t down : 1; // rotation direction
	uint8_t RGB : 3;
	uint8_t end : 1; // last line in sequence
	uint8_t skip : 1; // don't light led
	uint16_t offset; // line movement 
} L_seq;

/* data for one complete rotation*/
typedef struct L_data {
	struct L_prefix prefix;
	struct L_seq sequence;
	uint16_t strobe;
} L_data;

union {
	struct L_data dbuffer;
	uint8_t bbuffer[16];
} lbuffer;

static const L_data sequ[] = {
	{
		.prefix.cmd = 'u',
		.prefix.pos = 0,
		.strobe = 60000,
		.sequence.offset = 360,
		.sequence.down = 1,
		.sequence.RGB = R,
	},
	{
		.prefix.cmd = 'u',
		.prefix.pos = 1,
		.strobe = 50000,
		.sequence.offset = 360,
		.sequence.down = 1,
		.sequence.RGB = G,
		.sequence.end = 1,
	},
	{
		.prefix.cmd = 'E',
		.prefix.pos = 1,
	},
};

static L_data d_sequ[] = {
	{
		.prefix.cmd = 'u',
		.prefix.pos = 0,
		.strobe = 60000,
		.sequence.offset = 0,
		.sequence.down = 1,
		.sequence.RGB = R,
	},
	{
		.prefix.cmd = 'u',
		.prefix.pos = 1,
		.strobe = 50000,
		.sequence.offset = 0,
		.sequence.down = 1,
		.sequence.RGB = G + B,
		.sequence.end = 1,
	},
	{
		.prefix.cmd = 'E',
		.prefix.pos = 1,
	},
};

static const uint8_t init_string[] = "zzzzzzzi";

#pragma pack(pop)

struct termios options;
struct S_type s;

int usleep(int);

uint16_t deg_counts(double d)
{
	double val;

	if (d > 0.0) {
		val = timer_max - (d * timer_slope_deg);
	} else {
		val = timer_offset + (d * timer_slope_deg);
	}
	return(uint16_t) val;
}

uint16_t rad_counts(double d)
{
	double val;

	if (d > 0.0) {
		val = timer_max - (d * timer_slope_rad);
	} else {
		val = timer_offset + (d * timer_slope_rad);
	}
	return(uint16_t) val;
}

int open_port(void)
{
	int fd; /* File descriptor for the port */


	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		/*
		 * Could not open the port.
		 */

		perror("open_port: Unable to open /dev/ttyS0 - ");
	} else
		fcntl(fd, F_SETFL, 0);

	return(fd);
}

int l_pos_send(int fd, L_data l)
{
	int ret;

	lbuffer.dbuffer = l;
	//	lbuffer.bbuffer[0]=0x0f;
	//	lbuffer.bbuffer[1]=0xff;
	ret = write(fd, (uint8_t*) lbuffer.bbuffer, 7);
	usleep(15000);
	tcflush(fd, TCIOFLUSH);
	return ret;
}

int l_pos_send_cmd(int fd, L_data l)
{
	int ret;

	lbuffer.dbuffer = l;
	ret = write(fd, (uint8_t*) lbuffer.bbuffer, 2);
	usleep(15000);
	tcflush(fd, TCIOFLUSH);
	return ret;
}

/*
 * 
 */
int main(int argc, char** argv)
{
	s.n = 0;
	s.fd = open_port();
	if (s.fd == -1)
		return(EXIT_FAILURE);
	/*
	 * Set options for the port...
	 */
	s.deg = timer_deg_count / 360;
	s.rad = timer_deg_count / (2.0 * _PI);
	printf("\r\n %f timer counts per degree %f, counts per radian %f : counts %i ", timer_deg_count, s.deg, s.rad, (int) deg_counts(45.0));

	tcgetattr(s.fd, &options);
	cfsetispeed(&options, B19200);
	cfsetospeed(&options, B19200);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	tcsetattr(s.fd, TCSANOW, &options);

	write(s.fd, init_string, 8); // send init and info string

	l_pos_send(s.fd, sequ[0]);
	l_pos_send(s.fd, sequ[1]);
	l_pos_send_cmd(s.fd, sequ[2]);
	sleep(3);

	do {
		// state calc
		s.pos[0] = 1.0 + ((double) s.n * 0.09);
		s.pos[1] = 45.01 + ((double) s.n * 0.05);
		s.strobe[0] = deg_counts(s.pos[0]);
		s.strobe[1] = deg_counts(s.pos[1]);
		printf("\r\n %i %i  %f %f", (int) s.strobe[0], (int) s.strobe[1], s.pos[0], s.pos[1]);

		// transmit state
		d_sequ[0].strobe = s.strobe[0];
		d_sequ[1].strobe = s.strobe[1];

		//        d_sequ[0].strobe = (uint16_t) deg_counts(0.0);
		//        d_sequ[1].strobe = (uint16_t) deg_counts(360.0);

		l_pos_send(s.fd, d_sequ[0]);
		l_pos_send(s.fd, d_sequ[1]);

	} while (++s.n < 1450);

	sleep(1);
	close(s.fd);
	return(EXIT_SUCCESS);
}

