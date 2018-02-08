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
	uint8_t rot : 1; // rotation and sequence flags
	uint8_t seq : 1;
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
	},
	{
		.prefix.cmd = 'u',
		.prefix.pos = 2,
		.strobe = 40000,
		.sequence.offset = 200,
		.sequence.down = 0,
		.sequence.RGB = R + B,
	},
	{
		.prefix.cmd = 'u',
		.prefix.pos = 3,
		.strobe = 30000,
		.sequence.offset = 360,
		.sequence.down = 1,
		.sequence.RGB = G + R,
	},
	{
		.prefix.cmd = 'u',
		.prefix.pos = 4,
		.strobe = 55000,
		.sequence.offset = 100,
		.sequence.down = 1,
		.sequence.RGB = G,
	},
	{
		.prefix.cmd = 'u',
		.prefix.pos = 5,
		.strobe = 45000,
		.sequence.offset = 400,
		.sequence.down = 0,
		.sequence.RGB = B,
		.sequence.end = 1,
	},
	{
		.prefix.cmd = 'E',
		.prefix.pos = 5,
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

double l_log_deg(double ang)
{
	double tmp;
	tmp = 360.0 * log10(ang);
	if (tmp < 0.01)
		return 0.01;
	if (tmp > 360.0)
		return 360.0;
	return tmp;
}

/*
 * 
 */
int main(int argc, char** argv)
{
	s.n = 1;
	s.fd = open_port();
	if (s.fd == -1)
		return(EXIT_FAILURE);
	/*
	 * Set options for the port...
	 */
	s.deg = timer_deg_count / 360;
	s.rad = timer_deg_count / (2.0 * _PI);
	printf("\r\n %f timer counts per degree %f, counts per radian %f : counts %i ", timer_deg_count, s.deg, s.rad, (int) deg_counts(45.0));

	/* set port raw and speed options */
	tcgetattr(s.fd, &options);
	cfsetispeed(&options, B19200);
	cfsetospeed(&options, B19200);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	options.c_oflag &= ~(OPOST);
	options.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN); /* Clear ICANON and ECHO. */
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 0;
	tcsetattr(s.fd, TCSANOW, &options);
	tcflush(s.fd, TCIOFLUSH);

	write(s.fd, init_string, 8); // send init, info string and line data

	l_pos_send(s.fd, sequ[0]);
	l_pos_send(s.fd, sequ[1]);
	l_pos_send(s.fd, d_sequ[2]);
	l_pos_send(s.fd, d_sequ[3]);
	l_pos_send(s.fd, d_sequ[4]);
	l_pos_send(s.fd, d_sequ[5]);
	l_pos_send_cmd(s.fd, d_sequ[6]);
	sleep(3);

	do {
		// state calc
		s.pos[0] = 10.0 + ((double) s.n * 0.24);
		s.pos[1] = 45.00 + ((double) s.n * 0.15);
		s.pos[2] = l_log_deg((double) s.n / 100.0);
		s.strobe[0] = deg_counts(s.pos[0]);
		s.strobe[1] = deg_counts(s.pos[1]);
		s.strobe[2] = deg_counts(s.pos[2]);
		printf("\r\n %i %i %i : %f %f %f", (int) s.strobe[0], (int) s.strobe[1], s.strobe[2], s.pos[0], s.pos[1], s.pos[2]);

		// transmit state
		d_sequ[0].strobe = s.strobe[0];
		d_sequ[1].strobe = s.strobe[1];

		//        d_sequ[0].strobe = (uint16_t) deg_counts(9.0);
		//        d_sequ[1].strobe = (uint16_t) deg_counts(61.0);

		l_pos_send(s.fd, d_sequ[0]);
		l_pos_send(s.fd, d_sequ[1]);

	} while (++s.n < 1000);

	sleep(1);
	close(s.fd);
	return(EXIT_SUCCESS);
}

