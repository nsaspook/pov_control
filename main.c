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

#define R 1
#define G 2
#define B 4

typedef struct L_seq {
	uint8_t cmd;
	uint8_t pos;
	uint8_t down : 1; // rotation direction
	uint8_t RGB : 3;
	uint8_t end : 1; // last line in sequence
	uint8_t skip : 1; // don't light led
	uint16_t offset; // line movement 
} L_seq;

/* data for one complete rotation*/
typedef struct L_data {
	struct L_seq sequence;
	uint16_t strobe;
} L_data;

union {
	struct L_data dbuffer;
	uint8_t bbuffer[16];
} lbuffer;

static const L_data sequ[] = {
	{
		.sequence.cmd = 'u',
		.sequence.pos = 0,
		.strobe = 60000,
		.sequence.offset = 360,
		.sequence.down = 1,
		.sequence.RGB = R,
	},
	{
		.sequence.cmd = 'u',
		.sequence.pos = 1,
		.strobe = 50000,
		.sequence.offset = 60,
		.sequence.down = 1,
		.sequence.RGB = G,
	},
};

struct termios options;

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

/*
 * 
 */
int main(int argc, char** argv)
{
	int fd, n;

	fd = open_port();
	if (fd == -1)
		return(EXIT_FAILURE);
	/*
	 * Set options for the port...
	 */

	tcgetattr(fd, &options);
	cfsetispeed(&options, B57600);
	cfsetospeed(&options, B57600);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	tcsetattr(fd, TCSANOW, &options);

	lbuffer.dbuffer = sequ[0];
	write(fd, (uint8_t*) lbuffer.bbuffer, 7);
	write(fd, "U\2\0\0\0\0\0", 7);
	lbuffer.dbuffer = sequ[1];
	write(fd, (uint8_t*) lbuffer.bbuffer, 7);

	close(fd);
	return(EXIT_SUCCESS);
}

