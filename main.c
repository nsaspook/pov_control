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
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

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
	 * Get the current options for the port...
	 */

	tcgetattr(fd, &options);

	/*
	 * Set the baud rates to 19200...
	 */

	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);

	/*
	 * Enable the receiver and set local mode...
	 */

	options.c_cflag |= (CLOCAL | CREAD);

	/*
	 * Set the new options for the port...
	 */

	tcsetattr(fd, TCSANOW, &options);

	n = write(fd, "ATZ\r", 4);
	if (n < 0)
		fputs("write() of 4 bytes failed!\n", stderr);


	close(fd);
	return(EXIT_SUCCESS);
}

