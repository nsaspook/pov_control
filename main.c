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

#define R 1
#define G 2
#define B 4

#define timer_max 65535UL
#define timer_offset 24250UL
#define timer_range timer_max-timer_offset
#define timer_slope 114.680556l // timer counts per degree

double timer_deg_count = timer_range;

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
        .sequence.RGB = G,
    },
    {
        .prefix.cmd = 'E',
        .prefix.pos = 1,
    },
};

static const uint8_t init_string[] = "zzzzzzzi";

#pragma pack(pop)

struct termios options;

uint16_t deg_counts(double d) {
    double val;

    val = timer_max - (d * timer_slope);
    return (uint16_t) val;
}

int open_port(void) {
    int fd; /* File descriptor for the port */


    fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        /*
         * Could not open the port.
         */

        perror("open_port: Unable to open /dev/ttyS0 - ");
    } else
        fcntl(fd, F_SETFL, 0);

    return (fd);
}

/*
 * 
 */
int main(int argc, char** argv) {
    int fd, n;
    double deg;

    fd = open_port();
    if (fd == -1)
        return (EXIT_FAILURE);
    /*
     * Set options for the port...
     */
    deg = timer_deg_count / 360;
    printf("\r\n %f timer counts per degree %f : counts %i ", timer_deg_count, deg, deg_counts(45.0));

    tcgetattr(fd, &options);
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(fd, TCSANOW, &options);

    write(fd, init_string, 8); // send init and info string

    lbuffer.dbuffer = sequ[0];
    write(fd, (uint8_t*) lbuffer.bbuffer, 7);

    lbuffer.dbuffer = sequ[1];
    write(fd, (uint8_t*) lbuffer.bbuffer, 7);

    lbuffer.dbuffer = sequ[2];
    write(fd, (uint8_t*) lbuffer.bbuffer, 2);

    sleep(3);

    d_sequ[0].strobe = deg_counts(270.0);
    d_sequ[1].strobe = deg_counts(90.0);

    lbuffer.dbuffer = d_sequ[0];
    write(fd, (uint8_t*) lbuffer.bbuffer, 7);

    lbuffer.dbuffer = d_sequ[1];
    write(fd, (uint8_t*) lbuffer.bbuffer, 7);

    lbuffer.dbuffer = d_sequ[2];
    write(fd, (uint8_t*) lbuffer.bbuffer, 2);

    close(fd);
    return (EXIT_SUCCESS);
}

