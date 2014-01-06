#ifndef GYROREADER_H
#define GYROREADER_H

#include <iostream>
#include <fcntl.h>
#include <termios.h>

inline int init_serial_input (char * port) {
  int fd = 0;
  struct termios options;

  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
    return fd;
    fcntl(fd, F_SETFL, 0);    // clear all flags on descriptor, enable direct I/O
    tcgetattr(fd, &options);   // read serial port options
    // enable receiver, set 8 bit data, ignore control lines
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    // disable parity generation and 2 stop bits
    options.c_cflag &= ~(PARENB | CSTOPB);
    // set the new port options
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

inline int serial_read_Nbytes(int fd, int num_bytes){

    if(fd <= 0) {
        printf("Cannot read serial\n");
        return 0;
    }

    unsigned char b1;
    int v = 0;
    int base = 1;
    for(int i = 0; i < num_bytes; i++) {
        read(fd, &b1,1);
        v = (v<<8) + b1;
        base = base<<8;
    }

    if(v > base/2)
        return v-base;
    else
        return v;
}

#endif // GYROREADER_H
