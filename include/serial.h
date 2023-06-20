#ifndef SERIAL_H
#define SERIAL_H

#include <asm/termbits.h>  // redefine struct termios with <termios.h>, otherwise can't complier success
#include <cstdint>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

class Serial {
public:
    Serial();
    ~Serial();

    int  WriteData( const char* data, int datalength );
    int  ReadData( unsigned char* data, int datalength, bool block = true );
    void ClosePort();

    int OpenPort( const char* device, int baudrate_type, int speed = 115200, int data_bits = 8, int stop_bits = 1, int parity = 0 );
    int ConfigureTermios( int baudrate_type, int speed, int data_bits, int stop_bits, int parity );

    int fd() {
        return _fd;
    }

    operator bool() const {
        return ( _fd > 0 );
    }

private:
    int _fd;
};

#endif
