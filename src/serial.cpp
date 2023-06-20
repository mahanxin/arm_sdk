#include "serial.h"

#include <iomanip>
#include <iostream>

Serial::Serial() : _fd( -1 ) {}

Serial::~Serial() {}

int Serial::WriteData( const char* data, int datalength ) {
    if ( _fd < 0 ) {
        return -1;
    }
    int len = 0, total_len = 0;  // modify8.
    for ( total_len = 0; total_len < datalength; ) {
        len = write( _fd, &data[ total_len ], datalength - total_len );
        // printf("WriteData fd = %d ,len =%d,data = %s\n",fd,len,data);
        if ( len > 0 ) {
            total_len += len;
        }
        else if ( len <= 0 ) {
            len = -1;
            break;
        }
    }
    return len;
}

int Serial::ReadData( unsigned char* data, int datalength, bool block ) {
    if ( _fd < 0 ) {
        return -1;
    }
    size_t read_count = 0;
    size_t wait_iter  = 0;

    if ( block ) {
        while ( ( int )read_count < datalength ) {
            ssize_t read_result = read( _fd, data + read_count, datalength - read_count );
            if ( read_result < 0 ) {
                printf( "[Serial] failed to receive data!\n" );
                break;
            }
            read_count += read_result;
            wait_iter++;
            if ( wait_iter > 100 ) {
                printf( "[Serial] wait to read data over 100 loops!\n" );
                return 0;
            }
        }
        return datalength;
    }
    else {  // non block
        return read( _fd, data, datalength );
    }
}

void Serial::ClosePort() {
    if ( _fd != -1 ) {
        auto ret_val = close( _fd );
        if ( ret_val != 0 )
            std::cout << "[Serial] failed to close serial!" << std::endl;
        _fd = -1;
    }
}

/**
 * @brief Open the serial and set parameters
 *
 * @param device address of device
 * @param baudrate_type badurate type, 1 is standard, 2 is custom
 * @param speed baudrate
 * @param data_bits number of databits, 5 or 6 or 7 or 8
 * @param stop_bits number of stop bits, 1 or 2
 * @param parity parity bit, enable or disable
 * @return int
 */
int Serial::OpenPort( const char* device, int baudrate_type, int speed, int data_bits, int stop_bits, int parity ) {
    std::cout << "[Serial] begin to open serial common." << std::endl;
    _fd = open( device, O_RDWR | O_NOCTTY );  // O_RDWR | O_NOCTTY | O_NDELAY

    if ( _fd < 0 ) {
        std::cout << "[Serial] failed to open serial common!" << std::endl;
        return -1;
    }

    int ret_val = ConfigureTermios( baudrate_type, speed, data_bits, stop_bits, parity );
    if ( ret_val < -1 ) {
        std::cout << "[Serial] failed to setup serial common!" << std::endl;
        return -1;
    }
    else {
        std::cout << "[Serial] successed to open and setup serial common!" << std::endl;
        return 1;
    }
}

int Serial::ConfigureTermios( int baudrate_type, int speed, int data_bits, int stop_bits, int parity ) {
    struct termios2 tty;

    // Read in the terminal settings using ioctl
    ioctl( _fd, TCGETS2, &tty );

    /********** (.c_cflag) **********/

    // Set num of data bits
    tty.c_cflag &= ~CSIZE;  // CSIZE is a mask for the number of bits per character
    switch ( data_bits ) {
    case 5:
        tty.c_cflag |= CS5;
        break;
    case 6:
        tty.c_cflag |= CS6;
        break;
    case 7:
        tty.c_cflag |= CS7;
        break;
    case 8:
        tty.c_cflag |= CS8;
        break;
    default:
        tty.c_cflag |= CS8;
        break;
    }

    // Set parity
    switch ( parity ) {
    case 0:
        tty.c_cflag &= ~PARENB;  // Clear parity enable
        break;
    case 1:
        tty.c_cflag |= PARENB;   // Enable parity
        tty.c_cflag &= ~PARODD;  // Clearing PARODD
        break;
    case 2:
        tty.c_cflag |= PARENB;  // Enable parity
        tty.c_cflag |= ~PARODD;
        break;
    default:
        tty.c_cflag &= ~PARENB;  // Clear parity enable
        break;
    }

    // Set num. stop bits
    switch ( stop_bits ) {
    case 1:
        tty.c_cflag &= ~CSTOPB;
        break;
    case 2:
        tty.c_cflag |= CSTOPB;
        break;
    default:
        tty.c_cflag &= ~CSTOPB;
        break;
    }

    // Configure hardware flow control, default is off
    tty.c_cflag &= ~CRTSCTS;
    // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_cflag |= CREAD | CLOCAL;

    /********** (baud rate) **********/
    ( void )baudrate_type;
    // custom baudrate,
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= CBAUDEX;
    tty.c_ispeed = speed;
    tty.c_ospeed = speed;

    /********** (.c_oflag) **********/
    tty.c_oflag = 0;        // No remapping, no delays
    tty.c_oflag &= ~OPOST;  // Make raw

    /********** (.c_cc[]) **********/
    // c_cc[VTIME]: sets the inter-character timer, in units of 0.1s.
    //              Only meaningful when port is set to non-canonical mode
    // c_cc[VMIN]: sets the number of characters to block (wait) for when read() is called.
    //             Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode
    // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
    // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
    // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
    // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
    //                      after first character has elapsed
    tty.c_cc[ VTIME ] = 1;
    tty.c_cc[ VMIN ]  = 0;

    /********** (.c_iflag) **********/
    // software flow control, default is off
    tty.c_iflag &= ~( IXON | IXOFF | IXANY );

    tty.c_iflag &= ~( IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL );

    /********** (.c_lflag) **********/
    // Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
    // read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
    tty.c_lflag &= ~( ICANON | ECHO | ECHOE | ECHONL | ISIG );

    // Write terminal settings to file descriptor
    return ioctl( _fd, TCSETS2, &tty );
}