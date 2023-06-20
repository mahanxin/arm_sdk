#include "modbusHandler.h"

namespace ARM {

modbusHandler::modbusHandler( /* args */ ) {
    has_received_data = true;
}

modbusHandler::~modbusHandler() {}

uint16_t modbusHandler::crc16( uint8_t* buffer, uint16_t length ) {
    uint16_t temp = 0xffff;
    uint16_t i = 0, k = 0;
    while ( k < length ) {
        temp = temp ^ buffer[ k ];
        i    = 0;
        while ( i < 8 ) {
            if ( ( temp & 0x01 ) == 0 ) {
                temp = temp >> 1;
                i++;
            }
            else {
                temp = temp >> 1;
                temp = temp ^ 0xa001;
                i++;
            }
        }
        k++;
    }
    return temp;
}

template < typename T > void modbusHandler::multyBytesSeprate( T src, uint8_t* dest ) {
    int16_t len = sizeof( T );
    for ( int16_t i = 0; i < len; i++ ) {
        dest[ i ] = ( src >> ( ( len - 1 - i ) * 8 ) ) & 0xff;
    }
}

template < typename T > void modbusHandler::multyBytesCompose( uint8_t* src, T* dest ) {
    int16_t len = sizeof( T );
    *dest       = 0;
    for ( int16_t i = 0; i < len; i++ ) {
        *dest += src[ i ] * pow( 256, len - 1 - i );
    }
}

bool modbusHandler::sendAndReadPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data, bool isBlock ) {
    read_success = false;

    has_received_data = true;
    // send packet
    int16_t wait_iter = 0;
    while ( !read_success && wait_iter < 2 ) {
        sendPacket( id, func, register_add, data );
        if ( has_received_data )
            // read packet
            read_success = readPacket( isBlock );
        else {
            read_success = true;
            break;
        }
        wait_iter++;
    }
    if ( !read_success )
        printf( "[Communicate] Reading packet failed\n" );
    return read_success;
}

bool modbusHandler::sendAndReadPacket( uint8_t func, uint16_t register_add, uint32_t data, bool isBlock ) {
    // read_success = false;
    // // send packet
    // while (!read_success) {
    //   sendPacket(device_id, func, register_add, data);
    //   // read packet
    //   read_success = readPacket(isBlock);
    // }
    // return read_success;
    return sendAndReadPacket( device_id, func, register_add, data, isBlock );
}

bool modbusHandler::SendAndNotReadPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data[ 6 ], bool isBlock ) {
    read_success      = false;
    has_received_data = false;
    // send packet
    int16_t wait_iter = 0;
    while ( !read_success && wait_iter < 2 ) {
        SendPacket( id, func, register_add, data );
        if ( has_received_data )
            // read packet
            read_success = readPacket( isBlock );
        else {
            read_success = true;
            break;
        }
        wait_iter++;
    }
    if ( !read_success )
        printf( "[Communicate] Reading packet failed\n" );
    return read_success;
}

uint32_t modbusHandler::getResultData() {
    if ( read_success )
        return _read_packet.payload;
}

void modbusHandler::sendPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data ) {
    // set packet
    _write_packet.device_add   = id;
    _write_packet.func_type    = func;
    _write_packet.register_add = register_add;
    _write_packet.payload      = data;

    uint8_t tx_buff[ pack_len ] = { 0x00 };

    // copy front 2 bytes
    memcpy( tx_buff, &_write_packet, 2 );
    // register_add, 2bytes
    multyBytesSeprate( _write_packet.register_add, &tx_buff[ 2 ] );
    // payload data, 4bytes
    multyBytesSeprate( _write_packet.payload, &tx_buff[ 4 ] );

    // calu checksum
    _write_packet.check_sum = crc16( tx_buff, pack_len - 2 );

    // check_sum, 2bytes
    multyBytesSeprate( _write_packet.check_sum, &tx_buff[ 8 ] );

    // flush and write data on serial
    tcflush( _port->fd(), TCIOFLUSH );
    _port->WriteData( ( char* )tx_buff, pack_len );
    // for (int i = 0; i < pack_len; i++)
    //   printf("%x  ", tx_buff[i]);
    // printf("\n");

    usleep( 10 );
}

void modbusHandler::SendPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data[ 6 ] ) {
    static int8_t pack_length = 30;
    // set packet
    write_packet_all_joints_.device_add   = id;
    write_packet_all_joints_.func_type    = func;
    write_packet_all_joints_.register_add = register_add;
    for ( int8_t i = 0; i < 6; i++ )
        write_packet_all_joints_.payload[ i ] = data[ i ];

    uint8_t tx_buff[ pack_length ] = { 0x00 };

    // copy front 2 bytes
    memcpy( tx_buff, &write_packet_all_joints_, 2 );
    // register_add, 2bytes
    multyBytesSeprate( write_packet_all_joints_.register_add, &tx_buff[ 2 ] );
    // payload data, 4bytes
    for ( int8_t i = 0; i < 6; i++ )
        multyBytesSeprate( write_packet_all_joints_.payload[ i ], &tx_buff[ 4 + 4 * i ] );

    // calu checksum
    write_packet_all_joints_.check_sum = crc16( tx_buff, pack_length - 2 );

    // check_sum, 2bytes
    multyBytesSeprate( write_packet_all_joints_.check_sum, &tx_buff[ 4 + 4 * 6 ] );

    // flush and write data on serial
    tcflush( _port->fd(), TCIOFLUSH );
    _port->WriteData( ( char* )tx_buff, pack_length );
    // for (int i = 0; i < pack_length; i++)
    //   printf("%x  ", tx_buff[i]);
    // printf("\n");

    usleep( 10 );
}

bool modbusHandler::readPacket( bool isBlock ) {
    // read data
    uint8_t rx_buff[ pack_len ] = { 0x00 };
    int     err_num             = 0;

    while ( true ) {
        if ( err_num >= 10000 && err_num % 10000 == 0 ) {
            std::cerr << "[Serial] too many err char when waiting for header " << ( int )rx_buff[ 0 ] << ", " << ( int )rx_buff[ 1 ] << std::endl;

            if ( isBlock == false ) {
                return false;
            }
        }
        if ( _port->ReadData( rx_buff, 1, isBlock ) == 0 )
            return false;
        if ( rx_buff[ 0 ] == _write_packet.device_add ) {
            if ( _port->ReadData( &rx_buff[ 1 ], 1, isBlock ) == 0 )
                return false;
            if ( rx_buff[ 1 ] == _write_packet.func_type ) {
                _read_packet.device_add = rx_buff[ 0 ];
                _read_packet.func_type  = rx_buff[ 1 ];
                break;
            }
            else
                err_num++;
        }
        else
            err_num++;
        usleep( 10 );
    }
    // find header success
    if ( _port->ReadData( &rx_buff[ 2 ], pack_len - 2, isBlock ) == 0 )
        return false;

    // for (int i = 0; i < pack_len; i++)
    //   printf("%x  ", rx_buff[i]);
    // printf("\n");
    // check sum
    multyBytesCompose( &rx_buff[ pack_len - 2 ], &_read_packet.check_sum );

    if ( _read_packet.check_sum == crc16( rx_buff, pack_len - 2 ) ) {
        // read func_type, 2 bytes
        multyBytesCompose( &rx_buff[ 2 ], &_read_packet.register_add );
        // read payload data, 4 bytes
        multyBytesCompose( &rx_buff[ 4 ], &_read_packet.payload );

        return true;
    }
    else {
        std::cout << "[Serial] checksum is wrong, received data is :" << _read_packet.check_sum << " calu checksum is: " << crc16( rx_buff, pack_len - 2 ) << std::endl;

        return false;
    }
}

}  // namespace ARM
