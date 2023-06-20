/*!
 * @file modbusHandler.h
 * @brief A handler class to control communication by serial RS485
 *
 * This file contains: data packet structure, register address, function
 * address, a serial class to control write and read motor's data
 *
 * Authored by HanxinMa, 2022-12-10
 */
#ifndef MODBUS_HANDLER
#define MODBUS_HANDLER

#include <cmath>
#include <iostream>
#include <vector>

#include "modbus.h"
#include "serial.h"

namespace ARM {

typedef struct {
    uint8_t  device_add;
    uint8_t  func_type;
    uint16_t register_add;
    uint32_t payload;
    uint16_t check_sum;
} rs485_packet;

typedef struct {
    uint8_t  device_add;
    uint8_t  func_type;
    uint16_t register_add;
    uint32_t payload[ 6 ];
    uint16_t check_sum;
} Rs485PacketAllJoints;

enum RegisterAddress {
    DEVICE_BAUDRATE                    = 0x01,
    DEVICE_ADDRESS                     = 0x02,
    MOTOR_TMP                          = 0x03,
    MOTOR_IS_ENABLED                   = 0x10,
    MOTOR_POSITION_INPUT               = 0x13,
    MOTOR_VELOCITY_INPUT               = 0x14,
    MOTOR_POSITION_OUTPUT              = 0x15,
    MOTOR_CURRENT                      = 0x19,
    MOTOR_MOTION_PROCESS               = 0x1A,
    MOTOR_ERROR_FALG                   = 0x1B,
    MOTOR_POSCTRL_KP                   = 0x20,
    MOTOR_POSCTRL_KI                   = 0x21,
    MOTOR_POSCTRL_KD                   = 0x22,
    MOTION_POSCTRL_GOAL_ACC            = 0x27,
    MOTION_POSCTRL_GOAL_DEC            = 0x28,
    DEVICE_SAVE_PARAMS                 = 0x2D,
    MOTOR_POSCTRL_GOAL_VEL             = 0x2E,
    MOTOR_POSCTRL_GOAL_POS_WITHOUT_VEL = 0x81,
    MOTOR_POSCTRL_GOAL_POS_WITH_VEL    = 0x82,
    MOTOR_VELCTRL_FOAL_VEL             = 0x2F,
    MOTOR_CURCTRL_GOAL_CUR_            = 0x30,
    MOTOR_SET_ORIGINAL_POINT           = 0x31,
    MOTOR_REBACK_ORIGINAL_POINT        = 0x32,
    MOTOR_MOTION_STOP_PROCESS          = 0x33,

    MOTOR_POSCTRL_GOAL_POS_WITHOUT_VEL_ALL_DEVICES = 0x90
};

enum FunctionAdress { MOTOR_READ = 0x03, MOTOR_WRITE = 0x06 };

class modbusHandler {
private:
    rs485_packet _read_packet;
    rs485_packet _write_packet;
    uint16_t     pack_len = sizeof( rs485_packet ) - 2;
    Serial*      _port;

    Rs485PacketAllJoints write_packet_all_joints_;

    bool    read_success;
    uint8_t device_id;
    bool    has_received_data;

public:
    modbusHandler( /* args */ );
    ~modbusHandler();
    uint16_t crc16( uint8_t* buffer, uint16_t length );

    template < typename T > void multyBytesSeprate( T src, uint8_t* dest );
    template < typename T > void multyBytesCompose( uint8_t* src, T* dest );

    void setSerialPort( Serial* port ) {
        _port = port;
    }
    void setDeviceAdd( uint8_t id ) {
        device_id = id;
    }
    bool sendAndReadPacket( uint8_t func, uint16_t register_add, uint32_t data, bool isBlock );
    bool sendAndReadPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data, bool isBlock );
    void sendPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data );
    bool readPacket( bool isBlock );

    bool SendAndNotReadPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data[ 6 ], bool isBlock );
    void SendPacket( uint8_t id, uint8_t func, uint16_t register_add, uint32_t data[ 6 ] );

    uint32_t getResultData();

    void setIfHasReceivedData( bool flag ) {
        has_received_data = flag;
    }

    // void readMotorData(unsigned char id, unsigned char func);
};

}  // namespace ARM

#endif