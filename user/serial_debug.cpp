#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stack>
#include <string>
#include <thread>

#include "armController.h"

using namespace std;
using namespace ARM;

int main( int argc, char** argv ) {

    // new arm controller
    armController* arm_ctrl;
    arm_ctrl = new armController();

    ArmJointsState joints_state;
    joints_state.resizeDim( 6 );

    ArmJointsCommand joints_cmd;
    joints_cmd.resizeDim( 6 );

    bool first_run = true;
    while ( true ) {
        if ( first_run ) {
            arm_ctrl->initMotors();

            usleep( 1000 );

            arm_ctrl->updateData( &joints_state );
            // for ( int8_t j = 0; j < 6; j++ ) {
            //     // if( j != 3 )
            //         arm_ctrl->_jointHandlers.at(j)->getMotorUpdateData( &joints_state.q[ j ], &joints_state.qd[ j ] );
            //     usleep(1);
            // }

            std::cout << "[joints pos] " << std::endl;
            for ( int8_t j = 0; j < 6; j++ ) {
                std::cout << " " << joints_state.q.at( j );
            }
            std::cout << std::endl;
            std::cout << std::endl;

            usleep( 100 );
            first_run = false;
        }
        std::cout << "[joints cmd] " << std::endl;
        for ( int8_t j = 0; j < 6; j++ ) {
            joints_cmd.q_des.at( j ) = joints_state.q.at( j );
            std::cout << " " << joints_cmd.q_des.at( j );
        }
        std::cout << std::endl;
        std::cout << std::endl;
        usleep( 100 );
        arm_ctrl->updateCommand( &joints_cmd );
        usleep( 3000 );
    }

    /********** debug SerialPort **********/
    // std::cout<<"debug open source SerialPort!"<<std::endl;
    // Serial serial_port_handler;
    // serial_port_handler.OpenPort( "/dev/ttyUSB0" );

    // uint8_t tx_buff[ 10 ] = { 0x01, 0x03, 0x00, 0x13, 0x00, 0x00, 0x00, 0x02, 0xc5, 0xb6 };
    // uint8_t rx_buff[ 10 ] = { 0x00 };

    // while ( true ) {
    //     serial_port_handler.WriteData( ( char* )tx_buff, 10 );

    //     usleep( 10 );

    //     serial_port_handler.ReadData( rx_buff, 10, true );

    //     printf( "[Read] " );
    //     for ( int8_t i = 0; i < 10; i++ ) {
    //         printf( " %x ", rx_buff[ i ] );
    //     }
    //     printf( "\n" );
    //     usleep(1000);
    // }

    printf( "~ Exit ~\n" );

    // system("pause");
    return true;
}