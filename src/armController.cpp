#include "armController.h"

namespace ARM {

armController::armController( /* args */ ) {
    // set serial port
    _serial_port.OpenPort( ARM_COM_device, 1, 2250000 );

    // set interface data
    _jointsData.resizeDim( JOINTSNUM );
    _jointsCmd.resizeDim( JOINTSNUM );
    // set jointHandler
    _jointHandlers.resize( JOINTSNUM );
    _jointHandlers.at( 0 ) = new jointHandler( 0x01 );
    _jointHandlers.at( 1 ) = new jointHandler( 0x02 );
    _jointHandlers.at( 2 ) = new jointHandler( 0x03 );
    _jointHandlers.at( 3 ) = new jointHandler( 0x04 );
    _jointHandlers.at( 4 ) = new jointHandler( 0x05 );
    _jointHandlers.at( 5 ) = new jointHandler( 0x06 );
    for ( int i = 0; i < JOINTSNUM; i++ )
        // transfer serial port
        _jointHandlers.at( i )->setSerialPort( &_serial_port );
    general_joints_handler_ = new jointHandler( 0x00 );
    general_joints_handler_->setSerialPort( &_serial_port );
}

armController::~armController() {
    _serial_port.ClosePort();
}

void armController::zeroCommand() {
    _jointsCmd.zeroCmd();
}

void armController::initMotors() {
    std::cout << "[Init] start to initialize motor !" << std::endl;
    // check motor error state
    getErrorState();
    std::cout << "[Init] error state is: ";
    for ( int i = 0; i < JOINTSNUM; i++ )
        std::cout << _jointsData.error_mode.at( i ) << " ";
    std::cout << std::endl;
    setErrorClear();
    std::cout << "[Init] finished clear error state !" << std::endl;

    int32_t joint_pos_offset[ JOINTSNUM ];
    std::cout << "joint pos offset: ";
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        // and get zero point offset
        _jointHandlers.at( i )->getZeroOffset();
        // set acc and dec
        // _jointHandlers.at( i )->setMotorCtrlAcc( 100 );
        usleep( 10 );
        _jointHandlers.at( i )->ReturnZeroOffset( joint_pos_offset[ i ] );
        usleep( 10 );
        std::cout << "  " << joint_pos_offset[ i ];
    }
    std::cout << std::endl;
    // transfer to general joints handler
    general_joints_handler_->SetZeroOffsetForGeneraljoints( joint_pos_offset );

    std::cout << "[Init] finished getting zero offset and  setting acc !" << std::endl;
    // set pid, factory reset
    // int32_t kp_[JOINTSNUM] = {2000, 3000, 2000, 2000, 2000, 2000};
    // int32_t ki_[JOINTSNUM] = {20, 25, 20, 50, 50, 50};
    // int32_t kd_[JOINTSNUM] = {2000, 2500, 2000, 3000, 2500, 2500};
    // first version
    // int32_t kp_[JOINTSNUM] = {1200, 600, 600, 600, 600, 600};
    // int32_t ki_[JOINTSNUM] = {10, 10, 10, 10, 10, 10};
    // int32_t kd_[JOINTSNUM] = {2000, 2500, 2000, 3000, 2000, 2500};
    // debug
    int32_t kp_[ JOINTSNUM ] = { 1200, 1200, 600, 600, 600, 1000 };
    int32_t ki_[ JOINTSNUM ] = { 20, 20, 20, 20, 20, 20 };
    int32_t kd_[ JOINTSNUM ] = { 1000, 1000, 500, 500, 500, 500 };
    setPID( kp_, ki_, kd_ );
    std::cout << "[Init] finished setting pid !" << std::endl;
    // get pid
    getPID();
    std::cout << "[Init] set joints pid :" << std::endl;
    for ( uint8_t i = 0; i < 6; i++ ) {
        std::cout << "joint: " << ( int )i << "  pid: " << _jointsData.kp.at( i ) << "  " << _jointsData.ki.at( i ) << "  " << _jointsData.kd.at( i ) << std::endl;
    }

    // command set to be zero;
    zeroCommand();
    std::cout << "[Init] finished setting command zero !" << std::endl;
    // set enable
    setEnabled( true );
    std::cout << "[Init] finished setting motor enable !" << std::endl;
}
void armController::updateData( ArmJointsState* data ) {
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        // get pos
        // _jointHandlers.at(i)->getMotorUpdateData(&_jointsData.q[i], false);
        // get pos vel
        _jointHandlers.at( i )->getMotorUpdateData( &_jointsData.q[ i ], &_jointsData.qd[ i ] );
        // get pos vel and cur
        // _jointHandlers.at(i)->getMotorUpdateData(
        //     &_jointsData.q[i], &_jointsData.qd[i], &_jointsData.tau[i]);
        // get error mode
        // _jointsData.error_mode.at(i) = _jointHandlers.at(i)->getMotorErrorMode();
        usleep( 1 );
    }
    // update for external data
    data->q  = _jointsData.q;
    data->qd = _jointsData.qd;
    // data->tau = _jointsData.tau;

    // data->error_mode = _jointsData.error_mode;
}

void armController::updateAbsPosition( ArmJointsState* data ) {
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        // get pos vel and cur
        _jointHandlers.at( i )->getMotorUpdateData( &_jointsData.q[ i ], true );
        // usleep( 1 );
    }
    // update for external data
    data->q = _jointsData.q;
}

void armController::updateCommand( ArmJointsCommand* _cmd, bool with_vel ) {
    // receive cmd
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        _jointsCmd.q_des[ i ]  = _cmd->q_des[ i ];
        _jointsCmd.qd_des[ i ] = 1.0 * _cmd->qd_des[ i ];
    }
    // set command
#ifndef USE_GENERAL_JOINTS_HANDLER
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        if ( with_vel ) {
            // set pos vel and acc
            _jointHandlers.at( i )->setMotorPosCtrlCmd( _jointsCmd.q_des[ i ], _jointsCmd.qd_des[ i ], 0.0 );
            // set pos and vel
            // _jointHandlers.at(i)->setMotorPosCtrlCmd(_jointsCmd.q_des[i],
            //                                          _jointsCmd.qd_des[i]);
        }
        else {
            // set only pos
            _jointHandlers.at( i )->setMotorPosCtrlCmd( _jointsCmd.q_des[ i ] );
        }
        usleep( 1 );
    }
#else
    static double des_pos_joint[ 6 ];
    for ( int8_t i = 0; i < JOINTSNUM; i++ )
        des_pos_joint[ i ] = _jointsCmd.q_des[ i ];
    general_joints_handler_->setMotorPosCtrlCmd( des_pos_joint );
    usleep( 10 );
#endif
}

void armController::setBaudrate( uint8_t type ) {
    // set baudrate type
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        _jointHandlers.at( i )->setMotorBaudrate( type );
        usleep( 1000 );
        // save params
        _jointHandlers.at( i )->setSaveParams();
        usleep( 1000000 );
        std::cout << "motor: " << i + 1 << " has successed to change baudrate type!" << std::endl;
    }
}

void armController::getBaudrate() {
    for ( int i = 0; i < JOINTSNUM; i++ )
        _jointHandlers.at( i )->getMotorBaudrate( &_jointsData.baud_type.at( i ) );
}

void armController::setEnabled( bool enabled ) {
    for ( int i = 0; i < JOINTSNUM; i++ )
        _jointHandlers.at( i )->setMotorEnabled( enabled );
}

void armController::setEnabled( bool enabled, int8_t id ) {
    _jointHandlers.at( id )->setMotorEnabled( enabled );
}

void armController::getErrorState() {
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        _jointsData.error_mode.at( i ) = _jointHandlers.at( i )->getMotorErrorMode();
        usleep( 10 );
    }
}

void armController::setErrorClear() {
    for ( int i = 0; i < JOINTSNUM; i++ ) {
        if ( _jointsData.error_mode.at( i ) != 0 )
            _jointHandlers.at( i )->setClearMotorError();
        usleep( 10 );
    }
}

void armController::setRebackOriginalpoint() {
    for ( int i = 0; i < JOINTSNUM; i++ )
        _jointHandlers.at( i )->setRebackOriginalpoint();
}

void armController::setPID( int32_t* kp_des, int32_t* ki_des, int32_t* kd_des ) {
    for ( int i = 0; i < JOINTSNUM; i++ )
        _jointHandlers.at( i )->setMotorPosCtrlPID( kp_des[ i ], ki_des[ i ], kd_des[ i ] );
}
void armController::getPID() {
    for ( int i = 0; i < JOINTSNUM; i++ )
        _jointHandlers.at( i )->getMotorPosCtrlPID( &_jointsData.kp.at( i ), &_jointsData.ki.at( i ), &_jointsData.kd.at( i ) );
}

}  // namespace ARM
