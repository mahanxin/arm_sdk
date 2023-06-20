#include "jointHandler.h"

namespace ARM {
jointHandler::jointHandler( uint8_t id ) {
    _joint_id      = id;
    _modbusHandler = new modbusHandler();
    _modbusHandler->setDeviceAdd( _joint_id );

    input_zero_pos_encode  = 0;
    output_zero_pos_encode = 0;
    // for old version
    // if ( _joint_id <= 2 )
    //     trans_ratio = 101;
    // else
    //     trans_ratio = 51;
    // for new version, 2023-04-17
    trans_ratio       = 101;
    zero_offset_start = 0.0;
    max_step_pos_     = 1000.0 / 32767 * 2 * PI / trans_ratio;
}

jointHandler::~jointHandler() {}

int32_t jointHandler::readMotorStates( uint16_t register_add ) {
    _read_success = _modbusHandler->sendAndReadPacket( FunctionAdress::MOTOR_READ, register_add, 0x02, true );
    // usleep(10);
    if ( _read_success )
        return ( int32_t )_modbusHandler->getResultData();
}

bool jointHandler::writeMotorCommand( uint16_t register_add, int32_t wirte_data ) {
    _read_success = _modbusHandler->sendAndReadPacket( FunctionAdress::MOTOR_WRITE, register_add, ( uint32_t )wirte_data, true );
    // usleep(10);
    return _read_success;
}

bool jointHandler::writeMotorCommand( uint16_t register_add, uint32_t wirte_data[ 6 ] ) {
    _read_success = _modbusHandler->SendAndNotReadPacket( _joint_id, FunctionAdress::MOTOR_WRITE, register_add, wirte_data, true );
    // usleep(10);
    return _read_success;
}

// set motor enabled state
bool jointHandler::setMotorEnabled( bool enabled ) {
    int32_t cur_state;
    cur_state = readMotorStates( RegisterAddress::MOTOR_IS_ENABLED );
    if ( enabled == ( bool )cur_state ) {
        return true;
    }
    else {
        return writeMotorCommand( RegisterAddress::MOTOR_IS_ENABLED, enabled );
    }
}

// set serial baudrate
bool jointHandler::setMotorBaudrate( uint8_t baud_type ) {
    int32_t cur_state;
    cur_state = readMotorStates( RegisterAddress::DEVICE_BAUDRATE );
    std::cout << "motor: " << ( int )_joint_id << " current baudrate type is: " << cur_state << std::endl;
    if ( baud_type == ( uint8_t )cur_state ) {
        return true;
    }
    else {
        return writeMotorCommand( RegisterAddress::DEVICE_BAUDRATE, baud_type );
        // must operate to save params
        // return writeMotorCommand(RegisterAddress::DEVICE_SAVE_PARAMS, 0x01);
    }
}
// get serial baudrate type
bool jointHandler::getMotorBaudrate( uint8_t* baud_type ) {
    int32_t cur_state;
    cur_state  = readMotorStates( RegisterAddress::DEVICE_BAUDRATE );
    *baud_type = uint8_t( cur_state );
    return _read_success;
}
// save params
bool jointHandler::setSaveParams() {
    return writeMotorCommand( RegisterAddress::DEVICE_SAVE_PARAMS, 0x01 );
}
// get motor error mode
int32_t jointHandler::getMotorErrorMode() {
    return readMotorStates( RegisterAddress::MOTOR_ERROR_FALG );
}
// clear motor error state
bool jointHandler::setClearMotorError() {
    int32_t cur_state;
    cur_state = readMotorStates( RegisterAddress::MOTOR_ERROR_FALG );
    if ( !cur_state )
        return true;
    else
        return writeMotorCommand( RegisterAddress::MOTOR_ERROR_FALG, 0x00 );
}
// reback original point
bool jointHandler::setRebackOriginalpoint() {
    return writeMotorCommand( RegisterAddress::MOTOR_REBACK_ORIGINAL_POINT, 0x01 );
}
// set motor original point
bool jointHandler::setMotorOriginalpoint() {
    int32_t cur_state;
    bool    set_success;
    cur_state = readMotorStates( RegisterAddress::MOTOR_MOTION_PROCESS );
    if ( cur_state ) {
        printf( "[MOTOR] %d Set original point fail! Must stop motion at first!\n", cur_state );
        return false;
    }
    else {
        set_success = writeMotorCommand( RegisterAddress::MOTOR_SET_ORIGINAL_POINT, 0x01 );
        // get encode offset
        input_zero_pos_encode  = readMotorStates( RegisterAddress::MOTOR_POSITION_INPUT );
        output_zero_pos_encode = readMotorStates( RegisterAddress::MOTOR_POSITION_OUTPUT );
        return true;
    }
}
// get zero offset
void jointHandler::getZeroOffset() {
    int32_t inputCurPos, outputCurPos;

    inputCurPos = readMotorStates( RegisterAddress::MOTOR_POSITION_INPUT );

    outputCurPos = readMotorStates( RegisterAddress::MOTOR_POSITION_OUTPUT );

    if ( outputCurPos >= int32_t( 0.5 * output_encode_bound ) )
        outputCurPos = outputCurPos - output_encode_bound;

    input_zero_pos_encode = outputCurPos * trans_ratio - inputCurPos;
}
// get motor cur tmp
int32_t jointHandler::getMotorTemprate() {
    return readMotorStates( RegisterAddress::MOTOR_TMP );
}
// get motor control PID
bool jointHandler::getMotorPosCtrlPID( int32_t* kp, int32_t* ki, int32_t* kd ) {
    int32_t receivedData;
    // get kp
    receivedData = readMotorStates( RegisterAddress::MOTOR_POSCTRL_KP );
    *kp          = receivedData;
    // get ki
    receivedData = readMotorStates( RegisterAddress::MOTOR_POSCTRL_KI );
    *ki          = receivedData;
    // get kd
    receivedData = readMotorStates( RegisterAddress::MOTOR_POSCTRL_KD );
    *kd          = receivedData;
    return true;
}
// set motor control PID
bool jointHandler::setMotorPosCtrlPID( int32_t kp, int32_t ki, int32_t kd ) {
    int16_t  set_success = 0;
    uint32_t requestCmd;
    // set kp
    requestCmd = ( uint32_t )kp;
    requestCmd = requestCmd > 32000 ? 32000 : requestCmd;
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_KP, requestCmd );
    // set ki
    requestCmd = ( uint32_t )ki;
    requestCmd = requestCmd > 32000 ? 32000 : requestCmd;
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_KI, requestCmd );
    // set kd
    requestCmd = ( uint32_t )kd;
    requestCmd = requestCmd > 32000 ? 32000 : requestCmd;
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_KD, requestCmd );
    return set_success = 3 ? true : false;
}

// get motor update data( pos vel cur ) by input encoder
bool jointHandler::getMotorUpdateData( double* pos, double* vel, double* current ) {
    int32_t receivedData;
    // to rad
    receivedData = readMotorStates( RegisterAddress::MOTOR_POSITION_INPUT );
    if ( _read_success )
        *pos = 1.0 * ( receivedData + input_zero_pos_encode ) / input_encode_bound / trans_ratio * 2 * PI;
    // r/min to rad/s
    receivedData = readMotorStates( RegisterAddress::MOTOR_VELOCITY_INPUT );
    if ( _read_success )
        *vel = 1.0 * receivedData / trans_ratio / 9.55;
    // to A
    receivedData = 0.001 * readMotorStates( RegisterAddress::MOTOR_CURRENT );
    if ( _read_success )
        *current = 0.001 * receivedData;
    return true;
}
// get motor update data( pos vel ) by input encoder
bool jointHandler::getMotorUpdateData( double* pos, double* vel ) {
    int32_t receivedData;
    // to rad
    receivedData = readMotorStates( RegisterAddress::MOTOR_POSITION_INPUT );
    if ( _read_success )
        *pos = 1.0 * ( receivedData + input_zero_pos_encode ) / input_encode_bound / trans_ratio * 2 * PI;
    // r/min to rad/s
    receivedData = readMotorStates( RegisterAddress::MOTOR_VELOCITY_INPUT );
    if ( _read_success )
        *vel = 1.0 * receivedData / trans_ratio / 9.55;
    return true;
}
// get motor pos by output encoder Or input encoder
bool jointHandler::getMotorUpdateData( double* pos, bool output_flag ) {
    int32_t receivedData;
    // to rad
    if ( output_flag ) {
        // output abs encoder
        receivedData = readMotorStates( RegisterAddress::MOTOR_POSITION_OUTPUT );
        if ( _read_success ) {
            if ( receivedData < int32_t( 0.5 * output_encode_bound ) )
                *pos = 1.0 * receivedData / output_encode_bound * 2 * PI;
            else
                *pos = 1.0 * ( receivedData - output_encode_bound ) / output_encode_bound * 2 * PI;
        }
    }
    else {
        // input
        receivedData = readMotorStates( RegisterAddress::MOTOR_POSITION_INPUT );
        if ( _read_success )
            *pos = 1.0 * ( receivedData + input_zero_pos_encode ) / input_encode_bound / trans_ratio * 2 * PI;
    }
    return true;
}
// set motor command, pos ctrl ( pos vel acc)
bool jointHandler::setMotorPosCtrlCmd( double pos_des, double vel_des, double acc_des ) {
    uint32_t requestCmd;
    int16_t  set_success = 0;
    // set pos cmd
    requestCmd = uint32_t( 0.5 * ( pos_des ) / PI * trans_ratio * input_encode_bound - input_zero_pos_encode );
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_GOAL_POS_WITH_VEL, requestCmd );
    // set vel cmd
    static double motor_vel_des;
    motor_vel_des = vel_des * 9.55 * trans_ratio;
    motor_vel_des = abs( motor_vel_des ) > 3000 ? motor_vel_des / abs( motor_vel_des ) * 3000 : motor_vel_des;
    if ( motor_vel_des < 1.0 && motor_vel_des >= 0 ) {
        motor_vel_des = 1.0;
    }
    else if ( motor_vel_des > -1.0 && motor_vel_des < 0 ) {
        motor_vel_des = -1.0;
    }

    requestCmd = uint32_t( motor_vel_des );
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_GOAL_VEL, requestCmd );
    // set acc and dec
    requestCmd = acc_des <= 0 ? 100 : uint32_t( vel_des / acc_des * 1000 );
    requestCmd = requestCmd > 1000 ? 1000 : requestCmd;
    requestCmd = requestCmd < 100 ? 100 : requestCmd;
    set_success += writeMotorCommand( RegisterAddress::MOTION_POSCTRL_GOAL_ACC, requestCmd );
    set_success += writeMotorCommand( RegisterAddress::MOTION_POSCTRL_GOAL_DEC, requestCmd );
    return set_success = 4 ? true : false;
}
// set motor command, pos ctrl ( pos vel)
bool jointHandler::setMotorPosCtrlCmd( double pos_des, double vel_des ) {
    uint32_t requestCmd;
    int16_t  set_success = 0;
    // set pos cmd
    requestCmd = uint32_t( 0.5 * ( pos_des ) / PI * trans_ratio * input_encode_bound - input_zero_pos_encode );
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_GOAL_POS_WITH_VEL, requestCmd );
    // set vel cmd
    static double motor_vel_des;
    motor_vel_des = vel_des * 9.55 * trans_ratio;
    motor_vel_des = abs( motor_vel_des ) > 3000 ? motor_vel_des / abs( motor_vel_des ) * 3000 : motor_vel_des;
    if ( motor_vel_des < 1.0 && motor_vel_des >= 0 ) {
        motor_vel_des = 1.0;
    }
    else if ( motor_vel_des > -1.0 && motor_vel_des < 0 ) {
        motor_vel_des = -1.0;
    }

    requestCmd = uint32_t( motor_vel_des );
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_GOAL_VEL, requestCmd );
    return set_success = 2 ? true : false;
}
// set motor command, pos ctrl ( only pos )
bool jointHandler::setMotorPosCtrlCmd( double pos_des ) {
    uint32_t requestCmd;
    int16_t  set_success = 0;
    // set no has received data
    // setReceivedDataFalg(false);
    // set pos cmd
    requestCmd = uint32_t( 0.5 * ( pos_des ) / PI * trans_ratio * input_encode_bound - input_zero_pos_encode );
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_GOAL_POS_WITHOUT_VEL, requestCmd );
    // set no has received data
    // setReceivedDataFalg(true);
    return set_success = 1 ? true : false;
}

// set motor command, pos ctrl ( only pos ), for all joints
bool jointHandler::setMotorPosCtrlCmd( double pos_des[ 6 ] ) {
    uint32_t requestCmd[ 6 ];
    int16_t  set_success = 0;
    // set no has received data
    // setReceivedDataFalg(false);
    // set pos cmd
    for ( int8_t i = 0; i < 6; i++ )
        requestCmd[ i ] = uint32_t( 0.5 * ( pos_des[ i ] ) / PI * trans_ratio * input_encode_bound - record_joints_zero_pos_offset_[ i ] );
    set_success += writeMotorCommand( RegisterAddress::MOTOR_POSCTRL_GOAL_POS_WITHOUT_VEL_ALL_DEVICES, requestCmd );
    // set no has received data
    // setReceivedDataFalg(true);
    return set_success = 1 ? true : false;
}
// set motor acc and dec, time duration
bool jointHandler::setMotorCtrlAcc( int32_t acc ) {
    uint32_t requestCmd;
    int16_t  set_success = 0;
    requestCmd           = uint32_t( acc );
    requestCmd           = requestCmd > 1000 ? 1000 : requestCmd;
    requestCmd           = requestCmd < 100 ? 100 : requestCmd;
    set_success += writeMotorCommand( RegisterAddress::MOTION_POSCTRL_GOAL_ACC, requestCmd );
    set_success += writeMotorCommand( RegisterAddress::MOTION_POSCTRL_GOAL_DEC, requestCmd );
    return set_success = 2 ? true : false;
}
// get motor task state
bool jointHandler::getMotorTaskState() {
    int32_t cur_state;
    cur_state = readMotorStates( RegisterAddress::MOTOR_MOTION_PROCESS );
    return ( bool )cur_state;
}
// set motor stop current task
bool jointHandler::setMotorStopTask() {
    return writeMotorCommand( RegisterAddress::MOTOR_MOTION_STOP_PROCESS, 1 );
}
}  // namespace ARM