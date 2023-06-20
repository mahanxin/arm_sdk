/*!
 * @file jointHandle.h
 * @brief A handler class to control single joint
 *
 * This file contains: joint params, update data, command,
 * and some operation function for motor,
 * for example: enable、clear error、set original point and son on.
 *
 * Authored by HanxinMa, 2022-12-10
 */
#ifndef JOINT_HANDLER
#define JOINT_HANDLER

#include <iostream>

#include "modbusHandler.h"

namespace ARM {

#define PI 3.1415926

// struct JointState {
//   double q, qd, tau; // tau = current
//   int16_t error_mode;
//   int16_t motor_tmp;
// };

// struct JointCommand {
//   double q_des, qd_des, tau_des;
//   int32_t kp_des, ki_des, kd_des; // if need to write periodly
// };

enum Error_Mode { no_error = 0, under_voltage = 1, over_heat = 2, over_load = 3 };

// struct JointParams {
//   // after change baudrate and address, execute "save"
//   int16_t baud_type, address, tau_max; // tau_max = maxinum current(mA)
//   int32_t kp, ki, kd;
//   int16_t acc_max, dec_max;
//   bool enabled, stop_motion, clear_error;
//   bool reback_original_point, set_original_point;
//   int8_t motion_mode;
// };

class jointHandler {
private:
    /* joint */
    const int32_t input_encode_bound  = 32768;
    const int32_t output_encode_bound = 32768;
    const int32_t kpid_bound          = 32000;
    int32_t       trans_ratio         = 101;
    int32_t       input_zero_pos_encode;
    int32_t       output_zero_pos_encode;

    int32_t record_joints_zero_pos_offset_[ 6 ];

public:
    /* data */
    bool   _read_success;
    double zero_offset_start;
    double max_step_pos_;

    jointHandler( uint8_t id );
    ~jointHandler();
    uint8_t        _joint_id;
    modbusHandler* _modbusHandler;
    // transfer serial port
    void setSerialPort( Serial* port ) {
        _modbusHandler->setSerialPort( port );
    }
    // set joint id
    void setJointId( uint8_t id ) {
        _modbusHandler->setDeviceAdd( _joint_id );
    }
    // read operation
    int32_t readMotorStates( uint16_t register_add );

    // write operation, for one joint
    bool writeMotorCommand( uint16_t register_add, int32_t wirte_data );

    // write operation, for all joints
    bool writeMotorCommand( uint16_t register_add, uint32_t wirte_data[ 6 ] );

    // set if has received data
    void setReceivedDataFalg( bool flag ) {
        _modbusHandler->setIfHasReceivedData( flag );
    }
    // set motor enabled state
    bool setMotorEnabled( bool enabled );
    // set serial baudrate
    bool setMotorBaudrate( uint8_t baud_type );
    // get serial baudrate type
    bool getMotorBaudrate( uint8_t* baud_type );
    // save params
    bool setSaveParams();
    // get motor error mode
    int32_t getMotorErrorMode();
    // clear motor error state
    bool setClearMotorError();
    // reback original point
    bool setRebackOriginalpoint();
    // set motor original point
    bool setMotorOriginalpoint();
    // get motor cur tmp
    int32_t getMotorTemprate();
    // get motor control PID
    bool getMotorPosCtrlPID( int32_t* kp, int32_t* ki, int32_t* kd );
    // set motor control PID
    bool setMotorPosCtrlPID( int32_t kp, int32_t ki, int32_t kd );
    // get motor pos by output encoder Or input encoder
    bool getMotorUpdateData( double* pos, bool output_flag );
    // get motor update data( pos vel cur )
    bool getMotorUpdateData( double* pos, double* vel, double* current );
    // get motor update data( pos vel ) by input encoder
    bool getMotorUpdateData( double* pos, double* vel );
    // set motor command, pos ctrl ( pos vel acc)
    bool setMotorPosCtrlCmd( double pos_des, double vel_des, double acc_des );
    // set motor command, pos ctrl ( pos vel)
    bool setMotorPosCtrlCmd( double pos_des, double vel_des );
    // set motor command, pos ctrl ( only pos )
    bool setMotorPosCtrlCmd( double pos_des );
    // set motor command, pos ctrl ( only pos ), for all joints
    bool setMotorPosCtrlCmd( double pos_des[ 6 ] );
    // get zero offset
    void getZeroOffset();
    // return zero offset
    void ReturnZeroOffset( int32_t& offset ) {
        offset = input_zero_pos_encode;
    }
    // set zero offset of all joints
    void SetZeroOffsetForGeneraljoints( int32_t offset[ 6 ] ) {
        for ( int8_t i = 0; i < 6; i++ )
            record_joints_zero_pos_offset_[ i ] = offset[ i ];
    }
    // set motor acc and dec
    bool setMotorCtrlAcc( int32_t acc );
    // get motor task state
    bool getMotorTaskState();
    // set motor stop current task
    bool setMotorStopTask();
};

}  // namespace ARM

#endif  // JOINT_HANDLER