/*!
 * @file armController.h
 * @brief A handler class to control all joints of arm
 *
 * This file contains: joint params, update data, command,
 * and some operation function for motor,
 * for example: enable、clear error、set original point and son on.
 *
 * Authored by HanxinMa, 2022-12-10
 */
#ifndef ARM_CONTROLLER
#define ARM_CONTROLLER

#include <iostream>
#include <vector>

#include "jointHandler.h"

namespace ARM {
#define ARM_COM_device "/dev/ttyUSB0"
// using namespace Eigen;
#define JOINTSNUM 6
using std::vector;

// #define USE_GENERAL_JOINTS_HANDLER
// struct JointState {
//   double q, qd, tau;
//   int16_t error_mod;
//   int32_t kp, ki, kd;
// };

// struct JointCommand {
//   double q_des, qd_des, tau_des;
//   int32_t kp_des, ki_des, kd_des;
// };

// struct JointsData {
//   // Matrix< double, 6, 1 > q, qd, tau;
//   std::vector<JointState> _datas;
//   void resizeDim(const int dim) { _datas.resize(dim); }
// };

// struct JointsCommand {
//   // Matrix< double, 6, 1 > qdes, qddes, tau_des;
//   std::vector<JointCommand> _cmds;
//   void resizeDim(const int dim) { _cmds.resize(dim); }
// };

struct ArmJointsState {
    vector< double >  q, qd, tau;
    vector< int16_t > error_mode;
    // default is same for all joints
    vector< int32_t > kp, ki, kd;
    vector< uint8_t > baud_type;

    void resizeDim( const int dim ) {
        q.resize( dim );
        qd.resize( dim );
        tau.resize( dim );
        error_mode.resize( dim );
        kp.resize( dim );
        ki.resize( dim );
        kd.resize( dim );
    }
};

struct ArmJointsCommand {
    vector< double >  q_des, qd_des, tau_des;
    vector< int32_t > kp_des, ki_des, kd_des;

    void resizeDim( const int dim ) {
        q_des.resize( dim );
        qd_des.resize( dim );
        tau_des.resize( dim );
        kp_des.resize( dim );
        ki_des.resize( dim );
        kd_des.resize( dim );
    }

    void zeroCmd() {
        std::fill( q_des.begin(), q_des.end(), 0.0 );
        std::fill( qd_des.begin(), qd_des.end(), 0.0 );
        std::fill( tau_des.begin(), tau_des.end(), 0.0 );
    }
};

class armController {
private:
    vector< jointHandler* > _jointHandlers;

    jointHandler* general_joints_handler_;

    Serial _serial_port;

public:
    ArmJointsState   _jointsData;
    ArmJointsCommand _jointsCmd;
    armController( /* args */ );
    ~armController();
    bool _armEnabled = false;
    bool _clearError = false;

    void setBaudrate( uint8_t type );
    void getBaudrate();
    void zeroCommand();
    void setEnabled( bool enabled );
    void setEnabled( bool enabled, int8_t id );
    void getErrorState();
    void setErrorClear();
    void setRebackOriginalpoint();
    void initMotors();

    void setPID( int32_t* kp_des, int32_t* ki_des, int32_t* kd_des );
    void getPID();

    void updateAbsPosition( ArmJointsState* data );
    void updateData( ArmJointsState* data );
    void updateCommand( ArmJointsCommand* _cmd, bool with_vel = false );
};

}  // namespace ARM

#endif  // ARM_CONTROLLER