#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stack>
#include <string>
#include <thread>

// #include "jointHandler.h"
#include "armController.h"
// #include "hardware_bridge.h"

using namespace std;
using namespace ARM;
// #define test(a)  std::cout<<a<<std::endl

int main( int argc, char** argv ) {
    printf( "baudrate type is:\n" );
    printf( "1:B19200, 2:B57600, 3:B115200, 4:B2250000, 5:B4500000 \n" );
    if ( argc != 2 ) {
        printf( "please input one parameters!\n" );
    }
    // input desired baudrate type
    uint8_t type = 3;
    if ( argc >= 2 )
        type = argv[ 1 ][ 0 ] - 48;
    // change badurate
    armController* _armCtrl;
    _armCtrl = new armController();

    _armCtrl->setBaudrate( type );

    sleep( 3 );

    // printf("~ Exit ~\n");

    // sizeof(B);
    // system("pause");
    return true;
}