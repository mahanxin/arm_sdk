# Motor Driver

## 代码结构  
1. 本电机驱动代码分拆为四个文件，serial->modbus_handler->joint_handler->arm_handler,具体功能如下：  
serial: 包含串口设置及数据读写操作  
modbus_handler: 包含通信协议的定义，以及指令包的封装与分解  
joint_handler: 包含单关节的参数设置以及关节层发送指令、读取数据等操作函数的封装  
arm_handler: 包含多关节机械臂的参数设置及整机层面发送指令、读取数据等操作函数的封装  
## 注意事项  
1. 该通信协议基于MODBUS RTU协议自定义，传输接口采用基于串口的RS-485，传输方式为异步主从半双工方式。  
2. 波特率可选19200——4500000bps，代码中默认设置225000，可手动修改。  
3. 多关节同时发送位置指令，有两种指令包方式，一是各关节使用单独的指令包分别发送，二是整合为一个指令包一次发送。代码中使用宏定义#USE_GENERAL_JOINTS_HANDLER来选择，默认使用第一种方式。  
4. 代码中串口号使用宏定义配置ARM_COM_device，默认设置为"/dev/ttyUSB0"，请根据实际情况修改。
## 编译方法
1. git clone git@github.com:mahanxin/arm_sdk.git
2. cd arm_sdk && mkdir build
3. cmake ..
4. make -j4
> 在编译时，可能会报错"error: redefinition of ‘struct termios’",出现原因是termios2中对termios重复定义，目前解决方法是在/usr/include/asm-generic/termbits.h 文件中手动注释掉 struct termios！