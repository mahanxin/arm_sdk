include_directories( include/ )

file(GLOB LIBMODBUS_SRC "src/*.c" )

add_library( modbus STATIC ${LIBMODBUS_SRC} )