cmake_minimum_required(VERSION 3.16)

project(armController VERSION 1.0.0)

set(CMAKE_BUILD_TYPE DEBUG)

# flie(GLOB SRC_LIST "src/hello.cpp")

add_compile_options(-std=c++11)

find_package(Eigen3 3.3.7 EXACT REQUIRED)
find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)

# configure_file("${PROJECT_SOURCE_DIR}/include/modbus/modbus-version.h.in"
#     "${PROJECT_SOURCE_DIR}/include/modbus/modbus-version.h")

include_directories("common/")
include_directories("include/")

add_subdirectory( third_party/modbus )

list(APPEND EXTRA_LIBS modbus)
# list(APPEND EXTRA_LIBS legged_control)
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/third_party/modbus/include/")

file(GLOB_RECURSE source "src/*.cpp")

# message(${source})

add_executable(changeBaud user/changeBaud.cpp ${source})
target_link_libraries(changeBaud PUBLIC ${EXTRA_LIBS} pthread lcm ${YAML_CPP_LIBRARIES})
target_include_directories(changeBaud PUBLIC "${PROJECT_BINARY_DIR}" ${EXTRA_INCLUDES} )

add_executable(serial_dubug user/serial_debug.cpp ${source})
target_link_libraries(serial_dubug PUBLIC ${EXTRA_LIBS} pthread lcm ${YAML_CPP_LIBRARIES})
target_include_directories(serial_dubug PUBLIC "${PROJECT_BINARY_DIR}" ${EXTRA_INCLUDES} )