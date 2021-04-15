cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ipdss_rpico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# I've added pwm header from Pico's SDK, 'cause VSCode can't see as a native library
add_executable(ipdss_rpico main.cpp cmotor.h cmotor.cpp cpid.h cpid.cpp pwm.h pisystick.h pisystick.cpp)

pico_enable_stdio_usb(ipdss_rpico 1)
pico_enable_stdio_uart(ipdss_rpico 1)
pico_add_extra_outputs(ipdss_rpico)
target_link_libraries(ipdss_rpico pico_stdlib)