cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ipdss_rpico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

#set(SOURCES "libs/rotary-encoder/")

include_directories(libs/rotary-encoder)


add_executable(rpico_test main.c pisystick.h pisystick.c libs/rotary-encoder/rotary_encoder.c)

pico_enable_stdio_usb(rpico_test 1)
pico_enable_stdio_uart(rpico_test 1)
pico_add_extra_outputs(rpico_test)
target_link_libraries(rpico_test pico_stdlib)