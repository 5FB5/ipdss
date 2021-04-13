cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ipdss_rpico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ipdss_rpico main.c)

pico_enable_stdio_usb(ipdss_rpico 1)
pico_enable_stdio_uart(ipdss_rpico 1)
pico_add_extra_outputs(ipdss_rpico)
target_link_libraries(ipdss_rpico pico_stdlib)