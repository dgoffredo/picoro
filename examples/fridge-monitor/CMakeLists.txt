cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD "pico_w" CACHE STRING "The particular Raspberry Pi Pico board")
set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk" CACHE PATH "Path to the Raspberry Pi Pico SDK")

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(fridge-monitor CXX C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

# Require -Wall cleanliness for all code, including SDK headers.
add_compile_options(-Wall -Werror)

add_subdirectory(picoro)

add_executable(fridge-monitor fridge-monitor.cpp)

target_link_libraries(fridge-monitor
        picoro_coroutine
        picoro_drivers_dht22
        picoro_event_loop
        picoro_sleep

        pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(fridge-monitor)

# map stdio to the USB CDC, not to UART
pico_enable_stdio_usb(fridge-monitor 1)
pico_enable_stdio_uart(fridge-monitor 0)
