cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD "pico_w" CACHE STRING "The particular Raspberry Pi Pico board" FORCE)
set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk" CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(co2-server CXX C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

# Require -Wall cleanliness for all code, including SDK headers.
add_compile_options(-Wall -Werror)

add_executable(co2-server
        co2-server.cpp
        )

# Enable C++20 coroutines (GCC 10 requires a flag) and stricter warnings for our C++ code only.
set_source_files_properties(co2-server.cpp secrets.h lwipopts.h
        PROPERTIES COMPILE_OPTIONS -fcoroutines -Wextra -pedantic)

# Make our lwipopts.h visible to lwIP, which includes it.
target_include_directories(co2-server PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(co2-server
        hardware_i2c
        pico_async_context_poll
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(co2-server)

# map stdio to the USB CDC, not to UART
pico_enable_stdio_usb(co2-server 1)
pico_enable_stdio_uart(co2-server 0)
