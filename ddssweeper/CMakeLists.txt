cmake_minimum_required(VERSION 3.12)
#Pull in pico sdk
include(pico_sdk_import.cmake)
project(dds-sweeper C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

#initialize the SDK
pico_sdk_init()	
add_executable(dds-sweeper
        dds-sweeper.c
        ad9959.c
        ad9959.h
        )

pico_generate_pio_header(dds-sweeper ${CMAKE_CURRENT_LIST_DIR}/trigger_timer.pio)

target_link_libraries(dds-sweeper 
        pico_stdlib
        pico_multicore
        hardware_spi
        hardware_clocks
        hardware_pio
        hardware_dma
        hardware_flash
        )

# UART/USB config
pico_enable_stdio_usb(dds-sweeper 1)
pico_enable_stdio_uart(dds-sweeper 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(dds-sweeper)

# create map/bin/hex file etc.
