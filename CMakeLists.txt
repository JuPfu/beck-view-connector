# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(beck-view-connector C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(beck-view-connector beck-view-connector.c )

pico_set_program_name(beck-view-connector "beck-view-connector")
pico_set_program_version(beck-view-connector "0.5")

target_sources(beck-view-connector PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/lib/assert.c
        ${CMAKE_CURRENT_LIST_DIR}/ili9341/ili9341.c
        ${CMAKE_CURRENT_LIST_DIR}/touch/msp2807_touch.c
        ${CMAKE_CURRENT_LIST_DIR}/touch/msp2807_calibration.c
        ${CMAKE_CURRENT_LIST_DIR}/frame_timing.c
        ${CMAKE_CURRENT_LIST_DIR}/display.c
        ${CMAKE_CURRENT_LIST_DIR}/beck-view-connector.c
        )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_usb(beck-view-connector 0)
pico_enable_stdio_usb(beck-view-connector 1)

# Add the standard library to the build
target_link_libraries(beck-view-connector
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(beck-view-connector PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(beck-view-connector 
        pico_stdlib
        pico_divider
        pico_sync
        pico_multicore
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_timer
        hardware_watchdog
        hardware_clocks
        )

pico_add_extra_outputs(beck-view-connector)

