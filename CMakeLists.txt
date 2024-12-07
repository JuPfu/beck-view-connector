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

project(beck_view_connector C CXX ASM)


# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(beck_view_connector beck_view_connector.c )

pico_set_program_name(beck_view_connector "beck_view_connector")
pico_set_program_version(beck_view_connector "0.5")

# by default the header is generated into the build dir
pico_generate_pio_header(beck_view_connector ${CMAKE_CURRENT_LIST_DIR}/frame_signal.pio)

target_sources(beck_view_connector PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lib/assert.c
        ${CMAKE_CURRENT_LIST_DIR}/ili9341/ili9341.c
        ${CMAKE_CURRENT_LIST_DIR}/touch/msp2807_touch.c
        ${CMAKE_CURRENT_LIST_DIR}/touch/msp2807_calibration.c
        ${CMAKE_CURRENT_LIST_DIR}/frame_timing.c
        ${CMAKE_CURRENT_LIST_DIR}/display.c
        ${CMAKE_CURRENT_LIST_DIR}/beck_view_connector.c
        )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_usb(beck_view_connector 0)
pico_enable_stdio_usb(beck_view_connector 1)

# Add the standard include files to the build
target_include_directories(beck_view_connector PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(beck_view_connector PRIVATE
        pico_stdlib
        pico_divider
        pico_sync
        pico_multicore
        hardware_pio
        hardware_spi
        hardware_dma
        hardware_timer
        hardware_clocks
        )

pico_add_extra_outputs(beck_view_connector)

