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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
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

add_executable(beck_view_connector beck_view_connector.cpp)

pico_set_program_name(beck_view_connector "beck_view_connector")
pico_set_program_version(beck_view_connector "0.9")

# by default the header is generated into the build dir
pico_generate_pio_header(beck_view_connector ${CMAKE_CURRENT_LIST_DIR}/frame_signal.pio)

target_sources(beck_view_connector PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/st7789/ST7789_TFT.cpp
        ${CMAKE_CURRENT_LIST_DIR}/st7789/ST7789_TFT_Print.cpp
        ${CMAKE_CURRENT_LIST_DIR}/st7789/ST7789_TFT_graphics.cpp
        ${CMAKE_CURRENT_LIST_DIR}/st7789/ST7789_TFT_Font.cpp
        ${CMAKE_CURRENT_LIST_DIR}/st7789/ST7789_TFT_Bitmap_Data.cpp
        ${CMAKE_CURRENT_LIST_DIR}/frame_timing.c
        ${CMAKE_CURRENT_LIST_DIR}/display.cpp
        ${CMAKE_CURRENT_LIST_DIR}/beck_view_connector.cpp
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

