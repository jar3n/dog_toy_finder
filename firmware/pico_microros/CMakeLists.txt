# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(dtf_pico_fw C CXX ASM)

pico_sdk_init()

add_executable(dtf_pico_fw
    dtf_pico_fw_main.c
)

# TODO: add the micro ros library later
#  for now just simple main hello world

pico_set_program_name(dtf_pico_fw "dtf_pico_fw")
pico_set_program_version(dtf_pico_fw 1.0)

pico_enable_stdio_uart(dtf_pico_fw 0)
pico_enable_stdio_usb(dtf_pico_fw 1)

target_include_directories(dtf_pico_fw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(dtf_pico_fw
    pico_stdlib
)

pico_add_extra_outputs(dtf_pico_fw)