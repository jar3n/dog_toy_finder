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

# micro-ROS plus the log fan-out use more stack than the SDK default (2 KiB).
# Core 0's stack lives in the 4 KiB SCRATCH_Y region, so this is the maximum.
add_compile_definitions(PICO_STACK_SIZE=0x1000)

pico_sdk_init()

function(dtf_provide_local_config name)
    set(config_file "${CMAKE_CURRENT_LIST_DIR}/config/${name}.h")
    set(example_file "${CMAKE_CURRENT_LIST_DIR}/config/${name}.example.h")
    set(config_size 0)
    if(EXISTS "${config_file}")
        file(SIZE "${config_file}" config_size)
    endif()
    if(config_size EQUAL 0)
        configure_file("${example_file}" "${config_file}" COPYONLY)
        message(STATUS "Created ${config_file}; review its settings")
    endif()
endfunction()

# Only the Wi-Fi config holds credentials and is kept out of git. All other
# config headers are committed and edited in place.
dtf_provide_local_config(wifi_config)

add_executable(dtf_pico_fw
    dtf_pico_fw_main.c
    microros_time/microros_time.c
    udp_transport/picow_udp_transport.c
    wifi/wifi.c
    wifi/wifi_log.c
    ros/ros_node.c
    ros/ros_log.c
    imu/bno055.c
    imu/imu_ros.c
    encoder/quadrature_encoder.c
    encoder/encoder.c
    encoder/encoder_ros.c
    motor/tb6612fng.c
    motor/cmd_vel.c
    ir_obstacle/ir_obstacle.c
    ir_obstacle/ir_obstacle_ros.c
)

# micro ros library
add_library(micro_ros STATIC IMPORTED)
set_target_properties(micro_ros PROPERTIES
    IMPORTED_LOCATION
        "${CMAKE_CURRENT_LIST_DIR}/lib/libmicroros/libmicroros.a"
    INTERFACE_INCLUDE_DIRECTORIES
        "${CMAKE_CURRENT_LIST_DIR}/lib/libmicroros/include"
)

pico_set_program_name(dtf_pico_fw "dtf_pico_fw")
pico_set_program_version(dtf_pico_fw 1.0)

pico_enable_stdio_uart(dtf_pico_fw 0)
pico_enable_stdio_usb(dtf_pico_fw 1)

target_include_directories(dtf_pico_fw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/config
)

target_link_libraries(dtf_pico_fw
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_i2c
    hardware_pwm
    hardware_clocks
    micro_ros
)

pico_add_extra_outputs(dtf_pico_fw)
