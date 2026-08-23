cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(nite369_v2 C CXX ASM)

# Reserve the last 64KB of the 2MB flash for the Nite config store.
set(PICO_FLASH_SIZE_BYTES 2031616 CACHE STRING "Flash size in bytes (2MB - 64KB config reserve)")

pico_sdk_init()

# ── Auto-increment build version (same mechanism as v1) ───────────────
set(VERSION_COUNTER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/build_count.txt)
set(VERSION_HEADER_OUT ${CMAKE_CURRENT_BINARY_DIR}/firmware_version.h)
add_custom_target(bump_version ALL
    COMMAND ${CMAKE_COMMAND}
        -DVERSION_COUNTER_FILE=${VERSION_COUNTER_FILE}
        -DVERSION_HEADER_OUT=${VERSION_HEADER_OUT}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake
    COMMENT "Bumping firmware build version"
    VERBATIM
)
add_custom_command(
    OUTPUT ${VERSION_HEADER_OUT}
    COMMAND ${CMAKE_COMMAND} -E touch ${VERSION_HEADER_OUT}
    DEPENDS bump_version
    COMMENT "Ensure firmware_version.h exists"
)

# ── Directory layout (single source of truth for the SPI protocol) ────
set(SHARED_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/shared/spi_protocol)
set(SLAVE_COMMON  ${CMAKE_CURRENT_SOURCE_DIR}/slave_common)
set(MASTER_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/master)

# ioLibrary_Driver (W5500) — local clone
set(IOLIB_DIR ${CMAKE_CURRENT_LIST_DIR}/_ioLibrary_Driver)
if(NOT EXISTS ${IOLIB_DIR}/Ethernet/W5500/w5500.c)
    message(FATAL_ERROR "ioLibrary_Driver not found at ${IOLIB_DIR}. Run: git clone --depth 1 https://github.com/Wiznet/ioLibrary_Driver.git _ioLibrary_Driver")
endif()

# Common include dirs for every target
set(NITE_INCLUDE_DIRS
    ${SHARED_DIR}
    ${SLAVE_COMMON}
    ${MASTER_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# ========================================
# MASTER — W5500 Ethernet TCP server (SPI1 to slaves)
# ========================================
add_executable(spi_cmd_master_eth
    master/spi_cmd_master_eth.c
    master/gcode_parser.c
    master/w5500_tcp.c
    master/workspace_store.c
    slave_common/config_store.c
    shared/spi_protocol/frame.c
    ${VERSION_HEADER_OUT}
)
target_include_directories(spi_cmd_master_eth PRIVATE
    ${NITE_INCLUDE_DIRS}
    ${IOLIB_DIR}/Ethernet/W5500
    ${IOLIB_DIR}/Ethernet
)
target_sources(spi_cmd_master_eth PRIVATE
    ${IOLIB_DIR}/Ethernet/W5500/w5500.c
    ${IOLIB_DIR}/Ethernet/socket.c
    ${IOLIB_DIR}/Ethernet/wizchip_conf.c
)
target_link_libraries(spi_cmd_master_eth
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
)
target_compile_definitions(spi_cmd_master_eth PRIVATE
    _WIZCHIP_=5500
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(spi_cmd_master_eth 1)
pico_enable_stdio_uart(spi_cmd_master_eth 0)
pico_add_extra_outputs(spi_cmd_master_eth)

# ========================================
# SLAVE 1 — Arm Base (J1-J3, GPIO step, bit-bang SPI on GP2/3/4/5)
# ========================================
add_executable(spi_cmd_slave_s1
    slave1_arm_base/spi_cmd_slave_s1.c
    slave_common/motion_profile.c
    slave_common/motion_marlin.c
    slave_common/homing.c
    slave_common/config_store.c
    shared/spi_protocol/frame.c
    ${VERSION_HEADER_OUT}
)
target_include_directories(spi_cmd_slave_s1 PRIVATE ${NITE_INCLUDE_DIRS})
target_compile_definitions(spi_cmd_slave_s1 PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
target_link_libraries(spi_cmd_slave_s1
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_flash
    hardware_sync
)
pico_enable_stdio_usb(spi_cmd_slave_s1 1)
pico_enable_stdio_uart(spi_cmd_slave_s1 0)
pico_add_extra_outputs(spi_cmd_slave_s1)

# ========================================
# SLAVE 2 — Wrist + Gripper (J4-J6, TMC2209 UART, WS2812)
# ========================================
add_executable(spi_cmd_slave_s2
    slave2_wrist_gripper/spi_cmd_slave_s2.c
    slave_common/tmc_uart.c
    slave_common/ws2812.c
    slave_common/motion_profile.c
    slave_common/motion_marlin.c
    slave_common/homing.c
    slave_common/config_store.c
    shared/spi_protocol/frame.c
    ${VERSION_HEADER_OUT}
)
target_include_directories(spi_cmd_slave_s2 PRIVATE ${NITE_INCLUDE_DIRS})
target_compile_definitions(spi_cmd_slave_s2 PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
target_link_libraries(spi_cmd_slave_s2
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_uart
    hardware_pio
    hardware_flash
    hardware_sync
)
pico_enable_stdio_usb(spi_cmd_slave_s2 1)
pico_enable_stdio_uart(spi_cmd_slave_s2 0)
pico_add_extra_outputs(spi_cmd_slave_s2)
