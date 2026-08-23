cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(pico_linuxcnc_3pico C CXX ASM)

# Reserve the last 64KB of the 2MB flash for the Nite config store.
# The app is constrained below this region so flash writes never collide.
set(PICO_FLASH_SIZE_BYTES 2031616 CACHE STRING "Flash size in bytes (2MB - 64KB config reserve)")

pico_sdk_init()

if (TARGET picoレス) # Avoid unused warning
endif()

# ── Auto-increment build version ──────────────────────────────────────
# Every build of a firmware target bumps a counter and regenerates
# firmware_version.h with a unique version (1.001, 1.002, ...). The three
# production targets depend on the bump target so a rebuild always carries
# a new number (visible via #V / the boot banner).
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
# Make every target's compile depend on the generated header existing.
add_custom_command(
    OUTPUT ${VERSION_HEADER_OUT}
    COMMAND ${CMAKE_COMMAND} -E touch ${VERSION_HEADER_OUT}
    DEPENDS bump_version
    COMMENT "Ensure firmware_version.h exists"
)

# Use local clone of ioLibrary_Driver (cloned to _ioLibrary_Driver/)
set(IOLIB_DIR ${CMAKE_CURRENT_LIST_DIR}/_ioLibrary_Driver)
if(NOT EXISTS ${IOLIB_DIR}/Ethernet/W5500/w5500.c)
    message(FATAL_ERROR "ioLibrary_Driver not found at ${IOLIB_DIR}. Run: git clone --depth 1 https://github.com/Wiznet/ioLibrary_Driver.git _ioLibrary_Driver")
endif()
message(STATUS "ioLibrary_Driver source: ${IOLIB_DIR}")

# ========================================
# TARGET 1: Master Pico (Networking + SPI Routing)
# ========================================
add_executable(master_firmware master/nite_master.c)

# Generate PIO headers (must come after add_executable)
pico_generate_pio_header(master_firmware ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)

# ioLibrary_Driver path already set above

target_compile_definitions(master_firmware PRIVATE _WIZCHIP_=5500)
target_include_directories(master_firmware PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${IOLIB_DIR}/Ethernet/W5500
    ${IOLIB_DIR}/Ethernet
    ${IOLIB_DIR}/Internet/DHCP
)
target_sources(master_firmware PRIVATE
    inc/nite_config.c
    ${IOLIB_DIR}/Ethernet/W5500/w5500.c
    ${IOLIB_DIR}/Ethernet/socket.c
    ${IOLIB_DIR}/Ethernet/wizchip_conf.c
)
target_link_libraries(master_firmware
    pico_stdlib
    pico_multicore
    hardware_flash
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_watchdog
    hardware_timer
)
pico_enable_stdio_usb(master_firmware 1)
pico_enable_stdio_uart(master_firmware 0)
pico_add_extra_outputs(master_firmware)

# ========================================
# TARGET 2a: Servo Pico — SLAVE 1 (Arm Base, TMC2160 drivers)
# Encoders J1/J2A/J2B/J3 on GP12/13, 6/7, 8/9, 10/11 (see nite_servo.c)
# ========================================
add_executable(servo_firmware
    servo/nite_servo.c
    inc/nite_config.c
    inc/nite_tmc2209.c
    inc/nite_multicore.c
)

# Generate PIO headers (must come after add_executable)
pico_generate_pio_header(servo_firmware  ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
pico_generate_pio_header(servo_firmware  ${CMAKE_CURRENT_LIST_DIR}/pio/i2c.pio)

# Needed so servo/nite_servo.c can find PIO headers
target_include_directories(servo_firmware PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Slave 1 (arm base) uses TMC2160 drivers — NO TMC2209 UART bus.
# FIXED: was unconditionally defined, so Slave 1 ran the TMC2209 config
# sequence (and reported failures) on every boot.
target_compile_definitions(servo_firmware PRIVATE NITE_SLAVE_ID=1)

target_link_libraries(servo_firmware
    pico_stdlib
    pico_multicore
    hardware_flash
    hardware_spi
    hardware_pio
    hardware_clocks
    hardware_irq
    hardware_gpio
    hardware_timer
    hardware_dma
    hardware_sync
    hardware_uart
)
pico_enable_stdio_usb(servo_firmware 1)
pico_enable_stdio_uart(servo_firmware 0)
pico_add_extra_outputs(servo_firmware)

# ========================================
# TARGET 2b: Servo Pico — SLAVE 2 (Wrist, TMC2209 UART drivers)
# Encoders J4/J5/J6 on GP12/13, 8/9, 10/11; gripper axis = open-loop
# ========================================
add_executable(servo_firmware_s2
    servo/nite_servo.c
    inc/nite_config.c
    inc/nite_tmc2209.c
    inc/nite_multicore.c
)

pico_generate_pio_header(servo_firmware_s2  ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
pico_generate_pio_header(servo_firmware_s2  ${CMAKE_CURRENT_LIST_DIR}/pio/i2c.pio)

target_include_directories(servo_firmware_s2 PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Slave 2 (wrist) uses 4x TMC2209 on the half-duplex UART bus (GP0/GP1)
target_compile_definitions(servo_firmware_s2 PRIVATE NITE_SLAVE_ID=2 USE_TMC2209_UART)

target_link_libraries(servo_firmware_s2
    pico_stdlib
    pico_multicore
    hardware_flash
    hardware_spi
    hardware_pio
    hardware_clocks
    hardware_irq
    hardware_gpio
    hardware_timer
    hardware_dma
    hardware_sync
    hardware_uart
)
pico_enable_stdio_usb(servo_firmware_s2 1)
pico_enable_stdio_uart(servo_firmware_s2 0)
pico_add_extra_outputs(servo_firmware_s2)

# ========================================
# TEST: SPI Communication Test (Master → Slave 1 — Arm Base)
# CS1=GP9, SYNC=GP14
# ========================================
add_executable(spi_test_master test/spi_test_master.c)
target_include_directories(spi_test_master PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(spi_test_master
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(spi_test_master 1)
pico_enable_stdio_uart(spi_test_master 0)
pico_add_extra_outputs(spi_test_master)

# ========================================
# TEST: SPI Communication Test (Master → Slave 2 — Wrist)
# CS2=GP13, SYNC=GP14
# Flash to Master Pico for Slave 2 communication test.
# ========================================
add_executable(spi_test_master_slave2 test/spi_test_master_slave2.c)
target_include_directories(spi_test_master_slave2 PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(spi_test_master_slave2
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(spi_test_master_slave2 1)
pico_enable_stdio_uart(spi_test_master_slave2 0)
pico_add_extra_outputs(spi_test_master_slave2)

# ========================================
# NEW SPI Communication Test: Master -> Slave 2 (CS-per-byte, 9-byte CRC frames)
# Flash spi_test_s2_master to Master, spi_test_s2_slave to Slave 2 (Wrist).
# ========================================
add_executable(spi_test_s2_master test/spi_test_s2_master.c)
target_link_libraries(spi_test_s2_master pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(spi_test_s2_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000  # printf must never block the SPI loop
)
pico_enable_stdio_usb(spi_test_s2_master 1)
pico_enable_stdio_uart(spi_test_s2_master 0)
pico_add_extra_outputs(spi_test_s2_master)

add_executable(spi_test_s2_slave test/spi_test_s2_slave.c)
target_link_libraries(spi_test_s2_slave pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(spi_test_s2_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000  # printf must never block the SPI loop
)
pico_enable_stdio_usb(spi_test_s2_slave 1)
pico_enable_stdio_uart(spi_test_s2_slave 0)
pico_add_extra_outputs(spi_test_s2_slave)

# Variant of the master test targeting CS1 (GP9) = SLAVE 1 Pico.
# Use to flash spi_test_s2_slave onto the KNOWN-GOOD slave 1 Pico and verify
# the test firmware runs on working hardware.
add_executable(spi_test_s1_master test/spi_test_s2_master.c)
target_link_libraries(spi_test_s1_master pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(spi_test_s1_master PRIVATE
    TEST_CS1=1
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(spi_test_s1_master 1)
pico_enable_stdio_uart(spi_test_s1_master 0)
pico_add_extra_outputs(spi_test_s1_master)

# ========================================
# TEST: SPI Communication Test (Slave — hardware SPI)
# Works for both Slave 1 and Slave 2 (same SPI0 pins)
# SCK=GP2 MISO=GP3 MOSI=GP4 CS=GP5 SYNC=GP6
# ========================================
add_executable(spi_test_slave test/spi_test_slave.c)

target_include_directories(spi_test_slave PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(spi_test_slave
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(spi_test_slave 1)
pico_enable_stdio_uart(spi_test_slave 0)
pico_add_extra_outputs(spi_test_slave)

# ========================================
# TEST: SPI Master → Slave 2 ONLY (CS2=GP13)
# Same slave firmware (spi_test_slave) on Slave 2.
# Master uses CS2=GP13 instead of CS1=GP9.
# ========================================
add_executable(spi_test_master_s2 test/spi_test_master_s2.c)
target_link_libraries(spi_test_master_s2 pico_stdlib hardware_spi hardware_gpio)
pico_enable_stdio_usb(spi_test_master_s2 1)
pico_enable_stdio_uart(spi_test_master_s2 0)
pico_add_extra_outputs(spi_test_master_s2)

# ========================================
# SPI Cross-Wire Test — Central (default SPI0 pins)
# GP19(TX) -> Peripheral GP16, GP16(RX) <- Peripheral GP19
# ========================================
add_executable(spi_cross_central test/spi_cross_central.c)
target_link_libraries(spi_cross_central pico_stdlib hardware_spi)
pico_enable_stdio_usb(spi_cross_central 1)
pico_enable_stdio_uart(spi_cross_central 0)
pico_add_extra_outputs(spi_cross_central)

# ========================================
# SPI Cross-Wire Test — Peripheral (default SPI0 pins)
# GP16(RX) <- Central GP19, GP19(TX) -> Central GP16
# ========================================
add_executable(spi_cross_peripheral test/spi_cross_peripheral.c)
target_link_libraries(spi_cross_peripheral pico_stdlib hardware_spi)
pico_enable_stdio_usb(spi_cross_peripheral 1)
pico_enable_stdio_uart(spi_cross_peripheral 0)
pico_add_extra_outputs(spi_cross_peripheral)

# ========================================
# SPI Master → Slave 2 Simple (your wiring)
# Master SPI1: SCK=10 MOSI=11 MISO=12 CS=13
# Slave  SPI0: SCK=2  MOSI=3  MISO=4  CS=5
# No cross — your existing wiring
# ========================================
add_executable(spi_master_s2_simple test/spi_master_s2_simple.c)
target_link_libraries(spi_master_s2_simple pico_stdlib hardware_spi)
pico_enable_stdio_usb(spi_master_s2_simple 1)
pico_enable_stdio_uart(spi_master_s2_simple 0)
pico_add_extra_outputs(spi_master_s2_simple)

add_executable(spi_slave_s2_simple test/spi_slave_s2_simple.c)
target_link_libraries(spi_slave_s2_simple pico_stdlib hardware_spi)
pico_enable_stdio_usb(spi_slave_s2_simple 1)
pico_enable_stdio_uart(spi_slave_s2_simple 0)
pico_add_extra_outputs(spi_slave_s2_simple)

# ========================================
# SPI Speed Test — Master → Slave 2
# Tests 100kHz through 20MHz, reports PASS/FAIL per speed
# ========================================
add_executable(spi_speed_test_master test/spi_speed_test_master.c)
target_link_libraries(spi_speed_test_master pico_stdlib hardware_spi)
pico_enable_stdio_usb(spi_speed_test_master 1)
pico_enable_stdio_uart(spi_speed_test_master 0)
pico_add_extra_outputs(spi_speed_test_master)

# ========================================
# Wire Test — Master → Slave 2 (GPIO, no SPI)
# Verifies each wire individually: CS=GP13, SCK=GP10, MOSI=GP11, MISO=GP12
# Slave 2 reads: CS=GP5, SCK=GP2, MOSI=GP3, drives: MISO=GP4
# ========================================
add_executable(wire_test_s2_master test/wire_test_s2.c)
target_compile_definitions(wire_test_s2_master PRIVATE WIRE_TEST_S2_MASTER)
target_link_libraries(wire_test_s2_master pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_test_s2_master 1)
pico_enable_stdio_uart(wire_test_s2_master 0)
pico_add_extra_outputs(wire_test_s2_master)

add_executable(wire_test_s2_slave test/wire_test_s2.c)
target_compile_definitions(wire_test_s2_slave PRIVATE WIRE_TEST_S2_SLAVE)
target_link_libraries(wire_test_s2_slave pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_test_s2_slave 1)
pico_enable_stdio_uart(wire_test_s2_slave 0)
pico_add_extra_outputs(wire_test_s2_slave)

# ========================================
# WIRE CHECK — proper diagnostic for Slave 2 wiring
# Master drives one pin HIGH at a time for 2 seconds each
# Slave counts how many times it sees each pattern → auto PASS/FAIL
# ========================================
add_executable(wire_check_s2_master test/wire_check_s2.c)
target_compile_definitions(wire_check_s2_master PRIVATE WIRE_CHECK_S2_MASTER)
target_link_libraries(wire_check_s2_master pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_check_s2_master 1)
pico_enable_stdio_uart(wire_check_s2_master 0)
pico_add_extra_outputs(wire_check_s2_master)

add_executable(wire_check_s2_slave test/wire_check_s2.c)
target_compile_definitions(wire_check_s2_slave PRIVATE WIRE_CHECK_S2_SLAVE)
target_link_libraries(wire_check_s2_slave pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_check_s2_slave 1)
pico_enable_stdio_uart(wire_check_s2_slave 0)
pico_add_extra_outputs(wire_check_s2_slave)

# ========================================
# NEW SIMPLE wire test (no switching, no old code)
# wiretest_master -> Master Pico, wiretest_slave -> Slave
# ========================================
add_executable(wiretest_master test/wiretest_master.c)
target_link_libraries(wiretest_master pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_master 1)
pico_enable_stdio_uart(wiretest_master 0)
pico_add_extra_outputs(wiretest_master)

add_executable(wiretest_slave test/wiretest_slave.c)
target_link_libraries(wiretest_slave pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_slave 1)
pico_enable_stdio_uart(wiretest_slave 0)
pico_add_extra_outputs(wiretest_slave)

# ========================================
# FINAL wire test (master drives pins in sequence, slave reads every second)
# ========================================
add_executable(wiretest_final_master test/wiretest_final_master.c)
target_link_libraries(wiretest_final_master pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_final_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_final_master 1)
pico_enable_stdio_uart(wiretest_final_master 0)
pico_add_extra_outputs(wiretest_final_master)

add_executable(wiretest_final_slave test/wiretest_final_slave.c)
target_link_libraries(wiretest_final_slave pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_final_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_final_slave 1)
pico_enable_stdio_uart(wiretest_final_slave 0)
pico_add_extra_outputs(wiretest_final_slave)

# Wire test for SLAVE 2 on GP2/3/4/5 (CS1=GP9 on the master side)
add_executable(wiretest_s2_2345_master test/wiretest_s2_2345_master.c)
target_link_libraries(wiretest_s2_2345_master pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_s2_2345_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_s2_2345_master 1)
pico_enable_stdio_uart(wiretest_s2_2345_master 0)
pico_add_extra_outputs(wiretest_s2_2345_master)

add_executable(wiretest_s2_2345_slave test/wiretest_s2_2345_slave.c)
target_link_libraries(wiretest_s2_2345_slave pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_s2_2345_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_s2_2345_slave 1)
pico_enable_stdio_uart(wiretest_s2_2345_slave 0)
pico_add_extra_outputs(wiretest_s2_2345_slave)

# Wire test for SLAVE on GP10/11/12/13 (crossed MOSI/MISO)
add_executable(wiretest_10111213_master test/wiretest_10111213_master.c)
target_link_libraries(wiretest_10111213_master pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_10111213_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_10111213_master 1)
pico_enable_stdio_uart(wiretest_10111213_master 0)
pico_add_extra_outputs(wiretest_10111213_master)

add_executable(wiretest_10111213_slave test/wiretest_10111213_slave.c)
target_link_libraries(wiretest_10111213_slave pico_stdlib hardware_gpio)
target_compile_definitions(wiretest_10111213_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest_10111213_slave 1)
pico_enable_stdio_uart(wiretest_10111213_slave 0)
pico_add_extra_outputs(wiretest_10111213_slave)

# ========================================
# COMPLETELY NEW wiring test (pull-up MISO + slow slave TX square wave)
# ========================================
add_executable(wiretest2_master test/wiretest2_master.c)
target_link_libraries(wiretest2_master pico_stdlib hardware_gpio)
target_compile_definitions(wiretest2_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest2_master 1)
pico_enable_stdio_uart(wiretest2_master 0)
pico_add_extra_outputs(wiretest2_master)

add_executable(wiretest2_slave test/wiretest2_slave.c)
target_link_libraries(wiretest2_slave pico_stdlib hardware_gpio)
target_compile_definitions(wiretest2_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(wiretest2_slave 1)
pico_enable_stdio_uart(wiretest2_slave 0)
pico_add_extra_outputs(wiretest2_slave)

# ========================================
# SPI check: Master <-> Slave 2 raw PING/PONG diagnostic
# ========================================
add_executable(spicheck_master test/spicheck_master.c)
target_link_libraries(spicheck_master pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(spicheck_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(spicheck_master 1)
pico_enable_stdio_uart(spicheck_master 0)
pico_add_extra_outputs(spicheck_master)

add_executable(spicheck_slave test/spicheck_slave.c)
target_link_libraries(spicheck_slave pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(spicheck_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(spicheck_slave 1)
pico_enable_stdio_uart(spicheck_slave 0)
pico_add_extra_outputs(spicheck_slave)

# BIT-BANG SPI PING/PONG test — slave on GP2/3/4/5 (master CS1=GP9)
add_executable(spibang_2345_master test/spibang_2345_master.c)
target_link_libraries(spibang_2345_master pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(spibang_2345_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(spibang_2345_master 1)
pico_enable_stdio_uart(spibang_2345_master 0)
pico_add_extra_outputs(spibang_2345_master)

add_executable(spibang_2345_slave test/spibang_2345_slave.c)
target_link_libraries(spibang_2345_slave pico_stdlib hardware_gpio)
target_compile_definitions(spibang_2345_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(spibang_2345_slave 1)
pico_enable_stdio_uart(spibang_2345_slave 0)
pico_add_extra_outputs(spibang_2345_slave)

# BIT-BANG SPI PING/PONG — slave on GP6/7/8/9 (master CS2=GP13)
add_executable(spibang_6789_master test/spibang_6789_master.c)
target_link_libraries(spibang_6789_master pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(spibang_6789_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(spibang_6789_master 1)
pico_enable_stdio_uart(spibang_6789_master 0)
pico_add_extra_outputs(spibang_6789_master)

# FRESH bit-bang SPI test (new bb_spi.h library, no borrowed code)
add_executable(bb_spi_test_master test/bb_spi_test_master.c)
target_include_directories(bb_spi_test_master PRIVATE inc)
target_link_libraries(bb_spi_test_master pico_stdlib hardware_spi hardware_gpio)
target_compile_definitions(bb_spi_test_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(bb_spi_test_master 1)
pico_enable_stdio_uart(bb_spi_test_master 0)
pico_add_extra_outputs(bb_spi_test_master)

add_executable(bb_spi_test_slave test/bb_spi_test_slave.c)
target_include_directories(bb_spi_test_slave PRIVATE inc)
target_link_libraries(bb_spi_test_slave pico_stdlib hardware_gpio)
target_compile_definitions(bb_spi_test_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(bb_spi_test_slave 1)
pico_enable_stdio_uart(bb_spi_test_slave 0)
pico_add_extra_outputs(bb_spi_test_slave)

# Diagnostic: raw CS/SCK/MOSI pin monitor on the slave
add_executable(bb_spi_diag_slave test/bb_spi_diag_slave.c)
target_link_libraries(bb_spi_diag_slave pico_stdlib hardware_gpio)
target_compile_definitions(bb_spi_diag_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(bb_spi_diag_slave 1)
pico_enable_stdio_uart(bb_spi_diag_slave 0)
pico_add_extra_outputs(bb_spi_diag_slave)

# ========================================
# PURE GPIO wire test (no SPI, no protocol) — master drives + reads, slave too
# ========================================
add_executable(gpwire_master test/gpwire_master.c)
target_link_libraries(gpwire_master pico_stdlib hardware_gpio)
target_compile_definitions(gpwire_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(gpwire_master 1)
pico_enable_stdio_uart(gpwire_master 0)
pico_add_extra_outputs(gpwire_master)

add_executable(gpwire_slave test/gpwire_slave.c)
target_link_libraries(gpwire_slave pico_stdlib hardware_gpio)
target_compile_definitions(gpwire_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(gpwire_slave 1)
pico_enable_stdio_uart(gpwire_slave 0)
pico_add_extra_outputs(gpwire_slave)

# Internal loopback self-test: drive each pin, read it back on the same Pico
add_executable(gpwire_selftest test/gpwire_selftest.c)
target_link_libraries(gpwire_selftest pico_stdlib hardware_gpio)
target_compile_definitions(gpwire_selftest PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(gpwire_selftest 1)
pico_enable_stdio_uart(gpwire_selftest 0)
pico_add_extra_outputs(gpwire_selftest)

# ========================================
# ONE-DIRECTION GPIO sequence test: Master -> Slave 2
# Master drives SCK/MOSI/CS HIGH then LOW one at a time in order.
# Slave reads the 3 pins and prints them over USB serial.
# No SPI, no return wire. Proves the master -> slave wiring.
# ========================================
add_executable(gpio_seq_master test/gpio_seq_master.c)
target_link_libraries(gpio_seq_master pico_stdlib hardware_gpio)
target_compile_definitions(gpio_seq_master PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(gpio_seq_master 1)
pico_enable_stdio_uart(gpio_seq_master 0)
pico_add_extra_outputs(gpio_seq_master)

add_executable(gpio_seq_slave test/gpio_seq_slave.c)
target_link_libraries(gpio_seq_slave pico_stdlib hardware_gpio)
target_compile_definitions(gpio_seq_slave PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
pico_enable_stdio_usb(gpio_seq_slave 1)
pico_enable_stdio_uart(gpio_seq_slave 0)
pico_add_extra_outputs(gpio_seq_slave)
# ========================================
# DIAGNOSTIC: GPIO Loopback Test
# Flash one target to Master, the other to Slave
# ========================================
add_executable(gpio_loopback_master test/gpio_loopback.c)
target_compile_definitions(gpio_loopback_master PRIVATE GPIO_LOOPBACK_MASTER)
target_link_libraries(gpio_loopback_master pico_stdlib hardware_gpio)
pico_enable_stdio_usb(gpio_loopback_master 1)
pico_enable_stdio_uart(gpio_loopback_master 0)
pico_add_extra_outputs(gpio_loopback_master)

add_executable(gpio_loopback_slave test/gpio_loopback.c)
target_compile_definitions(gpio_loopback_slave PRIVATE GPIO_LOOPBACK_SLAVE)
target_link_libraries(gpio_loopback_slave pico_stdlib hardware_gpio)
pico_enable_stdio_usb(gpio_loopback_slave 1)
pico_enable_stdio_uart(gpio_loopback_slave 0)
pico_add_extra_outputs(gpio_loopback_slave)

# ========================================
# WIRE TEST: Master / Slave 2 connection
# ========================================
add_executable(wire_test_master test/wire_test_master.c)
target_link_libraries(wire_test_master pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_test_master 1)
pico_enable_stdio_uart(wire_test_master 0)
pico_add_extra_outputs(wire_test_master)

add_executable(wire_test_slave test/wire_test_slave.c)
target_link_libraries(wire_test_slave pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_test_slave 1)
pico_enable_stdio_uart(wire_test_slave 0)
pico_add_extra_outputs(wire_test_slave)

# ========================================
# DIAGNOSTIC: Bidirectional wire test
# Tests all 4 wires both directions using GPIO
# ========================================
add_executable(wire_bidi_master test/wire_bidi_master.c)
target_link_libraries(wire_bidi_master pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_bidi_master 1)
pico_enable_stdio_uart(wire_bidi_master 0)
pico_add_extra_outputs(wire_bidi_master)

add_executable(wire_bidi_slave test/wire_bidi_slave.c)
target_link_libraries(wire_bidi_slave pico_stdlib hardware_gpio)
pico_enable_stdio_usb(wire_bidi_slave 1)
pico_enable_stdio_uart(wire_bidi_slave 0)
pico_add_extra_outputs(wire_bidi_slave)

# ========================================
# DIAGNOSTIC: SPI Self-Loopback (tests one Pico in isolation)
# Jumper MOSI→MISO on the same Pico
# ========================================
add_executable(spi_loopback_spi1 test/spi_loopback_test.c)
target_compile_definitions(spi_loopback_spi1 PRIVATE SPI1_LOOPBACK)
target_link_libraries(spi_loopback_spi1 pico_stdlib hardware_spi)
pico_enable_stdio_usb(spi_loopback_spi1 1)
pico_enable_stdio_uart(spi_loopback_spi1 0)
pico_add_extra_outputs(spi_loopback_spi1)

add_executable(spi_loopback_spi0 test/spi_loopback_test.c)
target_compile_definitions(spi_loopback_spi0 PRIVATE SPI0_LOOPBACK)
target_link_libraries(spi_loopback_spi0 pico_stdlib hardware_spi)
pico_enable_stdio_usb(spi_loopback_spi0 1)
pico_enable_stdio_uart(spi_loopback_spi0 0)
pico_add_extra_outputs(spi_loopback_spi0)

# ========================================
# DIAGNOSTIC: GPIO-only MOSI→MISO loopback (no SPI peripheral)
# Tests if the MOSI→MISO wire works when pins are plain GPIO
# ========================================
add_executable(gpio_mosi_miso_spi1 test/gpio_mosi_miso_test.c)
target_compile_definitions(gpio_mosi_miso_spi1 PRIVATE GPIO_MOSI_MISO_SPI1)
target_link_libraries(gpio_mosi_miso_spi1 pico_stdlib hardware_gpio)
pico_enable_stdio_usb(gpio_mosi_miso_spi1 1)
pico_enable_stdio_uart(gpio_mosi_miso_spi1 0)
pico_add_extra_outputs(gpio_mosi_miso_spi1)

add_executable(gpio_mosi_miso_spi0 test/gpio_mosi_miso_test.c)
target_compile_definitions(gpio_mosi_miso_spi0 PRIVATE GPIO_MOSI_MISO_SPI0)
target_link_libraries(gpio_mosi_miso_spi0 pico_stdlib hardware_gpio)
pico_enable_stdio_usb(gpio_mosi_miso_spi0 1)
pico_enable_stdio_uart(gpio_mosi_miso_spi0 0)
pico_add_extra_outputs(gpio_mosi_miso_spi0)

# ========================================
# DIAGNOSTIC: SPI Register Dump — raw register-level SPI debug
# ========================================
add_executable(spi_reg_dump test/spi_reg_dump.c)
target_link_libraries(spi_reg_dump pico_stdlib hardware_spi hardware_gpio)
pico_enable_stdio_usb(spi_reg_dump 1)
pico_enable_stdio_uart(spi_reg_dump 0)
pico_add_extra_outputs(spi_reg_dump)

# ========================================
# DIAGNOSTIC: Wire test (separate files)
# Flash wire_test_master to Master, wire_test_slave to Slave
# ========================================


# ========================================
# DIAGNOSTIC: MCP23017 I2C Detection Test
# Flashes to Master Pico — detects MCP, dumps registers, reports DIP state
# ========================================
add_executable(mcp_detect_test test/mcp_detect_test.c)
target_link_libraries(mcp_detect_test pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(mcp_detect_test 1)
pico_enable_stdio_uart(mcp_detect_test 0)
pico_add_extra_outputs(mcp_detect_test)

# ========================================
# DIAGNOSTIC: MCP23017 LED Test — cycles POWER/MODE/LINK/FAULT LEDs
# Flashes to Master Pico — verifies MCP + LED + DIP hardware
# Commands: 0-3 toggle LED, A=all off, Q=chase, D=dump
# ========================================
add_executable(mcp_led_test test/mcp_led_test.c)
target_link_libraries(mcp_led_test pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(mcp_led_test 1)
pico_enable_stdio_uart(mcp_led_test 0)
pico_add_extra_outputs(mcp_led_test)

# ========================================
# TEST: SSD1306 OLED Display Test (JMD 0.96)
# Flashes to Master Pico — tests I2C OLED on GP0/GP1
# ========================================
add_executable(oled_test test/oled_test.c)
target_link_libraries(oled_test pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(oled_test 1)
pico_enable_stdio_uart(oled_test 0)
pico_add_extra_outputs(oled_test)

# ========================================
# TEST: OLED on GP2/GP3 (I2C1) — Master Pico
# Flashes to Master — 0.96" SSD1306 on I2C1
# ========================================
add_executable(oled_gp23_test test/oled_gp23_test.c)
target_link_libraries(oled_gp23_test pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(oled_gp23_test 1)
pico_enable_stdio_uart(oled_gp23_test 0)
pico_add_extra_outputs(oled_gp23_test)

# ========================================
# Simple MCP LED blink
# ========================================
add_executable(mcp_led_blink test/mcp_led_blink.c)
target_link_libraries(mcp_led_blink pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(mcp_led_blink 1)
pico_enable_stdio_uart(mcp_led_blink 0)
pico_add_extra_outputs(mcp_led_blink)

# ========================================
# Step Generator Library — reusable 4-axis trapezoidal step gen
# ========================================
# ========================================
# Step Generator — thin main using nite_stepgen source
# Header in inc/nite_stepgen.h is the reusable API.
# Other targets add servo/nite_stepgen.c to their sources.
# ========================================
add_executable(stepgen_new test/stepgen_new.c servo/nite_stepgen.c)
pico_generate_pio_header(stepgen_new ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(stepgen_new PRIVATE ${CMAKE_CURRENT_BINARY_DIR} inc)
target_link_libraries(stepgen_new
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_gpio
)
pico_enable_stdio_usb(stepgen_new 1)
pico_enable_stdio_uart(stepgen_new 0)
pico_add_extra_outputs(stepgen_new)

# ========================================
# Continuous Step Generator — Slave 2
# Standalone step/dir test on J4 (GP16/17) with serial control
# ========================================
add_executable(step_cont_s2 test/step_cont_s2.c)
target_link_libraries(step_cont_s2 pico_stdlib hardware_gpio)
pico_enable_stdio_usb(step_cont_s2 1)
pico_enable_stdio_uart(step_cont_s2 0)
pico_add_extra_outputs(step_cont_s2)

# ========================================
# All Motor Step Test — Slave 2
# Cycle all 4 joints (J4-J6+Gripper) with serial control
# ========================================
add_executable(step_all_s2 test/step_all_s2.c)
target_link_libraries(step_all_s2 pico_stdlib hardware_gpio)
pico_enable_stdio_usb(step_all_s2 1)
pico_enable_stdio_uart(step_all_s2 0)
pico_add_extra_outputs(step_all_s2)

# ========================================
# Slave 1 — Arm Base Pico
# SPI slave + 4 steppers + 4 encoders + limits
# ========================================
add_executable(slave1_arm_base test/slave1_arm_base.c)
pico_generate_pio_header(slave1_arm_base ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(slave1_arm_base PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(slave1_arm_base
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_clocks
    hardware_gpio
    hardware_i2c
)
pico_enable_stdio_usb(slave1_arm_base 1)
pico_enable_stdio_uart(slave1_arm_base 0)
pico_add_extra_outputs(slave1_arm_base)

# ========================================
# TEST: Nite Step Gen — Slave 1 (Library API)
# Uses reusable nite_stepgen library for 4-axis trapezoidal control
# with encoder feedback, limit switches, and USB serial commands.
# Flash to Slave 1 Pico (Arm Base).
# ========================================
add_executable(nite_stepgen_slave1_test
    test/nite_stepgen_slave1_test.c
    servo/nite_stepgen.c
)
pico_generate_pio_header(nite_stepgen_slave1_test ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(nite_stepgen_slave1_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR} inc)
target_link_libraries(nite_stepgen_slave1_test
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_clocks
    hardware_gpio
    hardware_i2c
)
pico_enable_stdio_usb(nite_stepgen_slave1_test 1)
pico_enable_stdio_uart(nite_stepgen_slave1_test 0)
pico_add_extra_outputs(nite_stepgen_slave1_test)

# ========================================
# TEST: Nite Step Gen — Slave 2 (Library API, Autonomous Bounce)
# 4-axis Wrist + Gripper (J4, J5, J6, Gripper)
# All 4 axes run simultaneously on boot with trapezoidal motion.
# No commands required — just power on and the wrist moves.
# Also supports USB serial commands for manual control.
# Flash to Slave 2 Pico (Wrist + Gripper).
# ========================================
add_executable(nite_stepgen_slave2_test
    test/nite_stepgen_slave2_test.c
    servo/nite_stepgen.c
)
pico_generate_pio_header(nite_stepgen_slave2_test ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(nite_stepgen_slave2_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR} inc)
target_link_libraries(nite_stepgen_slave2_test
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_timer
)
pico_enable_stdio_usb(nite_stepgen_slave2_test 1)
pico_enable_stdio_uart(nite_stepgen_slave2_test 0)
pico_add_extra_outputs(nite_stepgen_slave2_test)

# ========================================
# Servo Step Generator — 4-axis stepper with USB + SPI slave
# Flash to Servo Pico. USB for debug, SPI for Master Pico commands.
# ========================================
add_executable(servo_stepgen test/servo_stepgen.c)
pico_generate_pio_header(servo_stepgen ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(servo_stepgen PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(servo_stepgen
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_clocks
    hardware_gpio
    tinyusb_device
    tinyusb_board
)
pico_enable_stdio_usb(servo_stepgen 1)
pico_enable_stdio_uart(servo_stepgen 0)
pico_add_extra_outputs(servo_stepgen)

# ========================================
# OLED dedicated scanner
# ========================================
add_executable(oled_scan test/oled_scan.c)
target_link_libraries(oled_scan pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(oled_scan 1)
pico_enable_stdio_uart(oled_scan 0)
pico_add_extra_outputs(oled_scan)

# ========================================
# DIAGNOSTIC: I2C Bus Scanner — debug MCP detection failures
# Scans all addresses across multiple pin configs
# ========================================
add_executable(i2c_bus_scan test/i2c_bus_scan.c)
target_link_libraries(i2c_bus_scan pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(i2c_bus_scan 1)
pico_enable_stdio_uart(i2c_bus_scan 0)
pico_add_extra_outputs(i2c_bus_scan)

# ========================================
# TEST: Standalone Step Generator Continuous Run (No Commands)
# Just power on and the motor runs at 1000 Hz on GP16/GP17
# Flash to Slave 1 Pico for immediate stepper test
# ========================================
add_executable(step_continuous test/step_continuous.c)
pico_generate_pio_header(step_continuous ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(step_continuous PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(step_continuous
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_gpio
)
pico_enable_stdio_usb(step_continuous 1)
pico_enable_stdio_uart(step_continuous 0)
pico_add_extra_outputs(step_continuous)

# ========================================
# TEST: Standalone Step Generator Test
# Runs PIO freq_generator on GP16 (STEP) with GP17 (DIR) and GP28 (ENABLE)
# Flash to Slave 1 Pico for isolated stepper bring-up
# ========================================
add_executable(step_gen_test test/step_gen_test.c)
pico_generate_pio_header(step_gen_test ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(step_gen_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(step_gen_test
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_gpio
)
pico_enable_stdio_usb(step_gen_test 1)
pico_enable_stdio_uart(step_gen_test 0)
pico_add_extra_outputs(step_gen_test)

# ========================================
# Encoder Check — reads all 4 AS5600 encoders on Slave 1
# ========================================
add_executable(encoder_check test/encoder_check.c)
target_link_libraries(encoder_check pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(encoder_check 1)
pico_enable_stdio_uart(encoder_check 0)
pico_add_extra_outputs(encoder_check)

# ========================================
# Step Ramp Test — standalone accel/decel ramp on J1 (GP16/GP17)
# No commands. Just steps from 1 Hz to 10 kHz and back, looping.
# ========================================
add_executable(step_ramp test/step_ramp.c)
target_link_libraries(step_ramp pico_stdlib hardware_gpio)
pico_enable_stdio_usb(step_ramp 0)
pico_enable_stdio_uart(step_ramp 0)
pico_add_extra_outputs(step_ramp)

# ========================================
# DIAGNOSTIC: GPIO Pin State Monitor
# Monitors up to 8 pins, reports HI/LO changes over serial
# ========================================
add_executable(pin_monitor test/pin_monitor.c)
target_link_libraries(pin_monitor pico_stdlib hardware_gpio)
pico_enable_stdio_usb(pin_monitor 1)
pico_enable_stdio_uart(pin_monitor 0)
pico_add_extra_outputs(pin_monitor)

# ========================================
# DIAGNOSTIC: Raw GPIO Step Test (no PIO)
# Manual GPIO toggling to isolate hardware issues
# ========================================
add_executable(raw_step_test test/raw_step_test.c)
target_link_libraries(raw_step_test pico_stdlib hardware_gpio hardware_timer)
pico_enable_stdio_usb(raw_step_test 1)
pico_enable_stdio_uart(raw_step_test 0)
pico_add_extra_outputs(raw_step_test)

# ========================================
# TEST: AS5600 Single Encoder Test (J1 on Slave 1)
# Reads angle from AS5600 on GP12(SDA)/GP13(SCL)
# ========================================
add_executable(encoder_test test/encoder_test.c)
target_link_libraries(encoder_test pico_stdlib hardware_gpio)
pico_enable_stdio_usb(encoder_test 1)
pico_enable_stdio_uart(encoder_test 0)
pico_add_extra_outputs(encoder_test)

# ========================================
# TEST: Combined Encoder + Step Generator
# Minimal test: reads AS5600, drives PIO steps — no control loop
# ========================================
add_executable(combined_test test/combined_test.c)
pico_generate_pio_header(combined_test ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(combined_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(combined_test pico_stdlib hardware_gpio hardware_pio hardware_clocks hardware_timer)
pico_enable_stdio_usb(combined_test 1)
pico_enable_stdio_uart(combined_test 0)
pico_add_extra_outputs(combined_test)

# ========================================
# TEST: Stage 4 — Simple P-Control
# Single-core: encoder + PIO step + P-controller (no LADRC, no multicore)
# ========================================
add_executable(stage4_simple test/stage4_simple.c)
pico_generate_pio_header(stage4_simple ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(stage4_simple PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(stage4_simple pico_stdlib hardware_gpio hardware_pio hardware_clocks hardware_timer)
pico_enable_stdio_usb(stage4_simple 1)
pico_enable_stdio_uart(stage4_simple 0)
pico_add_extra_outputs(stage4_simple)

# ========================================
# TEST: E-Stop Relay Test
# Flashes to Master Pico — tests relay control, E-Stop button sense, MCP LEDs
# ========================================
add_executable(estop_relay_test test/estop_relay_test.c)
target_link_libraries(estop_relay_test pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(estop_relay_test 1)
pico_enable_stdio_uart(estop_relay_test 0)
pico_add_extra_outputs(estop_relay_test)

# ========================================
# DIAGNOSTIC: E-Stop Pin Poll (GP2)
# Prints GP2 state every 100ms — HI=normal, LO=estop
# ========================================
add_executable(estop_test test/estop_test.c)
target_link_libraries(estop_test pico_stdlib hardware_gpio)
pico_enable_stdio_usb(estop_test 1)
pico_enable_stdio_uart(estop_test 0)
pico_add_extra_outputs(estop_test)

# ========================================
# DIAGNOSTIC: Onboard LED Blink
# Flashes GP25 on/off every 500ms
# ========================================
add_executable(blink_test test/blink_test.c)
target_link_libraries(blink_test pico_stdlib hardware_gpio)
pico_enable_stdio_usb(blink_test 1)
pico_enable_stdio_uart(blink_test 0)
pico_add_extra_outputs(blink_test)

# ========================================
# DIAGNOSTIC: W5500 Detection Test
# SPI0: GP15(RST) GP16(MISO) GP17(CS) GP18(SCK) GP19(MOSI)
# Reads VERSIONR — should return 0x04
# ========================================
add_executable(w5500_detect_test test/w5500_detect_test.c)
target_link_libraries(w5500_detect_test pico_stdlib hardware_spi hardware_gpio)
pico_enable_stdio_usb(w5500_detect_test 1)
pico_enable_stdio_uart(w5500_detect_test 0)
pico_add_extra_outputs(w5500_detect_test)

# ========================================
# STAGE 4: Multicore P-Control (FIFO-based)
# Core 0: USB/Serial + command parsing → volatile globals
# Core 1: reads globals, 10kHz control loop
# ========================================
add_executable(stage4_multicore test/stage4_multicore.c)
pico_generate_pio_header(stage4_multicore ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(stage4_multicore PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(stage4_multicore
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_timer
    hardware_sync
)
pico_enable_stdio_usb(stage4_multicore 1)
pico_enable_stdio_uart(stage4_multicore 0)
pico_add_extra_outputs(stage4_multicore)

# ========================================
# TEST: W5500 UDP Echo Test
# Sends/receives UDP on port 27181, IP 10.10.10.10
# Uses ioLibrary_Driver for full socket API
# ========================================
add_executable(udp_echo_test test/udp_echo_test.c)
target_compile_definitions(udp_echo_test PRIVATE _WIZCHIP_=5500)
target_include_directories(udp_echo_test PRIVATE
    ${IOLIB_DIR}/Ethernet/W5500
    ${IOLIB_DIR}/Ethernet
    ${IOLIB_DIR}/Internet/DHCP
)
target_sources(udp_echo_test PRIVATE
    ${IOLIB_DIR}/Ethernet/W5500/w5500.c
    ${IOLIB_DIR}/Ethernet/socket.c
    ${IOLIB_DIR}/Ethernet/wizchip_conf.c
)
target_link_libraries(udp_echo_test
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(udp_echo_test 1)
pico_enable_stdio_uart(udp_echo_test 0)
pico_add_extra_outputs(udp_echo_test)

# ========================================
# DIAGNOSTIC: W5500 Bit-Bang Test
# Uses GPIO bit-bang instead of SPI peripheral
# Eliminates SPI config issues
# ========================================
add_executable(w5500_bitbang_test test/w5500_bitbang_test.c)
target_link_libraries(w5500_bitbang_test pico_stdlib hardware_gpio)
pico_enable_stdio_usb(w5500_bitbang_test 1)
pico_enable_stdio_uart(w5500_bitbang_test 0)
pico_add_extra_outputs(w5500_bitbang_test)

# ========================================
# TEST: TMC2209 Config + Step Generator (Combined)
# Phase 1: Configure all 4 TMC2209 drivers via UART
# Phase 2: Run step generator demo with trapezoidal motion
# Flash to Slave 2 Pico (Wrist + Gripper).
# ========================================
add_executable(tmc2209_config_test
    test/tmc2209_config_test.c
    servo/nite_stepgen.c
    inc/nite_tmc2209.c
)
pico_generate_pio_header(tmc2209_config_test ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(tmc2209_config_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR} inc)
target_link_libraries(tmc2209_config_test
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_uart
)
pico_enable_stdio_usb(tmc2209_config_test 1)
pico_enable_stdio_uart(tmc2209_config_test 0)
pico_add_extra_outputs(tmc2209_config_test)

# ========================================
# TEST: TMC2209 UART Communication Test
# Flashes to Slave 2 Pico
# Tests UART bus with 4x TMC2209 drivers on shared half-duplex UART
# Pins: GP0(TX) GP1(RX) GP28(ENABLE)
# ========================================
add_executable(tmc2209_uart_test
    test/tmc2209_uart_test.c
    inc/nite_tmc2209.c
)
target_include_directories(tmc2209_uart_test PRIVATE inc)
target_link_libraries(tmc2209_uart_test
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_timer
)
pico_enable_stdio_usb(tmc2209_uart_test 1)
pico_enable_stdio_uart(tmc2209_uart_test 0)
pico_add_extra_outputs(tmc2209_uart_test)

# ========================================
# Clean TMC UART library test (UART0, GP0/GP1, echo cancellation)
# ========================================
add_executable(tmc_uart_test test/tmc_uart_test.c inc/tmc_uart.c)
target_include_directories(tmc_uart_test PRIVATE inc)
target_link_libraries(tmc_uart_test pico_stdlib hardware_uart hardware_timer)
pico_enable_stdio_usb(tmc_uart_test 1)
pico_enable_stdio_uart(tmc_uart_test 0)
pico_add_extra_outputs(tmc_uart_test)

# ========================================
# Interactive TMC UART Config Test (scan, config, step, dump)
# Uses working tmc_uart.c library (LSB-first CRC + echo discard)
# ========================================
add_executable(tmc_uart_config_test test/tmc_uart_config_test.c inc/tmc_uart.c)
target_include_directories(tmc_uart_config_test PRIVATE inc)
target_link_libraries(tmc_uart_config_test pico_stdlib hardware_uart hardware_gpio hardware_timer)
pico_enable_stdio_usb(tmc_uart_config_test 1)
pico_enable_stdio_uart(tmc_uart_config_test 0)
pico_add_extra_outputs(tmc_uart_config_test)

# ========================================
# TEST: Multicore Serial Test — isolates USB IRQ on Core 1
# Core 0: LED blink (idle), Core 1: USB echo + heartbeat
# ========================================
add_executable(serial_multi_test test/serial_multi_test.c)
target_link_libraries(serial_multi_test
    pico_stdlib
    pico_multicore
    hardware_gpio
    hardware_irq
)
pico_enable_stdio_usb(serial_multi_test 1)
pico_enable_stdio_uart(serial_multi_test 0)
pico_add_extra_outputs(serial_multi_test)

# ========================================
# STAGE 4: Single-Axis LADRC Test (Original — continuous ESO, unstable at wo>~15)
# ========================================
add_executable(stage4_single_ladrc test/stage4_single_ladrc.c)
pico_generate_pio_header(stage4_single_ladrc ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(stage4_single_ladrc PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(stage4_single_ladrc
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_i2c
    hardware_sync
    hardware_timer
)
pico_enable_stdio_usb(stage4_single_ladrc 1)
pico_enable_stdio_uart(stage4_single_ladrc 0)
pico_add_extra_outputs(stage4_single_ladrc)

# ========================================
# STAGE 4 (FIXED): Single-Axis LADRC with discrete-time ESO
# Uses exact ZOH discretization + pole placement for guaranteed
# ESO stability at any wo. Also fixes zero_flag, relative STEP,
# output clamping, and deadband.
# ========================================
add_executable(stage4_single_ladrc_fixed test/stage4_single_ladrc_fixed.c)
pico_generate_pio_header(stage4_single_ladrc_fixed ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(stage4_single_ladrc_fixed PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(stage4_single_ladrc_fixed
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_sync
    hardware_timer
)
pico_enable_stdio_usb(stage4_single_ladrc_fixed 1)
pico_enable_stdio_uart(stage4_single_ladrc_fixed 0)
pico_add_extra_outputs(stage4_single_ladrc_fixed)

# ========================================
# STAGE 6: GRBL-like 2-Axis Open-Loop Step Generator
# Single core. Pure PIO step generation, NO encoder feedback.
# Position tracked by counting steps sent (open loop).
# Commands: G0/G1, G90/G91, G28, M17/M18, $ settings, ? status
#
# J1 (Axis 0): STEP=GP16 DIR=GP17 LIMIT=GP14
# J2 (Axis 1): STEP=GP18 DIR=GP19 LIMIT=GP15
# ========================================
add_executable(stage6_grbl_stepgen test/stage6_grbl_stepgen.c)
pico_generate_pio_header(stage6_grbl_stepgen ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(stage6_grbl_stepgen PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(stage6_grbl_stepgen
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_timer
)
pico_enable_stdio_usb(stage6_grbl_stepgen 1)
pico_enable_stdio_uart(stage6_grbl_stepgen 0)
pico_add_extra_outputs(stage6_grbl_stepgen)

# ========================================
# STAGE 5: Dual-Axis P-Control (J1 + J2, fixed PIO timing)
# Core 0: USB/Serial + command parsing
# Core 1: 1kHz control loop with encoder I2C + P-controller + PIO step gen
# Hardware (Slave 1):
#   J1: STEP=GP16 DIR=GP17 ENC=GP12/13 LIMIT=GP14
#   J2: STEP=GP18 DIR=GP19 ENC=GP6/7  LIMIT=GP15
# ========================================
add_executable(stage5_dual_axis test/stage5_dual_axis.c)
pico_generate_pio_header(stage5_dual_axis ${CMAKE_CURRENT_LIST_DIR}/pio/freq_generator.pio)
target_include_directories(stage5_dual_axis PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(stage5_dual_axis
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_timer
    hardware_sync
)
pico_enable_stdio_usb(stage5_dual_axis 1)
pico_enable_stdio_uart(stage5_dual_axis 0)
pico_add_extra_outputs(stage5_dual_axis)

# ========================================
# NITE369 Serial Jog — Simple 4-axis repeating-timer step controller
# Protocol: M<axis> <steps> <vel>, S=stop, E1/E0=enable, ?=status
# Flash to Slave 1 Pico (GP16-28).
# ========================================
add_executable(nite369_serial_jog
    serial_jog/main.c
)
target_link_libraries(nite369_serial_jog
    pico_stdlib
    hardware_timer
)
pico_enable_stdio_usb(nite369_serial_jog 1)
pico_enable_stdio_uart(nite369_serial_jog 0)
pico_add_extra_outputs(nite369_serial_jog)

# ========================================
# TEST: SPI Echo Test — Master (tests both CPHA=0 and CPHA=1)
# Flash to Master Pico. Sends pattern, expects echo from Slave.
# ========================================
add_executable(spi_echo_master_slave2 test/spi_echo_master_slave2.c)
target_include_directories(spi_echo_master_slave2 PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(spi_echo_master_slave2
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(spi_echo_master_slave2 1)
pico_enable_stdio_uart(spi_echo_master_slave2 0)
pico_add_extra_outputs(spi_echo_master_slave2)

# ========================================
# TEST: SPI Echo Test — Slave (minimal echo)
# Receives bytes and echoes them back. Flash to Slave 2 Pico.
# ========================================
add_executable(spi_echo_slave test/spi_echo_slave.c)
target_link_libraries(spi_echo_slave
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(spi_echo_slave 1)
pico_enable_stdio_uart(spi_echo_slave 0)
pico_add_extra_outputs(spi_echo_slave)

# ========================================
# DIAGNOSTIC: Axis 2 DIR Pin Test
# Toggles GP21 (DIR) every 2s, GP20 (STEP) every 500ms
# Flash to Slave 1 Pico. Probe GP21 with multimeter to verify direction control.
# ========================================
add_executable(diag_dir_pin test/diag_dir_pin.c)
target_link_libraries(diag_dir_pin pico_stdlib hardware_gpio)
pico_enable_stdio_usb(diag_dir_pin 1)
pico_enable_stdio_uart(diag_dir_pin 0)
pico_add_extra_outputs(diag_dir_pin)

# ========================================
# TEST: AS5600 x4 — reads all 4 encoders on Slave 1 using hardware I2C
# J1=GP12/13 J2A=GP6/7 J2B=GP8/9 J3=GP10/11
# Each encoder on separate SDA/SCL pair, same addr 0x36
# ========================================
add_executable(encoder_x4_test test/encoder_x4_test.c)
target_link_libraries(encoder_x4_test pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(encoder_x4_test 1)
pico_enable_stdio_uart(encoder_x4_test 0)
pico_add_extra_outputs(encoder_x4_test)

# ========================================
# TEST: AS5600 Slave 1 All-4 Encoder Check — bit-bang I2C
# Uses software bit-bang I2C (no pin-switching) for independent
# probe of all 4 encoders. More robust diagnostics than hardware I2C.
# J1=GP12/13 J2A=GP6/7 J2B=GP8/9 J3=GP10/11
# Includes bus scan + live angle table + detail view.
# ========================================
add_executable(encoder_slave1_all test/encoder_slave1_all.c)
target_link_libraries(encoder_slave1_all pico_stdlib hardware_gpio)
pico_enable_stdio_usb(encoder_slave1_all 1)
pico_enable_stdio_uart(encoder_slave1_all 0)
pico_add_extra_outputs(encoder_slave1_all)

# ========================================
# TEST: AS5600 Slave 2 Encoder Read (3x)
# J4=GP12/13(i2c0) J5=GP8/9(i2c0) J6=GP10/11(i2c1)
# Each encoder on separate SDA/SCL pair, all addr 0x36
# ========================================
add_executable(encoder_s2_test test/encoder_s2_test.c)
target_link_libraries(encoder_s2_test pico_stdlib hardware_i2c hardware_gpio)
pico_enable_stdio_usb(encoder_s2_test 1)
pico_enable_stdio_uart(encoder_s2_test 0)
pico_add_extra_outputs(encoder_s2_test)

# ========================================
# SPI relay: Master requests Slave 2 encoder data over SPI
# ========================================
add_executable(spi_enc_master test/spi_enc_master.c)
target_link_libraries(spi_enc_master pico_stdlib hardware_spi hardware_gpio)
pico_enable_stdio_usb(spi_enc_master 1)
pico_enable_stdio_uart(spi_enc_master 0)
pico_add_extra_outputs(spi_enc_master)

add_executable(spi_enc_slave_s2 test/spi_enc_slave_s2.c)
target_link_libraries(spi_enc_slave_s2 pico_stdlib hardware_spi hardware_i2c hardware_gpio)
pico_enable_stdio_usb(spi_enc_slave_s2 0)
pico_enable_stdio_uart(spi_enc_slave_s2 0)
pico_add_extra_outputs(spi_enc_slave_s2)

# ========================================
# SPI Command Slave 2 — Wrist (GPIO step, no PIO)
# 4 independent axes: J4, J5, J6, Gripper.
# Flash to Slave 2 Pico.
# ========================================
add_executable(spi_cmd_slave_s2 test/spi_cmd_slave_s2.c inc/tmc_uart.c inc/ws2812.c inc/motion_profile.c inc/motion_marlin.c inc/homing.c inc/config_store.c ${VERSION_HEADER_OUT})
target_include_directories(spi_cmd_slave_s2 PRIVATE inc ${CMAKE_CURRENT_BINARY_DIR})
target_compile_definitions(spi_cmd_slave_s2 PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000  # printf must never block the SPI loop when no USB host
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

# ========================================
# SPI Command Slave 1 — 8-byte protocol over SPI, GPIO step (no PIO)
# Flash to Slave 1 Pico (Arm Base).
# ========================================
add_executable(spi_cmd_slave_s1
    test/spi_cmd_slave_s1.c
    inc/motion_profile.c
    inc/motion_marlin.c
    inc/homing.c
    inc/config_store.c
    ${VERSION_HEADER_OUT}
)
target_include_directories(spi_cmd_slave_s1 PRIVATE inc ${CMAKE_CURRENT_BINARY_DIR})
target_compile_definitions(spi_cmd_slave_s1 PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000  # printf must never block the SPI loop when no USB host
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
# FACTORY RESET: erase the config sector on any Pico.
# Flash this ONCE to wipe stored config (step/accel/speed/limits), then
# re-flash the real firmware — it boots with factory defaults.
# ========================================
add_executable(config_wipe test/config_wipe.c)
target_link_libraries(config_wipe pico_stdlib hardware_flash hardware_gpio)
pico_enable_stdio_usb(config_wipe 1)
pico_enable_stdio_uart(config_wipe 0)
pico_add_extra_outputs(config_wipe)

# ========================================
# SPI Command Master — USB serial CLI → SPI commands to Slave 1 + Slave 2
# Flash to Master Pico. Dual CS: GP9=Slave1, GP13=Slave2.
# Flash to Master Pico. Connects to PC via USB, talks to Slave 2 via SPI.
# ========================================
add_executable(spi_cmd_master test/spi_cmd_master.c)
target_include_directories(spi_cmd_master PRIVATE inc)
target_link_libraries(spi_cmd_master
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(spi_cmd_master 1)
pico_enable_stdio_uart(spi_cmd_master 0)
pico_add_extra_outputs(spi_cmd_master)

# ========================================
# SPI Command Master with W5500 Ethernet
# TCP server on port 23, forwards Nite369 commands to slaves via SPI1.
# W5500 on SPI0 (GP15-19), Slaves on SPI1 (GP9-13).
# ========================================
add_executable(spi_cmd_master_eth
    test/spi_cmd_master_eth.c
    inc/w5500_tcp.c
    inc/gcode_parser.c
    inc/config_store.c
    ${VERSION_HEADER_OUT}
)
target_include_directories(spi_cmd_master_eth PRIVATE
    inc
    ${CMAKE_CURRENT_BINARY_DIR}
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
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000  # printf must never block the SPI loop when no USB host
)
pico_enable_stdio_usb(spi_cmd_master_eth 1)
pico_enable_stdio_uart(spi_cmd_master_eth 0)
pico_add_extra_outputs(spi_cmd_master_eth)

# ========================================
# Slave 1 Motor Test — simple GPIO step, J2A+J2B shoulder pairing
# Flash to Slave 1 Pico (Arm Base).
# ========================================
add_executable(motor_s1_test test/motor_s1_test.c)
target_link_libraries(motor_s1_test
    pico_stdlib
    hardware_gpio
    hardware_timer
)
pico_enable_stdio_usb(motor_s1_test 1)
pico_enable_stdio_uart(motor_s1_test 0)
pico_add_extra_outputs(motor_s1_test)

# ========================================
# USB serial echo test — Master (COM A) and Slave 1 (COM B)
# Two data cables to laptop, no SPI wiring. Two serial monitors.
# ========================================
add_executable(usb_echo_master test/usb_echo_test.c)
target_compile_definitions(usb_echo_master PRIVATE PICO_NAME="MASTER")
target_link_libraries(usb_echo_master pico_stdlib)
pico_enable_stdio_usb(usb_echo_master 1)
pico_enable_stdio_uart(usb_echo_master 0)
pico_add_extra_outputs(usb_echo_master)

add_executable(usb_echo_slave1 test/usb_echo_test.c)
target_compile_definitions(usb_echo_slave1 PRIVATE PICO_NAME="SLAVE1")
target_link_libraries(usb_echo_slave1 pico_stdlib)
pico_enable_stdio_usb(usb_echo_slave1 1)
pico_enable_stdio_uart(usb_echo_slave1 0)
pico_add_extra_outputs(usb_echo_slave1)

# ========================================
# DIAGNOSTIC: Echo test on CS1 path (GP9 -> GP5)
# Same as spi_echo_master_slave2 but targets Slave 1
# Flash echo_s1_diag to Master, spi_echo_slave to Slave 1
# ========================================
add_executable(echo_s1_diag test/echo_s1_diag.c)
target_link_libraries(echo_s1_diag
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_enable_stdio_usb(echo_s1_diag 1)
pico_enable_stdio_uart(echo_s1_diag 0)
pico_add_extra_outputs(echo_s1_diag)

# ========================================
# DIAGNOSTIC: Raw SPI byte transfer (no echo protocol)
# Master sends fixed pattern, slave counts received bytes
# ========================================
add_executable(spi_raw_master test/spi_raw_test.c)
target_compile_definitions(spi_raw_master PRIVATE SPI_RAW_MASTER)
target_link_libraries(spi_raw_master pico_stdlib hardware_spi hardware_gpio)
pico_enable_stdio_usb(spi_raw_master 1)
pico_enable_stdio_uart(spi_raw_master 0)
pico_add_extra_outputs(spi_raw_master)

add_executable(spi_raw_slave test/spi_raw_test.c)
target_compile_definitions(spi_raw_slave PRIVATE SPI_RAW_SLAVE)
target_link_libraries(spi_raw_slave pico_stdlib hardware_spi hardware_gpio)
pico_enable_stdio_usb(spi_raw_slave 1)
pico_enable_stdio_uart(spi_raw_slave 0)
pico_add_extra_outputs(spi_raw_slave)

# ========================================
# W5500 Ethernet hardware test
# Tests SPI comms, link detection, TCP echo server
# W5500 on SPI0: GP15=RST, GP16=MISO, GP17=CS, GP18=SCK, GP19=MOSI
# ========================================
add_executable(w5500_test
    test/w5500_test.c
)
target_include_directories(w5500_test PRIVATE
    inc
    ${IOLIB_DIR}/Ethernet/W5500
    ${IOLIB_DIR}/Ethernet
)
target_sources(w5500_test PRIVATE
    ${IOLIB_DIR}/Ethernet/W5500/w5500.c
    ${IOLIB_DIR}/Ethernet/socket.c
    ${IOLIB_DIR}/Ethernet/wizchip_conf.c
)
target_link_libraries(w5500_test
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
)
target_compile_definitions(w5500_test PRIVATE
    _WIZCHIP_=5500
)
pico_enable_stdio_usb(w5500_test 1)
pico_enable_stdio_uart(w5500_test 0)
pico_add_extra_outputs(w5500_test)

# ========================================
# W5500 UDP Master — ORIGINAL working version from initial testing
# Retrieved from git history (commit eaa1e09 firmware/master/main.c).
# UDP receiver on 10.10.10.10:27181, forwards 8x int32 to 2 servo Picos.
# NOTE: uses OLD slave SPI pins (CS1=13, CS2=11, SCK=14, MOSI=15, MISO=12).
# ========================================
add_executable(w5500_udp_master_test
    test/w5500_udp_master_test.c
)
target_include_directories(w5500_udp_master_test PRIVATE
    inc
    ${IOLIB_DIR}/Ethernet/W5500
    ${IOLIB_DIR}/Ethernet
)
target_sources(w5500_udp_master_test PRIVATE
    ${IOLIB_DIR}/Ethernet/W5500/w5500.c
    ${IOLIB_DIR}/Ethernet/socket.c
    ${IOLIB_DIR}/Ethernet/wizchip_conf.c
)
target_link_libraries(w5500_udp_master_test
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
)
target_compile_definitions(w5500_udp_master_test PRIVATE
    _WIZCHIP_=5500
)
pico_enable_stdio_usb(w5500_udp_master_test 1)
pico_enable_stdio_uart(w5500_udp_master_test 0)
pico_add_extra_outputs(w5500_udp_master_test)

