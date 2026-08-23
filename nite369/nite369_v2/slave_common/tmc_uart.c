#include "tmc_uart.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

static uint8_t tmc_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t cur = data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if ((crc >> 7) ^ (cur & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            cur >>= 1;
        }
    }
    return crc;
}

void tmc_uart_init(void) {
    uart_init(TMC_UART_ID, TMC_BAUD);
    gpio_set_function(TMC_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TMC_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(TMC_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(TMC_UART_ID, true);
}

static void tmc_flush_rx(void) {
    while (uart_is_readable(TMC_UART_ID)) {
        uart_getc(TMC_UART_ID);
    }
}

void tmc_write_register(uint8_t addr, uint8_t reg, uint32_t value) {
    uint8_t datagram[8];
    datagram[0] = 0x05;
    datagram[1] = addr;         // raw address
    datagram[2] = reg | 0x80;   // write flag on register byte
    datagram[3] = (value >> 24) & 0xFF;
    datagram[4] = (value >> 16) & 0xFF;
    datagram[5] = (value >> 8) & 0xFF;
    datagram[6] = value & 0xFF;
    datagram[7] = tmc_crc8(datagram, 7);

    tmc_flush_rx();
    uart_write_blocking(TMC_UART_ID, datagram, 8);

    // Read back echo (8 bytes) with per-byte timeout
    for (int i = 0; i < 8; i++) {
        absolute_time_t deadline = make_timeout_time_ms(20);
        while (!uart_is_readable(TMC_UART_ID)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) < 0) return;
        }
        uart_getc(TMC_UART_ID);
    }
}

bool tmc_read_register(uint8_t addr, uint8_t reg, uint32_t *value) {
    // TMC2209 reads use the SAME 8-byte datagram as writes: sync 0x05,
    // address, register byte with the read/write flag, then 4 zero data
    // bytes and CRC. The old 4-byte request made the driver reply with an
    // error datagram (writes worked because they used the full 8 bytes).
    uint8_t request[8];
    request[0] = 0x05;
    request[1] = addr;
    request[2] = reg | 0x80;   // read/write flag (set = read on TMC2209)
    request[3] = 0;
    request[4] = 0;
    request[5] = 0;
    request[6] = 0;
    request[7] = tmc_crc8(request, 7);

    tmc_flush_rx();
    uart_write_blocking(TMC_UART_ID, request, 8);

    // Read back the 8-byte reply (per-byte timeout).
    uint8_t reply[8];
    for (int i = 0; i < 8; i++) {
        absolute_time_t deadline = make_timeout_time_ms(20);
        while (!uart_is_readable(TMC_UART_ID)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) < 0) {
                tmc_flush_rx();
                return false;
            }
        }
        reply[i] = uart_getc(TMC_UART_ID);
    }

    if (tmc_crc8(reply, 7) != reply[7]) {
        return false;
    }

    *value = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16) |
             ((uint32_t)reply[5] << 8)  | reply[6];
    return true;
}
