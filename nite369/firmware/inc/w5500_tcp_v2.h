#ifndef W5500_TCP_V2_H
#define W5500_TCP_V2_H

/**
 * @file w5500_tcp_v2.h
 * @brief W5500 TCP server library — FIXED v2
 *
 * Drop-in replacement for inc/w5500_tcp.h. Use ONLY one of the two sets:
 * replace w5500_tcp.c/h with w5500_tcp_v2.c/h in the build (do NOT compile
 * both in the same target, or you will get duplicate-symbol link errors).
 *
 * Fixes over v1 (see w5500_tcp_v2.c header comment):
 *   1. Chip verification now reads VERSIONR (0x04), not MR.
 *   2. w5500_tcp_init() returns false when the chip is not detected.
 *   3. Burst SPI callbacks registered (used by ioLibrary for faster I/O).
 */

#include <stdint.h>
#include <stdbool.h>

#define W5500_TCP_PORT  23      // Default telnet/serial port
#define W5500_TX_BUF    2048
#define W5500_RX_BUF    2048

bool w5500_tcp_init(void);
bool w5500_tcp_is_link_up(void);
uint32_t w5500_tcp_get_ip(void);
bool w5500_tcp_poll(char *buf, int bufsize);
bool w5500_tcp_send(const char *data, int len);
void w5500_tcp_listen(void);

#endif // W5500_TCP_V2_H
