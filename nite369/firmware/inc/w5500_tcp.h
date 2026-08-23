#ifndef W5500_TCP_H
#define W5500_TCP_H

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
// True when a client is currently connected (SOCK_ESTABLISHED).
bool w5500_tcp_is_connected(void);

#endif
