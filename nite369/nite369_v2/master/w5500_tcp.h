#ifndef W5500_TCP_H
#define W5500_TCP_H

#include <stdint.h>
#include <stdbool.h>

#define W5500_TCP_PORT    23     // Telnet/serial port
#define W5500_TX_BUF      2048
#define W5500_RX_BUF      2048
#define W5500_MAX_CLIENTS 7      // Sockets 1-7 for clients (socket 0 = listener)
#define W5500_LISTEN_SOCK 0      // Socket 0 is always the TCP listener

// --- Init / link ---
bool w5500_tcp_init(void);
bool w5500_tcp_is_link_up(void);
uint32_t w5500_tcp_get_ip(void);

// --- Server lifecycle ---
void w5500_tcp_listen(void);

// --- Multi-client poll ---
// Returns true if any client sent data. Fills buf with the received data
// and sets *client_sock to the socket number that received it (1-7).
// The caller must pass client_sock to w5500_tcp_send_to() for the reply.
bool w5500_tcp_poll(char *buf, int bufsize, int *client_sock);

// --- Per-client send ---
// Send data to a specific client socket (1-7).
bool w5500_tcp_send_to(int sock, const char *data, int len);

// --- Per-client state queries ---
bool w5500_tcp_is_client_connected(int sock);

// --- Legacy single-client wrappers (compat during migration) ---
// w5500_tcp_send() and w5500_tcp_is_connected() are removed — use
// w5500_tcp_send_to(sock) and w5500_tcp_is_client_connected(sock).

#endif
