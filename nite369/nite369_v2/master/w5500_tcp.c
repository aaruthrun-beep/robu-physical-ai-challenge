#include "w5500_tcp.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500.h"

// --- W5500 SPI pins (SPI0 on Master) ---
#define W5500_SPI   spi0
#define W5500_SCK   18
#define W5500_MOSI  19
#define W5500_MISO  16
#define W5500_CS    17
#define W5500_RST   15

// --- Network config ---
static uint8_t MAC_ADDR[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
static uint8_t IP_ADDR[4]  = {192, 168, 1, 50};
static uint8_t SUBNET[4]   = {255, 255, 255, 0};
static uint8_t GATEWAY[4]  = {192, 168, 1, 1};

static bool link_up = false;
static uint32_t current_ip = 0;

// --- SPI callbacks for ioLibrary ---
static void w5500_cs_select(void)  { gpio_put(W5500_CS, 0); }
static void w5500_cs_deselect(void){ gpio_put(W5500_CS, 1); }
static uint8_t w5500_spi_read(void) {
    uint8_t rx;
    spi_read_blocking(W5500_SPI, 0xFF, &rx, 1);
    return rx;
}
static void w5500_spi_write(uint8_t data) {
    spi_write_blocking(W5500_SPI, &data, 1);
}

// --- Client tracking (sockets 0-7) ---
// Sockets cycle through three roles:
//   LISTENING  — waiting for a new TCP connection on port 23
//   CONNECTED  — serving an established client
//   CLOSED     — between disconnect and re-listen
//
// At any time, exactly one socket is in LISTENING state (the "next" listener).
// When that socket transitions to CONNECTED, we promote the next socket to
// LISTENING.  When a socket disconnects, it becomes the new LISTENING socket.
typedef struct {
    bool in_use;     // true = this socket has been opened as a server socket
} client_slot_t;

static client_slot_t slots[8];     // one per socket
static int listen_sock = 0;        // which socket is currently the listener
static uint32_t last_listen_ok_ms = 0; // to_ms_since_boot when listener was last confirmed healthy
#define LISTENER_WATCHDOG_MS  5000    // 5 s without a SOCK_LISTEN -> force recovery

static void open_listen(int sock) {
    if (sock < 0 || sock > 7) return;
    // Close if in any non-closed state
    uint8_t st = getSn_SR(sock);
    if (st != SOCK_CLOSED) {
        close(sock);
        sleep_ms(10);
    }
    int ret = socket(sock, Sn_MR_TCP, W5500_TCP_PORT, 0);
    sleep_ms(5);
    uint8_t st2 = getSn_SR(sock);
    if (st2 == SOCK_INIT) {
        int lr = listen(sock);
        sleep_ms(10);
        uint8_t st3 = getSn_SR(sock);
        printf("[W5500] sock%d: socket=%d listen=%d -> 0x%02X\n", sock, ret, lr, st3);
        if (st3 == SOCK_LISTEN || st3 == SOCK_ESTABLISHED) {
            slots[sock].in_use = true;
            if (st3 == SOCK_LISTEN) listen_sock = sock;
        }
    } else {
        printf("[W5500] sock%d: socket=%d -> 0x%02X (not SOCK_INIT, skipped listen)\n",
               sock, ret, st2);
    }
}

// ================================================================
//  Init
// ================================================================
bool w5500_tcp_init(void) {
    spi_init(W5500_SPI, 1 * 1000 * 1000);
    gpio_set_function(W5500_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(W5500_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(W5500_MISO, GPIO_FUNC_SPI);

    gpio_init(W5500_CS);
    gpio_set_dir(W5500_CS, GPIO_OUT);
    gpio_put(W5500_CS, 1);

    gpio_init(W5500_RST);
    gpio_set_dir(W5500_RST, GPIO_OUT);
    gpio_put(W5500_RST, 0);
    sleep_ms(50);
    gpio_put(W5500_RST, 1);
    sleep_ms(100);

    reg_wizchip_cs_cbfunc(w5500_cs_select, w5500_cs_deselect);
    reg_wizchip_spi_cbfunc(w5500_spi_read, w5500_spi_write);

    uint8_t txsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    uint8_t rxsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(txsize, rxsize);
    sleep_ms(100);

    setSHAR(MAC_ADDR);
    setSIPR(IP_ADDR);
    setSUBR(SUBNET);
    setGAR(GATEWAY);

    uint8_t ver = getMR();
    printf("W5500 MR: 0x%02X\n", ver);
    current_ip = (IP_ADDR[0] << 24) | (IP_ADDR[1] << 16) | (IP_ADDR[2] << 8) | IP_ADDR[3];
    printf("W5500 initialized. IP: %d.%d.%d.%d\n",
           IP_ADDR[0], IP_ADDR[1], IP_ADDR[2], IP_ADDR[3]);

    memset(slots, 0, sizeof(slots));
    return true;
}

// ================================================================
//  Link / IP
// ================================================================
bool w5500_tcp_is_link_up(void) {
    uint8_t phy = getPHYCFGR();
    link_up = (phy & 0x01) != 0;
    return link_up;
}

uint32_t w5500_tcp_get_ip(void) {
    return current_ip;
}

// ================================================================
//  Listen — open the first listener (socket 0)
// ================================================================
void w5500_tcp_listen(void) {
    // Close all sockets first
    for (int i = 0; i < 8; i++) {
        if (getSn_SR(i) != SOCK_CLOSED) {
            close(i);
            sleep_ms(5);
        }
    }
    memset(slots, 0, sizeof(slots));
    listen_sock = 0;
    printf("[W5500] Multi-client TCP server on port %d (max %d clients)\n",
           W5500_TCP_PORT, W5500_MAX_CLIENTS);
    open_listen(0);
}

// ================================================================
//  Watchdog: recover stuck listener
// ================================================================
//
// Called at the end of every poll cycle.  Detects and fixes three stuck
// states:
//
// 1. No socket in SOCK_LISTEN — listener was never opened or got
//    silently killed by W5500 glitch / SPI noise.
// 2. listen_sock stuck in intermediate state — SYN_RECV, SYN_SENT,
//    SOCK_INIT, or SOCK_CLOSED that never progressed to LISTEN or
//    ESTABLISHED.
// 3. Stale listener — listen_sock is SOCK_LISTEN but we haven't
//    reconfirmed it in LISTENER_WATCHDOG_MS (catches phantom LISTEN
//    where the W5500 register says LISTEN but the socket won't accept
//    new connections).
//
// Recovery cascade:
//  a. If any socket is in SOCK_LISTEN, trust it and reset the timer.
//  b. Otherwise, force-close listen_sock if stuck, then try every
//     socket (0..7) until open_listen succeeds.
//  c. Nuclear option: close ALL sockets and re-init from scratch.
static void listener_watchdog(uint32_t now_ms) {
    // --- Pass 1: scan for any socket actually in SOCK_LISTEN ---
    bool found_listen = false;
    for (int i = 0; i < 8; i++) {
        if (getSn_SR(i) == SOCK_LISTEN) {
            if (i != listen_sock) {
                printf("[W5500] watchdog: socket %d is SOCK_LISTEN (listen_sock was %d), adopting\n",
                       i, listen_sock);
                listen_sock = i;
            }
            found_listen = true;
            last_listen_ok_ms = now_ms;
            break;   // one listener is enough
        }
    }
    if (found_listen) return;

    // --- Pass 2: no SOCK_LISTEN found ---
    // If the designated listener is stuck in a recoverable intermediate
    // state, force-close it first to free the port.
    if (listen_sock >= 0 && listen_sock <= 7) {
        uint8_t st = getSn_SR(listen_sock);
        if (st != SOCK_LISTEN && st != SOCK_ESTABLISHED) {
            printf("[W5500] watchdog: sock%d stuck in 0x%02X, force-closing\n",
                   listen_sock, st);
            close(listen_sock);
            sleep_ms(10);
            slots[listen_sock].in_use = false;
        }
    }

    // Try every socket until one opens as listener.
    for (int i = 0; i < 8; i++) {
        if (getSn_SR(i) == SOCK_CLOSED) {
            printf("[W5500] watchdog: opening sock%d as replacement listener\n", i);
            listen_sock = i;
            open_listen(i);
            if (getSn_SR(i) == SOCK_LISTEN) {
                last_listen_ok_ms = now_ms;
                return;
            }
        }
    }

    // --- Pass 3: nuclear — all sockets occupied or corrupted ---
    if (now_ms - last_listen_ok_ms > LISTENER_WATCHDOG_MS * 2) {
        printf("[W5500] watchdog: FULL RESET (no free socket for %lu ms)\n",
               (unsigned long)(now_ms - last_listen_ok_ms));
        for (int i = 0; i < 8; i++) {
            close(i);
            slots[i].in_use = false;
        }
        sleep_ms(50);
        listen_sock = 0;
        open_listen(0);
        last_listen_ok_ms = now_ms;
    }
}

// ================================================================
//  Poll: check all sockets for state changes + data
// ================================================================
bool w5500_tcp_poll(char *buf, int bufsize, int *client_sock) {
    uint8_t phy = getPHYCFGR();
    link_up = (phy & 0x01) != 0;
    if (!link_up) return false;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    for (int sock = 0; sock < 8; sock++) {
        uint8_t st = getSn_SR(sock);

        if (st == SOCK_ESTABLISHED) {
            // Connected client — check for data
            uint16_t rx_size = getSn_RX_RSR(sock);
            if (rx_size > 0) {
                int len = recv(sock, (uint8_t *)buf, bufsize - 1);
                if (len > 0) {
                    buf[len] = '\0';
                    *client_sock = sock;
                    if (!slots[sock].in_use) {
                        printf("[W5500] Client connected on socket %d\n", sock);
                        slots[sock].in_use = true;
                    }
                    return true;
                }
            }
        }
        else if (st == SOCK_CLOSE_WAIT || st == SOCK_CLOSING ||
                 st == SOCK_TIME_WAIT) {
            printf("[W5500] Client disconnected on socket %d (st=0x%02X)\n", sock, st);
            close(sock);
            sleep_ms(10);
            slots[sock].in_use = false;
            listen_sock = sock;
            open_listen(sock);
            if (getSn_SR(sock) == SOCK_LISTEN) last_listen_ok_ms = now_ms;
        }
        else if (st == SOCK_CLOSED) {
            if (slots[sock].in_use) {
                printf("[W5500] Socket %d closed unexpectedly, re-listening\n", sock);
                slots[sock].in_use = false;
                listen_sock = sock;
                open_listen(sock);
                if (getSn_SR(sock) == SOCK_LISTEN) last_listen_ok_ms = now_ms;
            }
        }
        else if (st == SOCK_LISTEN) {
            if (sock != listen_sock) {
                printf("[W5500] Socket %d now listening (was listen_sock=%d)\n",
                       sock, listen_sock);
                listen_sock = sock;
            }
            last_listen_ok_ms = now_ms;
        }
    }

    // After the listener accepted a connection (went SOCK_ESTABLISHED),
    // open the next free socket as the new listener.
    if (listen_sock >= 0 && listen_sock <= 7) {
        uint8_t lst_st = getSn_SR(listen_sock);
        if (lst_st == SOCK_ESTABLISHED || lst_st == SOCK_CLOSE_WAIT) {
            for (int try_sock = 0; try_sock < 8; try_sock++) {
                if (try_sock == listen_sock) continue;
                uint8_t try_st = getSn_SR(try_sock);
                if (try_st == SOCK_CLOSED && !slots[try_sock].in_use) {
                    printf("[W5500] Listener moved: sock%d is client, opening sock%d as listener\n",
                           listen_sock, try_sock);
                    open_listen(try_sock);
                    if (getSn_SR(try_sock) == SOCK_LISTEN) last_listen_ok_ms = now_ms;
                    break;
                }
            }
        }
    }

    // --- Listener watchdog ---
    // Runs every poll cycle.  If no SOCK_LISTEN socket exists for
    // longer than LISTENER_WATCHDOG_MS, force-open one.
    if (now_ms - last_listen_ok_ms > LISTENER_WATCHDOG_MS) {
        listener_watchdog(now_ms);
    }

    return false;
}

// ================================================================
//  Send to specific client socket
// ================================================================
bool w5500_tcp_send_to(int sock, const char *data, int len) {
    if (sock < 0 || sock > 7) return false;
    uint8_t st = getSn_SR(sock);
    if (st != SOCK_ESTABLISHED) return false;
    int sent = send(sock, (uint8_t *)data, len);
    return sent == len;
}

// ================================================================
//  Client connected check
// ================================================================
bool w5500_tcp_is_client_connected(int sock) {
    if (sock < 0 || sock > 7) return false;
    return getSn_SR(sock) == SOCK_ESTABLISHED;
}
