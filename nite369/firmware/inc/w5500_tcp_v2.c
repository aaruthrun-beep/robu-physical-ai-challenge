#include "w5500_tcp_v2.h"
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
static bool server_connected = false;
static uint8_t server_sock = 0;

// --- SPI callbacks for ioLibrary ---
static void w5500_cs_select(void) {
    gpio_put(W5500_CS, 0);
}

static void w5500_cs_deselect(void) {
    gpio_put(W5500_CS, 1);
}

static uint8_t w5500_spi_read(void) {
    uint8_t rx;
    spi_read_blocking(W5500_SPI, 0xFF, &rx, 1);
    return rx;
}

static void w5500_spi_write(uint8_t data) {
    spi_write_blocking(W5500_SPI, &data, 1);
}

// --- v2: burst callbacks (ioLibrary falls back to byte mode without these,
//        which is correct but slower for multi-byte buffer transfers) ---
static void w5500_spi_read_burst(uint8_t *pBuf, uint16_t len) {
    spi_read_blocking(W5500_SPI, 0xFF, pBuf, len);
}

static void w5500_spi_write_burst(uint8_t *pBuf, uint16_t len) {
    spi_write_blocking(W5500_SPI, pBuf, len);
}

// --- Initialize W5500 ---
bool w5500_tcp_init(void) {
    // Init SPI0
    spi_init(W5500_SPI, 10 * 1000 * 1000);  // 10 MHz
    gpio_set_function(W5500_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(W5500_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(W5500_MISO, GPIO_FUNC_SPI);

    // CS pin
    gpio_init(W5500_CS);
    gpio_set_dir(W5500_CS, GPIO_OUT);
    gpio_put(W5500_CS, 1);

    // Reset pin
    gpio_init(W5500_RST);
    gpio_set_dir(W5500_RST, GPIO_OUT);
    gpio_put(W5500_RST, 0);
    sleep_ms(50);
    gpio_put(W5500_RST, 1);
    sleep_ms(100);

    // Register ioLibrary callbacks (byte + burst)
    reg_wizchip_cs_cbfunc(w5500_cs_select, w5500_cs_deselect);
    reg_wizchip_spi_cbfunc(w5500_spi_read, w5500_spi_write);
    reg_wizchip_spiburst_cbfunc(w5500_spi_read_burst, w5500_spi_write_burst);

    // Socket buffer allocation (2KB each) — does soft reset
    uint8_t txsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    uint8_t rxsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    if (wizchip_init(txsize, rxsize) != 0) {
        printf("w5500_tcp: wizchip_init FAILED\n");
        return false;
    }
    sleep_ms(100);

    // --- v2 FIX #1 & #2: verify the chip is really there ---
    // v1 read getMR() (the Mode Register — always 0x00 here) and always
    // returned true, so "W5500 init failed" could never trigger even with
    // no chip wired up. VERSIONR is hardwired to 0x04 on the W5500.
    uint8_t ver = getVERSIONR();
    printf("W5500 VERSIONR: 0x%02X (expect 0x04)\n", ver);
    if (ver != 0x04) {
        printf("w5500_tcp: W5500 NOT DETECTED (VERSIONR=0x%02X)\n", ver);
        printf("  Check wiring: SCK=GP%d MOSI=GP%d MISO=GP%d CS=GP%d RST=GP%d\n",
               W5500_SCK, W5500_MOSI, W5500_MISO, W5500_CS, W5500_RST);
        return false;
    }

    // Set network info (after init, so it sticks)
    setSHAR(MAC_ADDR);
    setSIPR(IP_ADDR);
    setSUBR(SUBNET);
    setGAR(GATEWAY);

    current_ip = (IP_ADDR[0] << 24) | (IP_ADDR[1] << 16) | (IP_ADDR[2] << 8) | IP_ADDR[3];
    printf("W5500 initialized. IP: %d.%d.%d.%d\n",
           IP_ADDR[0], IP_ADDR[1], IP_ADDR[2], IP_ADDR[3]);

    return true;
}

// --- Check link status ---
bool w5500_tcp_is_link_up(void) {
    uint8_t phy = getPHYCFGR();
    // PHYCFGR bit0 = LNK (1 = link up) — see PHYCFGR_LNK_ON in w5500.h
    link_up = (phy & PHYCFGR_LNK_ON) != 0;
    return link_up;
}

// --- Get current IP ---
uint32_t w5500_tcp_get_ip(void) {
    return current_ip;
}

// --- Open and listen on TCP socket ---
void w5500_tcp_listen(void) {
    printf("[W5500] Opening TCP socket...\n");
    int ret = socket(server_sock, Sn_MR_TCP, W5500_TCP_PORT, 0);
    printf("[W5500] socket() returned %d, Sn_SR=0x%02X\n", ret, getSn_SR(server_sock));

    // Wait for SOCK_INIT state
    int timeout = 500;
    while (getSn_SR(server_sock) != SOCK_INIT && timeout-- > 0) {
        sleep_ms(1);
    }
    printf("[W5500] After wait: Sn_SR=0x%02X (expect 0x13=INIT)\n", getSn_SR(server_sock));

    ret = listen(server_sock);
    printf("[W5500] listen() returned %d, Sn_SR=0x%02X\n", ret, getSn_SR(server_sock));

    // Verify we reached SOCK_LISTEN
    timeout = 500;
    while (getSn_SR(server_sock) != SOCK_LISTEN && timeout-- > 0) {
        sleep_ms(1);
    }
    printf("[W5500] Final Sn_SR=0x%02X (expect 0x14=LISTEN)\n", getSn_SR(server_sock));
}

// --- Poll for received data ---
static uint8_t last_sock_state = 0xFF;
bool w5500_tcp_poll(char *buf, int bufsize) {
    // Check link
    uint8_t phy = getPHYCFGR();
    link_up = (phy & PHYCFGR_LNK_ON) != 0;
    if (!link_up) {
        server_connected = false;
        return false;
    }

    uint8_t sock_status = getSn_SR(server_sock);

    // Log state changes
    if (sock_status != last_sock_state) {
        printf("[W5500] Socket state: 0x%02X (was 0x%02X)\n", sock_status, last_sock_state);
        last_sock_state = sock_status;
    }

    // Handle connection state
    if (sock_status == SOCK_CLOSED) {
        if (server_connected) {
            printf("[W5500] Socket closed by remote\n");
        }
        server_connected = false;
        printf("[W5500] Opening socket...\n");
        socket(server_sock, Sn_MR_TCP, W5500_TCP_PORT, 0);
        return false;
    }

    if (sock_status == SOCK_INIT) {
        printf("[W5500] Socket init, listening...\n");
        int ret = listen(server_sock);
        printf("[W5500] listen() returned %d\n", ret);
        return false;
    }

    if (sock_status == SOCK_LISTEN) {
        server_connected = false;
        return false;
    }

    if (sock_status == SOCK_CLOSE_WAIT) {
        printf("[W5500] CLOSE_WAIT — disconnecting\n");
        disconnect(server_sock);
        server_connected = false;
        return false;
    }

    if (sock_status == SOCK_ESTABLISHED) {
        if (!server_connected) {
            printf("[W5500] Client connected\n");
        }
        server_connected = true;
    }

    if (!server_connected) {
        return false;
    }

    // Check for received data
    uint16_t rx_size = getSn_RX_RSR(server_sock);
    if (rx_size == 0) {
        return false;
    }

    // Read data
    int len = recv(server_sock, (uint8_t *)buf, bufsize - 1);
    if (len <= 0) {
        return false;
    }

    buf[len] = '\0';
    return true;
}

// --- Send data to connected client ---
// v2: retry partial sends (ioLibrary send() may not transmit everything
// in one call when the TX buffer is full), so no data is silently dropped.
bool w5500_tcp_send(const char *data, int len) {
    if (!server_connected || len <= 0) {
        return false;
    }

    uint8_t sock_status = getSn_SR(server_sock);
    if (sock_status != SOCK_ESTABLISHED) {
        server_connected = false;
        return false;
    }

    int total = 0;
    while (total < len) {
        int sent = send(server_sock, (uint8_t *)(data + total), len - total);
        if (sent <= 0) {
            // 0 / busy — wait for the TX buffer to drain, then retry
            sleep_ms(1);
            if (getSn_SR(server_sock) != SOCK_ESTABLISHED) {
                server_connected = false;
                return false;
            }
            continue;
        }
        total += sent;
    }
    return true;
}
