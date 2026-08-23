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

// --- Initialize W5500 ---
bool w5500_tcp_init(void) {
    // Init SPI0
    spi_init(W5500_SPI, 1 * 1000 * 1000);  // 1 MHz (reduced from 10MHz to reduce SPI1 noise)
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

    // Register ioLibrary callbacks
    reg_wizchip_cs_cbfunc(w5500_cs_select, w5500_cs_deselect);
    reg_wizchip_spi_cbfunc(w5500_spi_read, w5500_spi_write);

    // Socket buffer allocation (2KB each) — does soft reset
    uint8_t txsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    uint8_t rxsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(txsize, rxsize);
    sleep_ms(100);

    // Set network info (after init, so it sticks)
    setSHAR(MAC_ADDR);
    setSIPR(IP_ADDR);
    setSUBR(SUBNET);
    setGAR(GATEWAY);

    // Verify chip
    uint8_t ver = getMR();
    printf("W5500 MR: 0x%02X\n", ver);

    current_ip = (IP_ADDR[0] << 24) | (IP_ADDR[1] << 16) | (IP_ADDR[2] << 8) | IP_ADDR[3];
    printf("W5500 initialized. IP: %d.%d.%d.%d\n",
           IP_ADDR[0], IP_ADDR[1], IP_ADDR[2], IP_ADDR[3]);

    return true;
}

// --- Check link status ---
bool w5500_tcp_is_link_up(void) {
    uint8_t phy = getPHYCFGR();
    link_up = (phy & 0x01) != 0;
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

// --- Initialize TCP server ---
static bool tcp_server_init(void) {
    uint8_t sock_status = getSn_SR(server_sock);
    if (sock_status == SOCK_ESTABLISHED) {
        server_connected = true;
        return true;
    }

    if (sock_status == SOCK_CLOSED) {
        // Open socket
        socket(server_sock, Sn_MR_TCP, W5500_TCP_PORT, 0);
        return false;
    }

    if (sock_status == SOCK_INIT) {
        // Listen for connections
        listen(server_sock);
        return false;
    }

    return false;
}

// --- Poll for received data ---
static uint8_t last_sock_state = 0xFF;
bool w5500_tcp_poll(char *buf, int bufsize) {
    // Check link
    uint8_t phy = getPHYCFGR();
    link_up = (phy & 0x01) != 0;
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
        close(server_sock);
        sleep_ms(100);

        // On cold boot the W5500 network registers (SIPR/GAR/SUBR) can read
        // as zero until the chip+PHY fully initialize. socket() returns
        // SOCKERR_SOCKINIT (-3) when SIPR==0.0.0.0 for a TCP socket, and the
        // socket never reaches SOCK_INIT, so the poll loop would spin forever.
        // Re-apply the network info and verify SIPR before opening, and back
        // off so we don't hammer socket() while the PHY is still coming up.
        uint8_t cur_ip[4];
        getSIPR(cur_ip);
        uint32_t ip32 = (cur_ip[0] << 24) | (cur_ip[1] << 16) | (cur_ip[2] << 8) | cur_ip[3];
        if (ip32 == 0) {
            // Network registers not ready yet — re-apply and wait for the PHY.
            setSHAR(MAC_ADDR);
            setSIPR(IP_ADDR);
            setSUBR(SUBNET);
            setGAR(GATEWAY);
            sleep_ms(500);
            printf("[W5500] Re-applied network info (SIPR was 0.0.0.0), retrying...\n");
            return false;
        }

        printf("[W5500] Opening socket...\n");
        int ret = socket(server_sock, Sn_MR_TCP, W5500_TCP_PORT, 0);
        printf("[W5500] socket() returned %d, Sn_SR=0x%02X\n", ret, getSn_SR(server_sock));
        sleep_ms(100);
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
bool w5500_tcp_is_connected(void) {
    return server_connected && (getSn_SR(server_sock) == SOCK_ESTABLISHED);
}

bool w5500_tcp_send(const char *data, int len) {
    if (!server_connected) {
        return false;
    }

    uint8_t sock_status = getSn_SR(server_sock);
    if (sock_status != SOCK_ESTABLISHED) {
        server_connected = false;
        return false;
    }

    int sent = send(server_sock, (uint8_t *)data, len);
    return sent == len;
}
