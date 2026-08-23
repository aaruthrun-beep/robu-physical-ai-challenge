#ifndef TMC_UART_H
#define TMC_UART_H

#include <stdint.h>
#include <stdbool.h>

#define TMC_UART_ID     uart0
#define TMC_UART_TX_PIN 0
#define TMC_UART_RX_PIN 1
#define TMC_BAUD        115200

#define TMC_REG_GCONF       0x00
#define TMC_REG_GSTAT       0x01
#define TMC_REG_IFCNT       0x02
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_CHOPCONF    0x6C
#define TMC_REG_DRV_STATUS  0x6F

void tmc_uart_init(void);
bool tmc_read_register(uint8_t addr, uint8_t reg, uint32_t *value);
void tmc_write_register(uint8_t addr, uint8_t reg, uint32_t value);

#endif
