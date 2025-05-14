#include <bcm2835.h>
#include <stdio.h>
#include <stdint.h>
#include "drivers/icm20948/icm20948.h"
#include "drivers/icm20948/icm20948_regs.h"

// Chip select assert/deassert for bcm2835
static void cs_assert(void) {
    bcm2835_gpio_write(8, LOW);
    printf("CS LOW\n");
    bcm2835_delayMicroseconds(1);
}
static void cs_deassert(void) {
    bcm2835_gpio_write(8, HIGH);
    printf("CS HIGH\n");
    bcm2835_delayMicroseconds(1);
}

// SPI transfer for bcm2835
static void spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint8_t out = tx ? tx[i] : 0xFF;
        uint8_t in = bcm2835_spi_transfer(out);
        if (rx) rx[i] = in;
    }
}

// Delay in microseconds
static void delay_us(uint32_t us) {
    bcm2835_delayMicroseconds(us);
}

// Platform SPI read function
static icm20948_return_code_t spi_read(uint8_t reg, uint8_t *data, uint32_t len) {
    if (!data) return ICM20948_RET_NULL_PTR;
    printf("CS LOW (spi_read)\n");
    bcm2835_gpio_write(8, LOW); // Use GPIO 8 for CS
    bcm2835_delayMicroseconds(5); // Increased delay slightly
    uint8_t cmd = reg | 0x80;
    printf("SPI READ: Sending cmd 0x%02X (reg=0x%02X|0x80)\n", cmd, reg);
    bcm2835_spi_transfer(cmd);
    bcm2835_delayMicroseconds(1);
    for (uint32_t i = 0; i < len; i++) {
        data[i] = bcm2835_spi_transfer(0xFF);
        printf("SPI READ: Received 0x%02X\n", data[i]);
    }
    bcm2835_delayMicroseconds(5); // Increased delay slightly
    bcm2835_gpio_write(8, HIGH); // Use GPIO 8 for CS
    printf("CS HIGH (spi_read)\n");
    return ICM20948_RET_OK;
}

// Platform SPI write function
static icm20948_return_code_t spi_write(uint8_t reg, uint8_t *data, uint32_t len) {
    if (!data) return ICM20948_RET_NULL_PTR;
    printf("CS LOW (spi_write)\n");
    bcm2835_gpio_write(8, LOW); // Use GPIO 8 for CS
    bcm2835_delayMicroseconds(5); // Increased delay slightly
    uint8_t cmd = reg & 0x7F;
    printf("SPI WRITE: Sending cmd 0x%02X (reg=0x%02X&0x7F)\n", cmd, reg);
    bcm2835_spi_transfer(cmd);
    bcm2835_delayMicroseconds(1);
    for (uint32_t i = 0; i < len; i++) {
        printf("SPI WRITE: Sending data 0x%02X\n", data[i]);
        bcm2835_spi_transfer(data[i]);
    }
    bcm2835_delayMicroseconds(5); // Increased delay slightly
    bcm2835_gpio_write(8, HIGH); // Use GPIO 8 for CS
    printf("CS HIGH (spi_write)\n");
    return ICM20948_RET_OK;
}

// Example initialization sequence
int icm20948_init_and_test(icm20948_dev_t *dev) {
    uint8_t whoami = 0;
    icm20948_return_code_t ret;

    // Platform setup
    // icm20948_platform_setup(dev);

    // Try reading WHO_AM_I
    ret = icm20948_read_who_am_i(dev, &whoami);
    printf("WHO_AM_I = 0x%02X\n", whoami);
    if (ret != ICM20948_RET_OK || whoami != 0xEA) {
        printf("ICM20948 not detected!\n");
        return -1;
    }
    printf("ICM20948 detected successfully.\n");
    return 0;
}
