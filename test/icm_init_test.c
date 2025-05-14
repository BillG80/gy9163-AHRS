#include <bcm2835.h>
#include <stdio.h>
#include "drivers/icm20948/icm20948.h"
#include "drivers/icm20948/icm20948_regs.h"

// Forward declarations from icm_init.c
void icm20948_platform_setup(icm20948_dev_t *dev);
int icm20948_init_and_test(icm20948_dev_t *dev);

int main(void) {
    printf("ICM20948 icm_init.c Standalone Test & SPI Loopback\n");

    // Initialize the BCM2835 library (required for SPI/GPIO)
    if (!bcm2835_init()) {
        printf("bcm2835_init failed!\n");
        return 1;
    }

    printf("----------------------------------------\n");
    // --- ICM20948 Initialization --- 
    printf("Initializing ICM20948 (Reconnect Sensor Wiring!)...\n");

    // Configure INT pin (GPIO25) as input for ICM20948 interrupt
    bcm2835_gpio_fsel(25, BCM2835_GPIO_FSEL_INPT);
    // Optionally, set pull-up if required:
    // bcm2835_gpio_set_pud(25, BCM2835_GPIO_PUD_UP);
    // --- Force CS (GPIO8/CE0) LOW before SPI begin to ensure SPI mode ---
    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP); // GPIO8 as output
    bcm2835_gpio_write(RPI_GPIO_P1_24, LOW);                   // Drive LOW
    bcm2835_delay(2000); // 2s for ICM20948 to latch SPI mode
    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_ALT0); // Back to SPI

    if (!bcm2835_spi_begin()) {
        printf("ERROR: bcm2835_spi_begin failed!\n");
        bcm2835_close();
        return 1;
    }
    // Set SPI clock divider to slowest (244kHz)
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);

    // Explicitly set SPI pins to ALT0 (SPI function)
    bcm2835_gpio_fsel(RPI_GPIO_P1_21, BCM2835_GPIO_FSEL_ALT0); // MISO (GPIO 9)
    bcm2835_gpio_fsel(RPI_GPIO_P1_19, BCM2835_GPIO_FSEL_ALT0); // MOSI (GPIO 10)
    bcm2835_gpio_fsel(RPI_GPIO_P1_23, BCM2835_GPIO_FSEL_ALT0); // SCLK (GPIO 11)
    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_ALT0); // CE0  (GPIO 8)
    // Wait 100ms for sensor to power up
    bcm2835_delay(100);

    // SPI configuration for ICM20948
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0); // Mode 0
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256); // ~1MHz
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE); // Manual CS control
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); // CS0 is GPIO 8

    // Set GPIO 8 (CS pin) as output and set it high initially
    bcm2835_gpio_fsel(8, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(8, HIGH);

    // --- Send dummy SPI commands to wake up sensor (optional, keep for now) ---
    printf("Sending dummy SPI commands...\n");
    for (int i = 0; i < 10; i++) {
        bcm2835_gpio_write(8, LOW); // Assert CS (GPIO 8)
        bcm2835_delayMicroseconds(5);
        bcm2835_spi_transfer(0xFF); // Send dummy byte
        bcm2835_delayMicroseconds(5);
        bcm2835_gpio_write(8, HIGH); // Deassert CS
        bcm2835_delayMicroseconds(20);
    }
    printf("Dummy commands sent.\n");

    // Setup ICM20948 device structure
    icm20948_dev_t dev = {0};
    icm20948_platform_setup(&dev); // Setup is mainly SPI transfers

    int result = icm20948_init_and_test(&dev); // Perform initialization and WHO_AM_I test
    if (result == 0) {
        printf("ICM20948 initialization and WHO_AM_I test PASSED.\n");
    } else {
        printf("ICM20948 initialization and WHO_AM_I test FAILED.\n");
    }

    bcm2835_spi_end();
    bcm2835_close();
    return result;
}
