// File: test/unit/test_spi1_loopback.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <bcm2835.h>

// SPI1 Pins (BCM numbering)
#define SPI1_MOSI_PIN RPI_V2_GPIO_P1_38 // GPIO 20 (Physical Pin 38)
#define SPI1_MISO_PIN RPI_V2_GPIO_P1_35 // GPIO 19 (Physical Pin 35)
#define SPI1_SCLK_PIN RPI_V2_GPIO_P1_40 // GPIO 21 (Physical Pin 40)
#define SPI1_CS0_PIN  RPI_V2_GPIO_P1_12 // GPIO 18 (Physical Pin 12) - Used as manual CS

// Test settings
#define SPI_CLOCK_DIV BCM2835_SPI_CLOCK_DIVIDER_4096 // ~61kHz

// Helper function to control CS pin (Needed for bcm2835_aux_spi_transfernb)
static inline void platform_cs_select() {
    bcm2835_gpio_write(SPI1_CS0_PIN, LOW);
}

static inline void platform_cs_deselect() {
    bcm2835_gpio_write(SPI1_CS0_PIN, HIGH);
}

int main(int argc, char **argv) {
    // Test patterns for loopback
    uint8_t tx_buf[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xAA, 0x55, 0x12, 0x34, 0xFF, 0x00 };
    uint8_t rx_buf[sizeof(tx_buf)];
    int i, errors = 0;

    printf("Initializing bcm2835 library for SPI1 Loopback Test...\n");
    if (!bcm2835_init()) {
        fprintf(stderr, "❌ bcm2835_init failed. Are you running as root?\n");
        return 1;
    }
     printf("✓ bcm2835 library initialized successfully.\n");

    printf("\nSetting up GPIO and AUX SPI interface (SPI1)...\n");

    // Configure CS pin as output, initially HIGH
    bcm2835_gpio_fsel(SPI1_CS0_PIN, BCM2835_GPIO_FSEL_OUTP);
    platform_cs_deselect(); // Set CS HIGH initially
    printf("✓ Configured GPIO %d (SPI1 CS0) as output HIGH\n", SPI1_CS0_PIN);

    // Initialize AUX SPI *before* setting clock and mode for bcm2835 library >= 1.72
    if (!bcm2835_aux_spi_begin()) {
        fprintf(stderr, "❌ Error initializing SPI1 (aux_spi_begin).\n");
        bcm2835_close();
        return 1;
    }

    // Set SPI clock speed (using low speed for stability initially)
    bcm2835_aux_spi_setClockDivider(SPI_CLOCK_DIV); // ~61kHz

    // SPI Data Mode for aux_spi defaults to Mode 0, no explicit setting needed/available

    printf("✓ AUX SPI interface configured (Manual CS on GPIO %d, ~61kHz, Mode 0)\n", SPI1_CS0_PIN);


    printf("\n--- Starting SPI1 Loopback Test ---\n");
    printf("Connect Pi Pin 38 (GPIO 20/MOSI) <--> Pi Pin 35 (GPIO 19/MISO)\n");
    printf("Press Enter when ready...\n");
    getchar(); // Wait for user to confirm wiring

    for (int loop = 0; loop < 5; ++loop) {
        printf("\nLoop %d:\n", loop + 1);
        memset(rx_buf, 0, sizeof(rx_buf)); // Clear RX buffer

        // Perform SPI transfer (loopback)
        platform_cs_select(); // Lower CS (GPIO 18)
        bcm2835_delayMicroseconds(5); // Short delay after CS select
        bcm2835_aux_spi_transfernb((char*)tx_buf, (char*)rx_buf, sizeof(tx_buf));
        bcm2835_delayMicroseconds(5); // Short delay before CS deselect
        platform_cs_deselect(); // Raise CS

        // Print results
        printf("  TX: ");
        for (i = 0; i < sizeof(tx_buf); i++) printf(" %02X", tx_buf[i]);
        printf("\n  RX: ");
        for (i = 0; i < sizeof(rx_buf); i++) printf(" %02X", rx_buf[i]);
        printf("\n");

        // Verify
        errors = 0;
        for (i = 0; i < sizeof(tx_buf); i++) {
            if (tx_buf[i] != rx_buf[i]) {
                errors++;
            }
        }
        if (errors == 0) {
            printf("  Result: PASS\n");
        } else {
            printf("  Result: FAIL (%d errors)\n", errors);
        }

        // Rotate the pattern for next loop
        uint8_t temp = tx_buf[0];
        memmove(tx_buf, tx_buf + 1, sizeof(tx_buf) - 1);
        tx_buf[sizeof(tx_buf) - 1] = temp;

        bcm2835_delay(1000); // Wait 1 second
    }

    printf("\n✓ Loopback test finished.\n");

    // Cleanup
    printf("\nCleaning up...\n");
    // Reset SPI pins to input
    bcm2835_gpio_fsel(SPI1_MOSI_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SPI1_MISO_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SPI1_SCLK_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SPI1_CS0_PIN, BCM2835_GPIO_FSEL_INPT); // Set manual CS pin to input
    printf("✓ Set SPI1 GPIOs (%d, %d, %d, %d) to INPUT\n",
           (int)SPI1_MOSI_PIN, (int)SPI1_MISO_PIN, (int)SPI1_SCLK_PIN, (int)SPI1_CS0_PIN);

    // End SPI
    bcm2835_aux_spi_end();
    printf("✓ AUX SPI ended\n");

    // Close library
    bcm2835_close();
    printf("✓ bcm2835 library closed.\n");

    return 0;
}