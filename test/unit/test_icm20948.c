#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <bcm2835.h>
#include <unistd.h>
#include <math.h>
#include "drivers/icm20948/icm20948.h"
#include "drivers/icm20948/icm20948_regs.h"

// SPI settings
#define SPI_CLOCK_DIV_1MHZ  250    // 250MHz/250 = 1MHz

#define SPI_CS_PIN      RPI_GPIO_P1_24  // CE0, GPIO 8
#define SPI_MISO_PIN    RPI_GPIO_P1_21  // GPIO 9
#define SPI_MOSI_PIN    RPI_GPIO_P1_19  // GPIO 10
#define SPI_CLK_PIN     RPI_GPIO_P1_23  // GPIO 11
#define MS5611_CS_PIN   RPI_GPIO_P1_26  // CE1, GPIO 7 (Used by MS5611)

// Global device instance
static icm20948_dev_t dev;

// Function prototypes
static void test_raw_spi(void);
static void test_who_am_i(void);
static void test_sensor_readings(void);
static void setup_spi(void);
static void cleanup_spi(void);

// --- Platform Specific Implementations (using bcm2835) ---

// Platform specific delay function
void platform_delay_us(uint32_t us) {
    bcm2835_delayMicroseconds(us);
}

// Platform specific CS assert function
void platform_cs_assert(void) {
    bcm2835_gpio_write(SPI_CS_PIN, LOW);
    // printf("CS Asserted (GPIO%d LOW)\n", SPI_CS_PIN); // Debug
    bcm2835_delayMicroseconds(5); // Small delay after assert
}

// Platform specific CS deassert function
void platform_cs_deassert(void) {
    bcm2835_gpio_write(SPI_CS_PIN, HIGH);
    // printf("CS Deasserted (GPIO%d HIGH)\n", SPI_CS_PIN); // Debug
}

#define MAX_SPI_TRANSFER_SIZE 256 // Define a reasonable max size

// Static buffer for dummy reads during write operations
static uint8_t dummy_rx_buffer[MAX_SPI_TRANSFER_SIZE];

// Platform specific SPI transfer function (using bcm2835_spi_transfernb)
icm20948_return_code_t platform_spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len) {
    // Use transfernb for block transfer
    // Note: bcm2835 requires tx and rx buffers to be different or non-overlapping for transfernb
    // The driver's _spi_read function already provides separate tx and rx buffers.
    // _spi_write only uses tx, so rx can be the same buffer or NULL (though bcm2835 might prefer a valid pointer).
    // We assume the calling functions (_spi_read/_spi_write) provide appropriate buffers.
    // If rx is NULL, bcm2835_spi_transfernb might still write received bytes to the tx buffer.
    // To be safe, let's ensure rx has a valid buffer, even if it's just tx itself for write operations.

    if (len > MAX_SPI_TRANSFER_SIZE) {
        fprintf(stderr, "Error: SPI transfer size %u exceeds max %d\n", len, MAX_SPI_TRANSFER_SIZE);
        return ICM20948_RET_BAD_ARG; // Or appropriate error
    }

    uint8_t* rx_ptr = rx ? rx : dummy_rx_buffer; // Use dummy buffer if rx is NULL

    bcm2835_spi_transfernb((char*)tx, (char*)rx_ptr, len);

    return ICM20948_RET_OK; // Return OK status (bcm2835 functions don't return status here)
}

// Test functions
void test_raw_spi(void) {
    printf("\nTesting raw SPI communication (using configured Mode 3)...\n\n");

    printf("  Using driver's SPI functions (manual CS on GPIO8)\n");
    printf("Selecting Bank 0...\n");
    uint8_t bank = 0;
    // Use the public function to select the bank
    icm20948_return_code_t ret = icm20948_select_bank(&dev, bank);

    if (ret == ICM20948_RET_OK) {
        printf("  Bank 0 selected successfully via driver.\n");
        printf("  Reading WHO_AM_I (Bank 0, Reg 0x00) via driver...\n");
        uint8_t who_am_i = 0xFF;
        ret = icm20948_read_who_am_i(&dev, &who_am_i);
        printf("    WHO_AM_I Read Result: %d\n", ret);
        if (ret == ICM20948_RET_OK) {
            printf("    WHO_AM_I = 0x%02X\n", who_am_i);
            // TEST_ASSERT_EQUAL_HEX8_MESSAGE(ICM20948_WHO_AM_I_VAL, who_am_i, "WHO_AM_I register mismatch");
        } else {
            // TEST_FAIL_MESSAGE("Failed to read WHO_AM_I register via driver");
        }
    } else {
        printf("  Failed to select Bank 0 via driver. Error: %d\n", ret);
        // TEST_FAIL_MESSAGE("Failed to select Bank 0 via driver");
    }
    printf("\n");
}

void test_who_am_i(void) {
    printf("\nTesting WHO_AM_I register...\n");
    
    uint8_t who_am_i;
    icm20948_return_code_t ret = icm20948_read_who_am_i(&dev, &who_am_i);
    if (ret != ICM20948_RET_OK) {
        printf("❌ Failed to read WHO_AM_I register\n");
        return;
    }
    
    printf("WHO_AM_I read returned: 0x%02X (Expected: 0x%02X)\n", who_am_i, ICM20948_WHO_AM_I_VAL);
    if (who_am_i == ICM20948_WHO_AM_I_VAL) {
        printf("✓ WHO_AM_I matches expected value\n");
    } else {
        printf("❌ WHO_AM_I mismatch! Expected 0x%02X, got 0x%02X\n", ICM20948_WHO_AM_I_VAL, who_am_i);
    }
}

void test_sensor_readings(void) {
    printf("\nTesting sensor readings...\n");

    // Placeholder variables for raw sensor data
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    int16_t mag_x_raw, mag_y_raw, mag_z_raw;

    // Placeholder variables for converted sensor data
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float mag_x_ut, mag_y_ut, mag_z_ut;
    float icm_temp_c; // Variable for ICM20948 internal temperature

    icm20948_return_code_t ret;
    time_t start_time = time(NULL);
    time_t current_time;

    // Print header
    printf("Time(s) Accel(g)             Gyro(dps)            Mag(uT)             Temp(C)\n");

    // Loop indefinitely, reading and printing sensor data
    for (;;) { // Run forever until Ctrl+C
        current_time = time(NULL);

        // Read accelerometer data
        ret = icm20948_read_accel(&dev, &accel_x_raw, &accel_y_raw, &accel_z_raw);
        if (ret != ICM20948_RET_OK) {
            printf("ERROR: Failed to read accel data (ret=%d)\n", ret);
            accel_x_g = accel_y_g = accel_z_g = NAN;
        } else {
            // Convert raw data to g (assuming default +/- 2g scale)
            accel_x_g = (float)accel_x_raw / 16384.0f;
            accel_y_g = (float)accel_y_raw / 16384.0f;
            accel_z_g = (float)accel_z_raw / 16384.0f;
        }

        // Read gyroscope data
        ret = icm20948_read_gyro(&dev, &gyro_x_raw, &gyro_y_raw, &gyro_z_raw);
        if (ret != ICM20948_RET_OK) {
            printf("ERROR: Failed to read gyro data (ret=%d)\n", ret);
            gyro_x_dps = gyro_y_dps = gyro_z_dps = NAN;
        } else {
            // Convert raw data to dps (assuming default +/- 250 dps scale)
            gyro_x_dps = (float)gyro_x_raw / 131.0f;
            gyro_y_dps = (float)gyro_y_raw / 131.0f;
            gyro_z_dps = (float)gyro_z_raw / 131.0f;
        }

        // Read magnetometer data
        ret = icm20948_read_mag(&dev, &mag_x_raw, &mag_y_raw, &mag_z_raw);
        if (ret != ICM20948_RET_OK) {
            printf("ERROR: Failed to read mag data (ret=%d)\n", ret);
            mag_x_ut = mag_y_ut = mag_z_ut = NAN;
        } else {
            // Convert raw data to uT (using typical sensitivity)
            // NOTE: This requires calibration for accuracy
            mag_x_ut = (float)mag_x_raw * 0.15f;
            mag_y_ut = (float)mag_y_raw * 0.15f;
            mag_z_ut = (float)mag_z_raw * 0.15f;
        }

        // Read temperature data
        ret = icm20948_read_temp(&dev, &icm_temp_c);
        if (ret != ICM20948_RET_OK) {
            printf("ERROR: Failed to read temp data (ret=%d)\n", ret);
            icm_temp_c = NAN;
        }

        // Print the readings
        printf("  %ld   %5.2f   %5.2f   %5.2f     %7.2f     %7.2f    %7.2f  %6.2f  %6.2f  %6.2f  %6.2f\n",
               current_time - start_time,
               accel_x_g, accel_y_g, accel_z_g,
               gyro_x_dps, gyro_y_dps, gyro_z_dps,
               mag_x_ut, mag_y_ut, mag_z_ut,
               icm_temp_c);

        // No delay needed here for continuous reading, Ctrl+C to stop
    }

    printf("\n✓ Sensor reading test completed\n\n");
}

void setup_spi(void) {
    printf("Setting up SPI...\n");
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }
    printf("✓ Initialized bcm2835 library\n");

    // --- Setup GPIO for Manual CS ---
    bcm2835_gpio_fsel(SPI_CS_PIN, BCM2835_GPIO_FSEL_OUTP); // Set CS pin to output
    bcm2835_gpio_write(SPI_CS_PIN, HIGH); // Start with CS high (inactive)
    printf("✓ Configured GPIO %d for manual CS\n", SPI_CS_PIN);

    // --- Setup GPIO for other SPI device CS (BMP388/MS5611 on GPIO 7) --- 
    bcm2835_gpio_fsel(MS5611_CS_PIN, BCM2835_GPIO_FSEL_OUTP); // Set other CS pin to output
    bcm2835_gpio_write(MS5611_CS_PIN, HIGH); // Keep other CS high (inactive)
    printf("✓ Configured GPIO %d for BMP388/MS5611 CS (inactive)\n", MS5611_CS_PIN);

    // --- Setup SPI ---
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3); // Use Mode 3 (CPOL=1, CPHA=1)
    bcm2835_spi_setClockDivider(SPI_CLOCK_DIV_1MHZ); // Set SPI clock speed
    // Note: We are using manual CS control, so chip select settings here are ignored
    // bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE); // Use NONE as we handle CS manually
    // bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); // Set CS polarity if using auto CS
    printf("✓ Configured SPI parameters (Mode 3, %dMHz)\n", 250 / SPI_CLOCK_DIV_1MHZ);

    // --- Initialize Driver Interface ---
    memset(&dev, 0, sizeof(dev)); // Clear the device structure

    // Link the platform-specific functions
    dev.intf.delay_us = platform_delay_us;
    dev.intf.cs_assert = platform_cs_assert;       // Assign manual CS assert
    dev.intf.cs_deassert = platform_cs_deassert;   // Assign manual CS deassert
    dev.intf.spi_transfer = platform_spi_transfer; // Assign the transfer function
    printf("✓ Linked platform-specific SPI functions to device struct\n");

    // Initialize the ICM20948 driver (this should handle reset and basic setup)
    icm20948_return_code_t ret = icm20948_init(&dev);
    if (ret != ICM20948_RET_OK) {
        fprintf(stderr, "icm20948_init failed with code %d\n", ret);
        cleanup_spi();
        exit(EXIT_FAILURE);
    }
    printf("✓ Initialized ICM20948 driver (icm20948_init returned OK)\n");

}

static void cleanup_spi(void) {
    printf("\nCleaning up...\n");
    bcm2835_spi_end();
    // Reset GPIO pins to input
    bcm2835_gpio_fsel(SPI_CS_PIN, BCM2835_GPIO_FSEL_INPT);
    printf("✓ Reset GPIO %d (ICM CS) to INPUT\n", SPI_CS_PIN);
    bcm2835_gpio_fsel(MS5611_CS_PIN, BCM2835_GPIO_FSEL_INPT);
    printf("✓ Reset GPIO %d (BMP/MS CS) to INPUT\n", MS5611_CS_PIN);
    bcm2835_close();
    printf("✓ Cleanup complete\n");
}

int main(void) {
    setup_spi();

    // Test reading WHO_AM_I register (should be 0xEA)
    // The icm20948_init should have selected bank 0 already.
    uint8_t who_am_i = 0;
    icm20948_return_code_t ret = icm20948_read_who_am_i(&dev, &who_am_i); // Correct helper

    if (ret == ICM20948_RET_OK) {
        printf("WHO_AM_I read returned: 0x%02X (Expected: 0x%02X)\n", who_am_i, ICM20948_WHO_AM_I_VAL);
        if (who_am_i == ICM20948_WHO_AM_I_VAL) {
            printf("✓ WHO_AM_I matches expected value\n");
        } else {
            printf("❌ WHO_AM_I mismatch! Expected 0x%02X, got 0x%02X\n", ICM20948_WHO_AM_I_VAL, who_am_i);
        }
    } else {
        printf("❌ Failed to read WHO_AM_I register\n");
    }

    // Run tests
    test_raw_spi();
    test_who_am_i();
    test_sensor_readings();

    // Cleanup
    cleanup_spi();
    return 0;
}