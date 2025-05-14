// Reference: Python test implementation in \\path\\to file
// d:\\RMRobot\\Elevator\\gy9163-sensor-system\\python\\test\\test_ms5611_i2c.py

#define _GNU_SOURCE  // enable POSIX/GNU prototypes (usleep, clock_gettime, nanosleep)

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h> // for usleep
#include "drivers/ms5611/ms5611.h"
#include <time.h>  // for clock_gettime and nanosleep
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <errno.h>

// --- Configuration ---
#define I2C_BUS_NUMBER  0                   // Use I2C bus 0 (pins 3, 5) - used for printing
#define MS5611_I2C_ADDR 0x77                // Use 0x77 (PS pin HIGH)

// --- Platform Specific Functions (I2C) ---
// Forward declaration for delay function
static void platform_delay_us(uint32_t period);

// Use correct return codes from ms5611.h (likely MS5611_RET_OK/MS5611_RET_ERROR)
static int i2c_fd;

static ms5611_return_code_t platform_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len) {
    // First, write the command/register address to read from
    if (write(i2c_fd, &reg_addr, 1) != 1) {
        perror("I2C write reg_addr for read");
        return MS5611_RET_ERROR;
    }
    // Add a small delay between write and read
    usleep(1000); // 1 ms

    // Now, read the requested number of bytes
    if (read(i2c_fd, reg_data, len) != (ssize_t)len) {
        perror("I2C read data");
        return MS5611_RET_ERROR;
    }
    return MS5611_RET_OK;
}

static ms5611_return_code_t platform_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len) {
    if (reg_data == NULL && len == 0) {
        if (write(i2c_fd, &reg_addr, 1) != 1) {
            perror("I2C write cmd");
            return MS5611_RET_ERROR;
        }
        if (reg_addr == MS5611_CMD_RESET) usleep(3000);
        return MS5611_RET_OK;
    }
    return MS5611_RET_BAD_ARG;
}

static void platform_delay_us(uint32_t period) {
    usleep(period);
}

// --- Main Test ---
int main(int argc, char *argv[]) {
    (void)argc; (void)argv;
    ms5611_dev_t ms5611_dev;
    ms5611_return_code_t ret;
    uint32_t temperature_raw; // D2
    uint32_t pressure_raw;    // D1
    int32_t temperature_degC_x100; // Compensated Temp (*100)
    int32_t pressure_hPa;           // Compensated Pressure (hPa)

    // Note: Removed bcm2835_init() as we are using /dev/i2c
    printf("Opening I2C bus 0 (/dev/i2c-0) for address 0x%02X...\n", MS5611_I2C_ADDR);
    i2c_fd = open("/dev/i2c-0", O_RDWR);
    if (i2c_fd < 0) {
        perror("Open /dev/i2c-0");
        return 1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, MS5611_I2C_ADDR) < 0) {
        perror("ioctl(I2C_SLAVE) failed");
        close(i2c_fd);
        return 1;
    }
    printf("✓ I2C Bus 0 opened and address 0x%02X set.\n", MS5611_I2C_ADDR);

    printf("\nInitializing MS5611 sensor...\n");
    // Pass platform functions as arguments to init
    ret = ms5611_init(&ms5611_dev, platform_i2c_read, platform_i2c_write, platform_delay_us);

    if (ret != MS5611_RET_OK) { // CORRECT RETURN CODE
        fprintf(stderr, "Failed to initialize MS5611 sensor, error code: %d", ret);
        if (ret == MS5611_RET_CRC_ERROR) {
            fprintf(stderr, " (PROM CRC Check Failed!)");
        }
        fprintf(stderr, "\n");
        close(i2c_fd);
        return 1;
    }
    printf("✓ MS5611 Initialized Successfully.\n");

    // Print PROM coefficients stored in the dev structure
    printf("\nPROM Coefficients (C1-C6):");
    for (int i = 0; i < 6; i++) {
        // Assuming prom_coeffs is the member name based on previous successful tests
        printf("\n  C%d: %u (0x%04X)", i+1, ms5611_dev.prom_coeffs[i], ms5611_dev.prom_coeffs[i]);
    }
    // C7 is the read CRC value
    printf("\n  CRC (read C7): %u", ms5611_dev.prom_coeffs[7]);
    printf("\n");

    // Perform a few readings using low-level ADC read and calculation
    printf("\nReading sensor data...\n");
    // printf("%12s %12s %10s %12s\n", "Raw D1(Pres)", "Raw D2(Temp)", "Temp (*100)", "Press (Pa)"); // Header for raw int values
    printf("%12s %12s %10s %12s\n", "Raw D1(Pres)", "Raw D2(Temp)", "Temp (C)", "Press (hPa)"); // Header for float temp, int pressure


    for (int i = 0; i < 5; i++) {
        // Read Raw Pressure (D1)
        ret = ms5611_read_adc(&ms5611_dev, (MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D1 | MS5611_CMD_ADC_OSR_4096), &pressure_raw);
        if (ret != MS5611_RET_OK) { // CORRECT RETURN CODE
            fprintf(stderr, "  Failed to read raw pressure D1, error code: %d\n", ret);
            continue;
        }

        // Read Raw Temperature (D2)
        ret = ms5611_read_adc(&ms5611_dev, (MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D2 | MS5611_CMD_ADC_OSR_4096), &temperature_raw);
        if (ret != MS5611_RET_OK) { // CORRECT RETURN CODE
            fprintf(stderr, "  Failed to read raw temperature D2, error code: %d\n", ret);
            continue;
        }

        printf("  Raw: D1=%u, D2=%u", pressure_raw, temperature_raw); // Print raw values first

        // Calculate compensated values using the correct function signature
        ms5611_calculate_pressure(&ms5611_dev, pressure_raw, temperature_raw, &pressure_hPa, &temperature_degC_x100);

        // Print compensated values as floats
        float temp_c = temperature_degC_x100 / 100.0f;
        // pressure_hPa is already in hPa
        printf("  Compensated: Temp=%.2f C, Press=%d hPa\n", temp_c, pressure_hPa);

        sleep(1); // Wait 1 second between readings
    }

    // Continuous measurement at ~50 Hz using OSR=2048
    printf("\nStarting continuous measurement at ~50Hz (OSR=2048)...\n");
    const double period = 0.02; // 50 Hz sample period (s)
    unsigned long long loop_count = 0; // Counter for periodic printing
    while (1) {
        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        // Read raw temperature (D2)
        ret = ms5611_read_adc(&ms5611_dev, MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D2 | MS5611_CMD_ADC_OSR_2048, &temperature_raw);
        if (ret != MS5611_RET_OK) {
            fprintf(stderr, "Error reading temperature ADC\n");
            continue;
        }

        // Read raw pressure (D1)
        ret = ms5611_read_adc(&ms5611_dev, MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D1 | MS5611_CMD_ADC_OSR_2048, &pressure_raw);
        if (ret != MS5611_RET_OK) {
            fprintf(stderr, "Error reading pressure ADC\n");
            continue;
        }

        // Calculate compensated values
        ms5611_calculate_pressure(&ms5611_dev, pressure_raw, temperature_raw, &pressure_hPa, &temperature_degC_x100);
        double temp_c = temperature_degC_x100 / 100.0;
        // pressure_hPa is already in hPa

        clock_gettime(CLOCK_MONOTONIC, &t_end);
        double dt = (t_end.tv_sec - t_start.tv_sec) + (t_end.tv_nsec - t_start.tv_nsec) / 1e9;
        double rate = 1.0 / dt;

        loop_count++;
        if (loop_count % 100 == 0) { // Print every 100 loops
            printf("Rate: %.1f Hz, Temp: %.2f C, Press: %d hPa (Loop %llu)\n", rate, temp_c, pressure_hPa, loop_count);
        }

        double sleep_time = period - dt;
        if (sleep_time > 0) {
            struct timespec ts;
            ts.tv_sec = (time_t)sleep_time;
            ts.tv_nsec = (long)((sleep_time - ts.tv_sec) * 1e9);
            nanosleep(&ts, NULL);
        }
    }

    printf("\nClosing I2C device...\n");
    close(i2c_fd);
    printf("✓ Done.\n");

    return 0;
}
