#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h> // For memset/memcmp
#include <unistd.h> // For usleep
#include <bcm2835.h>

#include "bcm_manager.h"
#include "drivers/bus/spi_bus.h"
#include "drivers/bus/i2c_bus.h" // Include I2C bus header
#include "common/common_types.h"

// --- Explicit extern declarations (Workaround for compiler warnings) ---
extern status_t spi_bus_close(spi_bus_handle_t handle);
extern status_t i2c_bus_close(i2c_bus_handle_t handle);
// We might need others if warnings appear (e.g., i2c_bus_write)
// extern status_t i2c_bus_write(i2c_bus_handle_t handle, uint8_t i2c_addr, const uint8_t* data, size_t len);
// --- End Workaround ---

// --- SPI Test Parameters ---
#define TEST_SPI_CHANNEL    0 // BCM2835_SPI_CS0
#define TEST_SPI_SPEED_HZ   1000000 // 1 MHz
#define TEST_BUFFER_SIZE    32

// --- I2C Test Parameters ---
#define I2C_BUS_NUM         1  // Standard Raspberry Pi I2C bus (used implicitly by i2c_bus.c)
#define SSD1306_I2C_ADDR    0x3C // Default address for SSD1306

// --- GPIO Test Parameters ---
#define LED1_PIN 12 // GPIO 12 on header pin 32 (Using direct BCM number)
#define LED2_PIN 16 // GPIO 16 on header pin 36 (Using direct BCM number)
#define BLINK_COUNT 5
#define BLINK_DELAY_MS 500

// Simple CHECK macro (can be used for SPI and I2C bus functions)
#define CHECK(op, msg) do { \
    status_t check_status = (op); \
    if (check_status != STATUS_OK) { \
        fprintf(stderr, "TEST FAILED: %s (%s returned %d)\n", msg, #op, check_status); \
        /* Consider adding cleanup here if needed before exit */ \
        exit(EXIT_FAILURE); \
    } else { \
        printf("TEST PASSED: %s\n", msg); \
    } \
} while(0)

// Helper function to convert status codes to strings
const char* status_to_string(status_t status) {
    switch (status) {
        case STATUS_OK: return "STATUS_OK";
        case ERROR_NULL_POINTER: return "ERROR_NULL_POINTER";
        case ERROR_INVALID_PARAM: return "ERROR_INVALID_PARAM";
        case ERROR_HARDWARE: return "ERROR_HARDWARE";
        case ERROR_COMMUNICATION: return "ERROR_COMMUNICATION"; // Covers I2C NACK etc.
        case ERROR_TIMEOUT: return "ERROR_TIMEOUT";
        case ERROR_SENSOR_OVERFLOW: return "ERROR_SENSOR_OVERFLOW";
        case ERROR_NOT_INITIALIZED: return "ERROR_NOT_INITIALIZED";
        case ERROR_ALREADY_INITIALIZED: return "ERROR_ALREADY_INITIALIZED";
        case ERROR_ALLOCATION_FAILED: return "ERROR_ALLOCATION_FAILED";
        default: return "OTHER_ERROR";
    }
}


int main() {
    printf("--- Starting Combined Bus and GPIO Test ---\n");

    spi_bus_handle_t spi_bus = NULL;
    i2c_bus_handle_t i2c_bus = NULL; // Handle for I2C
    uint8_t tx_buf[TEST_BUFFER_SIZE];
    uint8_t rx_buf[TEST_BUFFER_SIZE];
    status_t status;
    i2c_config_t i2c_conf; // Create a config struct for i2c_bus_init

    // Initialize the config struct (even if unused by current i2c_bus_init)
    // Set default values or leave empty if appropriate
    memset(&i2c_conf, 0, sizeof(i2c_conf));
    // i2c_conf.bus_number = I2C_BUS_NUM; // If config struct has these fields
    // i2c_conf.speed_hz = 100000;

    // ========================================
    // Initialize BCM Manager (once for all)
    // ========================================
    CHECK(bcm_manager_init(), "BCM Manager Init");

    // ========================================
    // SPI Bus Tests
    // ========================================
    printf("\n--- Initializing SPI Bus ---\n");
    printf("Initializing SPI Channel %d at %u Hz...\n", TEST_SPI_CHANNEL, TEST_SPI_SPEED_HZ);
    CHECK(spi_bus_init(&spi_bus, TEST_SPI_CHANNEL, TEST_SPI_SPEED_HZ), "SPI Bus Init");

    printf("\n--- Test 1: SPI Basic Transfer ---\n");
    memset(tx_buf, 0xA5, TEST_BUFFER_SIZE);
    memset(rx_buf, 0x00, TEST_BUFFER_SIZE);
    printf("Performing SPI transfer (%d bytes)...\n", TEST_BUFFER_SIZE);
    status = spi_bus_transfer(spi_bus, tx_buf, rx_buf, (size_t)TEST_BUFFER_SIZE);
    CHECK(status, "SPI Bus Transfer");
    printf("TX Buffer: "); for(int i=0; i<TEST_BUFFER_SIZE; i++) printf("0x%02X ", tx_buf[i]); printf("\n");
    printf("RX Buffer: "); for(int i=0; i<TEST_BUFFER_SIZE; i++) printf("0x%02X ", rx_buf[i]); printf("\n");
    if (memcmp(tx_buf, rx_buf, TEST_BUFFER_SIZE) == 0) {
        printf("SPI Loopback Test PASSED: RX data matches TX data.\n");
    } else {
        printf("SPI Loopback Test NOTE: RX data does not match TX data (expected if MISO/MOSI not connected).\n");
    }

    printf("\n--- Test 2: SPI NULL Pointer Checks ---\n");
    status = spi_bus_transfer(NULL, tx_buf, rx_buf, (size_t)TEST_BUFFER_SIZE);
    if (status == ERROR_NULL_POINTER) printf("TEST PASSED: spi_bus_transfer correctly handled NULL handle.\n");
    else fprintf(stderr, "TEST FAILED: spi_bus_transfer NULL handle check (returned %d).\n", status);
    status = spi_bus_transfer(spi_bus, NULL, rx_buf, (size_t)TEST_BUFFER_SIZE);
     if (status == ERROR_NULL_POINTER) printf("TEST PASSED: spi_bus_transfer correctly handled NULL tx_buffer.\n");
     else fprintf(stderr, "TEST FAILED: spi_bus_transfer NULL tx_buffer check (returned %d).\n", status);
     status = spi_bus_transfer(spi_bus, tx_buf, NULL, (size_t)TEST_BUFFER_SIZE);
     if (status == ERROR_NULL_POINTER) printf("TEST PASSED: spi_bus_transfer correctly handled NULL rx_buffer.\n");
     else fprintf(stderr, "TEST FAILED: spi_bus_transfer NULL rx_buffer check (returned %d).\n", status);

    printf("\n--- Test 3: SPI Zero Length Transfer ---\n");
     status = spi_bus_transfer(spi_bus, tx_buf, rx_buf, 0);
     if (status == STATUS_OK) printf("TEST PASSED: spi_bus_transfer correctly handled zero length.\n");
     else fprintf(stderr, "TEST FAILED: spi_bus_transfer zero length check (returned %d).\n", status);

    // ========================================
    // I2C Bus Tests
    // ========================================
    printf("\n--- Initializing I2C Bus ---\n");
    // Call i2c_bus_init with the config struct and handle pointer
    CHECK(i2c_bus_init(&i2c_conf, &i2c_bus), "I2C Bus Init");

    printf("\n--- Test 4: I2C Detect Device (SSD1306 @ 0x%02X) ---\n", SSD1306_I2C_ADDR);
    // Attempt a simple write to the device address to see if it ACKs.
    // Sending a single byte (e.g., 0x00 command) is common for probing.
    uint8_t i2c_probe_cmd = 0x00;
    status = i2c_bus_write(i2c_bus, SSD1306_I2C_ADDR, &i2c_probe_cmd, 1);
    if (status == STATUS_OK) {
        printf("  I2C ACK received from 0x%02X (Device likely present)\n", SSD1306_I2C_ADDR);
        CHECK(status, "I2C Device ACK Check"); // Macro will print PASSED
    } else {
        printf("  I2C NACK or Error (%d) from 0x%02X (Device likely absent or error occurred)\n", status, SSD1306_I2C_ADDR);
        // Don't use CHECK macro here as NACK is expected if device is absent
    }

    // ========================================
    // GPIO Tests
    // ========================================
    printf("\n--- Test 5: GPIO Configure Pins ---\n");
    printf("Configuring GPIO %d (Pin %d) as OUTPUT\n", 12, LED1_PIN);
    bcm2835_gpio_fsel(LED1_PIN, BCM2835_GPIO_FSEL_OUTP);
    printf("Configuring GPIO %d (Pin %d) as OUTPUT\n", 16, LED2_PIN);
    bcm2835_gpio_fsel(LED2_PIN, BCM2835_GPIO_FSEL_OUTP);

    printf("\n--- Test 6: GPIO Blink LEDs (%d times) ---\n", BLINK_COUNT);
    printf("Watch LEDs connected to GPIO %d and GPIO %d.\n", 12, 16);
    for (int i = 0; i < BLINK_COUNT; ++i) {
        printf("Blink %d: LED1 ON, LED2 OFF\n", i + 1);
        bcm2835_gpio_write(LED1_PIN, HIGH);
        bcm2835_gpio_write(LED2_PIN, LOW);
        bcm2835_delay(BLINK_DELAY_MS);

        printf("Blink %d: LED1 OFF, LED2 ON\n", i + 1);
        bcm2835_gpio_write(LED1_PIN, LOW);
        bcm2835_gpio_write(LED2_PIN, HIGH);
        bcm2835_delay(BLINK_DELAY_MS);
    }
    printf("Turning off LEDs.\n");
    bcm2835_gpio_write(LED1_PIN, LOW);
    bcm2835_gpio_write(LED2_PIN, LOW);

    // ========================================
    // Cleanup
    // ========================================
    printf("\n--- Test Finished ---\n");

    printf("Closing SPI Bus...\n");
    CHECK(spi_bus_close(spi_bus), "SPI Bus Close");
    spi_bus = NULL;

    printf("Closing I2C Bus...\n");
    CHECK(i2c_bus_close(i2c_bus), "I2C Bus Close");
    i2c_bus = NULL;

    printf("Closing BCM Manager...\n");
    bcm_manager_close();

    printf("Cleanup complete.\n");

    return EXIT_SUCCESS;
} 