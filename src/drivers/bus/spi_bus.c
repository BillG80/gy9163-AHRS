#include "drivers/bus/spi_bus.h"
#include "common/common_types.h" // Corrected include path
#include "bcm_manager.h" // Include bcm_manager for init check
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // For memset
// Remove include for bcm_manager if it's not used elsewhere in this file
// #include "../bcm_manager.h" // REMOVED or commented out

// Define the actual structure (implementation detail)
struct spi_bus_device {
    uint8_t spi_channel; // Store the channel (CS line)
    uint32_t speed_hz;   // Store the requested speed (for info)
    bcm2835SPIClockDivider clock_divider; // Store the actual divider used
};

// Remove local tracking of bcm2835 initialization
// static bool bcm2835_initialized = false; // REMOVED
// static int spi_instance_count = 0; // REMOVED (manager handles counts)

// --- Initialization ---
// Make sure this definition matches the declaration in spi_bus.h
status_t spi_bus_init(spi_bus_handle_t* handle_ptr, uint8_t spi_channel, uint32_t speed_hz) {
    if (!handle_ptr) {
        return ERROR_NULL_POINTER;
    }
    *handle_ptr = NULL; // Initialize output parameter

    // Allocate memory for the device structure
    spi_bus_handle_t handle = (spi_bus_handle_t)malloc(sizeof(struct spi_bus_device));
    if (!handle) {
        perror("ERROR: Failed to allocate memory for SPI handle");
        return ERROR_ALLOCATION_FAILED;
    }

    // Store configuration
    handle->spi_channel = spi_channel;
    handle->speed_hz = speed_hz; // Store requested speed

    // Initialize bcm2835 library (should be managed by bcm_manager)
    if (!bcm_manager_is_initialized()) {
        fprintf(stderr, "ERROR: BCM Manager not initialized before SPI init\n");
        free(handle);
        return ERROR_NOT_INITIALIZED;
    }

    // Configure SPI pins (handled by bcm2835_spi_begin)
    bcm2835_spi_begin(); // Initialize SPI pins

    // --- Calculate and Set SPI Clock Divider ---
    // bcm2835 uses power-of-2 dividers. Find the nearest suitable one.
    // BCM2835_CORE_CLK_HZ is typically 250MHz.
    uint32_t divider = BCM2835_CORE_CLK_HZ / speed_hz;
    // Round up to the nearest power of 2 divider supported by bcm2835
    // (The library uses specific enum values like BCM2835_SPI_CLOCK_DIVIDER_*)
    // Example: Find the closest power of 2 >= divider
    uint32_t power_of_2_divider = 2;
    while (power_of_2_divider < divider && power_of_2_divider < 65536) { // 65536 is max divider
        power_of_2_divider <<= 1;
    }
    // Ensure the divider is within the valid enum range (2 to 65536)
    if (power_of_2_divider < 2) power_of_2_divider = 2; // Min divider
    if (power_of_2_divider > 65536) power_of_2_divider = 65536; // Max divider

    // Cast the integer divider to the enum type.
    // Note: This relies on the enum values matching the integer divider values.
    // Check bcm2835.h to confirm BCM2835_SPI_CLOCK_DIVIDER_xxx = xxx
    handle->clock_divider = (bcm2835SPIClockDivider)power_of_2_divider;

    bcm2835_spi_setClockDivider(handle->clock_divider);
    uint32_t actual_speed = BCM2835_CORE_CLK_HZ / power_of_2_divider;
    // --- End Clock Divider Calculation ---


    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_chipSelect(handle->spi_channel);
    bcm2835_spi_setChipSelectPolarity(handle->spi_channel, LOW);

    printf("INFO: SPI Bus Initialized (Channel %d, Requested Speed %u Hz, Actual Speed %u Hz, Divider %u)\n",
           handle->spi_channel, speed_hz, actual_speed, power_of_2_divider);

    *handle_ptr = handle;
    return STATUS_OK;
}

// --- Close ---
status_t spi_bus_close(spi_bus_handle_t handle) {
    if (!handle) {
        // Optional: Return OK if already closed, or an error?
        // Let's return OK for idempotency.
        return STATUS_OK;
    }

    printf("INFO: Closing SPI Bus (Channel %d)\n", handle->spi_channel);

    // De-initialize SPI pins (handled by bcm2835_spi_end)
    // Note: bcm2835_spi_end affects all SPI, might be better managed globally
    // if multiple SPI devices are used. For now, close it here.
    bcm2835_spi_end();

    // Free the handle memory
    free(handle);

    return STATUS_OK;
}

// --- Transfer ---
status_t spi_bus_transfer(spi_bus_handle_t handle, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len) {
    if (!handle) {
        return ERROR_NULL_POINTER;
    }

    // Check if length is valid (e.g., not zero and not excessively large)
    // Note: bcm2835_spi_transfernb handles larger transfers internally,
    // but a sanity check is good.
    if (len == 0 || len > 65535) { // 65535 is max for bcm2835_spi_transfernb
        fprintf(stderr, "ERROR: spi_bus_transfer: Invalid transfer length %zu\n", len);
        return ERROR_INVALID_PARAM;
    }

    // Use %zu for size_t
    printf("Debug: spi_bus_transfer entering for len %zu\n", len);

    if (!tx_buffer || !rx_buffer) {
         fprintf(stderr, "ERROR: spi_bus_transfer: tx_buffer or rx_buffer is NULL\n");
        return ERROR_NULL_POINTER;
    }

    // Assume CS is active low
    //printf("Debug: spi_bus_transfer: Asserting CS (Pin %u LOW)\n", handle->spi_channel);
    //bcm2835_gpio_write(handle->spi_channel, LOW); // Assert CS
    //printf("Debug: spi_bus_transfer: CS Asserted\n");

    // Small delay after CS assert? Sometimes needed.
    // bcm2835_delayMicroseconds(5);

    //printf("Debug: spi_bus_transfer: Calling bcm2835_spi_transfernb...\n");
    // The core SPI transfer
    //bcm2835_spi_transfernb((char*)tx_buffer, (char*)rx_buffer, len);
    //printf("Debug: spi_bus_transfer: bcm2835_spi_transfernb returned\n");

    // Small delay before CS deassert? Sometimes needed.
    // bcm2835_delayMicroseconds(5);

    printf("Debug: spi_bus_transfer: Deasserting CS (Pin %u HIGH)\n", handle->spi_channel);
    bcm2835_gpio_write(handle->spi_channel, HIGH); // Deassert CS
    printf("Debug: spi_bus_transfer: CS Deasserted\n");

    printf("Debug: spi_bus_transfer exiting OK\n");
    return STATUS_OK;
} 