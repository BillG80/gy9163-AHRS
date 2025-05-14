#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <stdint.h>
#include <stddef.h> // For size_t
#include "common/common_types.h" // For status_t

// Define a practical max transfer size (adjust if needed)
#define SPI_MAX_TRANSFER_SIZE 4096

// --- Data Structures ---

// Opaque handle structure
typedef struct spi_bus_device* spi_bus_handle_t;

// Configuration structure for SPI initialization
typedef struct {
    uint8_t bus_num;        // SPI bus number (e.g., 0 for SPI0)
    uint16_t clock_divider; // Use BCM2835_SPI_CLOCK_DIVIDER_xxx values
    uint8_t spi_mode;       // Use BCM2835_SPI_MODE_x values
    // Chip select is handled manually by the driver using the bus (e.g. icm20948)
    // uint8_t cs_pin; // Not needed here if handled by driver
} spi_config_t;


// --- Function Prototypes ---

/**
 * @brief Initializes the SPI bus for communication on a specific channel.
 *
 * @param handle_ptr Pointer to a variable where the allocated SPI bus handle will be stored.
 * @param spi_channel The SPI channel (Chip Select) to use (e.g., 0 for CS0, 1 for CS1).
 * @param speed_hz The desired SPI clock speed in Hz.
 * @return STATUS_OK on success, or an error code otherwise.
 */
status_t spi_bus_init(spi_bus_handle_t* handle_ptr, uint8_t spi_channel, uint32_t speed_hz);

/**
 * @brief Cleans up resources used by the SPI bus handle.
 * De-initializes the SPI bus if necessary.
 *
 * @param handle Pointer to the bus handle.
 * @return STATUS_OK on success, or an error code otherwise.
 */
status_t spi_bus_cleanup(spi_bus_handle_t handle);

/**
 * @brief Performs an SPI data transfer (transmit and receive).
 * Assumes the Chip Select (CS) line is managed (asserted/de-asserted)
 * externally by the calling driver before/after calling this function.
 *
 * @param handle Handle to the initialized SPI bus.
 * @param tx_buffer Pointer to the buffer containing data to transmit.
 * @param rx_buffer Pointer to the buffer where received data will be stored.
 * @param len Number of bytes to transfer.
 * @return STATUS_OK on success, or an error code otherwise.
 */
status_t spi_bus_transfer(spi_bus_handle_t handle, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len);


#endif // SPI_BUS_H 