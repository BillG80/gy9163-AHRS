#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include <stddef.h> // For size_t
#include "common/common_types.h" // Include our common types and status codes

// I2C Speed Configuration
typedef enum {
    I2C_SPEED_100KHZ,
    I2C_SPEED_400KHZ,
    // Add other speeds if needed and supported by bcm2835/hardware
} i2c_speed_t;

// I2C Configuration Structure
typedef struct {
    uint8_t bus_num;    // I2C bus number (usually 1 for RPi main GPIO header)
    i2c_speed_t speed;  // Desired I2C clock speed
} i2c_config_t;

// Opaque handle for an I2C bus instance
typedef struct i2c_bus_device* i2c_bus_handle_t;

/**
 * @brief Initializes an I2C bus.
 *
 * @param config Pointer to the I2C configuration structure.
 * @param handle Pointer to store the created I2C bus handle.
 * @return STATUS_OK on success, or an error code otherwise.
 */
status_t i2c_bus_init(const i2c_config_t* config, i2c_bus_handle_t* handle);

/**
 * @brief Writes data to a specific I2C slave device.
 *
 * @param handle The I2C bus handle.
 * @param slave_addr The 7-bit I2C address of the target device.
 * @param data Pointer to the data buffer to transmit.
 * @param len Number of bytes to write.
 * @return STATUS_OK on success, ERROR_IO on NACK/error, or other error codes.
 */
status_t i2c_bus_write(i2c_bus_handle_t handle, uint8_t slave_addr, const uint8_t* data, size_t len);

/**
 * @brief Reads data from a specific I2C slave device.
 *
 * @param handle The I2C bus handle.
 * @param slave_addr The 7-bit I2C address of the target device.
 * @param buffer Pointer to the buffer to store received data.
 * @param len Number of bytes to read.
 * @return STATUS_OK on success, ERROR_IO on NACK/error, or other error codes.
 */
status_t i2c_bus_read(i2c_bus_handle_t handle, uint8_t slave_addr, uint8_t* buffer, size_t len);

/**
 * @brief Writes data (e.g., a register address) and then reads data from an I2C slave
 *        using a repeated start condition.
 *
 * @param handle The I2C bus handle.
 * @param slave_addr The 7-bit I2C address of the target device.
 * @param write_data Pointer to the data to write first.
 * @param write_len Number of bytes to write.
 * @param read_buffer Pointer to the buffer to store the read data.
 * @param read_len Number of bytes to read after the repeated start.
 * @return STATUS_OK on success, ERROR_IO on NACK/error, or other error codes.
 */
status_t i2c_bus_write_read(i2c_bus_handle_t handle, uint8_t slave_addr, const uint8_t* write_data, size_t write_len, uint8_t* read_buffer, size_t read_len);

// Keep ONLY the declaration (prototype) here:
status_t i2c_bus_cleanup(i2c_bus_handle_t handle);

#endif // I2C_BUS_H 