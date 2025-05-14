#ifndef SSD1306_H
#define SSD1306_H

#include "common/common_types.h" // For status_t
#include "drivers/bus/i2c_bus.h" // For i2c_bus_handle_t
#include <stddef.h> // For size_t
#include <stdint.h> // For uint8_t etc.

// --- Configuration ---

// Common display sizes (adjust if using a different model)
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_YELLOW_HEIGHT 16
#define SSD1306_BLUE_HEIGHT 48
// #define SSD1306_HEIGHT 32 // Uncomment for 128x32 displays

// Calculate buffer size based on dimensions (1 bit per pixel)
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

// Default I2C address (can be 0x3C or 0x3D depending on the module)
#define SSD1306_DEFAULT_I2C_ADDR 0x3C

typedef struct {
    i2c_bus_handle_t i2c_handle; // Handle to the initialized I2C bus
    uint8_t i2c_addr;            // I2C address of the display
    uint8_t width;               // Display width in pixels
    uint8_t height;              // Display height in pixels
    uint8_t* buffer;             // Pointer to the display buffer memory
    size_t buffer_size;          // Size of the buffer in bytes
    uint8_t dirty_min_page, dirty_max_page;   // Range of pages that need updating
    uint8_t dirty_min_col, dirty_max_col;     // Range of columns that need updating
    // Add other config like reset pin if needed
} ssd1306_handle_t;

// --- Public Functions ---

/**
 * @brief Initializes the SSD1306 display.
 *
 * Allocates memory for the handle and display buffer, configures the display
 * controller via I2C commands.
 *
 * @param i2c_bus Handle to an already initialized I2C bus driver.
 * @param i2c_address The 7-bit I2C address of the SSD1306 module.
 * @param width Display width (e.g., 128).
 * @param height Display height (e.g., 64 or 32).
 * @param out_handle Pointer to store the handle for the initialized display.
 * @return STATUS_OK on success, or an error code on failure.
 */
status_t ssd1306_init(i2c_bus_handle_t i2c_bus, uint8_t i2c_address, uint8_t width, uint8_t height, ssd1306_handle_t** out_handle);

status_t ssd1306_draw_pixel(ssd1306_handle_t* handle, int16_t x, int16_t y, uint8_t color);

status_t ssd1306_update(ssd1306_handle_t* handle);
/**
 * @brief Cleans up resources used by the SSD1306 driver.
 *
 * Frees the display buffer and the handle memory.
 *
 * @param handle Handle to the SSD1306 display instance.
 */
void ssd1306_cleanup(ssd1306_handle_t* handle);

/**
 * @brief Clears the internal display buffer (sets all pixels to off).
 *
 * Note: This only modifies the buffer in memory. Call ssd1306_display()
 *       to update the actual screen.
 *
 * @param handle Handle to the SSD1306 display instance.
 * @return STATUS_OK on success, ERROR_NULL_POINTER if handle or buffer is NULL.
 */
status_t ssd1306_clear_buffer(ssd1306_handle_t* handle);

/**
 * @brief Sends the internal display buffer content to the SSD1306 controller.
 *
 * This function updates the physical display based on the current state
 * of the buffer modified by functions like ssd1306_clear_buffer() and
 * ssd1306_draw_pixel().
 *
 * @param handle Handle to the SSD1306 display instance.
 * @return STATUS_OK on success, or an I2C communication error code.
 */
status_t ssd1306_display(ssd1306_handle_t* handle);

// --- Optional Advanced Functions (Add later if needed) ---
// ...

#endif // SSD1306_H 