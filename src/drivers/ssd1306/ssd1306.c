#include "drivers/ssd1306/ssd1306.h"
#include "drivers/bus/i2c_bus.h" // Include necessary headers
#include "common/common_types.h"        // Include necessary headers
#include <stdlib.h> // For malloc, free
#include <string.h> // For memset, memcpy
#include <stdio.h>  // For fprintf, stderr
#include <unistd.h> // For usleep

// --- SSD1306 Command Definitions ---
// Fundamental Commands
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4 // Resume normal display output from RAM
#define SSD1306_DISPLAYALLON 0xA5        // Force entire display ON, ignoring RAM
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

// Addressing Setting Commands
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21 // Requires 2 more bytes (start, end)
#define SSD1306_PAGEADDR 0x22   // Requires 2 more bytes (start, end)
// For MEMORYMODE 0x00 (Horizontal): Col Address Pointer increases, wraps page, Page Address Pointer resets
// For MEMORYMODE 0x01 (Vertical): Page Address Pointer increases, wraps column, Col Address Pointer resets
// For MEMORYMODE 0x02 (Page): Col Address Pointer increases, remains on same page (DEFAULT)

// Hardware Configuration Commands
#define SSD1306_SETSTARTLINE 0x40 // OR with 0-63
#define SSD1306_SEGREMAP 0xA0     // OR with 0x01 for column 127 mapped to SEG0
#define SSD1306_SETMULTIPLEX 0xA8 // Requires 1 more byte (15-63)
#define SSD1306_COMSCANINC 0xC0   // Normal COM0...COM[N-1] scan
#define SSD1306_COMSCANDEC 0xC8   // Remapped COM[N-1]...COM0 scan
#define SSD1306_SETDISPLAYOFFSET 0xD3 // Requires 1 more byte (0-63)
#define SSD1306_SETCOMPINS 0xDA       // Requires 1 more byte

// Timing & Driving Scheme Commands
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5 // Requires 1 more byte
#define SSD1306_SETPRECHARGE 0xD9       // Requires 1 more byte
#define SSD1306_SETVCOMDETECT 0xDB      // Requires 1 more byte

// Charge Pump Command
#define SSD1306_CHARGEPUMP 0x8D // Requires 1 more byte (0x10 disable, 0x14 enable)

// Define a chunk size for I2C transfers
#define SSD1306_I2C_CHUNK_SIZE 64 // Send 64 bytes of pixel data at a time

// Command to set Page Start Address for Page Addressing Mode
#define SSD1306_SETPAGESTARTADDR 0xB0 // OR with page number 0-7

// --- Internal Helper Functions ---

// --- Static Helper Function Forward Declarations ---
// Add these prototypes before any function implementations
static status_t ssd1306_send_command(ssd1306_handle_t* handle, uint8_t command);
static status_t ssd1306_send_data(ssd1306_handle_t* handle, const uint8_t* data, size_t size);

/**
 * @brief Sends a single command byte to the SSD1306.
 * @param handle Pointer to the SSD1306 handle.
 * @param command The command byte to send.
 * @return STATUS_OK on success, or an I2C error code.
 */
static status_t ssd1306_send_command(ssd1306_handle_t* handle, uint8_t command) {
    if (!handle || !handle->i2c_handle) {
        return ERROR_NULL_POINTER;
    }
    // For SSD1306 over I2C, commands are sent as data bytes following
    // a control byte with the Co (Continuation) bit 0 and D/C# (Data/Command) bit 0.
    // Control byte = 0x00
    uint8_t buffer[2] = {0x00, command};
    // Retry I2C write up to 3 times with small delay to avoid timeouts
    status_t status;
    for (int retry = 0; retry < 3; ++retry) {
        status = i2c_bus_write(handle->i2c_handle, handle->i2c_addr, buffer, 2);
        if (status == STATUS_OK) break;
        usleep(100);
    }
    return status;
}

// --- Public Function Implementations ---

status_t ssd1306_init(i2c_bus_handle_t i2c_bus, uint8_t i2c_address, uint8_t width, uint8_t height, ssd1306_handle_t** out_handle) {
    if (!i2c_bus || !out_handle) {
        return ERROR_NULL_POINTER;
    }
    if (width == 0 || height == 0 || (width * height / 8) == 0) {
        return ERROR_INVALID_PARAM;
    }

    status_t status = STATUS_OK;
    ssd1306_handle_t* handle = NULL;

    // 1. Allocate memory for the handle
    handle = (ssd1306_handle_t*)malloc(sizeof(ssd1306_handle_t));
    if (!handle) {
        return ERROR_NO_MEM;
    }
    memset(handle, 0, sizeof(ssd1306_handle_t)); // Clear handle memory

    // 2. Store configuration
    handle->i2c_handle = i2c_bus;
    handle->i2c_addr = i2c_address;
    handle->width = width;
    handle->height = height;
    handle->buffer_size = (size_t)width * height / 8;

    // 3. Allocate memory for the display buffer
    handle->buffer = (uint8_t*)malloc(handle->buffer_size);
    if (!handle->buffer) {
        fprintf(stderr, "ERROR: ssd1306_init: Failed to allocate display buffer (%zu bytes)\n", handle->buffer_size);
        free(handle); // Clean up handle allocation
        return ERROR_NO_MEM;
    }
    memset(handle->buffer, 0, handle->buffer_size); // Clear buffer initially

    // 4. Send COMPREHENSIVE initialization sequence
    printf("INFO: ssd1306_init: Sending initialization sequence ...\n");

    // Initialization Sequence using Adafruit's standard configuration
    const uint8_t init_sequence[] = {
        SSD1306_DISPLAYOFF,         // 0xAE - Display OFF

        SSD1306_SETDISPLAYCLOCKDIV, // 0xD5 - Set Clock Divide Ratio/Oscillator Frequency
        0x80,                       //   Default ratio

        SSD1306_SETMULTIPLEX,       // 0xA8 - Set Multiplex Ratio
        (uint8_t)(height - 1),      //   height - 1 (e.g., 63 for 128x64)

        SSD1306_SETDISPLAYOFFSET,   // 0xD3 - Set Display Offset
        0x00,                       //   No offset

        SSD1306_SETSTARTLINE | 0x0, // 0x40 - Set Display Start Line (line 0)

        SSD1306_CHARGEPUMP,         // 0x8D - Charge Pump Setting
        0x14,                       //   Enable charge pump (for internal VCC)

        SSD1306_MEMORYMODE,         // 0x20 - Set Memory Addressing Mode
        0x00,                       //   <<< 0x00 = Horizontal Addressing Mode >>>

        SSD1306_SEGREMAP | 0x1,     // 0xA1 - Set Segment Re-map (col 127 mapped to SEG0)
                                    //   This flips the display horizontally

        SSD1306_COMSCANDEC,         // 0xC8 - Set COM Output Scan Direction (scan COM[N-1] to COM0)
                                    //   This flips the display vertically

        SSD1306_SETCOMPINS,         // 0xDA - Set COM Pins Hardware Configuration
        (height == 64) ? 0x12 : 0x02, // 0x12 for 128x64, 0x02 for 128x32 (common values)

        SSD1306_SETCONTRAST,        // 0x81 - Set Contrast Control
        0xCF,                       //   Contrast value (adjust as needed)

        SSD1306_SETPRECHARGE,       // 0xD9 - Set Pre-charge Period
        0xF1,                       //   Pre-charge period (for internal VCC)

        SSD1306_SETVCOMDETECT,      // 0xDB - Set VCOMH Deselect Level
        0x40,                       //   VCOM deselect level (adjust as needed)

        SSD1306_DISPLAYALLON_RESUME,// 0xA4 - Entire Display ON (resume from RAM content)
        SSD1306_NORMALDISPLAY,      // 0xA6 - Set Normal Display mode

        SSD1306_DISPLAYON           // 0xAF - Display ON in normal mode
    };

    for (size_t i = 0; i < sizeof(init_sequence); ++i) {
        status = ssd1306_send_command(handle, init_sequence[i]);
        if (status != STATUS_OK) {
            fprintf(stderr, "ERROR: ssd1306_init: Failed command 0x%02X (index %zu), status=%d\n",
                    init_sequence[i], i, status);
            // Go to cleanup if allocation succeeded before failure
            goto error_cleanup;
        }
        usleep(100); // Small delay between commands might help stability
    }
    usleep(100 * 1000); // Delay after sequence

    // Set full-screen addressing window (horizontal mode)
    ssd1306_send_command(handle, SSD1306_PAGEADDR);
    ssd1306_send_command(handle, 0);
    ssd1306_send_command(handle, (handle->height / 8) - 1);
    ssd1306_send_command(handle, SSD1306_COLUMNADDR);
    ssd1306_send_command(handle, 0);
    ssd1306_send_command(handle, handle->width - 1);

    // Clear buffer and display initially
    status = ssd1306_clear_buffer(handle);
    if (status != STATUS_OK) { /* Handle error or warning */ }
    status = ssd1306_display(handle); // Send empty buffer
    if (status != STATUS_OK) { /* Handle error or warning */ }
    *out_handle = handle;
    printf("INFO: ssd1306_init: Initialization successful.\n");
    return STATUS_OK;

error_cleanup: // Label for cleaning up resources on error during init
    fprintf(stderr, "ERROR: ssd1306_init: Cleaning up after initialization failure.\n");
    if (handle) {
        if (handle->buffer) {
            free(handle->buffer);
            handle->buffer = NULL;
        }
        free(handle);
    }
    *out_handle = NULL;
    return status; // Return the error status that caused the jump
}

// --- Other Public Functions (To be implemented) ---

void ssd1306_cleanup(ssd1306_handle_t* handle) {
    if (!handle) {
        return; // Nothing to clean up if handle is NULL
    }

    // Optional: Turn the display off before cleaning up
    // Ignore potential errors here, as we are cleaning up anyway
    (void)ssd1306_send_command(handle, SSD1306_DISPLAYOFF);

    // Free the display buffer if it was allocated
    if (handle->buffer) {
        free(handle->buffer);
        handle->buffer = NULL; // Avoid dangling pointer
    }

    // Free the handle itself
    free(handle);

    printf("INFO: ssd1306_cleanup: Resources released.\n");
}

status_t ssd1306_clear_buffer(ssd1306_handle_t* handle) {
    if (!handle || !handle->buffer) return ERROR_NULL_POINTER;
    memset(handle->buffer, 0x00, handle->buffer_size); // Set all pixels OFF
    return STATUS_OK;
}

status_t ssd1306_draw_pixel(ssd1306_handle_t* handle, int16_t x, int16_t y, uint8_t color) {
    if (!handle || !handle->buffer) {
        return ERROR_NULL_POINTER;
    }
    // Check bounds
    if (x < 0 || x >= handle->width || y < 0 || y >= handle->height) {
        return ERROR_INVALID_PARAM; // Out of bounds
    }

    // Calculate buffer index based on SSD1306 page addressing:
    // Index = column + (page_number * columns_per_page)
    uint16_t buffer_index = x + (y / 8) * handle->width;

    // Calculate the specific bit within the byte to modify.
    uint8_t bit_mask = 1 << (y % 8);

    // Modify the buffer
    if (color) { // Set pixel
        handle->buffer[buffer_index] |= bit_mask;
    } else { // Clear pixel
        handle->buffer[buffer_index] &= ~bit_mask;
    }
    return STATUS_OK;
}

status_t ssd1306_display(ssd1306_handle_t* handle) {
    if (!handle || !handle->buffer || !handle->i2c_handle) {
        return ERROR_NULL_POINTER;
    }
    // Only send raw frame data; window set during init
    status_t status = ssd1306_send_data(handle, handle->buffer, handle->buffer_size);
    if (status != STATUS_OK) {
        fprintf(stderr, "ERROR: ssd1306_display: Failed to write buffer data, status=%d\n", status);
    }
    return status;
}

status_t ssd1306_update(ssd1306_handle_t* handle) {
    // Send entire buffer to display
    return ssd1306_display(handle);
}

// --- Helper Functions ---

// Sends a buffer of data bytes, chunking if necessary
static status_t ssd1306_send_data(ssd1306_handle_t* handle, const uint8_t* data, size_t size) {
    if (!handle || !handle->i2c_handle || !data) return ERROR_NULL_POINTER;
    if (size == 0) return STATUS_OK;

    // Full-buffer write for maximum throughput (~40 fps)
    uint8_t* full_buf = malloc(size + 1);
    if (!full_buf) return ERROR_NO_MEM;
    full_buf[0] = 0x40; // Data control byte
    memcpy(full_buf + 1, data, size);
    status_t status = i2c_bus_write(handle->i2c_handle, handle->i2c_addr,
                                    full_buf, size + 1);
    free(full_buf);
    return status;
}