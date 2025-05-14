#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // For sleep
#include <bcm2835.h>

#include "drivers/bcm_manager.h"
#include "drivers/bus/i2c_bus.h"
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/font_terminus_16x32.h"
#include "common/common_types.h" // Include for status codes

#define CHAR_SPACING 2 // Pixels between characters

// Define display properties (if not already in ssd1306.h)
#define SSD1306_I2C_ADDR_DEFAULT 0x3C
#define SSD1306_WIDTH_DEFAULT    128
#define SSD1306_HEIGHT_DEFAULT   64


// --- Drawing Functions using the new font (now require handle) ---

// Draw a single character using the font definition from the header
// Pass the handle down to ssd1306_draw_pixel
void ssd1306_draw_char(ssd1306_handle_t* handle, int x, int y, char c, uint8_t color) {
    if (!handle) return; // Safety check

    // Handle unsupported characters
    if (c < FONT_ASCII_START || c > FONT_ASCII_END) {
        c = '?';
        if (c < FONT_ASCII_START || c > FONT_ASCII_END) return;
    }

    const unsigned char* char_data_start = &font_terminus_16x32[(c - FONT_ASCII_START) * FONT_BYTES_PER_CHAR];

    for (int col = 0; col < FONT_WIDTH; col++) {
        const unsigned char* col_data_ptr = char_data_start + (col * FONT_BYTES_PER_COL);
        for (int row = 0; row < FONT_HEIGHT; row++) {
            int byte_index = row / 8;
            int bit_index = row % 8;
            if ((col_data_ptr[byte_index] >> bit_index) & 0x01) {
                // Pass the handle to ssd1306_draw_pixel
                ssd1306_draw_pixel(handle, x + col, y + row, color);
            }
        }
    }
}

// Draw a string using the new font (now requires handle)
void ssd1306_draw_string(ssd1306_handle_t* handle, int x, int y, const char* str, uint8_t color) {
    if (!handle) return; // Safety check
    int current_x = x;
    while (*str) {
        // Pass the handle to ssd1306_draw_char
        ssd1306_draw_char(handle, current_x, y, *str, color);
        current_x += FONT_WIDTH + CHAR_SPACING;
        str++;
    }
}


// --- Main Test ---
int main() {
    printf("--- Starting SSD1306 Large Font Scroll Test ---\n");

    i2c_bus_handle_t i2c_bus = NULL;
    ssd1306_handle_t* ssd1306_handle = NULL;
    i2c_config_t i2c_conf;
    status_t status;

    memset(&i2c_conf, 0, sizeof(i2c_conf));

    // Initialize BCM Manager
    status = bcm_manager_init();
    if (status != STATUS_OK) {
        fprintf(stderr, "Failed to initialize BCM Manager, status=%d\n", status);
        return EXIT_FAILURE;
    }

    // Initialize I2C Bus
    status = i2c_bus_init(&i2c_conf, &i2c_bus);
    if (status != STATUS_OK) {
        fprintf(stderr, "Failed to initialize I2C Bus, status=%d\n", status);
        bcm_manager_close();
        return EXIT_FAILURE;
    }

    // Initialize SSD1306
    status = ssd1306_init(i2c_bus,
                          SSD1306_I2C_ADDR_DEFAULT,
                          SSD1306_WIDTH_DEFAULT,
                          SSD1306_HEIGHT_DEFAULT,
                          &ssd1306_handle);
    if (status != STATUS_OK) {
         fprintf(stderr, "Failed to initialize SSD1306, status=%d\n", status);
         if (i2c_bus) {
             i2c_bus_cleanup(i2c_bus);
         }
         bcm_manager_close();
         return EXIT_FAILURE;
    }
    printf("SSD1306 Initialized.\n");

    // --- Scrolling Logic ---
    const char* text = "Daddy love Addie";
    int text_len = strlen(text);
    int char_pixel_width = FONT_WIDTH + CHAR_SPACING;
    int text_pixel_width = text_len * char_pixel_width;

    // Use defined height constants
    int start_row = (SSD1306_HEIGHT_DEFAULT - FONT_HEIGHT) / 2;
    if (start_row < 0) start_row = 0;

    int current_col = SSD1306_WIDTH_DEFAULT; // Use defined width constant

    printf("Starting scroll loop (Font: %dx%d)...\n", FONT_WIDTH, FONT_HEIGHT);
    for (int i = 0; i < 800; i++) {
        // 1. Clear the display buffer - Use correct function name
        status = ssd1306_clear_buffer(ssd1306_handle); // Renamed function
        if (status != STATUS_OK) {
            fprintf(stderr, "Warning: Failed to clear SSD1306 buffer, status=%d\n", status);
            // Decide if we should continue or break
        }


        // 2. Draw the string at the current position - Pass handle
        ssd1306_draw_string(ssd1306_handle, current_col, start_row, text, 1); // Color 1 = ON

        // 3. Update the physical display - Pass handle
        status = ssd1306_display(ssd1306_handle);
         if (status != STATUS_OK) {
            fprintf(stderr, "Warning: Failed to update SSD1306 display, status=%d\n", status);
            // Decide if we should continue or break
        }


        // 4. Update position for next frame (scroll left)
        current_col--;

        // 5. Wrap around logic
        if (current_col < -text_pixel_width) {
            current_col = SSD1306_WIDTH_DEFAULT; // Use defined width constant
        }

        // 6. Delay
        bcm2835_delay(40);
    }

    printf("Scroll loop finished.\n");

    // --- Cleanup ---
    printf("Clearing display...\n");
    if (ssd1306_handle) {
        ssd1306_clear_buffer(ssd1306_handle); // Renamed function
        ssd1306_display(ssd1306_handle);
    }


    printf("Cleaning up SSD1306 driver...\n");
    if (ssd1306_handle) {
        ssd1306_cleanup(ssd1306_handle);
        ssd1306_handle = NULL;
    }


    printf("Closing I2C Bus...\n");
    if (i2c_bus) {
        i2c_bus_cleanup(i2c_bus);
        i2c_bus = NULL;
    }


    printf("Closing BCM Manager...\n");
    bcm_manager_close();

    printf("Cleanup complete.\n");
    return EXIT_SUCCESS;
} 