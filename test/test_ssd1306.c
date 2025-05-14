#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> // For bool type
#include <unistd.h> // For usleep
#include <math.h>   // For floor in diagonal line drawing

// Include necessary headers
#include "common/common_types.h"
#include "bcm_manager.h"
#include "drivers/bus/i2c_bus.h"
#include "drivers/ssd1306/ssd1306.h" // Driver under test

// --- Test Configuration ---
#define TEST_I2C_BUS_NUM 0 // Usually I2C bus 1 on Raspberry Pi
#define TEST_I2C_SPEED 400000 // 400kHz, common for SSD1306
#define TEST_DISPLAY_WIDTH SSD1306_WIDTH // Use default from header
#define TEST_DISPLAY_HEIGHT SSD1306_HEIGHT // Use default from header
#define TEST_DISPLAY_ADDR SSD1306_DEFAULT_I2C_ADDR // Use default from header

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C

// Helper function to check buffer content
bool check_buffer(const uint8_t* buffer, size_t size, uint8_t expected_value) {
    for (size_t i = 0; i < size; ++i) {
        if (buffer[i] != expected_value) {
            printf("    FAIL: Buffer check failed at index %zu. Expected 0x%02X, Got 0x%02X\n",
                   i, expected_value, buffer[i]);
            return false;
        }
    }
    return true;
}

// Helper function to check a specific pixel bit
bool check_pixel_bit(const ssd1306_handle_t* handle, int16_t x, int16_t y, bool expected_state) {
     if (!handle || !handle->buffer) return false;
     if (x < 0 || x >= handle->width || y < 0 || y >= handle->height) return false; // Out of bounds

     size_t byte_index = (size_t)x + ((size_t)y / 8) * handle->width;
     uint8_t bit_position = (uint8_t)(y % 8);

     if (byte_index >= handle->buffer_size) return false; // Index out of bounds

     bool actual_state = (handle->buffer[byte_index] & (1 << bit_position)) != 0;

     if (actual_state != expected_state) {
         printf("    FAIL: Pixel (%d, %d) check failed. Expected %s, Got %s (Byte[0x%zX]=0x%02X, Bit=%d)\n",
                x, y, expected_state ? "ON" : "OFF", actual_state ? "ON" : "OFF",
                byte_index, handle->buffer[byte_index], bit_position);
         return false;
     }
     return true;
}

// Helper to print status
const char* status_to_string(status_t status) {
    // Make sure the cases match the definitions in common_types.h
    switch (status) {
        case STATUS_OK: return "STATUS_OK";
        case ERROR_NULL_POINTER: return "ERROR_NULL_POINTER";
        case ERROR_INVALID_PARAM: return "ERROR_INVALID_PARAM";
        // Remove or comment out cases not defined in common_types.h
        // case ERROR_HW_INIT_FAILED: return "ERROR_HW_INIT_FAILED";
        // case ERROR_HW_COMM_FAILED: return "ERROR_HW_COMM_FAILED";
        case ERROR_NO_MEM: return "ERROR_NO_MEM";
        case ERROR_TIMEOUT: return "ERROR_TIMEOUT";
        // Add other cases from common_types.h if they exist and are needed
        // case ERROR_NOT_IMPLEMENTED: return "ERROR_NOT_IMPLEMENTED";
        // case ERROR_NOT_INITIALIZED: return "ERROR_NOT_INITIALIZED"; // Example if it exists
        default: return "UNKNOWN_ERROR"; // Catch-all for unhandled codes
    }
}

#define CHECK(call, step_desc) \
    do { \
        printf("%d. %s...\n", step++, step_desc); \
        status = call; \
        if (status != STATUS_OK) { \
            fprintf(stderr, "   FAIL: %s failed with status %d (%s)\n", #call, status, status_to_string(status)); \
            goto cleanup; \
        } \
        printf("   PASS: %s successful.\n", #call); \
    } while (0)

#define CHECK_NO_GOTO(call, step_desc) \
    do { \
        printf("%d. %s...\n", step++, step_desc); \
        status = call; \
        if (status != STATUS_OK) { \
            fprintf(stderr, "   WARN: %s failed with status %d (%s)\n", #call, status, status_to_string(status)); \
            /* Don't goto cleanup, maybe subsequent steps can run */ \
        } else { \
            printf("   PASS: %s successful.\n", #call); \
        } \
    } while (0)

int main() {
    printf("--- SSD1306 Driver Test ---\n");
    int step = 1;
    status_t status = STATUS_OK;
    i2c_bus_handle_t i2c_bus = NULL;
    ssd1306_handle_t* oled_handle = NULL;

    // --- Define I2C configuration ---
    i2c_config_t i2c_conf = {
        .bus_num = TEST_I2C_BUS_NUM,
        .speed = TEST_I2C_SPEED
        // Add other fields if i2c_config_t has them, e.g., .pullups = true
    };

    CHECK(bcm_manager_init(), "Initializing BCM Manager");

    // --- Update the call to i2c_bus_init ---
    // Pass the address of the config structure.
    printf("%d. %s...\n", step++, "Initializing I2C Bus");
    status = i2c_bus_init(&i2c_conf, &i2c_bus); // Pass config address, handle address
    if (status != STATUS_OK) {
        fprintf(stderr, "   FAIL: i2c_bus_init failed with status %d (%s)\n", status, status_to_string(status));
        goto cleanup;
    }
     printf("   PASS: i2c_bus_init successful.\n");
    // --- End update ---

    if (i2c_bus) {
         printf("   I2C Bus Initialized (Handle: %p).\n", (void*)i2c_bus);
    }

    CHECK(ssd1306_init(i2c_bus, OLED_ADDR, OLED_WIDTH, OLED_HEIGHT, &oled_handle), "Testing ssd1306_init");
    if (oled_handle) {
        printf("      PASS: ssd1306_init returned OK (Handle: %p)\n", (void*)oled_handle);
        // Basic handle checks
        if (!oled_handle->i2c_handle) fprintf(stderr, "      FAIL: Handle i2c_handle is NULL\n"); else printf("      PASS: Handle i2c_handle OK.\n");
        if (oled_handle->i2c_addr != OLED_ADDR) fprintf(stderr, "      FAIL: Handle i2c_addr mismatch\n"); else printf("      PASS: Handle i2c_addr OK.\n");
        if (oled_handle->width != OLED_WIDTH) fprintf(stderr, "      FAIL: Handle width mismatch\n"); else printf("      PASS: Handle width OK.\n");
        if (oled_handle->height != OLED_HEIGHT) fprintf(stderr, "      FAIL: Handle height mismatch\n"); else printf("      PASS: Handle height OK.\n");
        if (oled_handle->buffer_size != (OLED_WIDTH * OLED_HEIGHT / 8)) fprintf(stderr, "      FAIL: Handle buffer_size mismatch\n"); else printf("      PASS: Handle buffer_size OK.\n");
        if (!oled_handle->buffer) fprintf(stderr, "      FAIL: Handle buffer is NULL\n"); else printf("      PASS: Handle buffer allocated.\n");
    }

    // --- Restore Original Test Pattern ---
    CHECK(ssd1306_clear_buffer(oled_handle), "Clearing buffer before drawing");

    printf("%d. Drawing test pattern...\n", step++);
    // Draw Border
    for (int16_t x = 0; x < oled_handle->width; ++x) {
        ssd1306_draw_pixel(oled_handle, x, 0, 1);                     // Top
        ssd1306_draw_pixel(oled_handle, x, oled_handle->height - 1, 1); // Bottom
    }
    for (int16_t y = 0; y < oled_handle->height; ++y) {
        ssd1306_draw_pixel(oled_handle, 0, y, 1);                     // Left
        ssd1306_draw_pixel(oled_handle, oled_handle->width - 1, y, 1); // Right
    }
    // Draw Horizontal Line
    for (int16_t x = 0; x < oled_handle->width; ++x) {
        ssd1306_draw_pixel(oled_handle, x, oled_handle->height / 2, 1);
    }
    // Draw Vertical Line
    for (int16_t y = 0; y < oled_handle->height; ++y) {
        ssd1306_draw_pixel(oled_handle, oled_handle->width / 2, y, 1);
    }
    // Draw Diagonal Line (Top-Left to Bottom-Right)
    for (int16_t i = 0; i < oled_handle->width && i < oled_handle->height; ++i) {
         ssd1306_draw_pixel(oled_handle, i, i, 1);
    }
    printf("   PASS: Test pattern drawn to buffer.\n");

    CHECK(ssd1306_display(oled_handle), "Displaying test pattern");
    printf("   Check Screen: Should show border, H/V lines, diagonal.\n");
    printf("   Pausing for 5 seconds...\n");
    sleep(5);
    // --- End of Restored Test Pattern ---


    // --- Test Cleanup ---
    printf("%d. Clearing screen before final cleanup...\n", step++);
    // Keep the final clear for a clean exit
    CHECK_NO_GOTO(ssd1306_clear_buffer(oled_handle), "Clearing buffer before cleanup");
    CHECK_NO_GOTO(ssd1306_display(oled_handle), "Displaying cleared buffer before cleanup");


cleanup:
    printf("%d. Testing ssd1306_cleanup...\n", step++);
    ssd1306_cleanup(oled_handle); // Should handle NULL gracefully
    printf("   INFO: ssd1306_cleanup called (check console for errors).\n");

    printf("%d. Cleaning up I2C Bus...\n", step++);
    i2c_bus_cleanup(i2c_bus); // Should handle NULL gracefully
    printf("   I2C Bus Cleaned Up.\n");

    printf("%d. Closing BCM Manager...\n", step++);
    bcm_manager_close();
    printf("   BCM Manager Closed.\n");

    printf("\n--- Test Summary ---\n");
    // Basic summary based on final status, could be more elaborate
    if (status == STATUS_OK) {
        printf("Overall Result: PASS (based on last required step)\n");
    } else {
        printf("Overall Result: FAIL (encountered error)\n");
    }

    return (status == STATUS_OK) ? 0 : 1;
}