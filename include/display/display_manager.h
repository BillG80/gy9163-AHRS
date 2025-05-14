#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "drivers/ssd1306/ssd1306.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCREEN_MAIN,
    SCREEN_SETTINGS,
    SCREEN_ERROR,
    // Add more screens as needed
} display_screen_t;

// Initialize display manager (pass SSD1306 handle)
void display_manager_init(ssd1306_handle_t* disp);

// Set which screen to show
void display_manager_set_screen(display_screen_t screen);

// Get the current screen
display_screen_t display_manager_get_screen(void);

// Call periodically to update the display (animations, sensor data, etc.)
void display_manager_update(uint32_t ms_since_boot, float pressure, float temperature, float altitude, float speed, float distance, float vvel, float vacc, bool is_stationary);

// Shutdown the async display thread and cleanup resources
void display_manager_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_MANAGER_H