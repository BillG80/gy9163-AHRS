#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <bcm2835.h>

#include "drivers/bus/i2c_bus.h"
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "display/icon_animation.h"
#include "fonts/SansCondensed.h"

#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64

int main(int argc, char *argv[]) {
    // Initialize BCM2835 library
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835_init failed\n");
        return 1;
    }
    
    // Initialize I2C bus
    i2c_bus_handle_t bus = NULL;
    i2c_config_t cfg = { .bus_num = 0, .speed = I2C_SPEED_400KHZ };
    if (i2c_bus_init(&cfg, &bus) != STATUS_OK) {
        fprintf(stderr, "I2C init failed\n");
        bcm2835_close();
        return 1;
    }

    // Init display
    ssd1306_handle_t* disp = NULL;
    if (ssd1306_init(bus, 0x3C, 128, 64, &disp) != STATUS_OK) {
        fprintf(stderr, "ssd1306_init failed\n");
        i2c_bus_cleanup(bus);
        bcm2835_close();
        return 1;
    }
    
    // Fixed parameters
    float upward_velocity = 2.0f;      // Positive = upward
    float upward_acceleration = 0.0f;
    float downward_velocity = -2.0f;   // Negative = downward
    float downward_acceleration = 0.0f;
    int arrow_y = 10;                  // Y position
    
    printf("Test Arrow and Tracking Dots\n");
    printf("----------------------------\n");
    printf("Press Ctrl+C to exit\n\n");
    
    // Main loop
    while (1) {
        // Clear display
        ssd1306_fill_rect(disp, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, 0);
        
        // Draw upward arrow (left side)
        draw_vacc_icon(disp, 20, arrow_y, 
                      upward_velocity, upward_acceleration, 
                      false, 0);
        
        // Draw downward arrow (right side)
        draw_vacc_icon(disp, 80, arrow_y, 
                      downward_velocity, downward_acceleration, 
                      false, 0);
        
        // Draw labels at bottom corners
        ssd1306_draw_text(disp, 10, DISPLAY_HEIGHT - 12, "UP", 1);
        ssd1306_draw_text(disp, DISPLAY_WIDTH - 30, DISPLAY_HEIGHT - 12, "DOWN", 1);
        
        // Update display
        ssd1306_display(disp);
        
        // Sleep to avoid hammering the CPU
        sleep(1);
    }
    
    // Cleanup (never reached in this example)
    ssd1306_cleanup(disp);
    i2c_bus_cleanup(bus);
    bcm2835_close();
    return 0;
}