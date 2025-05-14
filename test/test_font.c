#include <stdio.h>
#include <unistd.h>           // for sleep()
#include <string.h>           // for strlen()
#include <bcm2835.h>
#include "drivers/bus/i2c_bus.h"
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "fonts/SansCondensed.h"

int main(void) {
    // Initialize BCM2835 library
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835_init failed\n");
        return 1;
    }
    
    // Initialize I2C bus
    i2c_config_t cfg = { .bus_num = 0, .speed = I2C_SPEED_400KHZ };
    i2c_bus_handle_t bus = NULL;
    if (i2c_bus_init(&cfg, &bus) != STATUS_OK) {
        fprintf(stderr, "i2c_bus_init failed\n");
        bcm2835_close();
        return 1;
    }
    
    // Init display
    ssd1306_handle_t* disp = NULL;
    if (ssd1306_init(bus, 0x3C, 128, 64, &disp) != STATUS_OK) {
        fprintf(stderr, "ssd1306_init failed\n");
        bcm2835_close();
        return 1;
    }
    
    // Clear the display
    ssd1306_clear_buffer(disp);
    ssd1306_update(disp);
    sleep(1);  // Wait for the display to clear
    
    // Draw a simple border
    for (int x = 0; x < 128; x++) {
        ssd1306_draw_pixel(disp, x, 0, 1);      // Top edge
        ssd1306_draw_pixel(disp, x, 63, 1);     // Bottom edge
    }
    for (int y = 0; y < 64; y++) {
        ssd1306_draw_pixel(disp, 0, y, 1);      // Left edge
        ssd1306_draw_pixel(disp, 127, y, 1);    // Right edge
    }
    
    // Clear the center area to ensure no abnormal areas
    for (int x = 1; x < 127; x++) {
        for (int y = 1; y < 63; y++) {
            ssd1306_draw_pixel(disp, x, y, 0);
        }
    }
    
    // Test for character duplication
    printf("Testing for character duplication...\n");
    ssd1306_draw_text(disp, 10, 15, "LOVEvADDIE", 1);
    ssd1306_draw_text(disp, 10, 30, "1234567890",1);
    ssd1306_draw_text(disp, 10, 45, "Test_Temp:""25.5\xB0""C", 1);
    
    ssd1306_update(disp);
    sleep(10);  // Display for 10 seconds
    
    // Clean up
    ssd1306_cleanup(disp);
    i2c_bus_cleanup(bus);
    bcm2835_close();
    
    return 0;
}