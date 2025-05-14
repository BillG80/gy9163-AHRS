#include <unistd.h>
#include <bcm2835.h>
#include <stdio.h>
#include "drivers/bus/i2c_bus.h"
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "display/screens.h"

int main(void) {
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
    
    // Initialize display
    ssd1306_handle_t* disp = NULL;
    if (ssd1306_init(bus, 0x3C, 128, 64, &disp) != STATUS_OK) {
        fprintf(stderr, "SSD1306 init failed\n");
        i2c_bus_cleanup(bus);
        bcm2835_close();
        return 1;
    }
    
    printf("Display test starting...\n");
    
    // Clear display
    ssd1306_clear_buffer(disp);
    ssd1306_update(disp);
    sleep(1);
    
    // 1) Test upward arrow with full screen layout
    printf("Testing upward arrow with full screen layout...\n");
    draw_main_screen(
        disp,
        0,              // anim_time_ms
        1013.25f,       // pressure (hPa)
        25.5f,          // temperature (°C)
        100.0f,         // altitude (m)
        1.5f,           // speed (m/s)
        50.0f,          // distance (m)
        0.8f,           // vvel (positive = upward)
        0.2f,           // vacc
        false           // is_stationary
    );
    ssd1306_display(disp);
    sleep(3);
    
    // 2) Test downward arrow with full screen layout
    printf("Testing downward arrow with full screen layout...\n");
    draw_main_screen(
        disp,
        0,              // anim_time_ms
        1012.75f,       // pressure (hPa)
        25.3f,          // temperature (°C)
        95.0f,          // altitude (m)
        1.3f,           // speed (m/s)
        55.0f,          // distance (m)
        -0.8f,          // vvel (negative = downward)
        -0.2f,           // vacc
        false           // is_stationary
    );
    ssd1306_display(disp);
    sleep(3);
    
    // 3) Test stationary orbit with full screen layout
    printf("Testing stationary orbit animation with full screen layout...\n");
    for (int t = 0; t < 3000; t += 100) {  // 3 seconds
        draw_main_screen(
            disp,
            t,              // anim_time_ms
            1013.0f,        // pressure (hPa)
            25.4f,          // temperature (°C)
            100.0f,         // altitude (m)
            0.0f,           // speed (m/s)
            60.0f,          // distance (m)
            0.0f,           // vvel
            0.0f,           // vacc
            true            // is_stationary
        );
        ssd1306_display(disp);
        usleep(100000);  // 100ms delay
    }
    
    // 4) Test accelerating upward motion with full screen layout
    printf("Testing accelerating upward motion with full screen layout...\n");
    for (int t = 0; t < 3000; t += 100) {  // 3 seconds
        float vel = 0.2f + (t / 3000.0f) * 0.8f;  // Velocity increases from 0.2 to 1.0
        float acc = 0.3f;  // Constant acceleration
        float altitude = 100.0f + (vel * t / 1000.0f);  // Simple altitude calculation
        
        draw_main_screen(
            disp,
            t,                // anim_time_ms
            1013.0f - (altitude-100.0f)*0.01f,  // pressure decreases with altitude
            25.4f,            // temperature (°C)
            altitude,         // altitude (m)
            1.5f,             // speed (m/s)
            60.0f + (t/3000.0f)*10.0f,  // distance increases over time
            vel,              // vvel
            acc,              // vacc
            false             // is_stationary
        );
        ssd1306_display(disp);
        usleep(100000);  // 100ms delay
    }
    
    // 5) Test decelerating downward motion with full screen layout
    printf("Testing decelerating downward motion with full screen layout...\n");
    for (int t = 0; t < 3000; t += 100) {  // 3 seconds
        float vel = -0.8f + (t / 3000.0f) * 0.6f;  // Velocity increases from -0.8 to -0.2
        float acc = 0.2f;  // Positive acceleration (slowing down)
        float altitude = 100.0f + (vel * t / 1000.0f);  // Simple altitude calculation
        
        draw_main_screen(
            disp,
            t,                // anim_time_ms
            1013.0f - (altitude-100.0f)*0.01f,  // pressure decreases with altitude
            25.4f,            // temperature (°C)
            altitude,         // altitude (m)
            1.5f,             // speed (m/s)
            60.0f + (t/3000.0f)*10.0f,  // distance increases over time
            vel,              // vvel
            acc,              // vacc
            false             // is_stationary
        );
        ssd1306_display(disp);
        usleep(100000);  // 100ms delay
    }
    
    // 6) Test low velocity with high acceleration with full screen layout
    printf("Testing low velocity with high acceleration with full screen layout...\n");
    draw_main_screen(
        disp,
        0,              // anim_time_ms
        1013.0f,        // pressure (hPa)
        25.4f,          // temperature (°C)
        100.0f,         // altitude (m)
        1.5f,           // speed (m/s)
        60.0f,          // distance (m)
        0.05f,          // vvel (low velocity)
        0.8f,           // vacc (high acceleration)
        false           // is_stationary
    );
    ssd1306_display(disp);
    sleep(3);
    
    // 7) Test high velocity with low acceleration with full screen layout
    printf("Testing high velocity with low acceleration with full screen layout...\n");
    draw_main_screen(
        disp,
        0,              // anim_time_ms
        1013.0f,        // pressure (hPa)
        25.4f,          // temperature (°C)
        100.0f,         // altitude (m)
        1.5f,           // speed (m/s)
        60.0f,          // distance (m)
        2.0f,           // vvel (high velocity)
        0.05f,          // vacc (low acceleration)
        false           // is_stationary
    );
    ssd1306_display(disp);
    sleep(3);
    
    // 8) Test zero velocity with zero acceleration with full screen layout
    printf("Testing zero velocity with zero acceleration with full screen layout...\n");
    draw_main_screen(
        disp,
        0,              // anim_time_ms
        1013.0f,        // pressure (hPa)
        25.4f,          // temperature (°C)
        100.0f,         // altitude (m)
        0.0f,           // speed (m/s)
        60.0f,          // distance (m)
        0.0f,           // vvel
        0.0f,           // vacc
        false           // is_stationary (not marked as stationary to test edge case)
    );
    ssd1306_display(disp);
    sleep(3);
    
    // Clean up
    printf("Test complete, cleaning up...\n");
    ssd1306_cleanup(disp);
    i2c_bus_cleanup(bus);
    bcm2835_close();
    
    return 0;
}