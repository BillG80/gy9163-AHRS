#include <stdio.h>
#include <unistd.h>
#include <bcm2835.h>
#include "drivers/bus/i2c_bus.h"
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "display/icon_animation.h"
#include "fonts/SansCondensed.h"

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

    // Init display
    ssd1306_handle_t* disp = NULL;
    if (ssd1306_init(bus, 0x3C, 128, 64, &disp) != STATUS_OK) {
        fprintf(stderr, "ssd1306_init failed\n");
        i2c_bus_cleanup(bus);
        bcm2835_close();
        return 1;
    }

    // 3) Clear screen
    ssd1306_fill_rect(disp, 0, 0, 128, 64, 0);

    // Define icon position at left bottom corner
    int icon_x = 0;
    int icon_y = 64 - ICON_SIZE;
    
    // Function to draw the blue zone background and prepare display
    void draw_blue_zone_and_clear(ssd1306_handle_t* disp, int icon_x, int icon_y) {
        // Clear the entire display
        ssd1306_fill_rect(disp, 0, 0, 128, 64, 0);
        
        // Draw blue zone (inverted rectangle) for the icon area
        ssd1306_fill_rect(disp, icon_x, icon_y, ICON_SIZE, ICON_SIZE, 1);
        
        // Clear the icon area within the blue zone to prepare for icon drawing
        // This creates a border effect
        ssd1306_fill_rect(disp, icon_x + 1, icon_y + 1, ICON_SIZE - 2, ICON_SIZE - 2, 0);
    }

    // 4) Draw a static arrow (moving upward) and show it
    draw_blue_zone_and_clear(disp, icon_x, icon_y);
    draw_vacc_icon(disp, icon_x + 1, icon_y + 1,
                   0.8f,    // vvel (positive = upward arrow)
                   0.0f,    // vacc
                   false,   // is_stationary
                   0);      // anim_time_ms
    ssd1306_display(disp);
    sleep(2);
    
    // 5) Draw a static arrow (moving downward) and show it
    draw_blue_zone_and_clear(disp, icon_x, icon_y);
    draw_vacc_icon(disp, icon_x + 1, icon_y + 1,
                   -0.8f,   // vvel (negative = downward arrow)
                   0.0f,    // vacc
                   false,   // is_stationary
                   0);      // anim_time_ms
    ssd1306_display(disp);
    sleep(2);

    // 6) Animate stationary orbit for 3 seconds
    printf("Stationary orbit animation (3 seconds)...\n");
    for (int t = 0; t < 3000; t += 100) {  // 3000ms = 3 seconds
        draw_blue_zone_and_clear(disp, icon_x, icon_y);
        draw_vacc_icon(disp, icon_x + 1, icon_y + 1,
                       0.0f,    // vvel
                       0.0f,    // vacc
                       true,    // is_stationary
                       t);
        ssd1306_display(disp);
        usleep(100000);  // 100ms delay between frames
    }

    // 7) Simulate complete elevator motion cycle
    printf("Starting complete elevator motion simulation...\n");
    
    // Parameters for simulation
    float velocity = 0.0f;
    float max_velocity = 1.0f;    // Maximum velocity of 1.0 m/s
    float acceleration = 0.0f;
    float accel_rate = 0.5f;      // Acceleration of 0.5 m/s² (reaches 1 m/s in 2 seconds)
    float decel_rate = -0.5f;     // Deceleration rate matching acceleration
    int delay_ms = 100;           // Delay between frames (ms)
    int frames_to_max_speed = 20; // 20 frames * 100ms = 2 seconds
    
    // Stage 0: Start from stationary (circle)
    draw_blue_zone_and_clear(disp, icon_x, icon_y);
    draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, true, 0);
    ssd1306_display(disp);
    printf("Stationary\n");
    usleep(1000000);  // 1 second pause
    
    // Stage 1: Upward acceleration (0 to 1 m/s in 2s)
    printf("Stage 1: Upward acceleration (0 to 1 m/s)...\n");
    acceleration = accel_rate;
    for (int i = 0; i < frames_to_max_speed; i++) {
        // Update velocity based on acceleration
        velocity += acceleration * (delay_ms / 1000.0f);
        
        // Cap at max velocity
        if (velocity > max_velocity) {
            velocity = max_velocity;
        }
        
        // Display current state
        draw_blue_zone_and_clear(disp, icon_x, icon_y);
        draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
        ssd1306_display(disp);
        
        // Print current values
        printf("V: %.2f, A: %.2f\n", velocity, acceleration);
        
        // Delay
        usleep(delay_ms * 1000);
    }
    
    // Transition to constant velocity (gradually reduce acceleration to zero)
    printf("Transition to constant velocity...\n");
    for (int i = 0; i < 10; i++) {
        // Gradually reduce acceleration to zero
        acceleration = accel_rate * (1.0f - (i / 10.0f));
        
        // Display current state
        draw_blue_zone_and_clear(disp, icon_x, icon_y);
        draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
        ssd1306_display(disp);
        
        // Print current values
        printf("V: %.2f, A: %.2f\n", velocity, acceleration);
        
        // Delay
        usleep(delay_ms * 1000);
    }
    
    // Brief constant velocity period
    acceleration = 0.0f;
    draw_blue_zone_and_clear(disp, icon_x, icon_y);
    draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
    ssd1306_display(disp);
    usleep(500000);  // 0.5 second pause
    
    // Stage 2: Upward deceleration (1 m/s to 0 in 2s)
    printf("Stage 2: Upward deceleration (1 to 0 m/s)...\n");
    acceleration = decel_rate;
    for (int i = 0; i < frames_to_max_speed; i++) {
        // Update velocity based on acceleration
        velocity += acceleration * (delay_ms / 1000.0f);
        
        // Stop at zero velocity
        if (velocity <= 0.0f) {
            velocity = 0.0f;
            acceleration = 0.0f;
            break;
        }
        
        // Display current state
        draw_blue_zone_and_clear(disp, icon_x, icon_y);
        draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
        ssd1306_display(disp);
        
        // Print current values
        printf("V: %.2f, A: %.2f\n", velocity, acceleration);
        
        // Delay
        usleep(delay_ms * 1000);
    }
    
    // Brief stationary period
    printf("Brief stop...\n");
    draw_blue_zone_and_clear(disp, icon_x, icon_y);
    draw_vacc_icon(disp, icon_x + 1, icon_y + 1, 0.0f, 0.0f, true, 0);
    ssd1306_display(disp);
    usleep(1000000);  // 1 second pause
    
    // Stage 3: Downward acceleration (0 to -1 m/s in 2s)
    printf("Stage 3: Downward acceleration (0 to -1 m/s)...\n");
    acceleration = -accel_rate;  // Negative acceleration for downward motion
    for (int i = 0; i < frames_to_max_speed; i++) {
        // Update velocity based on acceleration
        velocity += acceleration * (delay_ms / 1000.0f);
        
        // Cap at max negative velocity
        if (velocity < -max_velocity) {
            velocity = -max_velocity;
        }
        
        // Display current state
        draw_blue_zone_and_clear(disp, icon_x, icon_y);
        draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
        ssd1306_display(disp);
        
        // Print current values
        printf("V: %.2f, A: %.2f\n", velocity, acceleration);
        
        // Delay
        usleep(delay_ms * 1000);
    }
    
    // Transition to constant velocity (gradually reduce acceleration to zero)
    printf("Transition to constant velocity...\n");
    for (int i = 0; i < 10; i++) {
        // Gradually reduce acceleration to zero
        acceleration = -accel_rate * (1.0f - (i / 10.0f));
        
        // Display current state
        draw_blue_zone_and_clear(disp, icon_x, icon_y);
        draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
        ssd1306_display(disp);
        
        // Print current values
        printf("V: %.2f, A: %.2f\n", velocity, acceleration);
        
        // Delay
        usleep(delay_ms * 1000);
    }
    
    // Brief constant velocity period
    acceleration = 0.0f;
    draw_blue_zone_and_clear(disp, icon_x, icon_y);
    draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
    ssd1306_display(disp);
    usleep(500000);  // 0.5 second pause
    
    // Stage 4: Downward deceleration (-1 m/s to 0 in 2s)
    printf("Stage 4: Downward deceleration (-1 to 0 m/s)...\n");
    acceleration = -decel_rate;  // Positive acceleration to slow down negative velocity
    for (int i = 0; i < frames_to_max_speed; i++) {
        // Update velocity based on acceleration
        velocity += acceleration * (delay_ms / 1000.0f);
        
        // Stop at zero velocity
        if (velocity >= 0.0f) {
            velocity = 0.0f;
            acceleration = 0.0f;
            break;
        }
        
        // Display current state
        draw_blue_zone_and_clear(disp, icon_x, icon_y);
        draw_vacc_icon(disp, icon_x + 1, icon_y + 1, velocity, acceleration, false, 0);
        ssd1306_display(disp);
        
        // Print current values
        printf("V: %.2f, A: %.2f\n", velocity, acceleration);
        
        // Delay
        usleep(delay_ms * 1000);
    }
    
    // Final stationary state
    printf("Back to stationary\n");
    draw_blue_zone_and_clear(disp, icon_x, icon_y);
    draw_vacc_icon(disp, icon_x + 1, icon_y + 1, 0.0f, 0.0f, true, 0);
    ssd1306_display(disp);
    usleep(1000000);  // 1 second pause

    // 8) Cleanup
    ssd1306_cleanup(disp);
    i2c_bus_cleanup(bus);
    bcm2835_close();
    return 0;
}
