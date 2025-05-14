#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "common/common_defs.h"

#define ICON_SIZE         48
#define ARROW_MIN_LEN     4
#define ARROW_MAX_LEN     24
#define DOT_RADIUS        2
#define TAIL_POINTS       3
#define ORBIT_DOTS        3
#define ORBIT_RADIUS      16
#define ORBIT_PERIOD_MS   4000.0f  // milliseconds per full revolution (slower)
#define PI                3.1415926f
#define MAX_VELOCITY      10.0f // Define maximum velocity

// Helper: draw a filled dot
static void draw_dot(ssd1306_handle_t* disp, int cx, int cy, int radius) {
    for (int y = -radius; y <= radius; ++y)
        for (int x = -radius; x <= radius; ++x)
            if (x*x + y*y <= radius*radius)
                ssd1306_draw_pixel(disp, cx + x, cy + y, 1);
}

// Function to draw a directional arrow with tracking dots
static void draw_directional_arrow(
    ssd1306_handle_t* disp,
    int cx, int cy,      // Center position
    float velocity,      // Velocity value (determines length and direction)
    float acceleration,  // Acceleration value (determines dot spacing)
    int y_min, int y_max // Bounds for drawing
) {
    // Local variables for arrow dimensions
    const int tri_width = 8;    // Triangle width (wider)
    const int rect_width = 4;   // Rectangle width (narrower)
    
    // Calculate arrow length based on velocity magnitude
    float abs_vel = fabsf(velocity);
    
    // Use logarithmic scaling for better visualization of low speeds
    float norm_vel;
    if (abs_vel > 0.001f) {
        // Log10 of velocity (0.001 to 1.0) gives range of -3 to 0
        // Add 3 to get range of 0 to 3, then divide by 3 to normalize to 0-1
        norm_vel = (log10f(abs_vel) + 3.0f) / 3.0f;
        if (norm_vel < 0.0f) norm_vel = 0.0f;
        if (norm_vel > 1.0f) norm_vel = 1.0f;
    } else {
        norm_vel = 0.0f;
    }
    
    // Scale arrow length from min to max based on normalized velocity
    int arrow_length = ARROW_MIN_LEN + (int)((ARROW_MAX_LEN - ARROW_MIN_LEN) * norm_vel);
    
    // Direction based on velocity sign (positive = up, negative = down)
    float dir = (velocity > 0.0f) ? 1.0f : -1.0f;
    
    // Triangle dimensions
    int tri_height = 5;
    
    // Calculate rectangle dimensions
    int rect_height = arrow_length - tri_height;
    if (rect_height < 0) rect_height = 0;  // Safety check
    
    // Draw the arrow (triangle + rectangle)
    if (dir > 0) {
        // Upward arrow (positive velocity)
        
        // First draw triangle (at the front)
        int tri_top_x = cx - tri_width / 2;
        int tri_top_y = cy - rect_height - tri_height; // Position triangle at the top
        
        // Clear the entire area where the triangle will be drawn to remove any artifacts
        for (int y = tri_top_y; y <= tri_top_y + tri_height; y++) {
            for (int x = tri_top_x; x <= tri_top_x + tri_width; x++) {
                ssd1306_draw_pixel(disp, x, y, 0);
            }
        }
        
        // Fill the triangle
        for (int y = tri_top_y; y <= tri_top_y + tri_height; y++) {
            // Calculate width at this y level (narrower at top, wider at bottom)
            float progress = (float)(y - tri_top_y) / tri_height;
            int width = (int)(progress * tri_width);
            int start_x = tri_top_x + (tri_width - width) / 2;
            
            // Draw horizontal line to fill
            ssd1306_draw_line(disp, start_x, y, start_x + width, y, 1);
        }
        
        // Then draw rectangle (body of the arrow)
        int rect_top_x = cx - rect_width / 2;
        int rect_top_y = cy - rect_height; // Position rectangle below triangle
        
        // Calculate the actual rectangle height (ensuring it doesn't overlap with triangle)
        int actual_rect_height = rect_height;
        
        // Draw rectangle
        ssd1306_fill_rect(disp, rect_top_x, rect_top_y, rect_width, actual_rect_height, 1);
    } else {
        // Downward arrow (negative velocity)
        
        // First draw rectangle (body of the arrow)
        int rect_top_x = cx - rect_width / 2;
        int rect_top_y = cy ; // Position rectangle above the triangle
        
        // Calculate the actual rectangle height (ensuring it doesn't overlap with triangle)
        int actual_rect_height = rect_height;
        
        // Draw rectangle
        ssd1306_fill_rect(disp, rect_top_x, rect_top_y, rect_width, actual_rect_height, 1);
        
        // Then draw triangle (at the front)
        int tri_top_x = cx - tri_width / 2;
        int tri_top_y = cy + rect_height; // Position triangle at the bottom
        
        // Clear the entire area where the triangle will be drawn to remove any artifacts
        for (int y = tri_top_y; y <= tri_top_y + tri_height; y++) {
            for (int x = tri_top_x; x <= tri_top_x + tri_width; x++) {
                ssd1306_draw_pixel(disp, x, y, 0);
            }
        }
        
        // Fill the triangle
        for (int y = tri_top_y; y <= tri_top_y + tri_height; y++) {
            // Calculate width at this y level (wider at bottom, narrower at top)
            float progress = (float)(y - tri_top_y) / tri_height;
            int width = (int)((1.0f - progress) * tri_width);
            int start_x = tri_top_x + (tri_width - width) / 2;
            
            // Draw horizontal line to fill
            ssd1306_draw_line(disp, start_x, y, start_x + width, y, 1);
        }
    }
    
    // No tracking dots when truly stationary
    float abs_acc = fabsf(acceleration);
    if (abs_vel < 0.01f && abs_acc < 0.01f) {
        return;  // Return early - no tracking dots for stationary state
    }
    
    // Tail points with standard distance during constant velocity
    float min_spacing = 1.0f;       // Minimum spacing when nearly stopped
    float standard_spacing = 4.0f;  // Standard spacing during constant velocity
    float spacing;
    
    // Joint formula for spacing that combines velocity and acceleration
    
    // Use logarithmic scaling for acceleration to make small values more significant
    float log_acc;
    if (abs_acc < 0.01f) {
        log_acc = 0.0f;
    } else {
        float log_acc_magnitude = (log10f(abs_acc) + 2.0f) / (0.021f + 2.0f);
        if (log_acc_magnitude > 1.0f) log_acc_magnitude = 1.0f;

        if (dir * acceleration < 0.0f) {
            log_acc = log_acc_magnitude;
        } else {
            log_acc = -log_acc_magnitude;
        }
    }
    
    // Calculate base spacing from velocity
    // - At zero velocity: minimum spacing
    // - At normal velocity: standard spacing
    float base_spacing = (standard_spacing - min_spacing) * norm_vel;
    
    // Apply acceleration effect - ensure consistent behavior for both directions
    if (dir * acceleration < 0.0f) {  // Acceleration is increasing speed (opposite sign to velocity)
        // Positive effective acceleration: increase spacing above base
        // Higher acceleration = wider spacing
        spacing = base_spacing + (standard_spacing - min_spacing) * log_acc;
    } else if (dir * acceleration > 0.0f) {  // Acceleration is decreasing speed (same sign as velocity)
        // Negative effective acceleration: decrease spacing below base
        // Higher deceleration = closer spacing
        // But never go below minimum
        spacing = base_spacing + (standard_spacing - min_spacing) * log_acc;
        
        // When velocity is very low during deceleration, make dots merge with arrow
        if (abs_vel < 0.1f) {
            // Scale spacing down as velocity approaches zero
            float merge_factor = abs_vel / 0.1f;
            spacing = spacing * merge_factor;
            if (spacing < 0.5f) spacing = 0.5f; // Minimum spacing during merge
        }
    } else {
        // Zero acceleration: use base spacing
        spacing = base_spacing;
    }
    
    // Determine number of visible points
    int visible_points = TAIL_POINTS;  // Always show all tail points for consistency
    if (abs_vel < 0.01f && abs_acc < 0.01f) {
        visible_points = 0;  // No dots when truly stationary
    }
    
    // py is the starting position for dots
    float py = cy;
    
    // Calculate merge factor for low velocity (clamped between 0.2 and 1.0)
    float merge_factor = 1.0f;
    if (abs_vel < 0.1f) {
        merge_factor = abs_vel / 0.1f;
        if (merge_factor < 0.2f) merge_factor = 0.2f; // Minimum merge factor
    }
    
    // Position the first dot at a consistent distance from the arrow body
    if (dir > 0) {
        // Upward arrow (positive velocity), dots below
        py = cy + spacing; // Start from bottom of rectangle
    } else {
        // Downward arrow (negative velocity), dots above
        py = cy - spacing; // Start from top of rectangle
    }
    
    // Draw the visible tail points
    for (int i = 0; i < visible_points; ++i) {
        // Calculate position for this dot
        float dot_spacing = spacing * merge_factor;
        float dot_py;
        
        if (dir > 0) {
            // Upward arrow (positive velocity), dots below
            dot_py = py + i * dot_spacing;
        } else {
            // Downward arrow (negative velocity), dots above
            dot_py = py - i * dot_spacing;
        }
        
        // Check if dot is within bounds
        if (dot_py < y_min + DOT_RADIUS || dot_py > y_max - DOT_RADIUS) break;
        
        // Draw the dot
        draw_dot(disp, cx, (int)dot_py, 1);
    }
}

// Main Vacc/motion icon animation
void draw_vacc_icon(
    ssd1306_handle_t* disp,
    int x, int y,
    DisplayStatus ds,
    float vvel,
    float vacc,
    uint32_t anim_time_ms
) {
    int cx = x + ICON_SIZE / 2;
    int cy;
    
    // Position the arrow based on direction
    if (ds == DISP_STATE_MOVING_UP) {
        // Upward arrow - position near top with enough space for the entire arrow
        cy = y + ARROW_MAX_LEN + 5; // Ensure the entire arrow (including triangle) stays within bounds
    } else if (ds == DISP_STATE_MOVING_DOWN) {
        // Downward arrow - position near bottom with enough space for the entire arrow
        cy = y + ICON_SIZE - ARROW_MAX_LEN - 5; // Position in bottom half with safety margin
    } else {
        // No velocity - center position
        cy = y + ICON_SIZE / 2;
    }

    if (ds == DISP_STATE_STATIONARY) {
        // Draw stationary dot - always centered
        int center_y = y + ICON_SIZE / 2;
        draw_dot(disp, cx, center_y, DOT_RADIUS);

        // Animate orbiting points around center
        for (int i = 0; i < ORBIT_DOTS; ++i) {
            float phase = 2 * PI * i / ORBIT_DOTS;
            // continuous rotation: full 2π rad per ORBIT_PERIOD_MS
            float anim_phase = 2.0f * PI * ((float)anim_time_ms / ORBIT_PERIOD_MS);
            float angle = phase + anim_phase;
            int px = (int)(cx + ORBIT_RADIUS * cosf(angle));
            int py = (int)(center_y + ORBIT_RADIUS * sinf(angle));
            // Clamp to region
            if (px < x) px = x;
            if (px >= x + ICON_SIZE) px = x + ICON_SIZE - 1;
            if (py < y) py = y;
            if (py >= y + ICON_SIZE) py = y + ICON_SIZE - 1;
            ssd1306_draw_pixel(disp, px, py, 1);
        }
    } else {
        // position arrow so it never clips
        cy = (ds==DISP_STATE_MOVING_UP)
            ? y + ARROW_MAX_LEN/2 +5
            : y + ICON_SIZE - ARROW_MAX_LEN/2 -5;
        draw_directional_arrow (disp, cx, cy, vvel, vacc, y, y+ICON_SIZE);
    }
}

// Optional: static rotation for stationary state (if used elsewhere)
void draw_static_rotation(ssd1306_handle_t* disp, int icon_x, int icon_y) {
    static float angle = 0.0f;
    angle += 0.02f;
    if (angle > 2 * PI) angle -= 2 * PI;

    int center_x = icon_x + ICON_SIZE / 2;
    int center_y = icon_y + ICON_SIZE / 2;
    for (int i = 0; i < 3; i++) {
        float particle_angle = angle + i * 2.094f;
        int px = center_x + 8 * cosf(particle_angle);
        int py = center_y + 8 * sinf(particle_angle);
        px = (px < icon_x) ? icon_x : (px >= icon_x + ICON_SIZE) ? icon_x + ICON_SIZE - 1 : px;
        py = (py < icon_y) ? icon_y : (py >= icon_y + ICON_SIZE) ? icon_y + ICON_SIZE - 1 : py;
        ssd1306_draw_pixel(disp, px, py, 1);
    }
}

// Draw right-side text data (altitude, speed, distance)
void draw_text_data(ssd1306_handle_t* disp, int x, int y, float altitude, float speed, float distance) {
    char buf[32];
    int line_height = 16; // Or use your font's height + spacing

    snprintf(buf, sizeof(buf), "ALT:%.1fm", altitude);
    ssd1306_draw_text(disp, x, y, buf, 1);

    snprintf(buf, sizeof(buf), "SPD:%.1fm/s", speed);
    ssd1306_draw_text(disp, x, y + line_height, buf, 1);

    snprintf(buf, sizeof(buf), "DST:%.1fm", distance);
    ssd1306_draw_text(disp, x, y + 2 * line_height, buf, 1);
}