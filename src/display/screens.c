#include "display/screens.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "drivers/ssd1306/ssd1306.h"
#include "display/icon_animation.h"
#include "fonts/SansCondensed.h"
#include "common/common_defs.h"
#include "display/screens.h"
#include <stdio.h>

// Draws the calibration confirmation screen
void draw_calib_confirm_screen(
    ssd1306_handle_t* disp,
    int countdown_s
) {
    // Clear display
    ssd1306_fill_rect(disp, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 0);

    // --- Yellow zone ---
    const char* yellow_msg = "IMU Calibrate?";
    int y0 = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    int x_center = (SSD1306_WIDTH - ssd1306_get_text_width(yellow_msg, 1)) / 2;
    ssd1306_draw_text(disp, x_center, y0, yellow_msg, 1);

    // --- Blue zone ---
    int blue_y1 = SSD1306_YELLOW_HEIGHT + 10;
    const char* blue_msg1 = "Click button = Y";
    int blue_x1 = (SSD1306_WIDTH - ssd1306_get_text_width(blue_msg1, 1)) / 2;
    ssd1306_draw_text(disp, blue_x1, blue_y1, blue_msg1, 1);

    char blue_msg2[32];
    snprintf(blue_msg2, sizeof(blue_msg2), "%ds to system", countdown_s);
    int blue_y2 = blue_y1 + SANS_FONT_HEIGHT + 8;
    int blue_x2 = (SSD1306_WIDTH - ssd1306_get_text_width(blue_msg2, 1)) / 2;
    ssd1306_draw_text(disp, blue_x2, blue_y2, blue_msg2, 1);

    ssd1306_update(disp);
}

// Draws the main screen UI
void draw_main_screen(
    ssd1306_handle_t* disp,
    uint32_t anim_time_ms,
    float pressure,      // hPa or Pa
    float temperature,   // deg C
    float altitude,      // meters
    float speed,         // m/s
    float distance,      // meters
    DisplayStatus display_state,  // UI motion state
    float vacc,          // vertical acceleration (m/s^2)
    bool is_stationary   // stationary flag
) {
    // Clear full display buffer
    ssd1306_fill_rect(disp, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 0);

    // --- Yellow zone: Pressure and Temperature (single line) ---
    char buf[32];
    int y0 = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    snprintf(buf, sizeof(buf), "P:%.2fhPa T:%.1f\xB0""C", pressure, temperature);
    ssd1306_draw_text(disp, 2, y0, buf, 1);

    // --- Blue zone: Vacc/motion icon (left 48x48) ---
    // Freeze vertical metrics when stationary
    static float last_distance_val = 0.0f;
    float disp_speed = is_stationary ? 0.0f : speed;
    float disp_distance;
    if (is_stationary) {
        disp_distance = last_distance_val;
    } else {
        disp_distance = distance;
        last_distance_val = distance;
    }
    draw_vacc_icon(disp, ICON_X, ICON_Y, display_state, disp_speed, vacc, anim_time_ms);

    // --- Blue zone: Text info (right 80x48) ---
    int text_zone_x = ICON_X + ICON_SIZE + 2;
    int text_zone_y = ICON_Y;
    int line_spacing = SANS_FONT_HEIGHT + 2;

    // "AL" Altitude
    snprintf(buf, sizeof(buf), "AL %.1fm", altitude);
    ssd1306_draw_text(disp, text_zone_x + 2, text_zone_y + 2, buf, 1);

    // "S" Speed
    snprintf(buf, sizeof(buf), "S  %.2fm/s", disp_speed);
    ssd1306_draw_text(disp, text_zone_x + 2, text_zone_y + 2 + line_spacing, buf, 1);

    // "D" Distance
    snprintf(buf, sizeof(buf), "D  %.2fm", disp_distance);
    ssd1306_draw_text(disp, text_zone_x + 2, text_zone_y + 2 + 2 * line_spacing, buf, 1);
}

// Function to display icon animation with detailed text labels for testing
void draw_test_screen(
    ssd1306_handle_t* disp,
    DisplayStatus display_state,
    uint32_t anim_time_ms,
    float vvel,          // vertical velocity (m/s)
    float vacc,          // vertical acceleration (m/s^2)
    bool is_stationary,  // stationary flag
    const char* title    // title text
) {
    // Clear display buffer
    ssd1306_fill_rect(disp, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 0);
    
    // Draw a border around the icon area
    ssd1306_fill_rect(disp, ICON_X - 1, ICON_Y - 1, ICON_SIZE + 2, ICON_SIZE + 2, 1);
    ssd1306_fill_rect(disp, ICON_X, ICON_Y, ICON_SIZE, ICON_SIZE, 0);
    
    // Draw the icon with the passed-in display_state
    draw_vacc_icon(disp, ICON_X, ICON_Y, display_state, vvel, vacc, anim_time_ms);
    
    // Draw text information
    char buf[32];
    int text_zone_x = ICON_X + ICON_SIZE + 5;
    int text_zone_y = ICON_Y;
    int line_spacing = SANS_FONT_HEIGHT + 2;
    
    // Title
    ssd1306_draw_text(disp, text_zone_x, text_zone_y, title, 1);
    
    // Velocity
    snprintf(buf, sizeof(buf), "Vel: %.2f m/s", vvel);
    ssd1306_draw_text(disp, text_zone_x, text_zone_y + line_spacing, buf, 1);
    
    // Acceleration
    snprintf(buf, sizeof(buf), "Acc: %.2f m/s^2", vacc);
    ssd1306_draw_text(disp, text_zone_x, text_zone_y + 2 * line_spacing, buf, 1);
    
    // Display direction text
    const char* direction;
    if (is_stationary) {
        direction = "Stationary";
    } else if (vvel > 0.01f) direction = "Moving Up";
    else if (vvel < -0.01f) direction = "Moving Down";
    else direction = "No Movement";
    ssd1306_draw_text(disp, text_zone_x, text_zone_y + 3 * line_spacing, direction, 1);
}

// Settings screen implementation
void draw_settings_screen(ssd1306_handle_t* disp, uint32_t anim_time_ms) {
    // Clear display buffer
    ssd1306_fill_rect(disp, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 0);
    // Display a settings title
    ssd1306_draw_text(disp, 2, 2, "Settings", 1);
}

// Error screen implementation
void draw_error_screen(ssd1306_handle_t* disp, uint32_t anim_time_ms) {
    // Clear display buffer
    ssd1306_fill_rect(disp, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 0);
    // Display an error message
    ssd1306_draw_text(disp, 2, 2, "Error", 1);
}