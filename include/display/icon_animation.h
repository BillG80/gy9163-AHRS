#ifndef ICON_ANIMATION_H
#define ICON_ANIMATION_H

#include <stdint.h>
#include <stdbool.h>
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "common/common_defs.h"

// --- Constants for layout (customize as needed) ---
#define ICON_SIZE         48
#define DOT_RADIUS        2
#define TAIL_POINTS       6
#define ORBIT_RADIUS      16

// --- Animation & UI helpers ---

/**
 * Draws the main Vacc/motion icon at (x, y) in a 48x48 region.
 * @param disp      SSD1306 display handle.
 * @param x, y      Top-left corner of the icon region.
 * @param vvel      Vertical velocity.
 * @param vacc      Vertical acceleration.
 * @param is_stationary True if stationary, false if moving.
 * @param anim_time_ms  Animation time in milliseconds.
 */
void draw_vacc_icon(
    ssd1306_handle_t* disp,
    int x, int y,
    DisplayStatus ds,
    float vvel,
    float vacc,            // vertical acceleration for dot spacing
    uint32_t anim_time_ms  // animation time in ms
);

/**
 * Draws a static rotation animation (optional, for stationary state).
 * @param disp      SSD1306 display handle.
 * @param icon_x, icon_y  Top-left of the icon region.
 */
void draw_static_rotation(ssd1306_handle_t* disp, int icon_x, int icon_y);

/**
 * Draws sensor data as text (altitude, speed, distance) in a vertical stack.
 * @param disp      SSD1306 display handle.
 * @param x, y      Top-left for the first text line.
 * @param altitude  Altitude value.
 * @param speed     Speed value.
 * @param distance  Distance value.
 */
void draw_text_data(
    ssd1306_handle_t* disp,
    int x, int y,
    float altitude, float speed, float distance
);

#endif // ICON_ANIMATION_H