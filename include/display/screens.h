#ifndef SCREENS_H
#define SCREENS_H

#include "drivers/ssd1306/ssd1306.h"
#include "display/icon_animation.h"
#include "core/navigation/elevator_nav.h"
#include "common/common_defs.h"
#include <stdbool.h>
#include <stdint.h>

#define ICON_X 0
#define SSD1306_YELLOW_HEIGHT 16
#define SSD1306_BLUE_HEIGHT 48
#define ICON_Y SSD1306_YELLOW_HEIGHT
#define ICON_SIZE 48

// 文本区域（蓝色右侧80x48）
#define TEXT_X (ICON_X + ICON_SIZE + 2)
#define TEXT_Y SSD1306_YELLOW_HEIGHT
#define TEXT_WIDTH 80
#define TEXT_HEIGHT 48

void draw_main_screen(
    ssd1306_handle_t* disp,
    uint32_t anim_time_ms,
    float pressure,
    float temperature,
    float altitude,
    float speed,
    float distance,
    DisplayStatus display_state,
    float vacc,
    bool is_stationary
);

void draw_settings_screen(ssd1306_handle_t* disp, uint32_t anim_time_ms);
void draw_error_screen(ssd1306_handle_t* disp, uint32_t anim_time_ms);

// Function to display icon animation with detailed text labels for testing
void draw_test_screen(
    ssd1306_handle_t* disp,
    DisplayStatus display_state,
    uint32_t anim_time_ms,
    float vvel,          // vertical velocity (m/s)
    float vacc,          // vertical acceleration (m/s^2)
    bool is_stationary,  // stationary flag
    const char* title    // title text
);

void draw_calib_confirm_screen(ssd1306_handle_t* disp, int countdown_s);

#endif