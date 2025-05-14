// src/core/fusion/calibration.c

#include "core/fusion/calibration.h"
#include "logging/calibration_data_log.h"
#include "core/fusion/calibration_config.h"
#include "display/calib_pose_icons.h"
#include <bcm2835.h>
#include "drivers/icm20948/icm20948.h"
#include <stdio.h>
#include <inttypes.h>
#include "drivers/led_indicator.h"
#include "ssd1306_graphics.h"
#include "core/fusion/svd3.h"
#include <math.h>
#include "fonts/SansCondensed.h"
#include <string.h>
#include <signal.h>
#include <stdbool.h>
#include <stdatomic.h>

// Helper: returns 1 if button held >BUTTON_HOLD_CANCEL_MS ms, else 0
// Uses CALIB_BUTTON_PIN (GPIO5) for calibration confirmation
static int check_button_hold_cancel() {
    int held_ms = 0;
    int btn_level = bcm2835_gpio_lev(CALIB_BUTTON_PIN);
    printf("[DEBUG] CALIB_BUTTON_PIN (GPIO5) state before hold check: %d\n", btn_level);
    if (btn_level) {
        while (bcm2835_gpio_lev(CALIB_BUTTON_PIN)) {
            bcm2835_delay(10);
            held_ms += 10;
            if (held_ms > BUTTON_HOLD_CANCEL_MS) return 1;
        }
    }
    return 0;
}


// Global flag for SIGINT
volatile sig_atomic_t calib_interrupted = 0;

void calib_sigint_handler(int sig) {
    calib_interrupted = 1;
}


// Externally defined in sensor_fusion.c:
extern icm20948_dev_t icm_dev;

// 1) Gyro bias only (static)
int calibrate_gyro_bias(ssd1306_handle_t* disp, calib_params_t* params) {
    // --- CSV Logging Setup ---
    uint64_t unix_time = bcm2835_st_read();
    char filename[64];
    make_calib_filename(filename, sizeof(filename), "gyro", unix_time);
    FILE *csv = fopen(filename, "w");
    if (csv) fprintf(csv, "timestamp_us,gx,gy,gz\n");
    // Set up SIGINT handler
    struct sigaction sa;
    sa.sa_handler = calib_sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    const int num_samples = GYRO_NUM_SAMPLES;
    int16_t gx_raw, gy_raw, gz_raw;
    double sum[3] = {0};
    int samples = 0;
    // Prompt: hold still (icon)
    //int y0 = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    // Draw hold still pose icon at fixed position x=0, y=16
    //draw_calib_pose_icon(disp, 0, 16, CALIB_POSE_HOLDSTILL);
    // Optionally show a minimal text prompt below the icon
    //int x_centered = (SSD1306_WIDTH - ssd1306_get_text_width("Hold Still...", 1)) / 2;
    //ssd1306_draw_text(disp, x_centered, y0 + CALIB_ICON_SIZE + 2, "Hold Still...", 1);
    //ssd1306_update(disp);
    //bcm2835_delay(5000);
    // Draw yellow zone hint and axis labels/bars in blue zone
    ssd1306_clear_buffer(disp);
    // Yellow zone: show 'Gyro Collecting...'
    int yellow_hint_y = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    //ssd1306_draw_text(disp, 2, yellow_hint_y, "Gyro Collecting...", 1); // Text can be made a macro if desired
    // Use unified bar and prompt positions from calibration_config.h
    
    const int bar_w = BAR_WIDTH;
    const int bar_h = BAR_THICKNESS;
    const int ys[3] = {BAR_Y_START1, BAR_Y_START2, BAR_Y_START3};
    
    const int prompt_y[3] = {PROMPT_Y_ROW1, PROMPT_Y_ROW2, PROMPT_Y_ROW3};
    const char* axis_prompts[3] = {"Gx", "Gy", "Gz"};

    for (int i = 0; i < num_samples; ++i) {
        // Draw UI
        ssd1306_clear_buffer(disp);
        // Draw pose icon (Z up) at fixed position x=0, y=16
        draw_calib_pose_icon(disp, 0, 16, CALIB_POSE_HOLDSTILL);
        // Alternate yellow zone text every 500ms, center-aligned
        const char* yellow_text = ((i/50) % 2 == 0) ? "Gyro Calibration" : "Hold still ...";
        int yellow_text_x = (SSD1306_WIDTH - ssd1306_get_text_width(yellow_text, 1)) / 2;
        ssd1306_draw_text(disp, yellow_text_x, yellow_hint_y, yellow_text, 1);
        // Blue zone: axis prompts and thick bars
        for (int j = 0; j < 3; ++j) {
            ssd1306_draw_text(disp, TEXT_X, prompt_y[j], axis_prompts[j], 1);
            ssd1306_draw_rect(disp, TEXT_X + 18, ys[j] + 2, bar_w, bar_h, 1);
        }
        // Fill progress bars
        int filled = ((i + 1) * (bar_w - 2)) / num_samples;
        for (int j = 0; j < 3; ++j) {
            ssd1306_fill_rect(disp, TEXT_X + 19, ys[j] + 3, filled, bar_h - 2, 1);
        }
        ssd1306_update(disp);
        bcm2835_delayMicroseconds(900);
        // Check for long button hold cancel
        if (check_button_hold_cancel()) {
            calib_interrupted = 1;
            
            calib_interrupted = 1;
            return -1;
        }
        if (icm20948_read_gyro(&icm_dev, &gx_raw, &gy_raw, &gz_raw) == ICM20948_RET_OK) {
    uint64_t now_us = bcm2835_st_read();
    if (csv) fprintf(csv, "%" PRIu64 ",%d,%d,%d\n", now_us, gx_raw, gy_raw, gz_raw);
            sum[0] += gx_raw / 131.0;
            sum[1] += gy_raw / 131.0;
            sum[2] += gz_raw / 131.0;
            samples++;
        }
    }
    
    // Compute and store bias
    params->gyro_bias[0] = samples ? (float)(sum[0] / samples) : 0.0f;
    params->gyro_bias[1] = samples ? (float)(sum[1] / samples) : 0.0f;
    params->gyro_bias[2] = samples ? (float)(sum[2] / samples) : 0.0f;
    // Display result
    char buf[32];
    ssd1306_clear_buffer(disp);
    snprintf(buf, sizeof(buf), "Gx=%.2f", params->gyro_bias[0]);
    ssd1306_draw_text(disp, 2, 2 + SANS_FONT_HEIGHT + 2, buf, 1);
    snprintf(buf, sizeof(buf), "Gy=%.2f", params->gyro_bias[1]);
    ssd1306_draw_text(disp, 2, 2 + 2 * (SANS_FONT_HEIGHT + 2), buf, 1);
    snprintf(buf, sizeof(buf), "Gz=%.2f", params->gyro_bias[2]);
    ssd1306_draw_text(disp, 2, 2 + 3 * (SANS_FONT_HEIGHT + 2), buf, 1);
    ssd1306_update(disp);
    // Give user ample time to rotate freely (increased from 2s to 6s)
    bcm2835_delay(8000);
    return 0;
}

// 2) Accel bias & scale (6-face method)
int calibrate_accel(ssd1306_handle_t* disp, calib_params_t* params) {
    // --- CSV Logging Setup ---
    uint64_t unix_time = bcm2835_st_read();
    char filename[64];
    make_calib_filename(filename, sizeof(filename), "accel", unix_time);
    FILE *csv = fopen(filename, "w");
    if (csv) fprintf(csv, "timestamp_us,ax,ay,az,face\n");
    printf(">> Accel bias & scale calibration\n");
    const int num_samples = ACCEL_NUM_SAMPLES;
    int16_t ax_raw, ay_raw, az_raw;
    double sum_pos[3] = {0}, sum_neg[3] = {0};
    const float conv = 1.0f/16384.0f; // raw to g for ±2g FS
    const int prompt_y[3] = {PROMPT_Y_ROW1, PROMPT_Y_ROW2, PROMPT_Y_ROW3};
    for (int f = 0; f < 6; ++f) {
    if (calib_interrupted) return -1;
        int axis = (f == 0 ? 2 : f == 5 ? 2 : (f-1)/2);  // map +Z/-Z axis to Z, others accordingly
        // Prompt face-up

        ssd1306_clear_buffer(disp);
        // Draw pose icon for current face
        calib_pose_icon_t pose_icon = CALIB_POSE_ZP;
        switch(f) {
            case 0: pose_icon = CALIB_POSE_HOLDSTILL; break; // +Z up uses holdstill icon
            case 1: pose_icon = CALIB_POSE_XP; break; // +X up
            case 2: pose_icon = CALIB_POSE_XN; break; // -X up
            case 3: pose_icon = CALIB_POSE_YP; break; // +Y up
            case 4: pose_icon = CALIB_POSE_YN; break; // -Y up
            case 5: pose_icon = CALIB_POSE_ZN; break; // -Z up
        }
        // For +Z up data collection, ensure holdstill icon is used
if (f == 0) pose_icon = CALIB_POSE_HOLDSTILL;
draw_calib_pose_icon(disp, 0, 16, pose_icon);
        // Draw yellow zone hint for click confirmation
char yellow_hint[32];
if (f == 5) {
    snprintf(yellow_hint, sizeof(yellow_hint), "Turn over & Keep still");
    // Turn on LED to indicate -Z data collection complete
    led_indicator_set_status(STATUS_CALIB_COMPLETE);
} else {
    snprintf(yellow_hint, sizeof(yellow_hint), "Align sensor to Arrow");
}
int yellow_hint_y = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
int x_centered = (SSD1306_WIDTH - ssd1306_get_text_width(yellow_hint, 1)) / 2;
ssd1306_draw_text(disp, x_centered - 2, yellow_hint_y, yellow_hint, 1);
        if (f > 0 && f < 5) {
            // Prompt for button confirmation in blue zone

            // Draw two-line hint aligned with axis prompt rows
            const char* ready_line1 = "Click Panel";
            const char* ready_line2 = "When Ready";
            ssd1306_draw_text(disp, TEXT_X, prompt_y[0], ready_line1, 1);
            ssd1306_draw_text(disp, TEXT_X, prompt_y[1], ready_line2, 1);
            ssd1306_update(disp);
            // Wait for short button click
            while (!bcm2835_gpio_lev(PIN_CAP_BUTTON) && !calib_interrupted) { }
if (calib_interrupted) return -1;
bcm2835_delay(200); // debounce
        } else if (f == 5) {
            // For -Z, wait for stable -g for >1s before collecting
            ssd1306_update(disp);
            const float conv = 1.0f/16384.0f;
            int16_t ax, ay, az;
            uint64_t stable_start = 0;
            while (!calib_interrupted) {
                if (icm20948_read_accel(&icm_dev, &ax, &ay, &az) == ICM20948_RET_OK) {
                    float z_g = az * conv;
                    if (z_g < -0.9f && z_g > -1.1f) {
                        if (!stable_start) stable_start = bcm2835_st_read();
                        if (bcm2835_st_read() - stable_start > 1000000) break;
                    } else {
                        stable_start = 0;
                    }
                }
                bcm2835_delay(10);
            }
            if (calib_interrupted) return -1;
            // Proceed to collection
            // (collection loop follows as normal)
        } else {
            // For +Z, auto-start after a delay
            ssd1306_update(disp);
            for (int d=0; d<2000 && !calib_interrupted; d+=10) bcm2835_delay(10);
            if (calib_interrupted) return -1;
        }
        // Use unified bar and prompt positions from calibration_config.h
        
        const int bar_w = BAR_WIDTH;
        const int bar_h = BAR_THICKNESS;
        const int ys[3] = {BAR_Y_START1, BAR_Y_START2, BAR_Y_START3};
        
        const int prompt_y[3] = {PROMPT_Y_ROW1, PROMPT_Y_ROW2, PROMPT_Y_ROW3};
        const char* ax_labels[3] = {"Ax", "Ay", "Az"}; // Already no colon
        // Sample readings with progress update
        for (int i = 0; i < num_samples; ++i) {
            // Draw UI
            ssd1306_clear_buffer(disp);
            // Always draw pose icon for current face
calib_pose_icon_t icon_to_draw = (f == 0) ? CALIB_POSE_HOLDSTILL : pose_icon;
draw_calib_pose_icon(disp, 0, 16, icon_to_draw);
            // Yellow zone: orientation + keep still
            char yellow_hint[32];
            if (f == 5) {
        snprintf(yellow_hint, sizeof(yellow_hint), "Turn over & Keep still");
    } else {
        snprintf(yellow_hint, sizeof(yellow_hint), "Align sensor to Arrow");
    }
    int yellow_hint_y = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    int x_centered = (SSD1306_WIDTH - ssd1306_get_text_width(yellow_hint, 1)) / 2;
    // Move 'Align sensor to Arrow' 1px left, 2px up
    if (check_button_hold_cancel()) {
            calib_interrupted = 1;
            
        calib_interrupted = 1;
        
        return -1;
    }
    if (strcmp(yellow_hint, "Align sensor to Arrow") == 0) {
        ssd1306_draw_text(disp, x_centered - 3, yellow_hint_y - 2, yellow_hint, 1);
    } else {
        ssd1306_draw_text(disp, x_centered - 2, yellow_hint_y, yellow_hint, 1);
    }
            // Blue zone: axis labels and bars
            for (int k = 0; k < 3; ++k) {
                ssd1306_draw_text(disp, TEXT_X, prompt_y[k], ax_labels[k], 1);
                ssd1306_draw_rect(disp, TEXT_X + 18, ys[k] + 2, bar_w, bar_h, 1);
            }
            // Fill progress bars
            int filled = ((i + 1) * (bar_w - 2)) / num_samples;
            for (int k = 0; k < 3; ++k) {
                ssd1306_fill_rect(disp, TEXT_X + 19, ys[k] + 3, filled, bar_h - 2, 1);
            }
            ssd1306_update(disp);

            if (icm20948_read_accel(&icm_dev, &ax_raw, &ay_raw, &az_raw) == ICM20948_RET_OK) {
    uint64_t now_us = bcm2835_st_read();
    if (csv) fprintf(csv, "%" PRIu64 ",%d,%d,%d,%d\n", now_us, ax_raw, ay_raw, az_raw, f);
                float val = (axis==0?ax_raw : axis==1?ay_raw : az_raw)*conv;
                if (f % 2 == 0) sum_pos[axis] += val;
                else            sum_neg[axis] += val;
            }
            bcm2835_delayMicroseconds(5000);
        }
    }
    
    // Compute and store accel bias and scale
    for (int axis = 0; axis < 3; ++axis) {
        float pos = sum_pos[axis] / num_samples;
        float neg = sum_neg[axis] / num_samples;
        // Bias is the average of positive and negative orientations
        params->accel_bias[axis] = (pos + neg) / 2.0f;
        // Scale: normalize so that (pos-neg)/2 = 1g
        float scale = (pos - neg) / 2.0f;
        float norm_scale = (scale != 0.0f) ? (1.0f / scale) : 1.0f;
        for (int j = 0; j < 3; ++j) {
            params->accel_scale[axis][j] = (j == axis) ? norm_scale : 0.0f;
        }
    }

    // Completed all 6 orientations, now signal completion
    led_indicator_set_status(STATUS_CALIB_COMPLETE);
    // For -Z, require button click to proceed, LED stays on
    // Only trigger this if the last face was -Z (f==5)
    if (calib_interrupted) return -1;
    while (!bcm2835_gpio_lev(PIN_CAP_BUTTON) && !calib_interrupted) { bcm2835_delay(10); }
    if (calib_interrupted) return -1;
    led_indicator_set_status(STATUS_OFF);
    return 0;
}

// 3) Mag hard/soft iron (ellipsoid fit)
int calibrate_mag(ssd1306_handle_t* disp, calib_params_t* params) {
    // --- CSV Logging Setup ---
    uint64_t unix_time = bcm2835_st_read();
    char filename[64];
    make_calib_filename(filename, sizeof(filename), "mag", unix_time);
    FILE *csv = fopen(filename, "w");
    if (csv) fprintf(csv, "timestamp_us,mx,my,mz\n");
    const int num_samples = MAG_NUM_SAMPLES;
    int16_t mx_raw, my_raw, mz_raw;
    float minv[3] = {1e9f,1e9f,1e9f}, maxv[3] = {-1e9f,-1e9f,-1e9f};
    const float conv = 0.15f; // uT per LSB for AK09916
    // Prompt: rotate sensor in all directions
    int y0 = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    ssd1306_clear_buffer(disp);
    // Center align 'Rotate all axes'
    const char* rotate_hint = "Rotate all axes";
    int x_centered = (SSD1306_WIDTH - ssd1306_get_text_width(rotate_hint, 1)) / 2;
    ssd1306_draw_text(disp, x_centered, y0, rotate_hint, 1);
    // Draw rotate icon at left
    draw_calib_pose_icon(disp, 0, 16, CALIB_POSE_FR);
    ssd1306_update(disp);
    // Give user ample time to rotate freely (increased from 2s to 6s)
    bcm2835_delay(6000);
    // Wait briefly for orientation clear, then start sampling
    bcm2835_delay(2000);
    // Set up unified bar UI
    ssd1306_clear_buffer(disp);
    // Center align 'Collecting Mag...'
    const char* collecting_mag = "Collecting Mag...";
    int x_cmag = (SSD1306_WIDTH - ssd1306_get_text_width(collecting_mag, 1)) / 2;
    ssd1306_draw_text(disp, x_cmag, y0, collecting_mag, 1);
    // (No rotate icon in blue zone during collection)
    // Bar and prompt positions (from calibration_config.h)
    
    const int bar_w = BAR_WIDTH;
    const int bar_h = BAR_THICKNESS;
    const int ys[3] = {BAR_Y_START1, BAR_Y_START2, BAR_Y_START3};
    
    const int prompt_y[3] = {PROMPT_Y_ROW1, PROMPT_Y_ROW2, PROMPT_Y_ROW3};
    const char* axis_prompts[3] = {"Mx", "My", "Mz"};
    ssd1306_update(disp);
    // Collect raw magnetometer data
    for (int i = 0; i < num_samples; ++i) {
        // Draw UI
        ssd1306_clear_buffer(disp);
        // Draw rotate icon at left during collection
        draw_calib_pose_icon(disp, 0, 16, CALIB_POSE_FR);
        ssd1306_draw_text(disp, x_cmag, y0, "Collecting Mag...", 1);
        for (int j = 0; j < 3; ++j) {
            ssd1306_draw_text(disp, TEXT_X, prompt_y[j], axis_prompts[j], 1);
            ssd1306_draw_rect(disp, TEXT_X + 18, ys[j] + 2, bar_w, bar_h, 1);
        }
        // Fill progress bars
        int filled = ((i + 1) * (bar_w - 2)) / num_samples;
        for (int j = 0; j < 3; ++j) {
            ssd1306_fill_rect(disp, TEXT_X + 19, ys[j] + 3, filled, bar_h - 2, 1);
        }
        ssd1306_update(disp);

        if (icm20948_read_mag(&icm_dev, &mx_raw, &my_raw, &mz_raw) == ICM20948_RET_OK) {
    uint64_t now_us = bcm2835_st_read();
    if (csv) fprintf(csv, "%" PRIu64 ",%d,%d,%d\n", now_us, mx_raw, my_raw, mz_raw);
            float m[3] = {mx_raw*conv, my_raw*conv, mz_raw*conv};
            for (int j = 0; j < 3; ++j) {
                if (m[j] < minv[j]) minv[j] = m[j];
                if (m[j] > maxv[j]) maxv[j] = m[j];
            }

        }
        bcm2835_delayMicroseconds(10000);
        // Check for long button hold cancel
        if (check_button_hold_cancel()) {
            calib_interrupted = 1;
            
            calib_interrupted = 1;
            return -1;
        }
    }
    
    // Compute hard-iron bias and axis-aligned soft-iron scale
    for (int j = 0; j < 3; ++j) {
        params->mag_bias[j] = (maxv[j] + minv[j]) * 0.5f;
        float rad = (maxv[j] - minv[j]) * 0.5f;
        params->mag_scale[j][j] = (rad > 0.0f ? 1.0f / rad : 1.0f);
        for (int k = 0; k < 3; ++k) {
            if (k != j) params->mag_scale[j][k] = 0.0f;
        }
    }
    // Display calibration results
    char buf[32];
    ssd1306_clear_buffer(disp);
    for (int j = 0; j < 3; ++j) {
        snprintf(buf, sizeof(buf), "Mb%d=%.1fuT", j, params->mag_bias[j]);
        ssd1306_draw_text(disp, 2, 2 + (j+1)*(SANS_FONT_HEIGHT+2), buf, 1);
        snprintf(buf, sizeof(buf), "Ms%d=%.3f", j, params->mag_scale[j][j]);
        ssd1306_draw_text(disp, SSD1306_WIDTH/2, 2 + (j+1)*(SANS_FONT_HEIGHT+2), buf, 1);
    }
    ssd1306_update(disp);
    bcm2835_delay(3000);
    return 0;
}

// 4a) Frame alignment: compute accel→mag rotation
// Note: Using fixed sample counts (e.g. 500 samples at ~100Hz ≈5s per pose).
// To adaptively stop when noise converges, implement an Allan-variance or running-variance based check.
int calibrate_frame_alignment(ssd1306_handle_t* disp, calib_params_t* params) {
    // Prompts and hints for all poses
    const char* collecting_hint = "FM Alignment";
    // New axis order: +Z, +X, -X, +Y, -Y, -Z
    const char* accel_prompts[6] = {"Az", "Ax", "Ax", "Ay", "Ay", "Az"};
    const char* mag_prompts[6] = {"Mz", "Mx", "Mx", "My", "My", "Mz"};
    // Set up SIGINT handler
    struct sigaction sa;
    sa.sa_handler = calib_sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    const int num_poses = 6, num_samples = 500;
    const char* poses[6] = {"+Z up","+X up","-X up","+Y up","-Y up","-Z up"};
    float accel_sum[6][3] = {{0}}, mag_sum[6][3] = {{0}};
    int counts[6] = {0};
    const float accel_conv = 1.0f/16384.0f, mag_conv = 0.15f;
    int y0 = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    for (int p = 0; p < num_poses; ++p) {
        // 1. Show pose in yellow, instruction in blue
        ssd1306_clear_buffer(disp);
        int yellow_y = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
        char yellow_hint[32];
        int x_centered;
        if (p == 5) {
            snprintf(yellow_hint, sizeof(yellow_hint), "Turn over & Keep still");
            // Turn on LED to indicate final pose of frame alignment
            led_indicator_set_status(STATUS_CALIB_COMPLETE);
        } else {
            snprintf(yellow_hint, sizeof(yellow_hint), "%s - Keep still ...", poses[p]);
        }
        x_centered = (SSD1306_WIDTH - ssd1306_get_text_width(yellow_hint, 1)) / 2;
        ssd1306_draw_text(disp, x_centered, yellow_y, yellow_hint, 1);
        int blue_msg_y = SSD1306_YELLOW_HEIGHT + (SSD1306_BLUE_HEIGHT - SANS_FONT_HEIGHT) / 2;
        // Draw pose icon for current pose
        calib_pose_icon_t pose_icon = CALIB_POSE_ZP;
        switch(p) {
            case 0: pose_icon = CALIB_POSE_ZP; break;   // +Z up
            case 1: pose_icon = CALIB_POSE_XP; break;   // +X up
            case 2: pose_icon = CALIB_POSE_XN; break;   // -X up
            case 3: pose_icon = CALIB_POSE_YP; break;   // +Y up
            case 4: pose_icon = CALIB_POSE_YN; break;   // -Y up
            case 5: pose_icon = CALIB_POSE_ZN; break;   // -Z up
        }
        draw_calib_pose_icon(disp, 0, 16, pose_icon);
        // Draw two-line blue zone prompt
        const char* ready_line1 = "Click Panel";
        const char* ready_line2 = "When Ready";
        ssd1306_draw_text(disp, TEXT_X, PROMPT_Y_ROW1, ready_line1, 1);
        ssd1306_draw_text(disp, TEXT_X, PROMPT_Y_ROW2, ready_line2, 1);
        ssd1306_update(disp);
        if (p == 5) {
            // For -Z up, wait for stable -g for >1s before collecting
            const float conv = 1.0f/16384.0f;
            int16_t ax, ay, az;
            uint64_t stable_start = 0;
            while (!calib_interrupted) {
                if (icm20948_read_accel(&icm_dev, &ax, &ay, &az) == ICM20948_RET_OK) {
                    float z_g = az * conv;
                    if (z_g < -0.9f && z_g > -1.1f) {
                        if (!stable_start) stable_start = bcm2835_st_read();
                        if (bcm2835_st_read() - stable_start > 1000000) break;
                    } else {
                        stable_start = 0;
                    }
                }
                bcm2835_delay(10);
            }
            if (calib_interrupted) return -1;
        } else {
            // Wait for button release (idle)
            while (bcm2835_gpio_lev(PIN_CAP_BUTTON)) { bcm2835_delay(10); }
            // Wait for button press
            do { bcm2835_delay(10); } while (!bcm2835_gpio_lev(PIN_CAP_BUTTON));
            // Debounce
            bcm2835_delay(200);
        }
        // 2. Show 'Collecting...' and progress bar
        ssd1306_clear_buffer(disp);
        draw_calib_pose_icon(disp, 0, 16, pose_icon);
        ssd1306_draw_text(disp, x_centered, yellow_y, yellow_hint, 1);
        // Draw the collecting hint above the progress bar
        int collecting_hint_y = blue_msg_y - SANS_FONT_HEIGHT - 2;
        if (collecting_hint_y < SSD1306_YELLOW_HEIGHT) collecting_hint_y = SSD1306_YELLOW_HEIGHT;
        // Row 1: Collecting hint, left-aligned at PROMPT_Y_ROW1
        int collecting_row1_y = PROMPT_Y_ROW1;
        ssd1306_draw_text(disp, TEXT_X, collecting_row1_y, collecting_hint, 1);
        // Row 2: Ax/Ay/Az prompt and progress bar
        ssd1306_draw_text(disp, TEXT_X, PROMPT_Y_ROW2, accel_prompts[p], 1);
        ssd1306_draw_rect(disp, TEXT_X + 18, PROMPT_Y_ROW2 + 2, BAR_WIDTH, BAR_THICKNESS, 1);
        // Row 3: Mx/My/Mz prompt and progress bar
        ssd1306_draw_text(disp, TEXT_X, PROMPT_Y_ROW3, mag_prompts[p], 1);
        ssd1306_draw_rect(disp, TEXT_X + 18, PROMPT_Y_ROW3 + 2, BAR_WIDTH, BAR_THICKNESS, 1);
        ssd1306_update(disp);
        // 3. Progress bar loop
        for (int i = 0; i < num_samples; ++i) {
    if (check_button_hold_cancel()) {
            calib_interrupted = 1;
            
        ssd1306_clear_buffer(disp);
        ssd1306_draw_text(disp, 2, SSD1306_HEIGHT/2 - SANS_FONT_HEIGHT/2, "Calibration Cancelled", 1);
        ssd1306_update(disp);
        bcm2835_delay(1000);
        return -1;
    }
    int16_t ax, ay, az, mx, my, mz;
    if (icm20948_read_accel(&icm_dev, &ax, &ay, &az) == ICM20948_RET_OK &&
        icm20948_read_mag  (&icm_dev, &mx, &my, &mz) == ICM20948_RET_OK) {
        accel_sum[p][0] += ax * accel_conv;
        accel_sum[p][1] += ay * accel_conv;
        accel_sum[p][2] += az * accel_conv;
        mag_sum[p][0]   += mx * mag_conv;
        mag_sum[p][1]   += my * mag_conv;
        mag_sum[p][2]   += mz * mag_conv;
        counts[p]++;
    }
    int filled = ((i + 1) * (BAR_WIDTH - 2)) / num_samples;
    // Draw filled progress bars for both accel and mag
    ssd1306_fill_rect(disp, TEXT_X + 19, PROMPT_Y_ROW2 + 3, filled, BAR_THICKNESS - 2, 1);
    ssd1306_fill_rect(disp, TEXT_X + 19, PROMPT_Y_ROW3 + 3, filled, BAR_THICKNESS - 2, 1);
    ssd1306_update(disp);
    bcm2835_delayMicroseconds(5000);
    if (calib_interrupted) {
        ssd1306_clear_buffer(disp);
        ssd1306_draw_text(disp, 2, SSD1306_HEIGHT/2 - SANS_FONT_HEIGHT/2, "Calibration Cancelled", 1);
        ssd1306_update(disp);
        bcm2835_delay(1000);
        return -1;
    }
}
    }
    // Compute mean accel and mag per pose
    float mean_a[num_poses][3], mean_m[num_poses][3];
    for (int p = 0; p < num_poses; ++p) {
        for (int j = 0; j < 3; ++j) {
            mean_a[p][j] = accel_sum[p][j] / counts[p];
            mean_m[p][j] = mag_sum[p][j]   / counts[p];
        }
    }
    // Compute centroids
    float ca[3] = {0}, cm[3] = {0};
    for (int p = 0; p < num_poses; ++p) {
        for (int j = 0; j < 3; ++j) {
            ca[j] += mean_a[p][j]; cm[j] += mean_m[p][j];
        }
    }
    for (int j = 0; j < 3; ++j) {
        ca[j] /= num_poses; cm[j] /= num_poses;
    }
    // Build covariance matrix H
    float H[3][3] = {{0}};
    for (int p = 0; p < num_poses; ++p) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                H[i][j] += (mean_a[p][i] - ca[i]) * (mean_m[p][j] - cm[j]);
            }
        }
    }
    // SVD: H = U * diag(S) * Vt
    float U[3][3], S[3], Vt[3][3];
    svd3(H, U, S, Vt);
    // Compute rotation R_am = V * U^T
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += Vt[k][i] * U[j][k];
            }
            params->R_am[i][j] = sum;
        }
    }
    // Display R_am
    printf("R_am:\n");
    for (int i = 0; i < 3; ++i) {
        printf("%+0.3f %+0.3f %+0.3f\n",
            params->R_am[i][0], params->R_am[i][1], params->R_am[i][2]);
    }
    ssd1306_clear_buffer(disp);
    ssd1306_draw_text(disp, 2, y0, "Frame Align Done", 1);
    ssd1306_update(disp);
    // Give user ample time to rotate freely (increased from 2s to 6s)
    bcm2835_delay(6000);


    return 0;
}

// 4b) Gyro scale & cross-coupling calibration
int calibrate_gyro_scale(ssd1306_handle_t* disp, calib_params_t* params) {
    const int num_axes = 3, num_samples = 500;
    const char* axes[3] = {"X","Y","Z"};
    float gyro_sum[3][3] = {{0}};
    int counts[3] = {0};
    int y0 = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    for (int a = 0; a < num_axes; ++a) {
    if (calib_interrupted) return -1;
        char buf[32];
        snprintf(buf, sizeof(buf), "Rotate 90\xB0 around %s", axes[a]);
        ssd1306_clear_buffer(disp);
        // Draw axis rotation icon for current axis (use icon_xr, icon_yr, icon_zr)
        calib_pose_icon_t pose_icon = CALIB_POSE_XR;
        if (a == 0) pose_icon = CALIB_POSE_XR;
        else if (a == 1) pose_icon = CALIB_POSE_YR;
        else if (a == 2) pose_icon = CALIB_POSE_ZR;
        draw_calib_pose_icon(disp, 0, 16, pose_icon);
        // Center align the yellow zone hint
        int hint_width = ssd1306_get_text_width(buf, 1);
        int x_centered = (SSD1306_WIDTH - hint_width) / 2;
        ssd1306_draw_text(disp, x_centered, y0, buf, 1);
        // Draw two-line blue zone prompt at standard positions
        ssd1306_draw_text(disp, TEXT_X, PROMPT_Y_ROW1, "Click Panel", 1);
        ssd1306_draw_text(disp, TEXT_X, PROMPT_Y_ROW2, "When Ready", 1);
        ssd1306_update(disp);
        // Wait for button release (idle)
        while (bcm2835_gpio_lev(PIN_CAP_BUTTON)) { bcm2835_delay(10); }
        // Wait for button press
        do { bcm2835_delay(10); } while (!bcm2835_gpio_lev(PIN_CAP_BUTTON));
        // Debounce
        bcm2835_delay(200);
        // Sample gyro
        ssd1306_clear_buffer(disp);
        // Draw pose icon for current axis (same as prompt)
        draw_calib_pose_icon(disp, 0, 16, pose_icon);
        // Center-align 'Collecting Gyro...'
        const char* collecting_hint = "Collecting Gyro...";
        int collecting_hint_width = ssd1306_get_text_width(collecting_hint, 1);
        int collecting_x = (SSD1306_WIDTH - collecting_hint_width) / 2;
        ssd1306_draw_text(disp, collecting_x, y0, collecting_hint, 1);
        // Use BAR_X_START, BAR_WIDTH, BAR_THICKNESS for the progress bar
        int bar_x = BAR_X_START;
        int bar_y = PROMPT_Y_ROW2 + 2; // middle of the blue zone
        ssd1306_draw_rect(disp, bar_x, bar_y, BAR_WIDTH, BAR_THICKNESS, 1);
        ssd1306_update(disp);
        for (int i = 0; i < num_samples; ++i) {
            // Check for long button hold cancel
            if (check_button_hold_cancel()) {
            calib_interrupted = 1;
            
                calib_interrupted = 1;
                return -1;
            }
            int16_t gx, gy, gz;
            if (icm20948_read_gyro(&icm_dev, &gx, &gy, &gz) == ICM20948_RET_OK) {
                gyro_sum[a][0] += gx;
                gyro_sum[a][1] += gy;
                gyro_sum[a][2] += gz;
                counts[a]++;
            }
            int filled = ((i + 1) * (BAR_WIDTH - 2)) / num_samples;
            ssd1306_fill_rect(disp, bar_x + 1, bar_y + 1, filled, BAR_THICKNESS - 2, 1);
            ssd1306_update(disp);
            bcm2835_delayMicroseconds(5000);
        }
    }
    // Compute raw mean matrix R_raw
    float R_raw[3][3], invR[3][3];
    for (int a = 0; a < 3; ++a) {
        for (int j = 0; j < 3; ++j) {
            R_raw[a][j] = gyro_sum[a][j] / counts[a];
        }
    }
    // Invert R_raw
    float det = R_raw[0][0] * (R_raw[1][1] * R_raw[2][2] - R_raw[1][2] * R_raw[2][1])
              - R_raw[0][1] * (R_raw[1][0] * R_raw[2][2] - R_raw[1][2] * R_raw[2][0])
              + R_raw[0][2] * (R_raw[1][0] * R_raw[2][1] - R_raw[1][1] * R_raw[2][0]);
    if (fabsf(det) > 1e-6f) {
        float invDet = 1.0f / det;
        invR[0][0] =  (R_raw[1][1]*R_raw[2][2] - R_raw[1][2]*R_raw[2][1]) * invDet;
        invR[0][1] = -(R_raw[0][1]*R_raw[2][2] - R_raw[0][2]*R_raw[2][1]) * invDet;
        invR[0][2] =  (R_raw[0][1]*R_raw[1][2] - R_raw[0][2]*R_raw[1][1]) * invDet;
        invR[1][0] = -(R_raw[1][0]*R_raw[2][2] - R_raw[1][2]*R_raw[2][0]) * invDet;
        invR[1][1] =  (R_raw[0][0]*R_raw[2][2] - R_raw[0][2]*R_raw[2][0]) * invDet;
        invR[1][2] = -(R_raw[0][0]*R_raw[1][2] - R_raw[0][2]*R_raw[1][0]) * invDet;
        invR[2][0] =  (R_raw[1][0]*R_raw[2][1] - R_raw[1][1]*R_raw[2][0]) * invDet;
        invR[2][1] = -(R_raw[0][0]*R_raw[2][1] - R_raw[0][1]*R_raw[2][0]) * invDet;
        invR[2][2] =  (R_raw[0][0]*R_raw[1][1] - R_raw[0][1]*R_raw[1][0]) * invDet;
    } else {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                invR[i][j] = (i == j ? 1.0f : 0.0f);
    }
    // Compute true rate from 90° over total sampling time
    float dt = 5000e-6f;
    float w_true = (M_PI/2.0f) / (num_samples * dt);
    // Populate the gyro_scale matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            params->gyro_scale[i][j] = w_true * invR[i][j];
        }
    }
    // Display gyro_scale
    printf("G_gyro:\n");
    for (int i = 0; i < 3; ++i)
        printf("%+0.3f %+0.3f %+0.3f\n",
               params->gyro_scale[i][0],
               params->gyro_scale[i][1],
               params->gyro_scale[i][2]);
    ssd1306_clear_buffer(disp);
    const char* gyro_done_msg = "Gyro Scale Done";
    int gyro_done_x = (SSD1306_WIDTH - ssd1306_get_text_width(gyro_done_msg, 1)) / 2;
    int gyro_done_y = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    ssd1306_draw_text(disp, gyro_done_x, gyro_done_y, gyro_done_msg, 1);
    ssd1306_update(disp);
    // Give user ample time to rotate freely (increased from 2s to 6s)
    bcm2835_delay(6000);
    return 0;
}

// 4c) Final cross-calibration
int cross_calibrate(ssd1306_handle_t* disp, calib_params_t* params) {
    const int num_samples = GYRO_NUM_SAMPLES;
    // const float accel_conv = 1.0f/16384.0f; // Removed unused variable
    const float mag_conv   = 0.15f;
    float H[3][3] = {{0}};
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    // Sample free rotation data
    for (int i = 0; i < num_samples; ++i) {
        // Check for long button hold cancel
        if (check_button_hold_cancel()) {
            calib_interrupted = 1;
            
            calib_interrupted = 1;
            return -1;
        }
        icm20948_read_accel(&icm_dev, &ax, &ay, &az);
        icm20948_read_mag  (&icm_dev, &mx, &my, &mz);
        icm20948_read_gyro (&icm_dev, &gx, &gy, &gz);
        // Convert mag into accel frame
        float mag_f[3] = {mx * mag_conv, my * mag_conv, mz * mag_conv};
        float mag_a[3] = {0};
        for (int m = 0; m < 3; ++m)
            for (int n = 0; n < 3; ++n)
                mag_a[m] += params->R_am[m][n] * mag_f[n];
        // Apply gyro scale
        float g_cal[3];
        for (int j = 0; j < 3; ++j)
            g_cal[j] = params->gyro_scale[j][0] * gx
                     + params->gyro_scale[j][1] * gy
                     + params->gyro_scale[j][2] * gz;
        // Accumulate correlation matrix H
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                H[r][c] += mag_a[r] * g_cal[c];
    }
    // Compute SVD of H
    float U[3][3], S[3], Vt[3][3];
    svd3(H, U, S, Vt);
    // R_ga = V * U^T
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += Vt[k][i] * U[j][k];
            params->R_ga[i][j] = sum;
        }
    // Step 1: During 4c, show icon_fr at (0,16) and 'Rotate Freely' in yellow zone, center aligned
    ssd1306_clear_buffer(disp);
    draw_calib_pose_icon(disp, 0, 16, CALIB_POSE_FR);
    const char* crosscal_msg = "Rotate Freely";
    int crosscal_x = (SSD1306_WIDTH - ssd1306_get_text_width(crosscal_msg, 1)) / 2;
    int crosscal_y = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    ssd1306_draw_text(disp, crosscal_x, crosscal_y, crosscal_msg, 1);
    ssd1306_update(disp);
    // Give user ample time to rotate freely (increased from 2s to 6s)
    bcm2835_delay(6000);

    // Step 2: After 4c complete, show icon_comp at (0,16), 3 lines at right side of blue zone, and 'Cross Cal Done' in yellow zone, center aligned, all at the same time
    ssd1306_clear_buffer(disp);
    // Draw icon_comp
    draw_calib_pose_icon(disp, 0, 16, CALIB_POSE_COMP);
    // Draw 'Cross Cal Done' in yellow zone, center aligned
    const char* done_msg = "Cross Cal Done";
    int done_x = (SSD1306_WIDTH - ssd1306_get_text_width(done_msg, 1)) / 2;
    int done_y = (SSD1306_YELLOW_HEIGHT - SANS_FONT_HEIGHT) / 2;
    ssd1306_draw_text(disp, done_x, done_y, done_msg, 1);
    // Draw 3 lines at right side of blue zone
    const char* line1 = "Congrats!";
    const char* line2 = "Calib Sucess";
    const char* line3 = "JSON filed";
    int prompt_x = TEXT_X;
    int prompt_y1 = SSD1306_YELLOW_HEIGHT + 2;
    int prompt_y2 = prompt_y1 + SANS_FONT_HEIGHT + 2;
    int prompt_y3 = prompt_y2 + SANS_FONT_HEIGHT + 2;
    ssd1306_draw_text(disp, prompt_x, prompt_y1, line1, 1);
    ssd1306_draw_text(disp, prompt_x, prompt_y2, line2, 1);
    ssd1306_draw_text(disp, prompt_x, prompt_y3, line3, 1);
    ssd1306_update(disp);
    // Give user ample time to rotate freely (increased from 2s to 6s)
    bcm2835_delay(6000);
    return 0;
}

// Runs all stages in sequence
typedef int (*stage_fn)(ssd1306_handle_t*, calib_params_t*);
int run_full_calibration(ssd1306_handle_t* disp, calib_params_t* params) {
    static stage_fn stages[] = {
        calibrate_gyro_bias,
        calibrate_accel,
        calibrate_mag,
        calibrate_frame_alignment,
        calibrate_gyro_scale,
        cross_calibrate,
    };
    for (int i = 0; i < (int)(sizeof(stages)/sizeof(*stages)); ++i) {
        int ret = stages[i](disp, params);
        if (ret) return -(i+1);
    }
    return 0;
}

// File I/O for persistent storage
int load_calibration(const char* filename, calib_params_t* params) {
    if (!filename || !params) return -1;
    FILE* f = fopen(filename, "r");
    if (!f) { perror("load_calibration: fopen"); return -1; }
    bool have_gyro_bias=false, have_gyro_scale=false;
    bool have_accel_bias=false, have_accel_scale=false;
    bool have_mag_bias=false, have_mag_scale=false;
    bool have_R_am=false;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "\"gyro_bias\"")) {
            int ret = sscanf(line, " \"gyro_bias\": [ %f , %f , %f ]",
                   &params->gyro_bias[0], &params->gyro_bias[1], &params->gyro_bias[2]);
            if (ret==3) have_gyro_bias=true;
            else { fprintf(stderr,"load_calibration: parse error gyro_bias\n"); fclose(f); return -1; }
        } else if (strstr(line, "\"gyro_scale\"")) {
            int count=0;
            for (int i=0; i<3; ++i) {
                if (!fgets(line,sizeof(line),f)) { fclose(f); return -1; }
                int ret2=sscanf(line, " [ %f , %f , %f ]",
                       &params->gyro_scale[i][0], &params->gyro_scale[i][1], &params->gyro_scale[i][2]);
                if (ret2==3) count++;
                else { fprintf(stderr,"load_calibration: parse error gyro_scale[%d]\n",i); fclose(f); return -1; }
            }
            have_gyro_scale=(count==3);
        } else if (strstr(line, "\"accel_bias\"")) {
            int ret = sscanf(line, " \"accel_bias\": [ %f , %f , %f ]",
                   &params->accel_bias[0], &params->accel_bias[1], &params->accel_bias[2]);
            if (ret==3) have_accel_bias=true;
            else { fprintf(stderr,"load_calibration: parse error accel_bias\n"); fclose(f); return -1; }
        } else if (strstr(line, "\"accel_scale\"")) {
            int count=0;
            for (int i=0; i<3; ++i) {
                if (!fgets(line,sizeof(line),f)) { fclose(f); return -1; }
                int ret2=sscanf(line, " [ %f , %f , %f ]",
                       &params->accel_scale[i][0], &params->accel_scale[i][1], &params->accel_scale[i][2]);
                if (ret2==3) count++;
                else { fprintf(stderr,"load_calibration: parse error accel_scale[%d]\n",i); fclose(f); return -1; }
            }
            have_accel_scale=(count==3);
        } else if (strstr(line, "\"mag_bias\"")) {
            int ret = sscanf(line, " \"mag_bias\": [ %f , %f , %f ]",
                   &params->mag_bias[0], &params->mag_bias[1], &params->mag_bias[2]);
            if (ret==3) have_mag_bias=true;
            else { fprintf(stderr,"load_calibration: parse error mag_bias\n"); fclose(f); return -1; }
        } else if (strstr(line, "\"mag_scale\"")) {
            int count=0;
            for (int i=0; i<3; ++i) {
                if (!fgets(line,sizeof(line),f)) { fclose(f); return -1; }
                int ret2=sscanf(line, " [ %f , %f , %f ]",
                       &params->mag_scale[i][0], &params->mag_scale[i][1], &params->mag_scale[i][2]);
                if (ret2==3) count++;
                else { fprintf(stderr,"load_calibration: parse error mag_scale[%d]\n",i); fclose(f); return -1; }
            }
            have_mag_scale=(count==3);
        } else if (strstr(line, "\"R_am\"")) {
            int count=0;
            for (int i=0; i<3; ++i) {
                if (!fgets(line,sizeof(line),f)) { fclose(f); return -1; }
                int ret2=sscanf(line, " [ %f , %f , %f ]",
                       &params->R_am[i][0], &params->R_am[i][1], &params->R_am[i][2]);
                if (ret2==3) count++;
                else { fprintf(stderr,"load_calibration: parse error R_am[%d]\n",i); fclose(f); return -1; }
            }
            have_R_am=(count==3);
        }
    }
    fclose(f);
    if (!(have_gyro_bias && have_gyro_scale && have_accel_bias && have_accel_scale &&
          have_mag_bias && have_mag_scale && have_R_am)) {
        fprintf(stderr,"load_calibration: incomplete calibration data\n");
        return -1;
    }
    return 0;
}

int save_calibration(const char* filename, const calib_params_t* params) {
    if (!filename || !params) {
        fprintf(stderr, "save_calibration: invalid arguments\n");
        return -1;
    }
    FILE* f = fopen(filename, "w");
    if (!f) {
        perror("save_calibration: fopen failed");
        return -1;
    }
    fprintf(f, "{\n");
    // Gyro bias
    fprintf(f, "  \"gyro_bias\": [%.6f, %.6f, %.6f],\n",
            params->gyro_bias[0], params->gyro_bias[1], params->gyro_bias[2]);
    // Gyro scale
    fprintf(f, "  \"gyro_scale\": [\n");
    for (int i = 0; i < 3; ++i) {
        fprintf(f, "    [%.6f, %.6f, %.6f]%s\n",
                params->gyro_scale[i][0], params->gyro_scale[i][1], params->gyro_scale[i][2],
                i < 2 ? "," : "");
    }
    fprintf(f, "  ],\n");
    // Accel bias & scale
    fprintf(f, "  \"accel_bias\": [%.6f, %.6f, %.6f],\n",
            params->accel_bias[0], params->accel_bias[1], params->accel_bias[2]);
    fprintf(f, "  \"accel_scale\": [\n");
    for (int i = 0; i < 3; ++i) {
        fprintf(f, "    [%.6f, %.6f, %.6f]%s\n",
                params->accel_scale[i][0], params->accel_scale[i][1], params->accel_scale[i][2],
                i < 2 ? "," : "");
    }
    fprintf(f, "  ],\n");
    // Mag bias & scale
    fprintf(f, "  \"mag_bias\": [%.6f, %.6f, %.6f],\n",
            params->mag_bias[0], params->mag_bias[1], params->mag_bias[2]);
    fprintf(f, "  \"mag_scale\": [\n");
    for (int i = 0; i < 3; ++i) {
        fprintf(f, "    [%.6f, %.6f, %.6f]%s\n",
                params->mag_scale[i][0], params->mag_scale[i][1], params->mag_scale[i][2],
                i < 2 ? "," : "");
    }
    fprintf(f, "  ],\n");
    // Frame alignment matrix R_am
    fprintf(f, "  \"R_am\": [\n");
    for (int i = 0; i < 3; ++i) {
        fprintf(f, "    [%.6f, %.6f, %.6f]%s\n",
                params->R_am[i][0], params->R_am[i][1], params->R_am[i][2],
                i < 2 ? "," : "");
    }
    fprintf(f, "  ]\n");
    fprintf(f, "}\n");
    fclose(f);
    return 0;
}