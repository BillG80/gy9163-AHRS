// include/core/fusion/calibration.h

#ifndef CORE_FUSION_CALIBRATION_H
#define CORE_FUSION_CALIBRATION_H

#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "display/screens.h"
#include <stdint.h>

// Holds all biases, scales and misalignment matrices
typedef struct {
    float gyro_bias[3];
    float gyro_scale[3][3];
    float accel_bias[3];
    float accel_scale[3][3];
    float mag_bias[3];
    float mag_scale[3][3];
    float R_am[3][3];    // accel→mag frame rotation
    float R_ga[3][3];    // gyro→accel frame rotation (cross-sensor alignment)
} calib_params_t;

// Stage routines
int calibrate_gyro_bias(ssd1306_handle_t* disp, calib_params_t* params);
int calibrate_accel    (ssd1306_handle_t* disp, calib_params_t* params);
int calibrate_mag      (ssd1306_handle_t* disp, calib_params_t* params);
int cross_calibrate    (ssd1306_handle_t* disp, calib_params_t* params);

// 4a) Frame alignment: compute accel→mag rotation
int calibrate_frame_alignment(ssd1306_handle_t* disp, calib_params_t* params);
// 4b) Gyro scale & cross-coupling calibration
int calibrate_gyro_scale     (ssd1306_handle_t* disp, calib_params_t* params);

// Wrapper: runs all stages in sequence
int run_full_calibration(ssd1306_handle_t* disp, calib_params_t* params);

// File I/O for persistent storage
int load_calibration(const char* filename, calib_params_t* params);
int save_calibration(const char* filename, const calib_params_t* params);

#endif // CORE_FUSION_CALIBRATION_H