#ifndef CALIBRATION_CONFIG_H
#define CALIBRATION_CONFIG_H

// UI Layout
#define BAR_THICKNESS           8
#define BAR_WIDTH               58
#define BAR_X_START             64
#define BAR_Y_START1            20
#define BAR_Y_START2            36
#define BAR_Y_START3            52
#define BAR_X_END               125

// Single progress bar (frame alignment, etc)
#define PROGRESS_BAR_Y          40

// prompt position
#define PROMPT_X                50
#define PROMPT_Y_ROW1           18
#define PROMPT_Y_ROW2           34
#define PROMPT_Y_ROW3           50

// Calibration Logic
#define GYRO_NUM_SAMPLES        30000
#define ACCEL_NUM_SAMPLES       30000
#define MAG_NUM_SAMPLES         30000
#define BUTTON_HOLD_CANCEL_MS   4000

#endif // CALIBRATION_CONFIG_H
