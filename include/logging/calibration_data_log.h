#ifndef CALIBRATION_DATA_LOG_H
#define CALIBRATION_DATA_LOG_H

#include <stdint.h>
#include <stdio.h>

// --- Data structures for logging calibration data with timestamps ---

typedef struct {
    uint64_t timestamp_us;
    int16_t gx, gy, gz;
} gyro_calib_sample_t;

typedef struct {
    uint64_t timestamp_us;
    int16_t ax, ay, az;
    int face_idx; // 0=+Z, 1=+X, 2=-X, 3=+Y, 4=-Y, 5=-Z
} accel_calib_sample_t;

typedef struct {
    uint64_t timestamp_us;
    int16_t mx, my, mz;
} mag_calib_sample_t;

// --- Utility for generating timestamped filenames ---
void make_calib_filename(char *buf, size_t buflen, const char *prefix, uint64_t unix_time);

#endif // CALIBRATION_DATA_LOG_H
