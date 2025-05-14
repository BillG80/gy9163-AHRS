#ifndef VERTICAL_MOTION_H
#define VERTICAL_MOTION_H

// Event codes for general motion detector
#define VERTICAL_MOTION_EVENT_NONE    0
#define VERTICAL_MOTION_EVENT_STARTED 1
#define VERTICAL_MOTION_EVENT_STOPPED 2

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float vertical_accel_mss;
    float vertical_velocity_ms;
    float vertical_distance_m;
    bool is_stationary;
} VerticalMotionState;

typedef struct {
    float stationary_threshold_g; // e.g., 0.03f
    float velocity_drift_reset_threshold_s; // e.g., 1.0f
} VerticalMotionConfig;

typedef struct {
    float threshold_g;
    float min_duration_s;
    float duration_s;
    bool in_motion;
} GeneralMotionDetector;

typedef struct {
    float acceleration_threshold_g; // e.g., 0.1f
    float duration_threshold_s; // e.g., 0.5f
} GeneralMotionDetectionConfig;

typedef struct {
    bool is_moving;
    uint32_t motion_start_time_ms;
} GeneralMotionDetectionState;

void vertical_motion_init(VerticalMotionConfig *config);
void vertical_motion_update(float accel_world_z_g, float dt, VerticalMotionState *state);

void general_motion_detection_init(GeneralMotionDetectionConfig *config);
void general_motion_detection_update(float accel_world_z_g, uint32_t timestamp_ms, GeneralMotionDetectionState *state);

#endif // VERTICAL_MOTION_H