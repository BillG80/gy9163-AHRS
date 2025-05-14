#include "core/navigation/vertical_motion.h"
#include <math.h>
#include <string.h> // for memset

// Internal persistent state
static VerticalMotionConfig g_config = {0};
static VerticalMotionState g_state = {0};
static float stationary_time = 0.0f;
// --- Online bias correction state ---
static float accel_bias_z = 0.0f;
static float bias_sum = 0.0f;
static int bias_count = 0;

// --- General vertical motion detection ---
#include <stdio.h>
#include <math.h>

void general_motion_detector_init(GeneralMotionDetector *det, float threshold_g, float min_duration_s) {
    if (!det) return;
    det->threshold_g = threshold_g;
    det->min_duration_s = min_duration_s;
    det->duration_s = 0.0f;
    det->in_motion = false;
}

// Returns VERTICAL_MOTION_EVENT_NONE, _STARTED, or _STOPPED
int general_motion_detector_update(GeneralMotionDetector *det, float accel_g, float dt) {
    if (!det) return VERTICAL_MOTION_EVENT_NONE;
    int event = VERTICAL_MOTION_EVENT_NONE;
    float abs_accel = fabsf(accel_g);
    if (!det->in_motion) {
        if (abs_accel > det->threshold_g) {
            det->duration_s += dt;
            if (det->duration_s >= det->min_duration_s) {
                det->in_motion = true;
                event = VERTICAL_MOTION_EVENT_STARTED;
                printf("[general_motion_detector] Motion started\n");
            }
        } else {
            det->duration_s = 0.0f;
        }
    } else {
        if (abs_accel <= det->threshold_g) {
            det->duration_s += dt;
            if (det->duration_s >= det->min_duration_s) {
                det->in_motion = false;
                event = VERTICAL_MOTION_EVENT_STOPPED;
                printf("[general_motion_detector] Motion stopped\n");
            }
        } else {
            det->duration_s = 0.0f;
        }
    }
    return event;
}

float vertical_motion_get_distance(void) {
    return g_state.vertical_distance_m;
}

void vertical_motion_init(VerticalMotionConfig *config) {
    accel_bias_z = 0.0f;
    bias_sum = 0.0f;
    bias_count = 0;
    stationary_time = 0.0f; // also reset here

    if (config) {
        g_config = *config;
    } else {
        g_config.stationary_threshold_g = 0.03f;
        g_config.velocity_drift_reset_threshold_s = 1.0f;
    }
    memset(&g_state, 0, sizeof(g_state));
    stationary_time = 0.0f;
}

void vertical_motion_update(float accel_world_z_g, float dt, VerticalMotionState *state) {
    // Stationary detection (assume accel_world_z_g ~ 1.0 when stationary)
    float accel_error = fabsf(accel_world_z_g - 1.0f);
    bool is_stationary = (accel_error < g_config.stationary_threshold_g);

    if (is_stationary) {
        stationary_time += dt;
        // --- Online bias correction: accumulate bias while stationary ---
        bias_sum += (accel_world_z_g - 1.0f) * 9.80665f; // m/s^2
        bias_count++;
        if (bias_count >= 100) { // Update bias every 100 stationary samples
            accel_bias_z = bias_sum / bias_count;
            printf("[vertical_motion] Updated accel_bias_z: %.6f m/s^2\n", accel_bias_z);
            bias_sum = 0.0f;
            bias_count = 0;
        }
        // If stationary for a while, reset velocity and optionally distance drift
        if (stationary_time > g_config.velocity_drift_reset_threshold_s) {
            g_state.vertical_velocity_ms = 0.0f;
            // Optionally: g_state.vertical_distance_m = 0.0f; // Uncomment to reset distance on stationary
        }
    } else {
        stationary_time = 0.0f;
        // Optionally, reset bias accumulation if you want only continuous stationary periods
        // bias_sum = 0.0f; bias_count = 0;
    }

    g_state.is_stationary = is_stationary;

    // --- Apply bias correction to acceleration ---
    float raw_accel_mss = (accel_world_z_g - 1.0f) * 9.80665f;
    float corrected_accel_mss = raw_accel_mss - accel_bias_z;
    g_state.vertical_accel_mss = corrected_accel_mss;

    // Velocity integration (only if not stationary)
    if (!is_stationary) {
        g_state.vertical_velocity_ms += corrected_accel_mss * dt;
        g_state.vertical_distance_m += g_state.vertical_velocity_ms * dt;
    } else {
        g_state.vertical_velocity_ms = 0.0f;
        // Optionally, clamp distance here if desired
    }

    // Output state
    if (state) {
        *state = g_state;
    }
}