#include "core/navigation/elevator_nav.h"
#include "core/navigation/vertical_motion.h"
#include <stdio.h>  // For printf in stubs
#include <math.h>   // For potentially needed math functions (e.g., sqrt, sin, cos)
#include <stdbool.h> // for bool

// --- Defines ---
#define GRAVITY_M_S2 9.80665f
// Calibration control
#define SLIDING_CALIBRATION 0 // set to 1 for continuous recalibration
static bool _calibration_done = false;

// Elevator motion detection parameters (used only for elevator mode)
#define MIN_MOTION_ACCEL_G 0.05f  // Minimum acceleration to detect motion (in g)
#define MAX_MOTION_ACCEL_G 0.2f    // Maximum acceleration to detect motion (in g)
#define MIN_MOTION_TIME_S 0.8f     // Minimum time for consistent acceleration to detect motion (in seconds)
#define MIN_STOP_TIME_S 0.8f       // Minimum time for consistent deceleration to detect stop (in seconds)

// Low-pass filter alpha for lin_norm
#define LIN_NORM_LP_ALPHA 0.8f
static float _lin_norm_lp = 0.0f;        // Filtered lin_norm magnitude

// Dynamic motion detection thresholds (elevator only)
static float _motion_thresh_min_g = MIN_MOTION_ACCEL_G;
static float _motion_thresh_max_g = MAX_MOTION_ACCEL_G;
static const float THRESHOLD_MULTIPLIER = 3.0f;
#define THRESH_LP_ALPHA     0.95f
#define MAX_THRESH_CAP_G    0.30f

// Elevator motion states
typedef enum {
    ELEVATOR_STATE_STATIONARY,
    ELEVATOR_STATE_ACCEL_UP,
    ELEVATOR_STATE_ACCEL_DOWN,
    ELEVATOR_STATE_MOVING_UP,
    ELEVATOR_STATE_MOVING_DOWN,
    ELEVATOR_STATE_DECEL_UP,
    ELEVATOR_STATE_DECEL_DOWN
} ElevatorState_T;

// --- Module Private Variables ---
static NavData_T _internal_nav_state; // Store the integrated state internally
static uint64_t _last_update_time_us = 0;

// Motion detection variables
static ElevatorState_T _elevator_state = ELEVATOR_STATE_STATIONARY;
static float _accel_duration_s = 0.0f;  // Duration of consistent acceleration
static float _decel_duration_s = 0.0f;  // Duration of consistent deceleration
static float _prev_accel_z = 0.0f;      // Previous vertical acceleration
static uint64_t _motion_start_time_us = 0; // Timestamp when motion started
static uint64_t _motion_stop_time_us = 0;  // Timestamp when motion stopped
static float _trip_start_altitude_m = 0.0f; // Altitude at trip start
static float _trip_distance_m = 0.0f;      // Distance traveled in current trip

// Data collection
#define MAX_DATA_POINTS 100
static ElevatorDataPoint_T _data_points[MAX_DATA_POINTS];
static int _data_point_count = 0;

// Cache for fusion data to use in data collection
static FusionData_T _fusion_data_cache;

// Add complementary filter weight and fused state
#define DIST_FUSE_ALPHA_MOVING 0.70f  // inertial vs baro fusion weight during movement (lower => more baro)

static float _fused_distance_m = 0.0f;

// Allan deviation computation (static samples)
#define ALLAN_BUF_SIZE 1024  // number of samples window
static float _allan_buf[ALLAN_BUF_SIZE];
static int _allan_idx = 0;
static int _allan_count = 0;

// Compute Allan deviation for tau = sample period
static float compute_allan_dev(const float *data, int N) {
    float sum = 0.0f;
    for (int i = 1; i < N; i++) {
        float diff = data[i] - data[i-1];
        sum += diff * diff;
    }
    return sqrtf(sum / (2.0f * (N - 1)));
}

// --- Helper Functions ---
static void quaternion_to_gravity_vector(float q0, float q1, float q2, float q3, float* gx, float* gy, float* gz) {
    // Calculates the components of the gravity vector in the sensor frame
    // based on the orientation quaternion.
    // See quaternion rotation formula.
    *gx = 2.0f * (q1 * q3 - q0 * q2);
    *gy = 2.0f * (q0 * q1 + q2 * q3);
    *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // This is g_z / |g|
}

// Estimate floor based on altitude change
static int estimate_floor(float altitude_change_m) {
    // Assuming average floor height is about 3 meters
    const float avg_floor_height_m = 3.0f;
    return (int)roundf(altitude_change_m / avg_floor_height_m);
}

// Add a data point to the collection
static void add_data_point(const FusionData_T *fusion_data, const NavData_T *nav_data) {
    if (_data_point_count >= MAX_DATA_POINTS) {
        // Array is full, can't add more points
        return;
    }
    
    ElevatorDataPoint_T *point = &_data_points[_data_point_count++];
    point->timestamp_us = fusion_data->timestamp_us;
    point->temperature_c = fusion_data->temperature_c;
    point->pressure_pa = fusion_data->pressure_pa;
    point->altitude_m = fusion_data->altitude_m;
    point->vertical_velocity_m_s = nav_data->vertical_velocity_m_s;
    point->vertical_accel_m_s2 = nav_data->vertical_accel_m_s2;
    
    // Calculate floor estimate based on altitude change from trip start
    float altitude_change_m = fusion_data->altitude_m - _trip_start_altitude_m;
    point->estimated_floor = estimate_floor(altitude_change_m);
    
    // Just log a brief message about data collection instead of printing all details
    // The actual data will be logged by file_logger
    printf("Collected data point %d at time %llu\n", 
           _data_point_count, 
           (unsigned long long)point->timestamp_us);
}

// Update elevator state based on acceleration
static void update_elevator_state(float accel_z, float dt_s) {
    // Convert acceleration to g units for easier comparison with thresholds
    float accel_g = accel_z / GRAVITY_M_S2;
    
    // State machine for elevator motion detection
    switch (_elevator_state) {
        case ELEVATOR_STATE_STATIONARY:
            // Check for start of upward acceleration
            if (accel_g >= _motion_thresh_min_g && accel_g <= _motion_thresh_max_g) {
                _elevator_state = ELEVATOR_STATE_ACCEL_UP;
                _accel_duration_s = dt_s;
                printf("State: STATIONARY -> ACCEL_UP (%.3fg)\n", accel_g);
            }
            // Check for start of downward acceleration
            else if (accel_g <= -_motion_thresh_min_g && accel_g >= -_motion_thresh_max_g) {
                _elevator_state = ELEVATOR_STATE_ACCEL_DOWN;
                _accel_duration_s = dt_s;
                printf("State: STATIONARY -> ACCEL_DOWN (%.3fg)\n", accel_g);
            }
            break;
            
        case ELEVATOR_STATE_ACCEL_UP:
            // Continue upward acceleration
            if (accel_g >= _motion_thresh_min_g && accel_g <= _motion_thresh_max_g) {
                _accel_duration_s += dt_s;
                // Check if acceleration has lasted long enough to confirm motion
                if (_accel_duration_s >= MIN_MOTION_TIME_S) {
                    _elevator_state = ELEVATOR_STATE_MOVING_UP;
                    _motion_start_time_us = _last_update_time_us;
                    _trip_start_altitude_m = _fusion_data_cache.altitude_m;
                    _trip_distance_m = 0.0f;
                    printf("State: ACCEL_UP -> MOVING_UP (Duration: %.2fs)\n", _accel_duration_s);
                    
                    // Add data point at start of motion
                    add_data_point(&_fusion_data_cache, &_internal_nav_state);
                }
            }
            // Acceleration stopped too soon
            else {
                _elevator_state = ELEVATOR_STATE_STATIONARY;
                _accel_duration_s = 0.0f;
                printf("State: ACCEL_UP -> STATIONARY (Accel stopped)\n");
            }
            break;
            
        case ELEVATOR_STATE_ACCEL_DOWN:
            // Continue downward acceleration
            if (accel_g <= -_motion_thresh_min_g && accel_g >= -_motion_thresh_max_g) {
                _accel_duration_s += dt_s;
                // Check if acceleration has lasted long enough to confirm motion
                if (_accel_duration_s >= MIN_MOTION_TIME_S) {
                    _elevator_state = ELEVATOR_STATE_MOVING_DOWN;
                    _motion_start_time_us = _last_update_time_us;
                    _trip_start_altitude_m = _fusion_data_cache.altitude_m;
                    _trip_distance_m = 0.0f;
                    printf("State: ACCEL_DOWN -> MOVING_DOWN (Duration: %.2fs)\n", _accel_duration_s);
                    
                    // Add data point at start of motion
                    add_data_point(&_fusion_data_cache, &_internal_nav_state);
                }
            }
            // Acceleration stopped too soon
            else {
                _elevator_state = ELEVATOR_STATE_STATIONARY;
                _accel_duration_s = 0.0f;
                printf("State: ACCEL_DOWN -> STATIONARY (Accel stopped)\n");
            }
            break;
            
        case ELEVATOR_STATE_MOVING_UP:
            // Check for deceleration (negative acceleration while moving up)
            if (accel_g <= -_motion_thresh_min_g && accel_g >= -_motion_thresh_max_g) {
                _elevator_state = ELEVATOR_STATE_DECEL_UP;
                _decel_duration_s = dt_s;
                printf("State: MOVING_UP -> DECEL_UP (%.3fg)\n", accel_g);
            }
            break;
            
        case ELEVATOR_STATE_MOVING_DOWN:
            // Check for deceleration (positive acceleration while moving down)
            if (accel_g >= _motion_thresh_min_g && accel_g <= _motion_thresh_max_g) {
                _elevator_state = ELEVATOR_STATE_DECEL_DOWN;
                _decel_duration_s = dt_s;
                printf("State: MOVING_DOWN -> DECEL_DOWN (%.3fg)\n", accel_g);
            }
            break;
            
        case ELEVATOR_STATE_DECEL_UP:
            // Continue deceleration
            if (accel_g <= -_motion_thresh_min_g && accel_g >= -_motion_thresh_max_g) {
                _decel_duration_s += dt_s;
                // Check if deceleration has lasted long enough to confirm stop
                if (_decel_duration_s >= MIN_STOP_TIME_S) {
                    _elevator_state = ELEVATOR_STATE_STATIONARY;
                    _motion_stop_time_us = _last_update_time_us;
                    printf("State: DECEL_UP -> STATIONARY (Duration: %.2fs, Trip: %.2fm)\n", 
                           _decel_duration_s, _trip_distance_m);
                    
                    // Add data point at end of motion
                    add_data_point(&_fusion_data_cache, &_internal_nav_state);
                    
                    // Reset velocity when stopped
                    _internal_nav_state.vertical_velocity_m_s = 0.0f;
                }
            }
            // Deceleration stopped too soon
            else {
                _elevator_state = ELEVATOR_STATE_MOVING_UP;
                _decel_duration_s = 0.0f;
                printf("State: DECEL_UP -> MOVING_UP (Decel stopped)\n");
            }
            break;
            
        case ELEVATOR_STATE_DECEL_DOWN:
            // Continue deceleration
            if (accel_g >= _motion_thresh_min_g && accel_g <= _motion_thresh_max_g) {
                _decel_duration_s += dt_s;
                // Check if deceleration has lasted long enough to confirm stop
                if (_decel_duration_s >= MIN_STOP_TIME_S) {
                    _elevator_state = ELEVATOR_STATE_STATIONARY;
                    _motion_stop_time_us = _last_update_time_us;
                    printf("State: DECEL_DOWN -> STATIONARY (Duration: %.2fs, Trip: %.2fm)\n", 
                           _decel_duration_s, _trip_distance_m);
                    
                    // Add data point at end of motion
                    add_data_point(&_fusion_data_cache, &_internal_nav_state);
                    
                    // Reset velocity when stopped
                    _internal_nav_state.vertical_velocity_m_s = 0.0f;
                }
            }
            // Deceleration stopped too soon
            else {
                _elevator_state = ELEVATOR_STATE_MOVING_DOWN;
                _decel_duration_s = 0.0f;
                printf("State: DECEL_DOWN -> MOVING_DOWN (Decel stopped)\n");
            }
            break;
    }
    
    // Update previous acceleration for next cycle
    _prev_accel_z = accel_z;
}

// --- Function Definitions ---

static ElevatorNavMode_T _nav_mode = ELEVATOR_NAV_MODE_ELEVATOR;

int elevator_nav_init(ElevatorNavMode_T mode) {
    _nav_mode = mode;
    _internal_nav_state.timestamp_us = 0;
    _internal_nav_state.vertical_accel_m_s2 = 0.0f;
    _internal_nav_state.vertical_velocity_m_s = 0.0f;
    _internal_nav_state.vertical_distance_m = 0.0f;
    _internal_nav_state.is_stationary = true; // Assume stationary at start
    _last_update_time_us = 0;
    
    // Initialize motion detection variables
    _elevator_state = ELEVATOR_STATE_STATIONARY;
    _accel_duration_s = 0.0f;
    _decel_duration_s = 0.0f;
    _prev_accel_z = 0.0f;
    _motion_start_time_us = 0;
    _motion_stop_time_us = 0;
    _trip_start_altitude_m = 0.0f;
    _trip_distance_m = 0.0f;
    
    // Initialize data collection
    _data_point_count = 0;

    // Set thresholds based on mode
    if (mode == ELEVATOR_NAV_MODE_ELEVATOR) {
        _motion_thresh_min_g = 0.025f;
        _motion_thresh_max_g = 0.2f;
        #undef MIN_MOTION_TIME_S
        #undef MIN_STOP_TIME_S
        #define MIN_MOTION_TIME_S 1.0f
        #define MIN_STOP_TIME_S 1.0f
        printf("Mode: ELEVATOR (min_g=0.025, min_time=1.0s)\n");
    } else {
        // General mode: thresholds not used here, handled by vertical_motion.c
        printf("Mode: GENERAL (motion detection handled by vertical_motion.c)\n");
    }

    printf("Elevator Navigation Initialized.\n");
    return 0;
}

int elevator_nav_update(const FusionData_T *fusion_data, NavData_T *nav_data) {
    if (!fusion_data || !nav_data) {
        return -1; // Invalid pointers
    }

    // Cache fusion data for use in data collection
    _fusion_data_cache = *fusion_data;
    
    // --- Calculate Delta Time ---
    uint64_t current_time_us = fusion_data->timestamp_us;
    uint64_t prev_time_us    = _last_update_time_us;
    float    dt_s            = 0.0f;
    if (prev_time_us > 0 && current_time_us > prev_time_us) {
        dt_s = (float)(current_time_us - prev_time_us) / 1000000.0f;
    }
    _last_update_time_us = current_time_us;

    if (dt_s <= 0.0f || dt_s > 0.5f) { // Basic sanity check on dt (ignore huge gaps for now)
        // Handle initial case or large time gaps - maybe reset integration?
         dt_s = 0.01f; // Use default dt for first run or if gap is too large? Risky.
         // Or just return without updating?
         printf("WARN: Invalid dt_s = %.4f s in nav update. Skipping integration.\n", dt_s);
          *nav_data = _internal_nav_state; // Output the last known state
          nav_data->timestamp_us = current_time_us; // Update timestamp
          return 0; // Return success, but didn't integrate
    }

    // Initialize trip baseline altitude on first valid update
    if (_trip_start_altitude_m == 0.0f) {
        _trip_start_altitude_m = _fusion_data_cache.altitude_m;
        _fused_distance_m = 0.0f;
    }

    // --- Calculate Vertical Acceleration (Subtract Gravity) ---
    // 1. Get gravity vector in sensor frame using AHRS quaternion
    float gravity_x, gravity_y, gravity_z;
    quaternion_to_gravity_vector(fusion_data->q0, fusion_data->q1, fusion_data->q2, fusion_data->q3,
                                 &gravity_x, &gravity_y, &gravity_z);
    // Debug: Print quaternion and gravity vector with its norm
    printf("[DBG] Quaternion: (q0=%.6f, q1=%.6f, q2=%.6f, q3=%.6f)\n", fusion_data->q0, fusion_data->q1, fusion_data->q2, fusion_data->q3);
    float grav_norm = sqrtf(gravity_x*gravity_x + gravity_y*gravity_y + gravity_z*gravity_z);
    printf("[DBG] Gravity vector: (%.6f, %.6f, %.6f), norm=%.6f\n", gravity_x, gravity_y, gravity_z, grav_norm);

    // 2. Calculate the Z-component of gravity in the sensor frame (scaled by g)
    float gravity_z_component = gravity_z * GRAVITY_M_S2;

    // 3. Convert measured Z acceleration from g to m/s², then subtract gravity component
    float measured_z_mss = fusion_data->accel_z * GRAVITY_M_S2;
    float linear_accel_z = measured_z_mss - gravity_z_component;
    _internal_nav_state.vertical_accel_m_s2 = linear_accel_z; // Store the linear acceleration

    // --- Update Elevator State Machine or General Motion Detector ---
    // Compute gravity-compensated accel vector magnitude
    float grav_x_mss = gravity_x * GRAVITY_M_S2;
    float grav_y_mss = gravity_y * GRAVITY_M_S2;
    float grav_z_mss = gravity_z * GRAVITY_M_S2;
    float lin_ax = fusion_data->accel_x * GRAVITY_M_S2 - grav_x_mss;
    float lin_ay = fusion_data->accel_y * GRAVITY_M_S2 - grav_y_mss;
    float lin_az = measured_z_mss - grav_z_mss;
    float lin_norm = sqrtf(lin_ax*lin_ax + lin_ay*lin_ay + lin_az*lin_az);
    _lin_norm_lp = LIN_NORM_LP_ALPHA * _lin_norm_lp + (1.0f - LIN_NORM_LP_ALPHA) * lin_norm;

    if (_nav_mode == ELEVATOR_NAV_MODE_GENERAL) {
        // Compute world-frame vertical acceleration in "g"
        float grav_norm = sqrtf(gravity_x*gravity_x + gravity_y*gravity_y + gravity_z*gravity_z);
        float accel_x = fusion_data->accel_x;
        float accel_y = fusion_data->accel_y;
        float accel_z = fusion_data->accel_z;
        // Project measured accel onto gravity direction (normalized)
        float accel_world_z_g = (accel_x * gravity_x + accel_y * gravity_y + accel_z * gravity_z) / (grav_norm * grav_norm);
        VerticalMotionState vm_state;
        vertical_motion_update(accel_world_z_g, dt_s, &vm_state);
        // Copy results to nav state
        _internal_nav_state.vertical_velocity_m_s = vm_state.vertical_velocity_ms;
        _internal_nav_state.vertical_accel_m_s2   = vm_state.vertical_accel_mss;
        _internal_nav_state.vertical_distance_m   = vm_state.vertical_distance_m;
        _internal_nav_state.is_stationary         = vm_state.is_stationary;
        _internal_nav_state.timestamp_us          = current_time_us;
        *nav_data = _internal_nav_state;
        return 0;
    }

    if (_nav_mode == ELEVATOR_NAV_MODE_ELEVATOR) {
        // Elevator-specific state machine
        if (_calibration_done) {
            update_elevator_state(_lin_norm_lp, dt_s);
        }
        // Collect static samples for Allan deviation
        if (_elevator_state == ELEVATOR_STATE_STATIONARY) {
            _allan_buf[_allan_idx++] = _lin_norm_lp;
            if (_allan_idx >= ALLAN_BUF_SIZE) _allan_idx = 0;
            if (_allan_count < ALLAN_BUF_SIZE) _allan_count++;
            if (_allan_count == ALLAN_BUF_SIZE) {
                float adev = compute_allan_dev(_allan_buf, ALLAN_BUF_SIZE);
                printf("Allan deviation (stationary, _lin_norm_lp): %.6f m/s^2\n", adev);
                float sigma_g     = adev / GRAVITY_M_S2;
                float comp_min_g  = fmaxf(THRESHOLD_MULTIPLIER * sigma_g, MIN_MOTION_ACCEL_G);
                float comp_max_g  = fmaxf(2.0f * comp_min_g, MAX_MOTION_ACCEL_G);
                _motion_thresh_min_g = THRESH_LP_ALPHA * _motion_thresh_min_g
                                    + (1.0f - THRESH_LP_ALPHA) * comp_min_g;
                _motion_thresh_max_g = THRESH_LP_ALPHA * _motion_thresh_max_g
                                    + (1.0f - THRESH_LP_ALPHA) * comp_max_g;
                _motion_thresh_min_g = fminf(_motion_thresh_min_g, MAX_THRESH_CAP_G);
                _motion_thresh_max_g = fminf(_motion_thresh_max_g, MAX_THRESH_CAP_G);
                if (!_calibration_done) {
                    printf("Initial thresholds: min=%.3fg, max=%.3fg\n",
                           _motion_thresh_min_g, _motion_thresh_max_g);
                    _calibration_done = true;
                } else {
                    printf("Smoothed thresholds: min=%.3fg, max=%.3fg\n",
                           _motion_thresh_min_g, _motion_thresh_max_g);
                }
                _allan_count = 0; // reset window
            }
        }
    }


    // --- ZUPT using bidirectional hysteresis ---
    static const float ZUPT_ENTRY_TIME_S     = 0.2f;    // seconds to enter freeze
    static const float ZUPT_EXIT_TIME_S      = 0.2f;    // seconds to exit freeze
    static const float ZUPT_ACC_NORM_THRESH  = 0.20f;   // m/s² accel threshold
    static const float ZUPT_VEL_THRESH       = 0.10f;   // m/s velocity threshold
    static float zupt_entry_timer            = 0.0f;
    static float zupt_exit_timer             = 0.0f;
    // Use previously computed grav_x_mss, grav_y_mss, grav_z_mss, lin_ax, lin_ay, lin_az
    float lin_accel_norm = sqrtf(lin_ax*lin_ax + lin_ay*lin_ay + lin_az*lin_az);
    bool under_thresh = lin_accel_norm < ZUPT_ACC_NORM_THRESH
                       && fabsf(_internal_nav_state.vertical_velocity_m_s) < ZUPT_VEL_THRESH;
    // Bidirectional hysteresis for freeze/unfreeze
    bool freeze = _internal_nav_state.is_stationary;
    if (!freeze) {
        if (under_thresh) {
            zupt_entry_timer += dt_s;
            if (zupt_entry_timer >= ZUPT_ENTRY_TIME_S) {
                freeze = true;
                zupt_exit_timer = 0.0f;
            }
        } else {
            zupt_entry_timer = 0.0f;
        }
    } else {
        if (!under_thresh) {
            zupt_exit_timer += dt_s;
            if (zupt_exit_timer >= ZUPT_EXIT_TIME_S) {
                freeze = false;
                zupt_entry_timer = 0.0f;
            }
        } else {
            zupt_exit_timer = 0.0f;
        }
    }
    _internal_nav_state.is_stationary = freeze;
    printf("DBG ZUPT: entry_timer=%.3f exit_timer=%.3f freeze=%d lin_norm=%.3f vel=%.3f D=%.3f\n",
           zupt_entry_timer, zupt_exit_timer,
           freeze,
           lin_accel_norm, _internal_nav_state.vertical_velocity_m_s,
           _fused_distance_m);
    // Smooth barometer altitude via simple 1D Kalman filter
    static bool baro_kalman_init = false;
    static float baro_x = 0.0f, baro_P = 1.0f;
    static const float baro_Q = 0.01f;
    static const float baro_R_moving = 0.1f;  // low noise to track movement
    if (!baro_kalman_init) {
        baro_x = _fusion_data_cache.altitude_m;
        baro_kalman_init = true;
    }
    baro_P += baro_Q;
    // only update with baro when moving, freeze holds altitude
    if (!freeze) {
        float baro_K = baro_P / (baro_P + baro_R_moving);
        baro_x += baro_K * (_fusion_data_cache.altitude_m - baro_x);
        baro_P *= (1 - baro_K);
    }


    bool is_motion = (_elevator_state == ELEVATOR_STATE_MOVING_UP || _elevator_state == ELEVATOR_STATE_MOVING_DOWN);
    // --- IMU-only integration for distance (no barometer fusion) ---
    if (freeze || !is_motion) {
        _internal_nav_state.vertical_velocity_m_s = 0.0f;
        // Hold D constant when stationary (no baro blending)
        // _fused_distance_m remains unchanged
        _internal_nav_state.vertical_distance_m = _fused_distance_m;
    } else {
        // Integrate acceleration only (no baro fusion)
        _internal_nav_state.vertical_velocity_m_s += linear_accel_z * dt_s;
        float distance_increment = _internal_nav_state.vertical_velocity_m_s * dt_s;
        _fused_distance_m += distance_increment;
        _internal_nav_state.vertical_distance_m = _fused_distance_m;
        _trip_distance_m += fabsf(distance_increment);
    }
    // Derive display_state: judge velocity first, then acceleration
    const float ACC_THRESH = 0.2f;   // m/s² accel threshold
    const float VEL_THRESH = 0.05f;  // m/s velocity threshold
    if (nav_data->vertical_velocity_m_s > VEL_THRESH) {
        nav_data->display_state = DISP_STATE_MOVING_UP;
    } else if (nav_data->vertical_velocity_m_s < -VEL_THRESH) {
        nav_data->display_state = DISP_STATE_MOVING_DOWN;
    } else if (nav_data->vertical_accel_m_s2 > ACC_THRESH) {
        nav_data->display_state = DISP_STATE_ACCEL_UP;
    } else if (nav_data->vertical_accel_m_s2 < -ACC_THRESH) {
        nav_data->display_state = DISP_STATE_ACCEL_DOWN;
    } else {
        nav_data->display_state = DISP_STATE_STATIONARY;
    }
    // --- Update Output Structure ---
    _internal_nav_state.timestamp_us = current_time_us;
    *nav_data = _internal_nav_state; // Copy internal state to output

    return 0; // Success
}

// Get the current elevator state as a string
const char* elevator_nav_get_state_str(void) {
    switch (_elevator_state) {
        case ELEVATOR_STATE_STATIONARY: return "Stationary";
        case ELEVATOR_STATE_ACCEL_UP: return "Accelerating Up";
        case ELEVATOR_STATE_ACCEL_DOWN: return "Accelerating Down";
        case ELEVATOR_STATE_MOVING_UP: return "Moving Up";
        case ELEVATOR_STATE_MOVING_DOWN: return "Moving Down";
        case ELEVATOR_STATE_DECEL_UP: return "Decelerating (Up)";
        case ELEVATOR_STATE_DECEL_DOWN: return "Decelerating (Down)";
        default: return "Unknown";
    }
}

// Get trip statistics
void elevator_nav_get_trip_stats(uint64_t *start_time_us, uint64_t *end_time_us, float *distance_m) {
    if (start_time_us) *start_time_us = _motion_start_time_us;
    if (end_time_us) *end_time_us = _motion_stop_time_us;
    if (distance_m) *distance_m = _trip_distance_m;
}

// Get collected data points
int elevator_nav_get_data_points(ElevatorDataPoint_T **data, int *count) {
    if (!data || !count) return -1;
    *data = _data_points;
    *count = _data_point_count;
    return 0;
}

// Get the latest data point (for real-time access)
int elevator_nav_get_latest_data_point(ElevatorDataPoint_T *data_point) {
    if (!data_point || _data_point_count == 0) {
        return -1; // No data available or invalid pointer
    }
    
    // Copy the latest data point
    *data_point = _data_points[_data_point_count - 1];
    return 0;
}

void elevator_nav_cleanup(void) {
    // Nothing to do for now as no dynamic memory is allocated
    printf("Elevator Navigation Cleaned Up.\n");
    
    // Print summary of collected data
    printf("Collected %d data points during operation.\n", _data_point_count);
}