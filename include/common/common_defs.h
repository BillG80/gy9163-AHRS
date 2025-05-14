#ifndef COMMON_DEFS_H_
#define COMMON_DEFS_H_

#include <stdint.h>
#include <stdbool.h>
#include <bcm2835.h> // For GPIO pin definitions

// System state for controlling display updates
typedef enum {
    SYSTEM_STATE_LOGO = 0,
    SYSTEM_STATE_CALIBRATION,
    SYSTEM_STATE_AHRS
} system_state_t;
extern system_state_t system_state;

#define COMMON_DEFS_H_

#include <stdint.h>
#include <stdbool.h>
#include <bcm2835.h> // For GPIO pin definitions

// --- GPIO Pin Definitions ---
// Use RPI_V2_GPIO_P1_XX or RPI_BPLUS_GPIO_J8_XX depending on your Pi model and header
#define PIN_LED_BLUE_GPIO12 RPI_V2_GPIO_P1_32 // GPIO 12 (PWM0) - Changed to BLUE
#define PIN_LED_RED_GPIO16 RPI_V2_GPIO_P1_36    // GPIO 16 - Changed to RED

#define PIN_LED_1 PIN_LED_BLUE_GPIO12 // GPIO 12 (PWM0) - Changed to BLUE
#define PIN_LED_2 PIN_LED_RED_GPIO16    // GPIO 16 - Changed to RED

// Capacitive Touch Button Pin
#define PIN_CAP_BUTTON RPI_V2_GPIO_P1_29 // GPIO 5
#define CALIB_BUTTON_PIN RPI_V2_GPIO_P1_29 // GPIO 5

// ICM20948 Interrupt Pin (Uncomment and set if using interrupts)
// #define PIN_INT_ICM RPI_V2_GPIO_P1_22 // GPIO 25

// #define PIN_BLE_TRIGGER RPI_V2_GPIO_P1_18 // GPIO 24 (SPI_CE0) - Placeholder, uncomment if needed

typedef enum {
  DISP_STATE_STATIONARY = 0,
  DISP_STATE_ACCEL_UP,
  DISP_STATE_ACCEL_DOWN,
  DISP_STATE_MOVING_UP,
  DISP_STATE_MOVING_DOWN
} DisplayStatus;

// --- Status Enum for LED Indicator and System State ---
typedef enum {
    STATUS_OFF,
    STATUS_INITIALIZING,
    STATUS_IDLE,                // Initialized, waiting or not actively processing significant events
    STATUS_RUNNING,             // Normal operation, processing data
    STATUS_ERROR_INIT,          // Fatal error during initialization
    STATUS_ERROR_RUNTIME,       // Non-fatal error during runtime (e.g., sensor read failure)
    STATUS_SHUTTING_DOWN,
    STATUS_CALIB_COMPLETE,  // Calibration complete: fast Red/Blue/Purple cycle
    // --- BLE Status Placeholders (Uncomment/modify as needed by BLE team) ---
    STATUS_BLE_ADVERTISING,
    STATUS_BLE_CONNECTED,
    STATUS_BLE_ERROR,

    STATUS_COUNT // Keep last for array sizing etc.
} StatusEnum;


// --- Data Structures ---

// Data output from the sensor fusion module (including AHRS)
typedef struct {
    uint64_t timestamp_us;      // System timestamp (microseconds)
    // Raw or calibrated sensor data
    float accel_x, accel_y, accel_z; // m/s^2 or g's
    float gyro_x, gyro_y, gyro_z;   // rad/s or deg/s
    float mag_x, mag_y, mag_z;     // uT or Gauss
    // Barometer data
    float pressure_pa;          // Pascals
    float temperature_c;        // Degrees Celsius
    float altitude_m;           // Calculated altitude (meters)
    // AHRS output (choose one representation or provide both)
    float q0, q1, q2, q3;       // Quaternion (w, x, y, z)
    // float roll, pitch, yaw;   // Euler angles (degrees or radians) - calculate from quaternion if needed
} FusionData_T;

// Data output from the navigation module
typedef struct {
    uint64_t timestamp_us;          // System timestamp (microseconds) matching fusion data if possible
    float vertical_accel_m_s2;    // Gravity compensated vertical acceleration
    float vertical_velocity_m_s;  // Estimated vertical velocity
    float vertical_distance_m;    // Estimated vertical distance travelled
    bool is_stationary;           // Flag indicating if elevator is considered stationary (for ZUPT)
    DisplayStatus display_state; // UI motion state
} NavData_T;

// Combined data structure holding all relevant output
typedef struct {
    FusionData_T fusion_data;
    NavData_T nav_data;
    // Add other relevant system data if needed
    // e.g., uint32_t system_uptime_ms;
} OUTPUT_DATA_T;


#endif // COMMON_DEFS_H_