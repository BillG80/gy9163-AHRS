#include <stdio.h>
#include "common/common_defs.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h> // For usleep, close
#include <fcntl.h>  // For O_RDWR
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <bcm2835.h>
#include <math.h>   // For sqrtf, NAN, M_PI
#include <signal.h>
#include <errno.h> // Include errno
#include <float.h>

#include "core/fusion/sensor_fusion.h"
#include "drivers/icm20948/icm20948.h"
#include "drivers/icm20948/icm20948_regs.h"
#include "drivers/ms5611/ms5611.h"
#include "logging/file_logger.h" // Include
#include "common/common_defs.h"  // Include (Contains FusionData_T, NavData_T)
#include "core/ahrs/ekf_ahrs.h"
#include "core/ahrs/madgwick_ahrs.h"
#include "core/ahrs/mahony_ahrs.h"
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "drivers/gpio/gpio.h"
#include "display/screens.h"
#include "core/fusion/calibration.h"
#include "fonts/SansCondensed.h"

//#define FONT_HEIGHT SansCondensed_FontHeight
//#define FONT_WIDTH SansCondensed_FontWidth
#define AHRS_BETA 0.1f
#define MAHONY_KP 2.0f
#define MAHONY_KI 0.0f
// --- Configuration ---
#define I2C_BUS              "/dev/i2c-1"
#define MS5611_I2C_ADDR      MS5611_ADDR_CSB_LOW // Or MS5611_ADDR_CSB_HIGH if CSB is high
#define MS5611_ADDR_CSB_LOW   0x77              // default address if CSB pin low
#define MS5611_ADDR_CSB_HIGH  0x76              // alternative address if CSB pin high
#define SPI_CLOCK_DIV        BCM2835_SPI_CLOCK_DIVIDER_64 // ~3.9MHz (250MHz / 64)
// --- Bus Configuration ---
// ICM20948: SPI0 (hardware SPI), chip select CE0 (GPIO8 => ICM20948_SPI_CS_PIN)
// MS5611: I2C0 (/dev/i2c-0 => I2C_BUS)
// Use RPI_GPIO_P1_24 from gpio.h for CE0, GPIO 8 (Confirm this matches your wiring)
#define ICM20948_SPI_CS_PIN  RPI_V2_GPIO_P1_24
#define HOLD_TIME_BLE_MS     3000           // ≥3s => BLE advertising
#define HOLD_TIME_CAL_MS     6000           // ≥6s => full IMU calibration
// Define other SPI pins if necessary (though bcm2835 handles them usually)
// #define SPI_MISO_PIN    RPI_GPIO_P1_21  // GPIO 9
// #define SPI_MOSI_PIN    RPI_GPIO_P1_19  // GPIO 10
// #define SPI_CLK_PIN     RPI_GPIO_P1_23  // GPIO 11

// --- Target Rates (Hz) ---
#define TARGET_RATE_ACCEL_GYRO 500.0
#define TARGET_RATE_MAG        100.0
#define TARGET_RATE_PRESS_TEMP 50.0
#define TARGET_RATE_AHRS_HZ    200.0

// Calculate loop period based on highest effective rate (~500Hz)
// Note: Actual ICM Accel/Gyro ODR is 1.125kHz, we'll read every loop but process less often
#define LOOP_RATE_HZ   1125.0 // Match default ICM Accel/Gyro ODR
#define LOOP_PERIOD_S  (1.0 / LOOP_RATE_HZ)
#define LOOP_PERIOD_US (LOOP_PERIOD_S * 1e6)

// --- AHRS Configuration ---
#define SAMPLE_RATE_HZ 100.0f         // Target sensor update rate
#define SAMPLE_PERIOD_S (1.0f / SAMPLE_RATE_HZ)
typedef enum {
    AHRS_NONE,
    AHRS_EKF,
    AHRS_MADGWICK,
    AHRS_MAHONY
} ahrs_filter_type_t;
static ahrs_filter_type_t selected_filter = AHRS_MADGWICK; // Default to Madgwick
#if defined(USE_AHRS_MADGWICK)
#define AHRS_BETA 0.041f            // Algorithm gain beta (Madgwick suggested default: sqrt(3/4) * omega_max_gyro_drift)
                                  // Common values: 0.033 to 0.1
#elif defined(USE_AHRS_MAHONY)
#define MAHONY_KP 2.0f
#define MAHONY_KI 0.0f
#endif
#define RAD_TO_DEG (180.0f / M_PI)
#define GRAVITY_MSS 9.80665f // Standard gravity in m/s^2

// --- Sensor Bias Storage ---
// These are populated from calibration file at startup
static float gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f}; // Bias values for Gx, Gy, Gz in degrees/sec
static float accel_bias_g[3] = {0.0f, 0.0f, 0.0f};
static float mag_bias_ut[3]  = {0.0f, 0.0f, 0.0f};
static float mag_scale[3]    = {1.0f, 1.0f, 1.0f};

// Calibration parameter structure (from calibration.h)
static calib_params_t calib_params;

// Loads calibration data from persistent storage and populates fusion module globals
// Returns 0 on success, nonzero on failure
static int load_calibration_results(const char* filename) {
    if (load_calibration(filename, &calib_params) != 0) {
        fprintf(stderr, "[Fusion] Failed to load calibration file: %s\n", filename);
        return -1;
    }
    // Copy biases/scales to fusion globals
    for (int i = 0; i < 3; ++i) {
        gyro_bias_dps[i] = calib_params.gyro_bias[i];
        accel_bias_g[i] = calib_params.accel_bias[i];
        mag_bias_ut[i] = calib_params.mag_bias[i];
        mag_scale[i] = calib_params.mag_scale[i][i]; // Assume diagonal for now
    }
    return 0;
}

// --- Global Device Instances ---
icm20948_dev_t icm_dev;
static ms5611_dev_t   ms_dev; 
static ekf_ahrs_t *ekf_instance = NULL;
static MadgwickAHRS *madgwick_instance = NULL;
static MahonyAHRS *mahony_instance = NULL; 

// --- Global variable for signal handling ---
volatile sig_atomic_t exit_request = 0;
volatile sig_atomic_t sigint_received = 0;

// --- Global i2c_fd variable
static int i2c_fd = -1;  // File descriptor for I2C bus

void sigint_handler(int sig) {
    (void)sig; // Unused parameter
    sigint_received = 1;
}

// --- Helper Functions for Quaternion Math ---

// Quaternion conjugate q* = [w, -x, -y, -z]
void quat_conjugate(float q_in[4], float q_out[4]) {
    q_out[0] =  q_in[0];
    q_out[1] = -q_in[1];
    q_out[2] = -q_in[2];
    q_out[3] = -q_in[3];
}

// Quaternion multiplication: q_res = q1 * q2
// q = [w, x, y, z]
void quat_multiply(float q1[4], float q2[4], float q_res[4]) {
    q_res[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]; // w
    q_res[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]; // x
    q_res[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]; // y
    q_res[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]; // z
}

// Rotate a 3D vector by a quaternion: v_rotated = q * v * q_conjugate
// Assumes v = [0, vx, vy, vz] as a pure quaternion
void quat_rotate_vector(float q[4], float v[3], float v_rotated[3]) {
    float q_conj[4];
    float v_quat[4] = {0.0f, v[0], v[1], v[2]};
    float temp_quat[4];
    float res_quat[4];

    quat_conjugate(q, q_conj);
    quat_multiply(q, v_quat, temp_quat);
    quat_multiply(temp_quat, q_conj, res_quat);

    v_rotated[0] = res_quat[1]; // x
    v_rotated[1] = res_quat[2]; // y
    v_rotated[2] = res_quat[3]; // z
}

// --- Signal Handler for SIGINT (Ctrl+C) ---
void handle_sigint(int sig) {
    (void)sig; // Unused parameter
    exit_request = 1;
}

// --- Platform Specific Implementations (Combined) ---

// Shared Delay (using bcm2835)
void platform_delay_us(uint32_t us) {
    bcm2835_delayMicroseconds(us);
}

// ICM20948 SPI CS Control (using bcm2835)
void platform_icm_cs_assert(void) {
    bcm2835_gpio_write(ICM20948_SPI_CS_PIN, LOW);
    bcm2835_delayMicroseconds(5); // Small delay after assert
}

void platform_icm_cs_deassert(void) {
    bcm2835_gpio_write(ICM20948_SPI_CS_PIN, HIGH);
    bcm2835_delayMicroseconds(5); // Small delay after deassert
}

// ICM20948 SPI Transfer (using bcm2835)
#define MAX_SPI_TRANSFER_SIZE 256
static uint8_t dummy_rx_buffer[MAX_SPI_TRANSFER_SIZE]; // Static buffer for dummy reads
icm20948_return_code_t platform_spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len) {
    if (len > MAX_SPI_TRANSFER_SIZE) {
        fprintf(stderr, "Error: SPI transfer size %u exceeds max %d\n", len, MAX_SPI_TRANSFER_SIZE);
        return ICM20948_RET_BAD_ARG;
    }
    uint8_t* rx_ptr = rx ? rx : dummy_rx_buffer;
    bcm2835_spi_transfernb((char*)tx, (char*)rx_ptr, len);
    return ICM20948_RET_OK;
}

// MS5611 I2C Platform Read (Matches ms5611_intf_t signature)
// NOTE: The 'reg' argument for I2C is usually the command byte written *before* reading.
ms5611_return_code_t platform_i2c_read(uint8_t reg, uint8_t *read_buf, uint32_t read_count) {
    if (i2c_fd < 0) return MS5611_RET_ERROR; // Use defined error code

    // Set target device address (assuming MS5611_I2C_ADDR is set correctly)
    if (ioctl(i2c_fd, I2C_SLAVE, MS5611_I2C_ADDR) < 0) {
        perror("I2C Read: Failed to set slave address");
        return MS5611_RET_ERROR; // Use defined error code
    }

    // Write the register/command byte first
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("I2C Read: Write register failed");
        return MS5611_RET_ERROR; // Use defined error code
    }

    // Perform the read
    if (read(i2c_fd, read_buf, read_count) != (ssize_t)read_count) { // Cast to ssize_t for comparison
        perror("I2C Read: Read failed");
        return MS5611_RET_ERROR; // Use defined error code
    }
    return MS5611_RET_OK;
}

// MS5611 I2C Platform Write (Matches ms5611_intf_t signature)
// NOTE: The 'reg' argument for I2C usually means writing the command byte *followed* by data.
ms5611_return_code_t platform_i2c_write(uint8_t reg, const uint8_t *write_buf, uint32_t write_count) {
     if (i2c_fd < 0) return MS5611_RET_ERROR; // Use defined error code

    // Set target device address
    if (ioctl(i2c_fd, I2C_SLAVE, MS5611_I2C_ADDR) < 0) {
        perror("I2C Write: Failed to set slave address");
        return MS5611_RET_ERROR; // Use defined error code
    }

    // Combine register/command and data into a single buffer for writing
    uint8_t combined_buf[write_count + 1];
    combined_buf[0] = reg;
    if (write_buf && write_count > 0) {
        memcpy(&combined_buf[1], write_buf, write_count);
    }
    size_t total_write_count = write_count + 1; // +1 for the register byte

    if (write(i2c_fd, combined_buf, total_write_count) != (ssize_t)total_write_count) { // Cast for comparison
        perror("I2C Write: Write failed");
        return MS5611_RET_ERROR; // Use defined error code
    }

    return MS5611_RET_OK;
}

// Forward declarations for MS5611 functions that might be missing in the header
ms5611_return_code_t ms5611_read_pressure(ms5611_dev_t *dev, uint32_t *pressure_raw);
ms5611_return_code_t ms5611_read_temperature(ms5611_dev_t *dev, uint32_t *temp_raw);
ms5611_return_code_t ms5611_calculate(ms5611_dev_t *dev, uint32_t d1, uint32_t d2, int32_t *pressure, int32_t *temperature);
ms5611_return_code_t ms5611_read_adc(ms5611_dev_t *dev, uint8_t command, uint32_t *result);

// Forward declare helper functions to fix implicit declaration errors
uint64_t get_timestamp_us(void);
void quaternion_to_gravity_vector(float q0, float q1, float q2, float q3, float* gx, float* gy, float* gz);

// --- Initialization Functions ---
// Forward declaration to avoid implicit declaration warning
void cleanup_hw(void);

int setup_spi(void) {
    printf("Setting up SPI...\n");
    // Assumes bcm2835_init() is called globally

    // --- Setup GPIO for Manual CS ---
    bcm2835_gpio_fsel(ICM20948_SPI_CS_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(ICM20948_SPI_CS_PIN, HIGH); // Start inactive
    printf("✓ Configured GPIO %d for ICM20948 manual CS\n", ICM20948_SPI_CS_PIN);

    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3); // ICM20948 uses Mode 0 or 3. Mode 3 is often more robust.
    bcm2835_spi_setClockDivider(SPI_CLOCK_DIV);
    // CS handled manually, no need to set CS pin or polarity here
    printf("✓ Configured SPI parameters (Mode 3, ClkDiv %d)\n", SPI_CLOCK_DIV);

    return 0; // Success
}

int setup_i2c(void) {
    printf("Setting up I2C...\n");
    if ((i2c_fd = open(I2C_BUS, O_RDWR)) < 0) {
        perror("Failed to open I2C bus");
        return -1;
    }
    printf("✓ Opened I2C bus %s\n", I2C_BUS);
    return 0; // Success
}

int initialize_sensors(ssd1306_handle_t* disp) {
    printf("Initializing Sensors...\n");

    // --- Initialize MS5611 (I2C) FIRST ---
    // Only allow display updates if in AHRS state

    printf("  Initializing MS5611 via I2C...\n");
    memset(&ms_dev, 0, sizeof(ms_dev));
    // Assign platform functions matching ms5611_intf_t signature
    ms_dev.intf.read = platform_i2c_read;
    ms_dev.intf.write = platform_i2c_write;
    ms_dev.intf.delay_us = platform_delay_us;
    // Note: The I2C address is handled within the platform read/write functions now
    ms5611_return_code_t ms_ret = ms5611_init(&ms_dev, platform_i2c_read, platform_i2c_write, platform_delay_us);
    if (ms_ret != MS5611_RET_OK) {
        fprintf(stderr, "❌ Failed to initialize MS5611, error: %d\n", ms_ret);
        return -1;
    }
    printf("  ✓ MS5611 initialized successfully.\n");

    // --- Initialize ICM20948 (SPI + Aux I2C) SECOND ---
    printf("  Initializing ICM20948 via SPI...\n");
    memset(&icm_dev, 0, sizeof(icm_dev));
    icm_dev.intf.delay_us = platform_delay_us;
    icm_dev.intf.cs_assert = platform_icm_cs_assert;
    icm_dev.intf.cs_deassert = platform_icm_cs_deassert;
    icm_dev.intf.spi_transfer = platform_spi_transfer;
    printf("Debug: Calling icm20948_init\n");
    icm20948_return_code_t icm_ret = icm20948_init(&icm_dev);
    if (icm_ret != ICM20948_RET_OK) {
        printf("Error in icm20948_init: %d\n", icm_ret);
    } else {
        printf("Debug: icm20948_init completed\n");
    }
    if (sigint_received) return 0;
    if (icm_ret != ICM20948_RET_OK) {
        fprintf(stderr, "❌ Failed to initialize ICM20948, error: %d\n", icm_ret);
        return -1;
    }
    printf("  ✓ ICM20948 initialized successfully.\n");

    // --- Calibration and bias computation removed: handled by calibration.c ---
// --- Only sensor hardware initialization and AHRS selection remain ---
// Load calibration data (biases and scales) from calibration module or config file
if (load_calibration_results("calibration.json") != 0) {
    fprintf(stderr, "[Fusion] WARNING: Using default (zero) biases and scales!\n");
}
printf("Sensor hardware initialized. Calibration data loaded.\n");

    // --- Initialize AHRS ---
    printf("Initializing AHRS...\n");
    switch (selected_filter) {
        case AHRS_EKF:
            printf("Initializing EKF AHRS...\n");
            ekf_instance = ekf_ahrs_create(1.0f / TARGET_RATE_AHRS_HZ);
            if (!ekf_instance) { fprintf(stderr, "ERROR: EKF init failed!\n"); cleanup_hw(); return 1; }
            printf("✓ EKF AHRS Initialized\n");
            break;
        case AHRS_MADGWICK:
            printf("Initializing Madgwick AHRS...\n");
            madgwick_instance = create_madgwick_ahrs(TARGET_RATE_AHRS_HZ, AHRS_BETA);
            if (!madgwick_instance) { fprintf(stderr, "ERROR: Madgwick init failed!\n"); cleanup_hw(); return 1; }
            printf("✓ Madgwick AHRS Initialized\n");
            break;
        case AHRS_MAHONY:
            printf("Initializing Mahony AHRS...\n");
            mahony_ahrs_init(mahony_instance, MAHONY_KP, MAHONY_KI, TARGET_RATE_AHRS_HZ);
            printf("✓ Mahony AHRS Initialized\n");
            break;
        default:
            fprintf(stderr, "ERROR: No AHRS filter selected or invalid filter type.\n");
            return 1;
    }

    // Calibration display and summary code removed. No display or buffer code should be present here.
    return 0; // Success;
}


void cleanup_hw(void) {
    printf("\nCleaning up...\n");
    // Attempt to reset the ICM20948 device first
    printf("Attempting ICM20948 device reset...\n");
    icm20948_return_code_t reset_ret = icm20948_device_reset(&icm_dev);
    if (reset_ret == ICM20948_RET_OK) {
        printf("✓ ICM20948 device reset command sent successfully.\n");
    } else {
        printf("WARN: Failed to send ICM20948 device reset command (err: %d).\n", reset_ret);
        // Continue cleanup regardless
    }

    if (i2c_fd >= 0) {
        close(i2c_fd);
        printf("✓ Closed I2C device.\n");
    }
    bcm2835_spi_end();
    printf("✓ Ended SPI.\n");
    // Reset CS GPIO to input
    bcm2835_gpio_fsel(ICM20948_SPI_CS_PIN, BCM2835_GPIO_FSEL_INPT);
    printf("✓ Reset GPIO %d (ICM CS) to INPUT\n", ICM20948_SPI_CS_PIN);
    bcm2835_close();
    printf("✓ Closed bcm2835 library.\n");
    printf("✓ Cleanup complete\n");
}

// Function to process sensor data and update fusion data
bool run_sensor_fusion_cycle(FusionData_T* fusion_data) {
    if (!fusion_data) return false;
    // Capture current timestamp for navigation
    fusion_data->timestamp_us = bcm2835_st_read();
    
    // --- Read ICM20948 Accel/Gyro/Temp ---
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t mx_raw, my_raw, mz_raw;
    float temp_icm_c = 0.0f;
    
    // Read accelerometer data
    icm20948_return_code_t ret = icm20948_read_accel(&icm_dev, &ax_raw, &ay_raw, &az_raw);
    if (ret != ICM20948_RET_OK) {
        fprintf(stderr, "Failed to read accelerometer data: %d\n", ret);
        return false;
    }
    
    // Read gyroscope data
    ret = icm20948_read_gyro(&icm_dev, &gx_raw, &gy_raw, &gz_raw);
    if (ret != ICM20948_RET_OK) {
        fprintf(stderr, "Failed to read gyroscope data: %d\n", ret);
        return false;
    }
    
    // Read temperature
    ret = icm20948_read_temp(&icm_dev, &temp_icm_c);
    if (ret != ICM20948_RET_OK) {
        fprintf(stderr, "Failed to read ICM temperature: %d\n", ret);
        // Continue anyway, temperature is not critical
    }
    
    // Read magnetometer data
    ret = icm20948_read_mag(&icm_dev, &mx_raw, &my_raw, &mz_raw);
    bool mag_valid = (ret == ICM20948_RET_OK);
    // --- Diagnostics: Print raw magnetometer values ---
    printf("[DIAG] Raw Mag: mx_raw=%d, my_raw=%d, mz_raw=%d, valid=%d\n", mx_raw, my_raw, mz_raw, mag_valid);
    
    // --- Convert raw values to physical units ---
    // --- Bias correction ---
    float accel_vec[3] = {
        (float)ax_raw / 16384.0f - accel_bias_g[0],
        (float)ay_raw / 16384.0f - accel_bias_g[1],
        (float)az_raw / 16384.0f - accel_bias_g[2]
    };
    float gyro_vec[3] = {
        (float)gx_raw / 131.0f - gyro_bias_dps[0],
        (float)gy_raw / 131.0f - gyro_bias_dps[1],
        (float)gz_raw / 131.0f - gyro_bias_dps[2]
    };
    float mag_vec[3] = {0.0f, 0.0f, 0.0f};
    if (mag_valid) {
        mag_vec[0] = (float)mx_raw * 0.15f - mag_bias_ut[0];
        mag_vec[1] = (float)my_raw * 0.15f - mag_bias_ut[1];
        mag_vec[2] = (float)mz_raw * 0.15f - mag_bias_ut[2];
    }
    // --- Diagnostics: Print bias-corrected magnetometer values ---
    printf("[DIAG] Bias-corrected Mag: mx=%.3f, my=%.3f, mz=%.3f\n", mag_vec[0], mag_vec[1], mag_vec[2]);

    // --- Scale correction (matrix multiplication) ---
    float accel_corr[3] = {0}, gyro_corr[3] = {0}, mag_corr[3] = {0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            accel_corr[i] += calib_params.accel_scale[i][j] * accel_vec[j];
            gyro_corr[i]  += calib_params.gyro_scale[i][j]  * gyro_vec[j];
            mag_corr[i]   += calib_params.mag_scale[i][j]   * mag_vec[j];
        }
    }
    float ax_g = accel_corr[0];
    float ay_g = accel_corr[1];
    float az_g = accel_corr[2];
    float gx_dps = gyro_corr[0];
    float gy_dps = gyro_corr[1];
    float gz_dps = gyro_corr[2];
    // --- Diagnostics: Print scale-corrected magnetometer values ---
    printf("[DIAG] Scale-corrected Mag: mx=%.3f, my=%.3f, mz=%.3f\n", mag_corr[0], mag_corr[1], mag_corr[2]);
    float mag_corr_norm = sqrtf(mag_corr[0]*mag_corr[0] + mag_corr[1]*mag_corr[1] + mag_corr[2]*mag_corr[2]);
    printf("[DIAG] Scale-corrected Mag Norm: %.3f\n", mag_corr_norm);
    // --- Apply accel-to-mag alignment matrix (R_am) ---
    float mag_aligned[3] = {0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mag_aligned[i] += calib_params.R_am[i][j] * mag_corr[j];
        }
    }
    float mx_ut = mag_aligned[0];
    float my_ut = mag_aligned[1];
    float mz_ut = mag_aligned[2];
    // --- Diagnostics: Print aligned magnetometer values and norm ---
    float mag_aligned_norm = sqrtf(mx_ut*mx_ut + my_ut*my_ut + mz_ut*mz_ut);
    printf("[DIAG] Aligned Mag: mx=%.3f, my=%.3f, mz=%.3f, norm=%.3f\n", mx_ut, my_ut, mz_ut, mag_aligned_norm);
    
    // --- Read MS5611 Pressure/Temperature ---
    uint32_t d1_raw, d2_raw;
    int32_t pressure_pa = 0;
    int32_t temp_ms5611_degC_x100 = 0; // Compensated temperature (degC * 100)
    float temp_ms5611_c = 0.0f;
    
    // Read pressure (D1)
    ms5611_return_code_t ms_ret = ms5611_read_pressure(&ms_dev, &d1_raw);
    if (ms_ret != MS5611_RET_OK) {
        fprintf(stderr, "Failed to read MS5611 pressure: %d\n", ms_ret);
        return false;
    }
    
    // Read temperature (D2)
    ms_ret = ms5611_read_temperature(&ms_dev, &d2_raw);
    if (ms_ret != MS5611_RET_OK) {
        fprintf(stderr, "Failed to read MS5611 temperature: %d\n", ms_ret);
        return false;
    }
    
    // Calculate compensated pressure and temperature
    ms_ret = ms5611_calculate(&ms_dev, d1_raw, d2_raw, &pressure_pa, &temp_ms5611_degC_x100);
    if (ms_ret != MS5611_RET_OK) {
        fprintf(stderr, "Failed to calculate MS5611 pressure/temperature: %d\n", ms_ret);
        return false;
    }
    
    temp_ms5611_c = (float)temp_ms5611_degC_x100 / 100.0f;
    
    // Store temperature in fusion data
    fusion_data->temperature_c = temp_ms5611_c;
    fusion_data->pressure_pa = (float)pressure_pa;
    
    // --- Update AHRS ---
    // Convert gyro to rad/s for AHRS
    float gx_rad = gx_dps * (M_PI / 180.0f);
    float gy_rad = gy_dps * (M_PI / 180.0f);
    float gz_rad = gz_dps * (M_PI / 180.0f);
    
    float current_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Default to identity
    
    switch (selected_filter) {
        case AHRS_NONE:
            // Just use identity quaternion
            break;
            
        case AHRS_EKF:
            if (ekf_instance) {
                ekf_ahrs_predict(ekf_instance, gx_rad, gy_rad, gz_rad);
                ekf_ahrs_update_accel(ekf_instance, ax_g, ay_g, az_g);
                if (mag_valid) {
                    printf("[DIAG] Using magnetometer for EKF yaw correction.\n");
                    ekf_ahrs_update_mag(ekf_instance, mx_ut, my_ut, mz_ut);
                } else {
                    printf("[DIAG] Magnetometer NOT used for EKF (invalid data).\n");
                }
                current_q[0] = ekf_instance->q0;
                current_q[1] = ekf_instance->q1;
                current_q[2] = ekf_instance->q2;
                current_q[3] = ekf_instance->q3;
            }
            break;
            
        case AHRS_MADGWICK:
            if (madgwick_instance) {
                if (mag_valid) {
                    printf("[DIAG] Using magnetometer for Madgwick yaw correction.\n");
                    madgwick_ahrs_update(madgwick_instance,
                                         gx_rad, gy_rad, gz_rad,
                                         ax_g, ay_g, az_g,
                                         mx_ut, my_ut, mz_ut);
                } else {
                    printf("[DIAG] Magnetometer NOT used for Madgwick (invalid data).\n");
                    madgwick_ahrs_update_imu(madgwick_instance,
                                             gx_rad, gy_rad, gz_rad,
                                             ax_g, ay_g, az_g);
                }
                current_q[0] = madgwick_instance->q0;
                current_q[1] = madgwick_instance->q1;
                current_q[2] = madgwick_instance->q2;
                current_q[3] = madgwick_instance->q3;
            }
            break;
            
        case AHRS_MAHONY:
            if (mahony_instance) {
                if (mag_valid) {
                    printf("[DIAG] Using magnetometer for Mahony yaw correction.\n");
                    mahony_ahrs_update(mahony_instance,
                                       gx_rad, gy_rad, gz_rad,
                                       ax_g, ay_g, az_g,
                                       mx_ut, my_ut, mz_ut);
                } else {
                    printf("[DIAG] Magnetometer NOT used for Mahony (invalid data).\n");
                    mahony_ahrs_update_imu(mahony_instance,
                                           gx_rad, gy_rad, gz_rad,
                                           ax_g, ay_g, az_g);
                }
                current_q[0] = mahony_instance->q0;
                current_q[1] = mahony_instance->q1;
                current_q[2] = mahony_instance->q2;
                current_q[3] = mahony_instance->q3;
            }
            break;
    }
    
    // --- Calculate Vertical Acceleration ---
    // Gravity vector in sensor frame
    float grav_x, grav_y, grav_z;
    quaternion_to_gravity_vector(current_q[0], current_q[1], current_q[2], current_q[3], 
                                 &grav_x, &grav_y, &grav_z);
    
    // Gravity-compensated acceleration
    float acc_x_mss = ax_g * GRAVITY_MSS;
    float acc_y_mss = ay_g * GRAVITY_MSS;
    float acc_z_mss = az_g * GRAVITY_MSS;
    
    // Remove gravity component
    acc_x_mss -= grav_x * GRAVITY_MSS;
    acc_y_mss -= grav_y * GRAVITY_MSS;
    acc_z_mss -= grav_z * GRAVITY_MSS;

    // --- Diagnostics: Print norms and values after gravity compensation ---
    float accel_corr_norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    printf("[DIAG] Calibrated Accel (g): x=%.6f y=%.6f z=%.6f norm=%.6f\n", ax_g, ay_g, az_g, accel_corr_norm);
    float grav_norm = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    printf("[DIAG] Gravity Vector: x=%.6f y=%.6f z=%.6f norm=%.6f\n", grav_x, grav_y, grav_z, grav_norm);
    float lin_acc_norm = sqrtf(acc_x_mss*acc_x_mss + acc_y_mss*acc_y_mss + acc_z_mss*acc_z_mss);
    printf("[DIAG] Linear Accel (m/s^2): x=%.6f y=%.6f z=%.6f norm=%.6f\n", acc_x_mss, acc_y_mss, acc_z_mss, lin_acc_norm);
 
    // Project acceleration onto gravity vector to get vertical component
    float vertical_accel_mss = -(acc_x_mss * grav_x + acc_y_mss * grav_y + acc_z_mss * grav_z);
    printf("[DIAG] Vertical Accel (m/s^2): %.6f\n", vertical_accel_mss);

    
    // --- Calculate Altitude ---
    // Standard barometric formula: h = (1 - (P/P0)^(1/5.257)) * (T + 273.15) / 0.0065
    const float P0 = 101325.0f; // Standard pressure at sea level (Pa)
    const float T0 = 288.15f;   // Standard temperature at sea level (K)
    
    float altitude_m = T0 / 0.0065f * (1.0f - powf((float)pressure_pa / P0, 0.190295f));
    
    // --- Update Fusion Data ---
    fusion_data->accel_x = ax_g;
    fusion_data->accel_y = ay_g;
    fusion_data->accel_z = az_g;
    fusion_data->gyro_x = gx_dps;
    fusion_data->gyro_y = gy_dps;
    fusion_data->gyro_z = gz_dps;
    if (mag_valid) {
        fusion_data->mag_x = mx_ut;
        fusion_data->mag_y = my_ut;
        fusion_data->mag_z = mz_ut;
    }
    fusion_data->altitude_m = altitude_m;
    fusion_data->q0 = current_q[0];
    fusion_data->q1 = current_q[1];
    fusion_data->q2 = current_q[2];
    fusion_data->q3 = current_q[3];
    
    // Update navigation data in the global OUTPUT_DATA_T structure
    NavData_T* nav_data = &((OUTPUT_DATA_T*)fusion_data)->nav_data;
    
    // Simple integration for vertical velocity (with high-pass filter)
    static float prev_velocity = 0.0f;
    static float prev_accel = 0.0f;
    static uint64_t prev_time = 0;
    
    uint64_t current_time = fusion_data->timestamp_us;
    float dt = (prev_time == 0) ? 0.01f : (float)(current_time - prev_time) / 1000000.0f;
    prev_time = current_time;
    
    // Apply high-pass filter to acceleration to remove bias
    const float alpha = 0.8f;
    float filtered_accel = alpha * (vertical_accel_mss - prev_accel + prev_velocity);
    prev_accel = vertical_accel_mss;
    
    // Integrate velocity
    float velocity = prev_velocity + filtered_accel * dt;
    prev_velocity = velocity;
    
    // Detect if stationary (low acceleration and velocity)
    bool is_stationary = (fabsf(vertical_accel_mss) < 0.1f && fabsf(velocity) < 0.05f);
    
    // If stationary, reset velocity to zero (zero velocity update)
    if (is_stationary) {
        velocity = 0.0f;
        prev_velocity = 0.0f;
    }
    
    // Update navigation data
    nav_data->timestamp_us = current_time;
    nav_data->vertical_accel_m_s2 = vertical_accel_mss; // Already in m/s^2
    nav_data->vertical_velocity_m_s = velocity;
    nav_data->vertical_distance_m = altitude_m;  // Using barometric altitude
    nav_data->is_stationary = is_stationary;
    
    return true;
}

// Helper function to get current timestamp in microseconds
uint64_t get_timestamp_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// Function to convert quaternion to gravity vector
void quaternion_to_gravity_vector(float q0, float q1, float q2, float q3, float* gx, float* gy, float* gz) {
    *gx = 2.0f * (q1 * q3 - q0 * q2);
    *gy = 2.0f * (q0 * q1 + q2 * q3);
    *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

// --- Sensor Fusion Application ---
int sensor_fusion_main(int argc, char *argv[]) {
    printf("=== Sensor Fusion Application ===\n");

    // --- Initialize BCM2835 Library ---
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835_init failed. Are you running as root??\n");
        return 1;
    }
    printf("✓ Initialized bcm2835 library\n");

    signal(SIGINT, sigint_handler);

    // --- Button-held gesture to select BLE or calibration ---
    bcm2835_gpio_fsel(CALIB_BUTTON_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud (CALIB_BUTTON_PIN, BCM2835_GPIO_PUD_DOWN);
    bcm2835_delay(50);
    if (bcm2835_gpio_lev(CALIB_BUTTON_PIN)) {
        uint64_t t0 = get_timestamp_us();
        // wait until release or max hold
        while (bcm2835_gpio_lev(CALIB_BUTTON_PIN)) {
            if ((get_timestamp_us() - t0)/1000 >= HOLD_TIME_CAL_MS) break;
        }
        uint32_t dt = (uint32_t)((get_timestamp_us() - t0)/1000);
        if (dt >= HOLD_TIME_CAL_MS) {
            printf("Holding >%ums: running full IMU calibration...\n", dt);
            // Initialize I2C bus and display for calibration
            i2c_config_t display_cfg = { .bus_num = 0, .speed = I2C_SPEED_400KHZ };
            i2c_bus_handle_t display_bus = NULL;
            if (i2c_bus_init(&display_cfg, &display_bus) != STATUS_OK) {
                fprintf(stderr, "Display I2C bus init failed\n");
                bcm2835_close();
                return 1;
            }
            ssd1306_handle_t* disp = NULL;
            if (ssd1306_init(display_bus, 0x3C, 128, 64, &disp) != STATUS_OK) {
                fprintf(stderr, "SSD1306 init failed\n");
                i2c_bus_cleanup(display_bus);
                bcm2835_close();
                return 1;
            }
            calib_params_t params = {0};
            run_full_calibration(disp, &params);
            printf("Calibration done.\n");
            // Clean up display after calibration
            ssd1306_cleanup(disp);
            i2c_bus_cleanup(display_bus);
        } else if (dt >= HOLD_TIME_BLE_MS) {
            printf("Holding %ums: starting BLE advertising...\n", dt);
            // TODO: call BLE advertising init here
        }
        bcm2835_close();
        return 0;
    }

    // --- Parse Command Line Arguments ---
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <FILTER_TYPE> or --calibrate\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "EKF") == 0) {
        selected_filter = AHRS_EKF;
    } else if (strcmp(argv[1], "MADGWICK") == 0) {
        selected_filter = AHRS_MADGWICK;
    } else if (strcmp(argv[1], "MAHONY") == 0) {
        selected_filter = AHRS_MAHONY;
    } else {
        fprintf(stderr, "ERROR: Invalid filter type '%s'. Use EKF, MADGWICK, or MAHONY.\n", argv[1]);
        return 1;
    }

    // --- Setup Hardware Interfaces ---
    if (setup_i2c() != 0) {
        bcm2835_close();
        return 1;
    }
    if (setup_spi() != 0) {
        cleanup_hw();
        return 1;
    }

    // --- Initialize Sensors ---
    // Initialize I2C bus and display for sensors
    i2c_config_t display_cfg = { .bus_num = 0, .speed = I2C_SPEED_400KHZ };
    i2c_bus_handle_t display_bus = NULL;
    if (i2c_bus_init(&display_cfg, &display_bus) != STATUS_OK) {
        fprintf(stderr, "Display I2C bus init failed\n");
        bcm2835_close();
        return 1;
    }
    ssd1306_handle_t* disp = NULL;
    if (ssd1306_init(display_bus, 0x3C, 128, 64, &disp) != STATUS_OK) {
        fprintf(stderr, "SSD1306 init failed\n");
        i2c_bus_cleanup(display_bus);
        bcm2835_close();
        return 1;
    }
    if (initialize_sensors(disp) != 0) {
        ssd1306_cleanup(disp);
        i2c_bus_cleanup(display_bus);
        cleanup_hw();
        return 1;
    }

    // Register the signal handler for SIGINT
    signal(SIGINT, handle_sigint);

    printf("\nStarting main sensor loop (Rate: %.1f Hz, Period: %.1f us)...\n", LOOP_RATE_HZ, LOOP_PERIOD_US);
    printf("Press Ctrl+C to exit.\n\n");
    printf("Loop | Accel (g)      | Gyro (dps)     | Mag (uT)       | Pressure(hPa)| Temp(C) | ICM_Temp(C) | S (m/s) | Roll | Pitch | Yaw \n");
    printf("-----+----------------+----------------+----------------+--------------+---------+-------------+---------+------+-------+-----\n");

    // --- Timing and Rate Control Variables ---
    struct timespec t_start, t_end, t_loop_start, t_sleep;
    double loop_duration_s = 0;
    uint64_t loop_count = 0;
    static const uint32_t loops_per_mag_read = (uint32_t)(LOOP_RATE_HZ / TARGET_RATE_MAG + 0.5f);
    static const uint32_t loops_per_ahrs_update = (uint32_t)(LOOP_RATE_HZ / TARGET_RATE_AHRS_HZ + 0.5f);
    static const uint32_t loops_per_press_temp_read = (uint32_t)(LOOP_RATE_HZ / TARGET_RATE_PRESS_TEMP + 0.5f);

    // --- Sensor Data Variables ---
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t mx_raw, my_raw, mz_raw;
    uint32_t d1_raw, d2_raw; // MS5611 raw press/temp
    int32_t pressure_pa = 0;         // Compensated pressure (Pascals)
    int32_t temp_ms5611_degC_x100 = 0; // Compensated temperature (degC * 100)
    float temp_icm_c = NAN;

    float ax_g = NAN, ay_g = NAN, az_g = NAN;
    float gx_dps = NAN, gy_dps = NAN, gz_dps = NAN;
    float mx_ut = NAN, my_ut = NAN, mz_ut = NAN;
    float temp_ms5611_c = NAN;

    float roll_deg = NAN, pitch_deg = NAN, yaw_deg = NAN;

    float vertical_accel_mss = 0.0f;
    float vertical_velocity_ms = 0.0f;

    float ahrs_dt = 1.0f / TARGET_RATE_AHRS_HZ; // Time step for AHRS and integration

    // Declare data structures for logging
    FusionData_T fusion_data;
    NavData_T nav_data;

    // --- Main Loop ---
    clock_gettime(CLOCK_MONOTONIC, &t_start); // Reference time

    while (!exit_request) { // Loop until exit request
        clock_gettime(CLOCK_MONOTONIC, &t_loop_start);

        // --- Read ICM20948 Accel/Gyro/Temp (Every Loop - 1125 Hz) ---
        icm20948_read_accel(&icm_dev, &ax_raw, &ay_raw, &az_raw);
        icm20948_read_gyro(&icm_dev, &gx_raw, &gy_raw, &gz_raw);
        icm20948_read_temp(&icm_dev, &temp_icm_c); // Read ICM temp

        // TODO: Implement processing less often for effective 500Hz rate if needed
        // Convert Accel/Gyro raw to meaningful units (adjust divisors based on actual scale set)
        ax_g = (float)ax_raw / 16384.0f - accel_bias_g[0]; // Assuming +/- 2g
        ay_g = (float)ay_raw / 16384.0f - accel_bias_g[1];
        az_g = (float)az_raw / 16384.0f - accel_bias_g[2];
        gx_dps = (float)gx_raw / 131.0f - gyro_bias_dps[0];   // Assuming +/- 250 dps and applying bias correction
        gy_dps = (float)gy_raw / 131.0f - gyro_bias_dps[1];
        gz_dps = (float)gz_raw / 131.0f - gyro_bias_dps[2];

        // --- Read ICM20948 Mag (Rate Limited - 100 Hz) ---
        if (loop_count % loops_per_mag_read == 0) {
            if (icm20948_read_mag(&icm_dev, &mx_raw, &my_raw, &mz_raw) == ICM20948_RET_OK) {
                 // Convert Mag raw to uT (adjust multiplier based on calibration/sensitivity)
                 mx_ut = (float)mx_raw * 0.15f - mag_bias_ut[0];
                 my_ut = (float)my_raw * 0.15f - mag_bias_ut[1];
                 mz_ut = (float)mz_raw * 0.15f - mag_bias_ut[2];
                 mx_ut *= mag_scale[0];
                 my_ut *= mag_scale[1];
                 mz_ut *= mag_scale[2];
            } else {
                // Handle error, maybe keep old value or set to NAN
                mx_ut = my_ut = mz_ut = NAN;
            }
        }

        // --- Update AHRS (Rate Limited - 200 Hz) ---
        if (loop_count % loops_per_ahrs_update == 0) {
            // Convert gyro to rad/s
            float gx_rad = gx_dps * (M_PI / 180.0f);
            float gy_rad = gy_dps * (M_PI / 180.0f);
            float gz_rad = gz_dps * (M_PI / 180.0f);

            float current_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Default to identity

            switch (selected_filter) {
                case AHRS_EKF:
                    if (ekf_instance) {
                        ekf_ahrs_predict(ekf_instance, gx_rad, gy_rad, gz_rad);
                        ekf_ahrs_update_accel(ekf_instance, ax_g, ay_g, az_g);
                        ekf_ahrs_update_mag(ekf_instance, mx_ut, my_ut, mz_ut);
                        current_q[0] = ekf_instance->q0;
                        current_q[1] = ekf_instance->q1;
                        current_q[2] = ekf_instance->q2;
                        current_q[3] = ekf_instance->q3;
                    }
                    break;
                case AHRS_MADGWICK:
                    if (madgwick_instance) {
                        if (!isnan(mx_ut)) {
                            madgwick_ahrs_update(madgwick_instance,
                                                 gx_rad, gy_rad, gz_rad,
                                                 ax_g, ay_g, az_g,
                                                 mx_ut, my_ut, mz_ut);
                        } else {
                            madgwick_ahrs_update_imu(madgwick_instance,
                                                     gx_rad, gy_rad, gz_rad,
                                                     ax_g, ay_g, az_g);
                        }
                        current_q[0] = madgwick_instance->q0;
                        current_q[1] = madgwick_instance->q1;
                        current_q[2] = madgwick_instance->q2;
                        current_q[3] = madgwick_instance->q3;
                    }
                    break;
                case AHRS_MAHONY:
                    mahony_ahrs_update(mahony_instance,
                                      gx_rad, gy_rad, gz_rad,
                                      ax_g, ay_g, az_g,
                                      mx_ut, my_ut, mz_ut);
                    mahony_get_euler_angles(mahony_instance);
                    current_q[0] = mahony_instance->q0;
                    current_q[1] = mahony_instance->q1;
                    current_q[2] = mahony_instance->q2;
                    current_q[3] = mahony_instance->q3;
                    break;
                default:
                    // No filter - maybe set q to identity or NAN?
                    current_q[0] = 1.0f; current_q[1] = 0.0f; current_q[2] = 0.0f; current_q[3] = 0.0f;
                    break;
            }

            // --- Calculate World Frame Acceleration & Vertical Velocity ---
            float accel_g[3] = {ax_g, ay_g, az_g}; // Body frame accel in g
            float world_accel_g[3];                // World frame accel in g

            quat_rotate_vector(current_q, accel_g, world_accel_g);

            // Z-axis world acceleration minus gravity (1g)
            vertical_accel_mss = (world_accel_g[2] - 1.0f) * GRAVITY_MSS;

            // Integrate velocity (simple Euler integration)
            vertical_velocity_ms += vertical_accel_mss * ahrs_dt;

            // Euler conversion
            roll_deg  = atan2f(2*(current_q[0]*current_q[1] + current_q[2]*current_q[3]),
                               1 - 2*(current_q[1]*current_q[1] + current_q[2]*current_q[2])) * RAD_TO_DEG;
            pitch_deg = asinf(2*(current_q[0]*current_q[2] - current_q[3]*current_q[1])) * RAD_TO_DEG;
            yaw_deg   = atan2f(2*(current_q[0]*current_q[3] + current_q[1]*current_q[2]),
                               1 - 2*(current_q[2]*current_q[2] + current_q[3]*current_q[3])) * RAD_TO_DEG;
        }

        // --- Read MS5611 Pressure/Temp (Rate Limited - 50 Hz) ---
        if (loop_count % loops_per_press_temp_read == 0) {
            // Read D2 (Temperature) - ms5611_read_adc handles conversion start, delay, and read
            uint8_t conv_cmd_d2 = MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D2 | MS5611_CMD_ADC_OSR_4096; // Use highest OSR
            if (ms5611_read_adc(&ms_dev, conv_cmd_d2, &d2_raw) == MS5611_RET_OK) {
                // Read D1 (Pressure)
                uint8_t conv_cmd_d1 = MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D1 | MS5611_CMD_ADC_OSR_4096;
                if (ms5611_read_adc(&ms_dev, conv_cmd_d1, &d1_raw) == MS5611_RET_OK) {
                    // Calculate compensated values
                    ms5611_calculate(&ms_dev, d1_raw, d2_raw, &pressure_pa, &temp_ms5611_degC_x100);
                    temp_ms5611_c = (float)temp_ms5611_degC_x100 / 100.0f;
                } else {
                    pressure_pa = -99999; temp_ms5611_c = NAN; printf("MS Read D1 Failed\n"); // Read D1 failed
                }
            } else {
                pressure_pa = -99999; temp_ms5611_c = NAN; printf("MS Read D2 Failed\n"); // Read D2 failed
            }
        }

        // --- Print Data (Example - print every ~100 loops) ---
        if (loop_count % 100 == 0) {
             printf("%4lu | %+.2f %+.2f %+.2f | %+.2f %+.2f %+.2f | %+.1f %+.1f %+.1f | %12.3f | %7.1f | %10.1f | %7.2f %7.2f %7.2f\n",
                   (unsigned long)loop_count,
                   ax_g, ay_g, az_g,
                   gx_dps, gy_dps, gz_dps,
                   mx_ut, my_ut, mz_ut,
                   (float)pressure_pa / 100.0f, // Convert Pa to hPa for printing
                   temp_ms5611_c,
                   temp_icm_c,
                   roll_deg, pitch_deg, yaw_deg);
        }

        // --- Populate data structures for logging ---
        fusion_data.timestamp_us = bcm2835_st_read(); // Use current system time
        fusion_data.accel_x = ax_g;
        fusion_data.accel_y = ay_g;
        fusion_data.accel_z = az_g;
        fusion_data.gyro_x = gx_dps;
        fusion_data.gyro_y = gy_dps;
        fusion_data.gyro_z = gz_dps;
        fusion_data.mag_x = mx_ut;
        fusion_data.mag_y = my_ut;
        fusion_data.mag_z = mz_ut;
        fusion_data.pressure_pa = (float)pressure_pa; // Assuming pressure_pa holds the value
        fusion_data.temperature_c = temp_ms5611_c; // Use MS5611 temp, or choose ICM temp
        fusion_data.altitude_m = NAN; // Altitude calculation not shown, set NAN or calculate
        switch (selected_filter) {
            case AHRS_EKF:
                if (ekf_instance) {
                    fusion_data.q0 = ekf_instance->q0;
                    fusion_data.q1 = ekf_instance->q1;
                    fusion_data.q2 = ekf_instance->q2;
                    fusion_data.q3 = ekf_instance->q3;
                }
                break;
            case AHRS_MADGWICK:
                if (madgwick_instance) {
                    fusion_data.q0 = madgwick_instance->q0;
                    fusion_data.q1 = madgwick_instance->q1;
                    fusion_data.q2 = madgwick_instance->q2;
                    fusion_data.q3 = madgwick_instance->q3;
                }
                break;
            case AHRS_MAHONY:
                fusion_data.q0 = mahony_instance->q0;
                fusion_data.q1 = mahony_instance->q1;
                fusion_data.q2 = mahony_instance->q2;
                fusion_data.q3 = mahony_instance->q3;
                break;
            default:
                // No filter - set to identity or NAN?
                fusion_data.q0 = 1.0f; fusion_data.q1 = 0.0f; fusion_data.q2 = 0.0f; fusion_data.q3 = 0.0f;
                break;
        }

        nav_data.timestamp_us = fusion_data.timestamp_us; // Use same timestamp
        nav_data.vertical_accel_m_s2 = vertical_accel_mss; // Already in m/s^2
        nav_data.vertical_velocity_m_s = vertical_velocity_ms;
        nav_data.vertical_distance_m = 0.0f; // Distance integration wasn't implemented
        // Basic stationary check (example, refine later)
        float accel_mag_sq = (ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g);
        nav_data.is_stationary = (fabsf(accel_mag_sq - 1.0f) < 0.05f); // Check if magnitude is close to 1g
        if (nav_data.is_stationary) {
            vertical_velocity_ms = 0.0f;  // Reset velocity when stationary to prevent drift
        }

        // --- Log Data ---
        if (file_logger_log_data(&fusion_data, &nav_data) != 0) {
            // int log_errno = errno; // Capture errno immediately after the failed call
            // fprintf(stderr, "Warning: file_logger_log_data failed in main loop. errno = %d (%s)\n", log_errno, strerror(log_errno));
            // fflush(stderr); // Ensure it's flushed
            // Decide if this is critical - maybe stop logging?
        }

        // --- Print AHRS Test Data (Rate Limited) ---
        #define PRINT_INTERVAL_LOOPS 100 // Print roughly every 100 loops (~10Hz @ 1kHz loop)
        if (loop_count % PRINT_INTERVAL_LOOPS == 0) {
             printf("AHRS: R=%.1f P=%.1f Y=%.1f | VAcc=%.3f m/s^2 | VVel=%.3f m/s | Stat=%d\n",
                   roll_deg, pitch_deg, yaw_deg,
                   vertical_accel_mss,
                   vertical_velocity_ms,
                   nav_data.is_stationary);
        }

        // --- Loop Timing Control ---
        loop_count++;
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        loop_duration_s = (t_end.tv_sec - t_loop_start.tv_sec) + (t_end.tv_nsec - t_loop_start.tv_nsec) / 1e9;
        double sleep_s = LOOP_PERIOD_S - loop_duration_s;

        if (sleep_s > 0) {
            t_sleep.tv_sec = (time_t)sleep_s;
            t_sleep.tv_nsec = (long)((sleep_s - t_sleep.tv_sec) * 1e9);
            nanosleep(&t_sleep, NULL);
        } else if (sleep_s < 0) {
            // Optional: Warn if loop overran
            // printf("WARN: Loop overrun by %.3f ms\n", -sleep_s * 1000.0);
        }
    } // End while(1)

    // Cleanup (though loop is infinite, good practice to have)
    if (ekf_instance) { free(ekf_instance); ekf_instance = NULL; printf("EKF cleaned up.\n"); }
    if (madgwick_instance) { free(madgwick_instance); madgwick_instance = NULL; printf("Madgwick cleaned up.\n"); }
    // Mahony is likely stack/global, no free needed unless dynamically allocated
    cleanup_hw();
    return 0;
}