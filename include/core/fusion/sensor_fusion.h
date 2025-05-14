#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

// Includes for types potentially used in future shared structures/functions
#include <stdint.h>
#include <stdbool.h>
#include "common/common_defs.h"
#include "drivers/ssd1306/ssd1306.h"  // For display prompts

int initialize_sensors(ssd1306_handle_t* disp);
void cleanup_hw(void);
bool run_sensor_fusion_cycle(FusionData_T* fusion_data); // Adjust type as needed

// --- Fused Sensor Data Structure (Example - Populate later) ---
/*
typedef struct {
    // Timestamp
    struct timespec timestamp;

    // IMU Data (e.g., in standard units)
    float accel_x, accel_y, accel_z; // g
    float gyro_x, gyro_y, gyro_z;   // dps
    float mag_x, mag_y, mag_z;     // uT

    // Barometer Data
    float pressure_hpa;            // hPa
    float temp_ms5611_c;           // degrees C

    // Reference Temperature
    float temp_icm_c;              // degrees C

    // Status/Validity flags?
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
    bool baro_valid;

} fused_sensor_data_t;
*/

// --- Public Function Declarations (if any are added later) ---
// e.g., void init_sensor_fusion(void);
// e.g., bool get_latest_fused_data(fused_sensor_data_t *data);


#endif // SENSOR_FUSION_H