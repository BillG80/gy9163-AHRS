#ifndef ELEVATOR_NAV_H_
#define ELEVATOR_NAV_H_

#include "common/common_defs.h" // For FusionData_T and NavData_T

/**
 * @brief Data structure for elevator motion data collection
 * Contains sensor readings and derived data at a specific timestamp
 */
typedef struct {
    uint64_t timestamp_us;
    float temperature_c;
    float pressure_pa;
    float altitude_m;
    float vertical_velocity_m_s;
    float vertical_accel_m_s2;
    int estimated_floor;  // Estimated floor number
} ElevatorDataPoint_T;

/**
 * @brief Initializes the elevator navigation module.
 * Sets initial velocity, distance, etc.
 * @return 0 on success, -1 on error.
 */
typedef enum {
    ELEVATOR_NAV_MODE_ELEVATOR = 0,
    ELEVATOR_NAV_MODE_GENERAL = 1
} ElevatorNavMode_T;

int elevator_nav_init(ElevatorNavMode_T mode);

/**
 * @brief Updates the navigation estimate based on new sensor fusion data.
 * Calculates vertical acceleration, integrates for velocity and distance,
 * potentially uses altitude for drift correction and detects stationary periods.
 * @param fusion_data Pointer to the latest sensor fusion output data.
 * @param nav_data Pointer to the navigation data structure to be populated.
 * @return 0 on success, -1 on error.
 */
int elevator_nav_update(const FusionData_T *fusion_data, NavData_T *nav_data);

/**
 * @brief Gets the current elevator state as a string.
 * @return String representation of the current elevator state.
 */
const char* elevator_nav_get_state_str(void);

/**
 * @brief Gets statistics about the current or last elevator trip.
 * @param start_time_us Pointer to store the trip start timestamp (microseconds).
 * @param end_time_us Pointer to store the trip end timestamp (microseconds).
 * @param distance_m Pointer to store the total trip distance (meters).
 */
void elevator_nav_get_trip_stats(uint64_t *start_time_us, uint64_t *end_time_us, float *distance_m);

/**
 * @brief Gets the collected elevator data points.
 * @param data Pointer to store the array of data points.
 * @param count Pointer to store the number of data points.
 * @return 0 on success, -1 on error.
 */
int elevator_nav_get_data_points(ElevatorDataPoint_T **data, int *count);

/**
 * @brief Gets the most recent data point collected.
 * @param data_point Pointer to store the latest data point.
 * @return 0 on success, -1 on error (e.g., no data points collected).
 */
int elevator_nav_get_latest_data_point(ElevatorDataPoint_T *data_point);

/**
 * @brief Cleans up the elevator navigation module.
 * (Might not be needed if there's no dynamically allocated memory).
 */
void elevator_nav_cleanup(void);


#endif // ELEVATOR_NAV_H_