//=====================================================================================================
// madgwick_ahrs.h
//=====================================================================================================
//
// Madgwick's implementation of Madgwick's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
// 08/31/2020	Disi A	        Object-oriented fashion
// [Current Date] Cascade/USER    Integrated into gy9163-sensor-system
//
//=====================================================================================================
#ifndef __MADGWICK_AHRS_H__
#define __MADGWICK_AHRS_H__

#include <stdint.h>
#include <math.h>
#include "ahrs.h" // Defines MA_PRECISION, inv_sqrt, COMPUTE_EULER_ANGLE, M_PI
#include <stdlib.h> // For malloc/free

typedef struct {
    MA_PRECISION sample_rate; // Sample frequency in Hz
    MA_PRECISION beta;        // Algorithm gain beta
    MA_PRECISION q0;          // Quaternion components
    MA_PRECISION q1;
    MA_PRECISION q2;
    MA_PRECISION q3;

    // Result variables (Euler angles in radians)
    MA_PRECISION yaw;
    MA_PRECISION pitch;
    MA_PRECISION roll;
} MadgwickAHRS;

//----------------------------------------------------------------------------------------------------
// Function declarations

/**
 * @brief Creates and initializes a MadgwickAHRS instance.
 * @param sample_rate The initial sample rate in Hz.
 * @param beta The algorithm gain beta (a good starting value is 0.033 to 0.1).
 * @return Pointer to the created MadgwickAHRS instance, or NULL on failure.
 */
MadgwickAHRS* create_madgwick_ahrs(MA_PRECISION sample_rate, MA_PRECISION beta);

/**
 * @brief Frees the memory allocated for a MadgwickAHRS instance.
 * @param workspace Pointer to the MadgwickAHRS instance to free.
 */
void free_madgwick_ahrs(MadgwickAHRS* workspace);

/**
 * @brief Updates the sample rate used by the algorithm. Resets quaternion if changed.
 * @param workspace Pointer to the MadgwickAHRS instance.
 * @param sample_rate The new sample rate in Hz.
 */
void madgwick_ahrs_update_sample_rate(MadgwickAHRS* workspace, MA_PRECISION sample_rate);

/**
 * @brief Updates the filter state using only gyroscope and accelerometer data (6-axis).
 * Assumes accelerometer measures gravity and sensor is not accelerating significantly.
 * Gyroscope units must be radians/second. Accelerometer units are arbitrary (but consistent).
 * @param workspace Pointer to the MadgwickAHRS instance.
 * @param gx Gyroscope x-axis measurement (rad/s).
 * @param gy Gyroscope y-axis measurement (rad/s).
 * @param gz Gyroscope z-axis measurement (rad/s).
 * @param ax Accelerometer x-axis measurement.
 * @param ay Accelerometer y-axis measurement.
 * @param az Accelerometer z-axis measurement.
 */
void madgwick_ahrs_update_imu(MadgwickAHRS* workspace, MA_PRECISION gx, MA_PRECISION gy, MA_PRECISION gz, MA_PRECISION ax, MA_PRECISION ay, MA_PRECISION az);

/**
 * @brief Updates the filter state using gyroscope, accelerometer, and magnetometer data (9-axis).
 * Gyroscope units must be radians/second. Accelerometer and Magnetometer units are arbitrary (but consistent).
 * @param workspace Pointer to the MadgwickAHRS instance.
 * @param gx Gyroscope x-axis measurement (rad/s).
 * @param gy Gyroscope y-axis measurement (rad/s).
 * @param gz Gyroscope z-axis measurement (rad/s).
 * @param ax Accelerometer x-axis measurement.
 * @param ay Accelerometer y-axis measurement.
 * @param az Accelerometer z-axis measurement.
 * @param mx Magnetometer x-axis measurement.
 * @param my Magnetometer y-axis measurement.
 * @param mz Magnetometer z-axis measurement.
 */
void madgwick_ahrs_update(MadgwickAHRS* workspace, MA_PRECISION gx, MA_PRECISION gy, MA_PRECISION gz, MA_PRECISION ax, MA_PRECISION ay, MA_PRECISION az, MA_PRECISION mx, MA_PRECISION my, MA_PRECISION mz);

#endif /* __MADGWICK_AHRS_H__ */