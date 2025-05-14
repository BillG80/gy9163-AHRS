#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <stdint.h>
#include <stdbool.h>

//----------------------------------------------------------------------------------------------------
// Variable declaration

typedef struct {
    float sample_freq;    // sample frequency in Hz
    float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    float integral_fb_x;  // integral error terms scaled by Ki
    float integral_fb_y;
    float integral_fb_z;
    float kp;             // proportional gain
    float ki;             // integral gain
    // Euler angles (degrees)
    float roll;
    float pitch;
    float yaw;
} MahonyAHRS;

//---------------------------------------------------------------------------------------------------
// Public API

/// Initialize filter state (q=[1,0,0,0], zero integral).
void mahony_ahrs_init(MahonyAHRS *ahrs,
                      float sample_frequency,
                      float kp_gain,
                      float ki_gain);

/// AHRS update with 9-axis (gyro in rad/s, accel in g, mag in uT).
void mahony_ahrs_update(MahonyAHRS *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz);

/// AHRS update with only 6-axis (gyro + accel).
void mahony_ahrs_update_imu(MahonyAHRS *ahrs,
                            float gx, float gy, float gz,
                            float ax, float ay, float az);

/// Recompute roll/pitch/yaw (degrees) from the current quaternion.
void mahony_get_euler_angles(MahonyAHRS *ahrs);

#endif // MAHONY_AHRS_H