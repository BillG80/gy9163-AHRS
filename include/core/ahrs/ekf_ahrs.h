#ifndef EKF_AHRS_H
#define EKF_AHRS_H

#include <stdint.h>

// EKF state: quaternion + gyro bias (3) + accel bias (3)
typedef struct {
    // Orientation
    float q0, q1, q2, q3;
    // Bias states
    float bg[3];
    float ba[3];
    // Timestep
    float dt;
    // (You can add covariance P, process noise Q, measurement noise R here)
} ekf_ahrs_t;

// Create/destroy
ekf_ahrs_t* ekf_ahrs_create(float dt);
void         ekf_ahrs_destroy(ekf_ahrs_t* ekf);

// Predict step: propagate quaternion & bias (bias is random walk)
void ekf_ahrs_predict(ekf_ahrs_t* ekf,
                      float wx, float wy, float wz);

// Update with accel measurement (g): aligns gravity vector → correct q & ba
void ekf_ahrs_update_accel(ekf_ahrs_t* ekf,
                           float ax, float ay, float az);

// Update with mag measurement (uT): aligns magnetic north → correct q
void ekf_ahrs_update_mag(ekf_ahrs_t* ekf,
                         float mx, float my, float mz);

#endif // EKF_AHRS_H