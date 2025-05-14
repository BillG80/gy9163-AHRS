#include "core/ahrs/ekf_ahrs.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Internal helper: normalize quaternion
static void normalize_q(ekf_ahrs_t* ekf) {
    float n = sqrtf(ekf->q0*ekf->q0 + ekf->q1*ekf->q1 +
                    ekf->q2*ekf->q2 + ekf->q3*ekf->q3);
    ekf->q0 /= n; ekf->q1 /= n; ekf->q2 /= n; ekf->q3 /= n;
}

ekf_ahrs_t* ekf_ahrs_create(float dt) {
    ekf_ahrs_t* ekf = (ekf_ahrs_t*)malloc(sizeof(*ekf));
    if (!ekf) return NULL;
    // init state: identity quaternion, zero biases
    ekf->q0 = 1; ekf->q1 = ekf->q2 = ekf->q3 = 0;
    ekf->bg[0]=ekf->bg[1]=ekf->bg[2]=0;
    ekf->ba[0]=ekf->ba[1]=ekf->ba[2]=0;
    ekf->dt = dt;
    // TODO: alloc/init covariance P, Q, R
    return ekf;
}

void ekf_ahrs_destroy(ekf_ahrs_t* ekf) {
    if (ekf) {
        // TODO: free any matrices if allocated
        free(ekf);
    }
}

void ekf_ahrs_predict(ekf_ahrs_t* ekf,
                      float wx, float wy, float wz)
{
    // Remove gyro bias
    wx -= ekf->bg[0];
    wy -= ekf->bg[1];
    wz -= ekf->bg[2];
    // Quaternion derivative: ½·Ω(ω)·q
    float dt = ekf->dt * 0.5f;
    float dq0 = (-ekf->q1*wx - ekf->q2*wy - ekf->q3*wz) * dt;
    float dq1 = ( ekf->q0*wx + ekf->q2*wz - ekf->q3*wy) * dt;
    float dq2 = ( ekf->q0*wy - ekf->q1*wz + ekf->q3*wx) * dt;
    float dq3 = ( ekf->q0*wz + ekf->q1*wy - ekf->q2*wx) * dt;
    ekf->q0 += dq0; ekf->q1 += dq1;
    ekf->q2 += dq2; ekf->q3 += dq3;
    normalize_q(ekf);
    // TODO: propagate covariance P = F·P·Fᵀ + Q
}

void ekf_ahrs_update_accel(ekf_ahrs_t* ekf,
                           float ax, float ay, float az)
{
    // Silence unused parameter warnings until implemented
    (void)ekf;
    (void)ax;
    (void)ay;
    (void)az;

    // TODO:
    // 1) Predict measured gravity in body frame: g_pred = R(q)ᵀ·[0,0,1]
    // 2) Compute innovation: y = [ax,ay,az] - g_pred
    // 3) Compute Kalman gain K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
    // 4) Update state: x = x + K·y  (adjust q and ba)
    // 5) Update covariance: P = (I - K·H)·P
}

void ekf_ahrs_update_mag(ekf_ahrs_t* ekf,
                         float mx, float my, float mz)
{
    // Silence unused parameter warnings until implemented
    (void)ekf;
    (void)mx;
    (void)my;
    (void)mz;

    // TODO: same pattern, using magnetometer model
    // to correct heading (yaw) and optionally biases
}