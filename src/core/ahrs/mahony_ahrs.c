#include "core/ahrs/mahony_ahrs.h"
#include <math.h>
#include <stdint.h>

// Fast inverse square-root
static float inv_sqrt(float x) {
    float half = 0.5f * x;
    union { float f; uint32_t i; } u;
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    u.f = u.f * (1.5f - half * u.f * u.f);
    return u.f;
}

void mahony_ahrs_init(MahonyAHRS *ahrs,
                      float sample_frequency,
                      float kp_gain,
                      float ki_gain)
{
    ahrs->sample_freq   = sample_frequency;
    ahrs->kp            = kp_gain;
    ahrs->ki            = ki_gain;
    ahrs->q0 = 1.0f;  ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;  ahrs->q3 = 0.0f;
    ahrs->integral_fb_x = 0.0f;
    ahrs->integral_fb_y = 0.0f;
    ahrs->integral_fb_z = 0.0f;
    ahrs->roll  = 0.0f;
    ahrs->pitch = 0.0f;
    ahrs->yaw   = 0.0f;
}

void mahony_ahrs_update(MahonyAHRS *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz)
{
    float recip_norm;
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float halfex = 0, halfey = 0, halfez = 0;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float qa, qb, qc;

    // Apply integral feedback if Ki > 0
    if (ahrs->ki > 0.0f) {
        gx += ahrs->integral_fb_x;
        gy += ahrs->integral_fb_y;
        gz += ahrs->integral_fb_z;
    }

    // Normalize accelerometer
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recip_norm;  ay *= recip_norm;  az *= recip_norm;

        // Normalize magnetometer
        recip_norm = inv_sqrt(mx*mx + my*my + mz*mz);
        mx *= recip_norm;  my *= recip_norm;  mz *= recip_norm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f*(mx*(0.5f - q2*q2 - q3*q3) + 
                  my*(q1*q2 - q0*q3)     + 
                  mz*(q1*q3 + q0*q2));
        hy = 2.0f*(mx*(q1*q2 + q0*q3)     +
                  my*(0.5f - q1*q1 - q3*q3) +
                  mz*(q2*q3 - q0*q1));
        bx = sqrtf(hx*hx + hy*hy);
        bz = 2.0f*(mx*(q1*q3 - q0*q2) +
                  my*(q2*q3 + q0*q1) +
                  mz*(0.5f - q1*q1 - q2*q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = q0*q0 - 0.5f + q3*q3;
        halfwx = bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2);
        halfwy = bx*(q1*q2 - q0*q3)         + bz*(q0*q1 + q2*q3);
        halfwz = bx*(q0*q2 + q1*q3)         + bz*(0.5f - q1*q1 - q2*q2);

        // Error is sum of cross products
        halfex += (ay*halfvz - az*halfvy) + (my*halfwz - mz*halfwy);
        halfey += (az*halfvx - ax*halfvz) + (mz*halfwx - mx*halfwz);
        halfez += (ax*halfvy - ay*halfvx) + (mx*halfwy - my*halfwz);

        // Integral feedback
        if (ahrs->ki > 0.0f) {
            ahrs->integral_fb_x += ahrs->ki * halfex * (1.0f/ahrs->sample_freq);
            ahrs->integral_fb_y += ahrs->ki * halfey * (1.0f/ahrs->sample_freq);
            ahrs->integral_fb_z += ahrs->ki * halfez * (1.0f/ahrs->sample_freq);
        }
        // Proportional feedback
        gx += ahrs->kp * halfex;
        gy += ahrs->kp * halfey;
        gz += ahrs->kp * halfez;
    }

    // Integrate quaternion rate and normalize
    gx *= (0.5f * (1.0f/ahrs->sample_freq));  
    gy *= (0.5f * (1.0f/ahrs->sample_freq));  
    gz *= (0.5f * (1.0f/ahrs->sample_freq));  
    qa = q0; qb = q1; qc = q2;
    ahrs->q0 += (-qb*gx - qc*gy - q3*gz);
    ahrs->q1 += ( qa*gx + qc*gz - q3*gy);
    ahrs->q2 += ( qa*gy - qb*gz + q3*gx);
    ahrs->q3 += ( qa*gz + qb*gy - qc*gx);

    // Normalize quaternion
    recip_norm = inv_sqrt(
      ahrs->q0*ahrs->q0 + ahrs->q1*ahrs->q1 +
      ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3);
    ahrs->q0 *= recip_norm;
    ahrs->q1 *= recip_norm;
    ahrs->q2 *= recip_norm;
    ahrs->q3 *= recip_norm;

    // Update Euler angles
    mahony_get_euler_angles(ahrs);
}

void mahony_ahrs_update_imu(MahonyAHRS *ahrs,
                            float gx, float gy, float gz,
                            float ax, float ay, float az)
{
    // Same as above but skip magnetometer steps
    float recip_norm, halfex=0, halfey=0, halfez=0;
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float qa, qb, qc;

    // Integral feedback
    if (ahrs->ki > 0.0f) {
        gx += ahrs->integral_fb_x;
        gy += ahrs->integral_fb_y;
        gz += ahrs->integral_fb_z;
    }

    // Normalize accel
    if (!((ax==0) && (ay==0) && (az==0))) {
        recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recip_norm; ay *= recip_norm; az *= recip_norm;

        // Estimated gravity direction
        halfex = (ay*(q0*q1+q2*q3) - az*(q0*q2-q1*q3));
        halfey = (az*(q0*q2+q1*q3) - ax*(q0*q1-q2*q3));
        halfez = (ax*(q0*q1+q2*q3) - ay*(q0*q1-q2*q3));

        // Integral
        if (ahrs->ki > 0.0f) {
            ahrs->integral_fb_x += ahrs->ki * halfex * (1.0f/ahrs->sample_freq);
            ahrs->integral_fb_y += ahrs->ki * halfey * (1.0f/ahrs->sample_freq);
            ahrs->integral_fb_z += ahrs->ki * halfez * (1.0f/ahrs->sample_freq);
        }
        // Proportional
        gx += ahrs->kp * halfex;
        gy += ahrs->kp * halfey;
        gz += ahrs->kp * halfez;
    }

    // Integrate and normalize (same as above)
    gx *= (0.5f * (1.0f/ahrs->sample_freq));
    gy *= (0.5f * (1.0f/ahrs->sample_freq));
    gz *= (0.5f * (1.0f/ahrs->sample_freq));
    qa = q0; qb = q1; qc = q2;
    ahrs->q0 += (-qb*gx - qc*gy - q3*gz);
    ahrs->q1 += ( qa*gx + qc*gz - q3*gy);
    ahrs->q2 += ( qa*gy - qb*gz + q3*gx);
    ahrs->q3 += ( qa*gz + qb*gy - qc*gx);

    recip_norm = inv_sqrt(
      ahrs->q0*ahrs->q0 + ahrs->q1*ahrs->q1 +
      ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3);
    ahrs->q0 *= recip_norm;
    ahrs->q1 *= recip_norm;
    ahrs->q2 *= recip_norm;
    ahrs->q3 *= recip_norm;

    mahony_get_euler_angles(ahrs);
}

void mahony_get_euler_angles(MahonyAHRS *ahrs)
{
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    const float rad2deg = 57.2957795131f;

    // Roll (x-axis)
    float sinr =  2.0f*(q0*q1 + q2*q3);
    float cosr =  1.0f - 2.0f*(q1*q1 + q2*q2);
    ahrs->roll  = atan2f(sinr, cosr) * rad2deg;

    // Pitch (y-axis)
    float sinp =  2.0f*(q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f)
        ahrs->pitch = copysignf(90.0f, sinp);
    else
        ahrs->pitch = asinf(sinp) * rad2deg;

    // Yaw (z-axis)
    float siny =  2.0f*(q0*q3 + q1*q2);
    float cosy =  1.0f - 2.0f*(q2*q2 + q3*q3);
    ahrs->yaw   = atan2f(siny, cosy) * rad2deg;
}