#ifndef __AHRS_H__
#define __AHRS_H__

#include <math.h> // Need this for M_PI, atan2f, asinf, sqrtf
#include <stdint.h>

#define MA_DOUBLE_PRECISION 0

#if defined(_WIN32) && !defined(M_PI) // Define M_PI if not defined (e.g., older MSVC)
#define M_PI 3.141592653589793238462643383279502884197163993751
#endif

#if MA_DOUBLE_PRECISION
#define MA_PRECISION double

#define ATAN2 atan2
#define ASIN asin
#define MAGIC_R 0x5fe6eb50c7b537a9
#define SQRT sqrt

#else
#define MA_PRECISION float
#define ATAN2 atan2f
#define ASIN asinf
#define SQRT sqrtf
#define MAGIC_R 0x5f3759df
#endif

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// Calculates Euler angles (radians) from quaternion and stores in workspace
// Assumes ZYX extrinsic rotation order (Yaw, Pitch, Roll)
#define COMPUTE_EULER_ANGLE(WS) \
	do { \
        /* Roll (x-axis rotation) */ \
        MA_PRECISION sinr_cosp = 2.0f * (WS->q0 * WS->q1 + WS->q2 * WS->q3); \
        MA_PRECISION cosr_cosp = 1.0f - 2.0f * (WS->q1 * WS->q1 + WS->q2 * WS->q2); \
        WS->roll = ATAN2(sinr_cosp, cosr_cosp); \
        /* Pitch (y-axis rotation) */ \
        MA_PRECISION sinp = 2.0f * (WS->q0 * WS->q2 - WS->q3 * WS->q1); \
        if (fabsf(sinp) >= 1.0f) \
            WS->pitch = copysignf(M_PI / 2.0f, sinp); /* Use 90/-90 degrees if out of range */ \
        else \
            WS->pitch = ASIN(sinp); \
        /* Yaw (z-axis rotation) */ \
        MA_PRECISION siny_cosp = 2.0f * (WS->q0 * WS->q3 + WS->q1 * WS->q2); \
        MA_PRECISION cosy_cosp = 1.0f - 2.0f * (WS->q2 * WS->q2 + WS->q3 * WS->q3); \
        WS->yaw = ATAN2(siny_cosp, cosy_cosp); \
    } while(0)


// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static inline MA_PRECISION inv_sqrt(MA_PRECISION x) {
    // Fast inverse sqrt (bit-level hack) without violating aliasing
    MA_PRECISION halfx = 0.5f * x;
    MA_PRECISION y = x;
#if MA_DOUBLE_PRECISION
    union { MA_PRECISION f; uint64_t i; } u;
    u.f = y;
    u.i = MAGIC_R - (u.i >> 1);
    y = u.f;
#else
    union { MA_PRECISION f; uint32_t i; } u;
    u.f = y;
    u.i = MAGIC_R - (u.i >> 1);
    y = u.f;
#endif
    y = y * (1.5f - (halfx * y * y));
    return y;
}

#endif // __AHRS_H__