#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h> // For fixed-width integer types like uint8_t, uint64_t
#include <stdbool.h> // For bool type

#ifdef ENABLE_DEBUG
  #define DPRINTF(...) printf(__VA_ARGS__)
#else
  #define DPRINTF(...) (void)0
#endif

// --- Basic Vector Types ---
// Used for accelerometer, gyroscope, magnetometer, position, velocity etc.

// 3D vector with single-precision float components
typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

// 3D vector with double-precision float components (optional, if higher precision needed)
typedef struct {
    double x;
    double y;
    double z;
} vector3d_t;


// --- Quaternion Type ---
// Used for representing orientation in 3D space (AHRS output)
typedef struct {
    float w; // Scalar part
    float x; // Vector part x
    float y; // Vector part y
    float z; // Vector part z
} quaternion_t;


// --- Timestamp Type ---
// Using 64 bits to avoid rollover for a very long time
typedef uint64_t timestamp_us_t; // Timestamp in microseconds


// --- Status/Error Codes ---
// Common return type for functions to indicate success or failure
/**
 * @brief Standard status codes for function return values.
 *
 * Based on common embedded system practices. Zero is success, negative values are errors.
 */
typedef enum {
    STATUS_OK = 0,              // Operation successful
    ERROR_NULL_POINTER = -1,    // Null pointer passed as argument
    ERROR_INVALID_PARAM = -2,   // Invalid parameter value
    ERROR_HARDWARE = -3,         // General hardware error (init failed, communication failed, etc.)
    ERROR_COMMUNICATION = -4,    // Specific communication error (e.g., NACK, timeout on bus)
    ERROR_TIMEOUT = -5,
    ERROR_SENSOR_OVERFLOW = -6,  // Sensor reading overflowed
    ERROR_NOT_INITIALIZED = -7,
    ERROR_ALREADY_INITIALIZED = -8,
    ERROR_ALLOCATION_FAILED = -9, // Memory allocation failed
    ERROR_NO_MEM = -3,
    ERROR_UNKNOWN = -99,
    ERROR_MEMORY_ALLOC = -3,    // Memory allocation failed
    ERROR_BUSY = -5,            // Device or resource is busy
    ERROR_NOT_SUPPORTED = -7,   // Feature or operation not supported
    ERROR_BUFFER_FULL = -9,     // Buffer is full
    ERROR_BUFFER_EMPTY = -10,   // Buffer is empty
    ERROR_DEVICE_NOT_FOUND = -11, // Device not found or incorrect ID
    ERROR_UNEXPECTED_VALUE = -12, // An unexpected value was read or encountered// --- Add the new error codes here ---
    ERROR_HW_INIT_FAILED = -13, // Specific hardware init failure (e.g., wrong WHO_AM_I)
    ERROR_NOT_IMPLEMENTED = -14, // Functionality is not yet coded
    ERROR_INITIALIZATION_FAILED = -15, // Generic initialization failure
    // Add more specific error codes as needed, continuing the negative sequence

} status_t;

#endif // COMMON_TYPES_H 