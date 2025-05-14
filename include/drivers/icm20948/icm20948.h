#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>
#include <stdbool.h>

// Constants for configuration
#define ICM20948_WHO_AM_I_VAL    0xEA

// Scale factors
extern const float ACCEL_SCALE;  // LSB/g
extern const float GYRO_SCALE;   // LSB/dps
extern const float MAG_SCALE;    // LSB/uT

// Return codes
typedef enum {
    ICM20948_RET_OK = 0,
    ICM20948_RET_NOT_INIT,
    ICM20948_RET_BAD_ARG,     ///< Bad argument passed to function
    ICM20948_RET_NULL_PTR,     ///< Null pointer passed to function
    ICM20948_RET_GEN_FAIL,     ///< General failure (e.g., SPI/I2C comms, unexpected device state)
    ICM20948_RET_MAG_DATA_ERROR ///< Magnetometer data read error (e.g., overflow, overrun)
} icm20948_return_code_t;

// Function pointer types
typedef icm20948_return_code_t (*icm20948_read_fptr_t)(uint8_t reg, uint8_t *data, uint32_t len);
typedef icm20948_return_code_t (*icm20948_write_fptr_t)(uint8_t reg, uint8_t *data, uint32_t len);
typedef void (*icm20948_delay_us_fptr_t)(uint32_t us);

// Device structure
typedef void (*icm20948_cs_assert_fptr_t)(void);
typedef void (*icm20948_cs_deassert_fptr_t)(void);
typedef icm20948_return_code_t (*icm20948_spi_transfer_fptr_t)(const uint8_t *tx, uint8_t *rx, uint32_t len);

typedef struct {
    struct {
        icm20948_read_fptr_t read;
        icm20948_write_fptr_t write;
        icm20948_delay_us_fptr_t delay_us;
        icm20948_cs_assert_fptr_t cs_assert;
        icm20948_cs_deassert_fptr_t cs_deassert;
        icm20948_spi_transfer_fptr_t spi_transfer;
    } intf;
    uint8_t current_bank;       /**< Current user bank selection */
} icm20948_dev_t;

// Raw data structure
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    uint8_t accel_status;  // Status flags
    uint8_t gyro_status;   // Status flags
    uint8_t mag_status;    // Status flags
} icm20948_sensor_data_t;

// Function prototypes
icm20948_return_code_t icm20948_init(icm20948_dev_t *dev);
icm20948_return_code_t icm20948_read_reg(icm20948_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len);
icm20948_return_code_t icm20948_write_reg(icm20948_dev_t *dev, uint8_t reg, uint8_t value);
icm20948_return_code_t icm20948_read_sensor_data(icm20948_dev_t *dev, icm20948_sensor_data_t *data);
icm20948_return_code_t icm20948_read_temp_raw(icm20948_dev_t *dev, int16_t *temp);
icm20948_return_code_t icm20948_read_accel(icm20948_dev_t *dev, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);
icm20948_return_code_t icm20948_read_gyro(icm20948_dev_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
icm20948_return_code_t icm20948_read_mag(icm20948_dev_t *dev, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
icm20948_return_code_t icm20948_read_who_am_i(icm20948_dev_t *dev, uint8_t *who_am_i);

// Public Functions
icm20948_return_code_t icm20948_select_bank(icm20948_dev_t *dev, uint8_t bank);

/**
 * @brief Reads the raw temperature sensor value and converts it to degrees Celsius.
 *
 * @param dev Pointer to the device structure.
 * @param temp_c Pointer to a float where the temperature in degrees Celsius will be stored.
 * @return icm20948_return_code_t Status code indicating success or failure.
 */
icm20948_return_code_t icm20948_read_temp(icm20948_dev_t *dev, float *temp_c);

// Function to trigger a software reset of the device
icm20948_return_code_t icm20948_device_reset(icm20948_dev_t *dev);

#ifdef __cplusplus
extern "C" {
#endif

#endif // ICM20948_H