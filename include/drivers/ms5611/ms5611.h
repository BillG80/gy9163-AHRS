#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>

// --- MS5611 Commands ---
#define MS5611_CMD_RESET             0x1E // ADC reset command
#define MS5611_CMD_ADC_READ          0x00 // ADC read command
#define MS5611_CMD_ADC_CONV          0x40 // ADC conversion command base
#define MS5611_CMD_ADC_D1            0x00 // D1 conversion command offset
#define MS5611_CMD_ADC_D2            0x10 // D2 conversion command offset
#define MS5611_CMD_ADC_OSR_256       0x00 // OSR = 256 conversion command offset
#define MS5611_CMD_ADC_OSR_512       0x02 // OSR = 512 conversion command offset
#define MS5611_CMD_ADC_OSR_1024      0x04 // OSR = 1024 conversion command offset
#define MS5611_CMD_ADC_OSR_2048      0x06 // OSR = 2048 conversion command offset
#define MS5611_CMD_ADC_OSR_4096      0x08 // OSR = 4096 conversion command offset
#define MS5611_CMD_PROM_RD_BASE      0xA0 // PROM read command base
#define MS5611_CMD_PROM_RD_C1        0xA0 // Read C1 (Pressure Sensitivity)
#define MS5611_CMD_PROM_RD_C2        0xA2 // Read C2 (Pressure Offset)
#define MS5611_CMD_PROM_RD_C3        0xA4 // Read C3 (Temp Coeff Pressure Sens)
#define MS5611_CMD_PROM_RD_C4        0xA6 // Read C4 (Temp Coeff Pressure Offset)
#define MS5611_CMD_PROM_RD_C5        0xA8 // Read C5 (Reference Temperature)
#define MS5611_CMD_PROM_RD_C6        0xAA // Read C6 (Temp Coeff Temperature)
#define MS5611_CMD_PROM_RD_CRC       0xAE // Read C7 (CRC)

// Convenience macros for full conversion commands (OSR = 4096)
#define MS5611_CMD_CONV_D1_OSR4096   (MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D1 | MS5611_CMD_ADC_OSR_4096)
#define MS5611_CMD_CONV_D2_OSR4096   (MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D2 | MS5611_CMD_ADC_OSR_4096)

// Conversion time (max microseconds per OSR)
#define MS5611_CONV_TIME_OSR256_US   600u
#define MS5611_CONV_TIME_OSR512_US  1200u
#define MS5611_CONV_TIME_OSR1024_US 2300u
#define MS5611_CONV_TIME_OSR2048_US 4600u
#define MS5611_CONV_TIME_OSR4096_US 9040u

// --- Return Codes ---
typedef enum {
    MS5611_RET_OK = 0,
    MS5611_RET_ERROR = -1,
    MS5611_RET_NULL_PTR = -2,
    MS5611_RET_TIMEOUT = -3,
    MS5611_RET_BAD_ARG = -4,  // Invalid argument passed to function
    MS5611_RET_CRC_ERROR = -5, // CRC check for PROM failed
} ms5611_return_code_t;

// Structure to hold the platform interface functions
typedef struct {
    // Function pointer for bus read (I2C).
    // Args: reg (command/register), data buffer, length
    ms5611_return_code_t (*read)(uint8_t reg, uint8_t *data, uint32_t len);
    
    // Function pointer for bus write (I2C).
    // Args: reg (command/register), data buffer, length
    ms5611_return_code_t (*write)(uint8_t reg, const uint8_t *data, uint32_t len);

    // Function pointer for delay in microseconds
    void (*delay_us)(uint32_t period);

} ms5611_intf_t;

// Main device structure
typedef struct {
    ms5611_intf_t intf;          // Platform interface functions
    uint16_t prom_coeffs[8];     // PROM Coefficients C1-C6 (reserve 8 for CRC)
    // Add other device state if needed (e.g., OSR setting)
} ms5611_dev_t;

/**
 * @brief Initializes the MS5611 device structure and the sensor.
 *        This function assigns platform interface functions, resets the sensor,
 *        and reads the PROM calibration coefficients.
 * @param dev Pointer to the ms5611 device structure to initialize.
 * @param platform_read Pointer to the platform-specific read function.
 * @param platform_write Pointer to the platform-specific write function.
 * @param platform_delay_us Pointer to the platform-specific delay function.
 * @return MS5611_RET_OK on success, error code otherwise.
 */
ms5611_return_code_t ms5611_init(ms5611_dev_t *dev, 
                                 ms5611_return_code_t (*platform_read)(uint8_t reg, uint8_t *data, uint32_t len),
                                 ms5611_return_code_t (*platform_write)(uint8_t reg, const uint8_t *data, uint32_t len),
                                 void (*platform_delay_us)(uint32_t period));

/**
 * @brief Sends the RESET command to the MS5611.
 * @param dev Pointer to the ms5611 device structure.
 * @return MS5611_RET_OK on success, error code otherwise.
 */
ms5611_return_code_t ms5611_reset(ms5611_dev_t *dev);

/**
 * @brief Reads all 6 PROM coefficients (C1-C6) from the sensor.
 * @param dev Pointer to the ms5611 device structure. Coefficients are stored in dev->prom_coeffs.
 * @return MS5611_RET_OK on success, error code otherwise.
 */
ms5611_return_code_t ms5611_read_prom(ms5611_dev_t *dev);

/**
 * @brief Starts an ADC conversion and reads the result.
 * @param dev Pointer to the ms5611 device structure.
 * @param conversion_cmd The conversion command (e.g., 0x48 for pressure OSR 4096, 0x58 for temp OSR 4096).
 * @param adc_value Pointer to store the raw 24-bit ADC result.
 * @return MS5611_RET_OK on success, error code otherwise.
 */
ms5611_return_code_t ms5611_read_adc(ms5611_dev_t *dev, uint8_t conversion_cmd, uint32_t *adc_value);

/**
 * @brief Calculates compensated pressure (Pascals) and temperature (degrees C * 100).
 * @param dev Pointer to the ms5611 device structure (must have PROM coeffs loaded).
 * @param D1 Raw pressure ADC value.
 * @param D2 Raw temperature ADC value.
 * @param pressure Pointer to store the calculated pressure in Pascals.
 * @param temperature Pointer to store the calculated temperature in degrees C * 100.
 */
void ms5611_calculate_pressure(ms5611_dev_t *dev, uint32_t D1, uint32_t D2, int32_t *pressure, int32_t *temperature);

/* Commands derived from datasheet:
 * Reset: 0x1E
 * Convert D1 (Pressure): 0x40 + OSR_offset
 * Convert D2 (Temperature): 0x50 + OSR_offset
 * ADC Read: 0x00
 * PROM Read C1: 0xA2
 * PROM Read C2: 0xA4
 * ...
 * PROM Read C6: 0xAC
 * PROM Read CRC: 0xAE
 */

#endif // MS5611_H
