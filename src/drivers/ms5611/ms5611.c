#include "drivers/ms5611/ms5611.h"
#include <stdio.h> // For error prints, etc.
#include "common/common_types.h"

#ifdef ENABLE_DEBUG
#define MS5611_DBG(...) printf(__VA_ARGS__)
#else
#define MS5611_DBG(...) do {} while(0)
#endif

ms5611_return_code_t ms5611_init(ms5611_dev_t *dev, 
                                 ms5611_return_code_t (*platform_read)(uint8_t reg, uint8_t *data, uint32_t len),
                                 ms5611_return_code_t (*platform_write)(uint8_t reg, const uint8_t *data, uint32_t len),
                                 void (*platform_delay_us)(uint32_t period))
{
    if (!dev || !platform_read || !platform_write || !platform_delay_us) {
        return MS5611_RET_NULL_PTR;
    }

    // Assign platform interface functions
    dev->intf.read = platform_read;
    dev->intf.write = platform_write;
    dev->intf.delay_us = platform_delay_us;

    ms5611_return_code_t ret;

    // Reset the sensor
    ret = ms5611_reset(dev);
    if (ret != MS5611_RET_OK) {
        fprintf(stderr, "MS5611 Reset failed!\n");
        return ret;
    }

    ret = ms5611_read_prom(dev);
    if (ret != MS5611_RET_OK) {
        fprintf(stderr, "MS5611 PROM read failed!\n");
        return ret;
    }

    MS5611_DBG("MS5611 Initialized successfully.\n");
    return MS5611_RET_OK;
}

/**
 * @brief Sends the RESET command to the MS5611.
 */
ms5611_return_code_t ms5611_reset(ms5611_dev_t *dev) {
    if (!dev || !dev->intf.write || !dev->intf.delay_us) {
        return MS5611_RET_NULL_PTR;
    }

    ms5611_return_code_t ret = dev->intf.write(MS5611_CMD_RESET, NULL, 0);
    if (ret == MS5611_RET_OK) {
        // Wait for reset sequence (Python example used 100ms)
        dev->intf.delay_us(100000);
    }
    return ret;
}

/**
 * @brief Calculates the CRC-4 checksum for the PROM data.
 *        Based on the application note from Sensirion/Measurement Specialties.
 */
static uint8_t ms5611_crc4(uint16_t n_prom[]) {
    int cnt;            // simple counter
    uint16_t n_rem = 0;         // crc reminder (use uint16_t explicitly)
    uint8_t n_bit;

    // Per datasheet / working python code, only Word 7's lower byte (CRC byte) is zeroed for calculation.
    // Do NOT modify n_prom[0].
    n_prom[7] &= 0xFF00; // Zero the lower byte of Word 7

    for (cnt = 0; cnt < 16; cnt++) {    // operation is performed on bytes
        // choose LSB or MSB
        if (cnt % 2 == 1) n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8); // Already uint16_t

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else n_rem = (n_rem << 1);
            // Ensure n_rem remains 16-bit, matching Python's n_rem &= 0xFFFF
            // (Implicit due to uint16_t type, but good practice)
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit reminder is CRC code
    return (uint8_t)n_rem; // Cast and remove redundant XOR
}

/**
 * @brief Reads all 8 PROM coefficients (Words 0-7) from the sensor.
 */
ms5611_return_code_t ms5611_read_prom(ms5611_dev_t *dev) {
    if (!dev || !dev->intf.read) {
        return MS5611_RET_NULL_PTR;
    }

    ms5611_return_code_t ret = MS5611_RET_OK;
    uint8_t prom_data[2];

    MS5611_DBG("Reading MS5611 PROM coefficients:\n");
    // Read all 8 PROM words (addresses 0xA0 to 0xAE)
    for (int i = 0; i < 8; i++) { // Words 0..7
        // PROM read commands are 0xA0, 0xA2, ..., 0xAE
        uint8_t cmd = MS5611_CMD_PROM_RD_BASE + (i * 2); // Start at PROM read base for Word 0
        ret = dev->intf.read(cmd, prom_data, 2);
        if (ret != MS5611_RET_OK) {
            fprintf(stderr, "Error reading PROM address 0x%02X\n", cmd);
            // Clear coeffs on error?
            for (int j = i; j < 8; j++) dev->prom_coeffs[j] = 0;
            return ret;
        }
        // Combine MSB and LSB
        MS5611_DBG("    Raw bytes read for cmd 0x%02X: MSB=0x%02X, LSB=0x%02X\n", cmd, prom_data[0], prom_data[1]);
        dev->prom_coeffs[i] = (prom_data[0] << 8) | prom_data[1];

        // Print appropriately
        if (i == 0) {
            MS5611_DBG("  Word %d/Factory (Cmd 0x%02X): %u\n", i, cmd, dev->prom_coeffs[i]);
        } else if (i < 7) { // C1 to C6 are words 1 to 6
            MS5611_DBG("  Word %d/C%d (Cmd 0x%02X): %u\n", i, i, cmd, dev->prom_coeffs[i]);
        } else { // Word 7 contains CRC
            MS5611_DBG("  Word %d/CRC (Cmd 0x%02X): %u\n", i, cmd, dev->prom_coeffs[i]);
        }
        // Add a small delay after each read, potentially helps with timing
        if (dev->intf.delay_us) dev->intf.delay_us(1000); // 1 ms delay
    }

    // Extract the read CRC from the last word (Word 7)
    // The CRC is the lower 4 bits of prom_coeffs[7]
    uint8_t read_crc = (uint8_t)(dev->prom_coeffs[7] & 0x000F);
    MS5611_DBG("  Extracted CRC from Word 7: %u\n", read_crc);

    // Perform CRC check
    uint16_t prom_copy[8]; // Copy for CRC calculation (CRC function modifies the array)
    for(int i=0; i<8; i++) prom_copy[i] = dev->prom_coeffs[i]; // Copy all 8 words read

    // CRC function expects Word 0 Crc byte and Word 7 to be zeroed for calculation
    // ms5611_crc4 modifies the array in-place, so we use the copy.
    uint8_t calculated_crc = ms5611_crc4(prom_copy);
    MS5611_DBG("  Calculated CRC: %u\n", calculated_crc);

    if (calculated_crc != read_crc) {
        fprintf(stderr, "ERROR: PROM CRC mismatch! Read=%u, Calculated=%u\n", read_crc, calculated_crc);
        return MS5611_RET_CRC_ERROR;
    }
    MS5611_DBG("  CRC Check: OK\n");

    return MS5611_RET_OK;
}

/**
 * @brief Starts an ADC conversion and reads the result.
 * TODO: Add support for different OSR settings.
 */
ms5611_return_code_t ms5611_read_adc(ms5611_dev_t *dev, uint8_t conversion_cmd, uint32_t *adc_value) {
    if (!dev || !dev->intf.write || !dev->intf.read || !dev->intf.delay_us || !adc_value) {
        return MS5611_RET_NULL_PTR;
    }

    // Send conversion command
    ms5611_return_code_t ret = dev->intf.write(conversion_cmd, NULL, 0);
    if (ret != MS5611_RET_OK) {
        fprintf(stderr, "Error sending conversion command 0x%02X\n", conversion_cmd);
        return ret;
    }

    // Determine conversion time based on OSR
    uint32_t conversion_delay_us = 0;
    uint8_t osr_bits = conversion_cmd & 0x0F; // Isolate OSR bits

    switch (osr_bits) {
        case 0x00: // 256
            conversion_delay_us = MS5611_CONV_TIME_OSR256_US;
            break;
        case 0x02: // 512
            conversion_delay_us = MS5611_CONV_TIME_OSR512_US;
            break;
        case 0x04: // 1024
            conversion_delay_us = MS5611_CONV_TIME_OSR1024_US;
            break;
        case 0x06: // 2048
            conversion_delay_us = MS5611_CONV_TIME_OSR2048_US;
            break;
        case 0x08: // 4096
        default: // Default to max OSR if unknown
            conversion_delay_us = MS5611_CONV_TIME_OSR4096_US;
            break;
    }

    dev->intf.delay_us(conversion_delay_us); // Wait for conversion

    // Read ADC result (3 bytes)
    uint8_t adc_data[3];
    ret = dev->intf.read(MS5611_CMD_ADC_READ, adc_data, 3);
    if (ret != MS5611_RET_OK) {
        fprintf(stderr, "Error reading ADC value\n");
        *adc_value = 0;
        return ret;
    }

    // Combine 3 bytes into 24-bit value
    *adc_value = ((uint32_t)adc_data[0] << 16) | ((uint32_t)adc_data[1] << 8) | adc_data[2];

    return MS5611_RET_OK;
}

/**
 * @brief Calculates compensated pressure (Pascals) and temperature (degrees C * 100).
 * Based on datasheet formulas.
 */

// Wrapper for reading pressure using ms5611_read_adc
ms5611_return_code_t ms5611_read_pressure(ms5611_dev_t *dev, uint32_t *pressure_raw) {
    return ms5611_read_adc(dev, MS5611_CMD_CONV_D1_OSR4096, pressure_raw);
}

// Wrapper for reading temperature using ms5611_read_adc
ms5611_return_code_t ms5611_read_temperature(ms5611_dev_t *dev, uint32_t *temp_raw) {
    return ms5611_read_adc(dev, MS5611_CMD_CONV_D2_OSR4096, temp_raw);
}

// Wrapper for ms5611_calculate_pressure that returns a status code
ms5611_return_code_t ms5611_calculate(ms5611_dev_t *dev, uint32_t d1, uint32_t d2, int32_t *pressure, int32_t *temperature) {
    ms5611_calculate_pressure(dev, d1, d2, pressure, temperature);
    return MS5611_RET_OK;
}

void ms5611_calculate_pressure(ms5611_dev_t *dev, uint32_t D1, uint32_t D2, int32_t *pressure_pa, int32_t *temperature_c100) {
    if (!dev || !pressure_pa || !temperature_c100) {
        if (pressure_pa) *pressure_pa = 0;
        if (temperature_c100) *temperature_c100 = 0;
        return;
    }

    // Extract coefficients for easier use (cast to signed where appropriate for calculations)
    uint16_t C1 = dev->prom_coeffs[1]; // Pressure sensitivity (Word 1)
    uint16_t C2 = dev->prom_coeffs[2]; // Pressure offset (Word 2)
    uint16_t C3 = dev->prom_coeffs[3]; // Temperature coefficient of pressure sensitivity (Word 3)
    uint16_t C4 = dev->prom_coeffs[4]; // Temperature coefficient of pressure offset (Word 4)
    uint16_t C5 = dev->prom_coeffs[5]; // Reference temperature (Word 5)
    uint16_t C6 = dev->prom_coeffs[6]; // Temperature coefficient of the temperature (Word 6)
    
    // 1st order temperature and pressure calculations
    int64_t dT = (int64_t)D2 - ((int64_t)C5 << 8);
    
    int64_t TEMP = 2000 + ((dT * (int64_t)C6) >> 23);  // temp in 0.01°C
    
    int64_t OFF = ((int64_t)C2 << 16) + (((int64_t)C4 * dT) >> 7);
    int64_t SENS = ((int64_t)C1 << 15) + (((int64_t)C3 * dT) >> 8);

    // 2nd order compensation
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        // Low temperature
        T2 = (dT * dT) >> 31;
        OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 2;
        
        if (TEMP < -1500) {
            // Very low temperature (< -15°C)
            OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 11 * (TEMP + 1500) * (TEMP + 1500) / 2;
        }
        
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }
    
    *temperature_c100 = (int32_t)TEMP;

    // Final pressure (Pa)
    int64_t P_pa = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;
    *pressure_pa = (int32_t)P_pa; // Return pressure in Pascals
}
