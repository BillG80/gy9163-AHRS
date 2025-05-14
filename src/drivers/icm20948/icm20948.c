#include "drivers/icm20948/icm20948.h"
#include "drivers/icm20948/icm20948_regs.h"
#include <stddef.h> // For NULL
#include <stdio.h>  // For printf
#include <string.h> // For memset
#include <math.h> // For NAN
#include "common/common_types.h"

// --- Scale Factor Definitions ---
// Values correspond to default sensitivities:
// Accel: +/- 2g (16384 LSB/g)
// Gyro:  +/- 250 dps (131 LSB/dps)
// Mag:   AK09916 (0.15 uT/LSB -> ~6.67 LSB/uT - but datasheet formula is different)
const float ACCEL_SCALE = 1.0f / 16384.0f; // LSB/g
const float GYRO_SCALE  = 1.0f / 131.0f;   // LSB/dps
const float MAG_SCALE   = 0.15f;           // uT/LSB (Direct sensitivity from AK09916 datasheet)

// --- Helper Functions ---

// --- Forward Declarations for Internal Static Helpers ---
static icm20948_return_code_t _select_bank(icm20948_dev_t *dev, uint8_t bank);
static icm20948_return_code_t _spi_write(icm20948_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len);
static icm20948_return_code_t _spi_read(icm20948_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len);
static icm20948_return_code_t _write_reg(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, uint8_t value);
static icm20948_return_code_t _read_regs(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, uint8_t *data, uint32_t len);
static icm20948_return_code_t _ak09916_write_reg(icm20948_dev_t *dev, uint8_t reg, uint8_t value);
static icm20948_return_code_t _ak09916_init(icm20948_dev_t *dev);

// --- Function to assign platform-specific function pointers ---
void icm20948_platform_setup(icm20948_dev_t *dev) {
    // This function doesn't need to *do* anything anymore, as the pointers
    // are assigned directly by the caller (e.g., in test_icm20948.c).
    // It can optionally be used to validate that the necessary pointers are set.
    if (!dev) {
        // Optionally log an error or handle it
        return;
    }
    // Check if essential functions are provided
    if (!dev->intf.read || !dev->intf.write || !dev->intf.delay_us || 
        !dev->intf.cs_assert || !dev->intf.cs_deassert || !dev->intf.spi_transfer) {
        // Optionally log a warning/error that setup is incomplete
        // printf("Warning: ICM20948 platform setup incomplete. Some interface functions are NULL.\n");
    }
    // No assignments needed here anymore.
}

// --- Internal Helper Functions ---

// Helper to select the register bank
static icm20948_return_code_t _select_bank(icm20948_dev_t *dev, uint8_t bank) {
    // printf("    [_select_bank] Selecting bank %d\n", bank);
    // Check pointers needed for _spi_write (which this function uses)
    if (!dev || !dev->intf.cs_assert || !dev->intf.cs_deassert || !dev->intf.spi_transfer || !dev->intf.delay_us) {
        // printf("    [_select_bank] Error: NULL pointer check failed (dev=%p, assert=%p, deassert=%p, transfer=%p, delay=%p)\n",
        //        (void*)dev, (void*)dev->intf.cs_assert, (void*)dev->intf.cs_deassert, (void*)dev->intf.spi_transfer, (void*)dev->intf.delay_us);
        return ICM20948_RET_BAD_ARG;
    }

    // Read current bank selection register
    // printf("    [_select_bank] Reading current bank (Reg 0x%02X)...\n", REG_BANK_SEL);
    uint8_t current_bank;
    icm20948_return_code_t ret = _spi_read(dev, REG_BANK_SEL, &current_bank, 1);
    if (ret != ICM20948_RET_OK) {
        // printf("    [_select_bank] Error reading current bank. Code: %d\n", ret);
        return ret; // Propagate error
    }
    // printf("    [_select_bank] Read value: 0x%02X\n", current_bank);
    current_bank = (current_bank >> 4) & 0x03;
    // printf("    [_select_bank] Current bank is %u\n", current_bank);

    // If already correct bank, do nothing
    if (current_bank == bank) {
        // printf("    [_select_bank] Already on bank %d, no change needed.\n", bank);
        return ICM20948_RET_OK;
    }

    // Write new bank selection
    uint8_t bank_val = bank << 4;
    // printf("    [_select_bank] Writing bank %u (Value 0x%02X) to Reg 0x%02X...\n", bank, bank_val, REG_BANK_SEL);
    ret = _spi_write(dev, REG_BANK_SEL, &bank_val, 1);
    if (ret != ICM20948_RET_OK) {
        // printf("    [_select_bank] Error writing new bank %u. Code: %d\n", bank, ret);
        return ret; // Propagate error
    }

    // Verify bank change
    // printf("    [_select_bank] Verifying bank change (Reading Reg 0x%02X)...\n", REG_BANK_SEL);
    ret = _spi_read(dev, REG_BANK_SEL, &current_bank, 1);
    if (ret != ICM20948_RET_OK) {
        // printf("    [_select_bank] Error verifying bank change. Code: %d\n", ret);
        return ret; // Propagate error
    }
    // printf("    [_select_bank] Read value for verification: 0x%02X\n", current_bank);
    current_bank = (current_bank >> 4) & 0x03;

    if (current_bank != bank) {
        // printf("    [_select_bank] Bank verification failed! Expected %u, got %u\n", bank, current_bank);
        return ICM20948_RET_GEN_FAIL;
    }

    printf("Debug: _select_bank delay of 1ms starting\n");
    dev->intf.delay_us(1000); // 1ms delay
    printf("Debug: _select_bank delay of 1ms completed\n");
    // printf("    [_select_bank] Bank %d selected successfully.\n", bank);
    return ICM20948_RET_OK;
}

static icm20948_return_code_t _spi_write(icm20948_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len) {
    //printf("Debug: _spi_write entering for reg 0x%02X, len %u\n", reg, len);
    // printf("    [_spi_write] Start: reg=0x%02X, len=%u\n", reg, len);
    if (!dev || !dev->intf.cs_assert || !dev->intf.cs_deassert || !dev->intf.spi_transfer) {
         // printf("    [_spi_write] Error: NULL pointer check failed (dev=%p, assert=%p, deassert=%p, transfer=%p)\n",
        //        (void*)dev, (void*)dev->intf.cs_assert, (void*)dev->intf.cs_deassert, (void*)dev->intf.spi_transfer);
        return ICM20948_RET_BAD_ARG;
    }

    uint8_t tx_buf[len + 1];
    uint8_t rx_dummy_buf[len + 1]; // Dummy buffer for receiving data during write
    tx_buf[0] = reg & 0x7F;
    memcpy(&tx_buf[1], data, len);
    // printf("    [_spi_write] TX: 0x%02X", tx_buf[0]);
    // for(uint32_t i=0; i<len; ++i) printf(" 0x%02X", tx_buf[i+1]);
    // printf("\n");

    dev->intf.cs_assert();
    // printf("    [_spi_write] Asserted CS\n");
    icm20948_return_code_t ret = dev->intf.spi_transfer(tx_buf, rx_dummy_buf, len + 1);
    //printf("Debug: _spi_write spi_transfer returned %d\n", ret);
    // printf("    [_spi_write] spi_transfer returned: %d\n", ret);
    dev->intf.cs_deassert();
    // printf("    [_spi_write] Deasserted CS\n");

    //printf("Debug: _spi_write exiting with ret %d\n", ret);

    if (ret != ICM20948_RET_OK) {
        // printf("    [_spi_write] Error: spi_transfer failed with code %d\n", ret);
        return ret;
    }

    return ICM20948_RET_OK;
}

static icm20948_return_code_t _spi_read(icm20948_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len) {
    if (!dev || !dev->intf.cs_assert || !dev->intf.cs_deassert || !dev->intf.spi_transfer) {
        return ICM20948_RET_BAD_ARG; // Or NULL_PTR if dev is NULL
    }
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1]; // Separate buffer for receiving data
    tx_buf[0] = reg | 0x80; // Set MSB for read
    memset(&tx_buf[1], 0xFF, len); // Send dummy bytes (0xFF) to read

    // printf("    [_spi_read] Start: reg=0x%02X, len=%u\n", reg, len);
    // printf("    [_spi_read] TX: 0x%02X", tx_buf[0]);
    // for(uint32_t i=0; i<len; ++i) printf(" 0x%02X", tx_buf[i+1]);
    // printf("\n");

    dev->intf.cs_assert();
    // printf("    [_spi_read] Asserted CS\n");
    icm20948_return_code_t ret = dev->intf.spi_transfer(tx_buf, rx_buf, len + 1);
    // printf("    [_spi_read] spi_transfer returned: %d\n", ret);
    dev->intf.cs_deassert();
    // printf("    [_spi_read] Deasserted CS\n");

    if (ret != ICM20948_RET_OK) {
        // printf("    [_spi_read] Error: spi_transfer failed with code %d\n", ret);
        return ret;
    }

    // The actual data bytes received start from rx_buf[1]
    memcpy(data, &rx_buf[1], len);
    // printf("    [_spi_read] RX: ");
    // for(uint32_t i=0; i<len; ++i) printf(" 0x%02X", data[i]);
    // printf("\n");

    return ICM20948_RET_OK;
}

static icm20948_return_code_t _write_reg(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, uint8_t value) {
    // printf("  [_write_reg] Start: bank=%d, reg=0x%02X, value=0x%02X\n", bank, reg, value); // Debug Start
    // Check pointers needed for _select_bank and _spi_write
    if (!dev || !dev->intf.cs_assert || !dev->intf.cs_deassert || !dev->intf.spi_transfer || !dev->intf.delay_us) {
        // printf("  [_write_reg] Error: NULL pointer check failed (dev=%p, assert=%p, deassert=%p, transfer=%p, delay=%p)\n",
        //        (void*)dev, (void*)dev->intf.cs_assert, (void*)dev->intf.cs_deassert, (void*)dev->intf.spi_transfer, (void*)dev->intf.delay_us);
        return ICM20948_RET_BAD_ARG;
    }

    icm20948_return_code_t ret;

    // Select the correct bank first
    ret = _select_bank(dev, bank);
    if (ret != ICM20948_RET_OK) return ret;

    // Then write the value to the target register within the selected bank
    // printf("  [_write_reg] Calling _spi_write to write register (reg=0x%02X, value=0x%02X)\n", reg, value);
    ret = _spi_write(dev, reg, &value, 1);
    // printf("  [_write_reg] _spi_write (reg write) returned: %d\n", ret);
     if (ret != ICM20948_RET_OK) {
        // printf("  [_write_reg] Error writing reg 0x%02X in bank %d, ret = %d\n", reg, bank, ret);
        // Even if write fails, maybe attempt to switch back to bank 0 for safety?
        // For now, just return the error.
        return ret;
    }

    // Removed the automatic switch back to bank 0. Caller should manage bank context if needed.

    // printf("  [_write_reg] End\n"); // Debug End
    return ICM20948_RET_OK; // Return OK from the register write itself
}

static icm20948_return_code_t _read_regs(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, uint8_t *data, uint32_t len) {
    //printf("Debug: _read_regs entering for bank %u, reg 0x%02X, len %u\n", bank, reg, len);
    if (!dev || !data || !dev->intf.cs_assert || !dev->intf.cs_deassert || !dev->intf.spi_transfer || !dev->intf.delay_us) { // Check pointers needed for _select_bank
        // printf("    [_read_regs] Error: NULL pointer check failed\n");
        return ICM20948_RET_NULL_PTR;
    }

    // Select the correct bank first
    icm20948_return_code_t ret = _select_bank(dev, bank);
    if (ret != ICM20948_RET_OK) {
        // printf("    [_read_regs] Error: _select_bank failed with code %d\n", ret);
        return ret;
    }

    // Perform the SPI read operation
    // printf("    [_read_regs] Calling _spi_read: reg=0x%02X, len=%u\n", reg, len);
    ret = _spi_read(dev, reg, data, len);
    //printf("Debug: _read_regs exiting with ret %d\n", ret);

    return ret;
}

// --- I2C Master related helpers ---
// Note: These helpers (_ak09916_read_regs, _ak09916_write_reg, _read_i2c_slave_reg, _write_i2c_slave_reg)
// internally call _write_reg and _read_regs, which now handle bank switching via _select_bank.
// Their own pointer checks might need review if I2C master functionality is used.

static icm20948_return_code_t _ak09916_write_reg(icm20948_dev_t *dev, uint8_t reg, uint8_t value) {
    if (!dev) {
        return ICM20948_RET_NULL_PTR;
    }

    icm20948_return_code_t ret;
    uint8_t status;
    uint8_t timeout_counter = 0;

    // Configure SLV4 for the write operation
    ret = _write_reg(dev, 3, REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR); // Write address
    if (ret != ICM20948_RET_OK) return ret;
    ret = _write_reg(dev, 3, REG_I2C_SLV4_REG, reg);
    if (ret != ICM20948_RET_OK) return ret;
    ret = _write_reg(dev, 3, REG_I2C_SLV4_DO, value);
    if (ret != ICM20948_RET_OK) return ret;
    ret = _write_reg(dev, 3, REG_I2C_SLV4_CTRL, ICM20948_I2C_SLV_EN); // Corrected constant
    if (ret != ICM20948_RET_OK) return ret;

    // Add a small delay *after* enabling SLV4 and *before* polling
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 5ms starting\n");
        dev->intf.delay_us(5000); // 5ms delay
        //printf("Debug: Delay of 5ms completed\n");
    }

    // Wait for the I2C transaction to complete
    do {
        timeout_counter++;
        if (dev->intf.delay_us) {
            //printf("Debug: Delay of 5ms starting\n");
            dev->intf.delay_us(5000); // Use the increased 5ms delay
            //printf("Debug: Delay of 5ms completed\n");
        }
        ret = _read_regs(dev, 0, REG_I2C_MST_STATUS, &status, 1); // Read from Bank 0
        if (ret != ICM20948_RET_OK) {
            printf("ERROR: Failed to read I2C_MST_STATUS during SLV4 poll.\n");
            _write_reg(dev, 3, REG_I2C_SLV4_CTRL, 0x00); // Attempt to disable SLV4
            return ret;
        }
        if (timeout_counter > 100) {
            printf("ERROR: Timeout waiting for SLV4 transaction completion (Status=0x%02X).\n", status);
            _write_reg(dev, 3, REG_I2C_SLV4_CTRL, 0x00); // Attempt to disable SLV4
            return ICM20948_RET_GEN_FAIL; // Timeout error
        }
    } while (!(status & ICM20948_I2C_MST_STATUS_SLV4_DONE));

    // Check for NACK
    if (status & ICM20948_I2C_MST_STATUS_SLV4_NACK) {
        printf("ERROR: _ak09916_write_reg - SLV4 NACK received! Status=0x%02X\n", status);
        _write_reg(dev, 3, REG_I2C_SLV4_CTRL, 0x00); // Attempt to disable SLV4
        return ICM20948_RET_GEN_FAIL; // Return a failure code
    }  

    // Disable SLV4 (best effort)
    _write_reg(dev, 3, REG_I2C_SLV4_CTRL, 0x00);

    return ICM20948_RET_OK;
}

static icm20948_return_code_t _ak09916_init(icm20948_dev_t *dev) {
    icm20948_return_code_t ret;
    // uint8_t who_am_i[2] = {0xFF, 0xFF}; // WHO_AM_I check removed for SLV0 auto-read

    // AK09916 soft reset using helper function
    printf("Debug: Resetting AK09916 via SLV4\n");
    ret = _ak09916_write_reg(dev, AK09916_REG_CNTL3, 0x01); // Soft reset
    if (ret != ICM20948_RET_OK) {
        printf("ERROR: Failed AK09916 soft reset via SLV4.\n");
        return ret;
    }
    // Wait after AK09916 reset (MEM: ~10ms needed)
    if (dev->intf.delay_us) {
         //printf("Debug: Delay of 10ms starting after AK09916 reset\n");
         dev->intf.delay_us(10000);
         //printf("Debug: Delay of 10ms completed after AK09916 reset\n");
    }

    // Set AK09916 mode using helper function
    printf("Debug: Setting AK09916 mode via SLV4\n");
    ret = _ak09916_write_reg(dev, AK09916_REG_CNTL2, 0x08); // Continuous 100Hz mode
    if (ret != ICM20948_RET_OK) {
        printf("ERROR: Failed to set AK09916 mode via SLV4.\n");
        return ret;
    }
    // Small delay after setting mode might be prudent
     if (dev->intf.delay_us) {
         //printf("Debug: Delay of 1ms starting after AK09916 mode set\n");
         dev->intf.delay_us(1000);
         //printf("Debug: Delay of 1ms completed after AK09916 mode set\n");
    }

    // Configure I2C SLV0 to read 8 bytes starting from AK09916_REG_HXL (0x11)
    // This reads HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2
    ret = _write_reg(dev, 3, REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // Use 0x80 for I2C read flag
    if (ret != ICM20948_RET_OK) return ret;
    ret = _write_reg(dev, 3, REG_I2C_SLV0_REG, AK09916_REG_HXL); // Start address for read
    if (ret != ICM20948_RET_OK) return ret;
    ret = _write_reg(dev, 3, REG_I2C_SLV0_CTRL, ICM20948_I2C_SLV_EN | 8); // Enable SLV0, read 8 bytes
    if (ret != ICM20948_RET_OK) return ret;

    return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_read_mag(icm20948_dev_t *dev, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
    if (!dev || !mag_x || !mag_y || !mag_z) {
        return ICM20948_RET_NULL_PTR;
    }

    icm20948_return_code_t ret;
    uint8_t buf[8]; // Buffer to hold HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2

    // Read the 8 bytes of data stored by I2C Master from SLV0
    // These registers are in Bank 0
    ret = _read_regs(dev, 0, REG_EXT_SLV_SENS_DATA_00, buf, 8);
    if (ret != ICM20948_RET_OK) {
        printf("ERROR: Failed to read EXT_SLV_SENS_DATA for magnetometer. ret=%d\n", ret);
        *mag_x = 0;
        *mag_y = 0;
        *mag_z = 0;
        return ret;
    }

    // Check AK09916 ST2 register (buf[7]) for data validity
    // Check for Data Overrun (DOR) bit 2 (0x04) and Sensor Overflow (HOFL) bit 3 (0x08)
    if (buf[7] & 0x0C) {
        // DOR or HOFL occurred, data is invalid.
        *mag_x = 0; // Zero out potentially invalid data
        *mag_y = 0;
        *mag_z = 0;
        return ICM20948_RET_MAG_DATA_ERROR;
    }

    // Reconstruct the 16-bit values (Little Endian: Low Byte first)
    *mag_x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *mag_y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    *mag_z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

    return ICM20948_RET_OK;
}

// Read raw temperature data
icm20948_return_code_t icm20948_read_temp_raw(icm20948_dev_t *dev, int16_t *temp) {
    if (!dev || !temp) {
        return ICM20948_RET_NULL_PTR;
    }

    uint8_t data[2];
    icm20948_return_code_t ret = _read_regs(dev, 0, REG_TEMP_OUT_H, data, 2);
    if (ret != ICM20948_RET_OK) return ret;

    *temp = (int16_t)(((uint16_t)data[0] << 8) | data[1]);

    return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_read_temp(icm20948_dev_t *dev, float *temp_c) {
    if (!dev || !temp_c) {
        return ICM20948_RET_NULL_PTR;
    }

    uint8_t data[2];
    icm20948_return_code_t ret;

    // Temperature registers are in Bank 0
    ret = _read_regs(dev, 0, REG_TEMP_OUT_H, data, 2);
    if (ret != ICM20948_RET_OK) {
        printf("ERROR: Failed to read temperature registers (ret=%d)\n", ret);
        *temp_c = NAN; // Indicate failure
        return ret;
    }

    // Combine high and low bytes
    int16_t temp_raw = (int16_t)((data[0] << 8) | data[1]);

    // Apply conversion formula from datasheet
    // Temperature (°C) = (TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity + 21.0
    // RoomTemp_Offset = 0 (typically)
    // Temp_Sensitivity = 333.87 LSB/°C
    *temp_c = ((float)temp_raw / 333.87f) + 21.0f;

    return ICM20948_RET_OK;
}

// Read the main ICM20948 WHO_AM_I register
icm20948_return_code_t icm20948_read_who_am_i(icm20948_dev_t *dev, uint8_t *who_am_i) {
    if (!dev || !who_am_i) return ICM20948_RET_NULL_PTR;
    icm20948_return_code_t ret = _read_regs(dev, 0, REG_WHO_AM_I, who_am_i, 1);
    if (ret != ICM20948_RET_OK) return ret;
    // Don't check value here, let caller do it
    return ICM20948_RET_OK;
}

// Read all sensor data at once
icm20948_return_code_t icm20948_read_sensor_data(icm20948_dev_t *dev, icm20948_sensor_data_t *data) {
    if (!dev || !data) {
        return ICM20948_RET_NULL_PTR;
    }

    icm20948_return_code_t ret;

    // Read accelerometer
    ret = icm20948_read_accel(dev, &data->accel_x, &data->accel_y, &data->accel_z);
    if (ret != ICM20948_RET_OK) return ret;

    // Read gyroscope
    ret = icm20948_read_gyro(dev, &data->gyro_x, &data->gyro_y, &data->gyro_z);
    if (ret != ICM20948_RET_OK) return ret;

    // Read magnetometer (using the new SLV0 auto-read method)
    ret = icm20948_read_mag(dev, &data->mag_x, &data->mag_y, &data->mag_z);
    if (ret != ICM20948_RET_OK) {
        if (ret == ICM20948_RET_MAG_DATA_ERROR) {
            // Propagate the specific magnetometer data error
            return ret;
        } else {
            // For other errors (e.g., communication failure), log warning, zero data, and continue
            printf("WARNING: icm20948_read_mag failed (ret=%d). Setting mag data to 0.\n", ret);
            data->mag_x = 0;
            data->mag_y = 0;
            data->mag_z = 0;
            // Fall through to read temperature, allowing Acc/Gyro to still be used
        }
    }

    // Read temperature
    ret = icm20948_read_temp_raw(dev, &data->temp);
    if (ret != ICM20948_RET_OK) return ret; 
    
    // Note: Status fields in icm20948_sensor_data_t are not populated by this function
    data->accel_status = 0; // Placeholder
    data->gyro_status = 0;  // Placeholder
    data->mag_status = 0;   // Placeholder

    return ICM20948_RET_OK;
}

// Public function to select register bank
icm20948_return_code_t icm20948_select_bank(icm20948_dev_t *dev, uint8_t bank) {
    // Basic validation
    if (!dev) return ICM20948_RET_NULL_PTR;
    if (bank > 3) {
        return ICM20948_RET_BAD_ARG; // Invalid bank number
    }
    // Call the internal helper function
    return _select_bank(dev, bank);
}

// --- Public API Functions ---

icm20948_return_code_t icm20948_write_reg(icm20948_dev_t *dev, uint8_t reg, uint8_t value) {
    if (!dev || !dev->intf.spi_transfer || !dev->intf.cs_assert || !dev->intf.cs_deassert) {
        return ICM20948_RET_NULL_PTR;
    }

    uint8_t tx_buf[2];
    icm20948_return_code_t ret;

    tx_buf[0] = reg & ~SPI_READ_FLAG; // Register address with write flag (MSB=0)
    tx_buf[1] = value;                // Data byte to write

    dev->intf.cs_assert();
    ret = dev->intf.spi_transfer(tx_buf, NULL, 2); // Transfer 2 bytes (reg + data)
    dev->intf.cs_deassert();

    if (ret == ICM20948_RET_OK && dev->intf.delay_us) {
        //printf("Debug: Delay of 10us starting\n");
        dev->intf.delay_us(10); // Small delay after write might be needed
        //printf("Debug: Delay of 10us completed\n");
    }

    return ret;
}

icm20948_return_code_t icm20948_read_reg(icm20948_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len) {
    if (!dev || !dev->intf.spi_transfer || !dev->intf.cs_assert || !dev->intf.cs_deassert || !data) {
        return ICM20948_RET_NULL_PTR;
    }
    if (len == 0) {
        return ICM20948_RET_BAD_ARG;
    }

    // Max transfer size check might be needed depending on platform buffer limitations
    // For now, assume len is reasonable.

    // Need temporary buffers for transfer, size = 1 (reg addr) + len (data)
    // Using stack allocation; consider dynamic if len can be very large
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];
    icm20948_return_code_t ret;

    tx_buf[0] = reg | SPI_READ_FLAG; // Register address with read flag (MSB=1)
    // Fill rest of tx_buf with dummy bytes (0xFF often used)
    for(uint32_t i = 1; i <= len; ++i) {
        tx_buf[i] = 0xFF;
    }

    dev->intf.cs_assert();
    // Transfer 1 byte (register address) + len bytes (for reading data)
    ret = dev->intf.spi_transfer(tx_buf, rx_buf, len + 1);
    dev->intf.cs_deassert();

    if (ret == ICM20948_RET_OK) {
        // Copy received data (skip first byte which corresponds to reg addr transfer)
        memcpy(data, &rx_buf[1], len);
    }

    return ret;
}

/**
 * @brief Initializes the ICM20948 sensor.
 */
icm20948_return_code_t icm20948_init(icm20948_dev_t *dev) {
    printf("Debug: Starting ICM20948 init - Verifying WHO_AM_I\n");
    // Check essential platform functions needed by the internal SPI helpers
    if (!dev || !dev->intf.spi_transfer || !dev->intf.cs_assert || !dev->intf.cs_deassert || !dev->intf.delay_us) {
        return ICM20948_RET_NULL_PTR;
    }

    icm20948_return_code_t ret;
    uint8_t data;

    // Add startup delay
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 100ms starting\n");
        dev->intf.delay_us(100000);  // 100ms startup delay
        //printf("Debug: Delay of 100ms completed\n");
    }

    // Initialize current_bank state
    dev->current_bank = 0xFF; // Force initial bank selection

    // Select bank 0 and verify WHO_AM_I first
    ret = _select_bank(dev, 0);
    if (ret != ICM20948_RET_OK) return ret;
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 1ms starting\n");
        dev->intf.delay_us(1000);
        //printf("Debug: Delay of 1ms completed\n");
    }

    printf("Debug: Verifying WHO_AM_I register\n");
    printf("Debug: Attempting WHO_AM_I read\n");
    ret = _read_regs(dev, 0, REG_WHO_AM_I, &data, 1);
    printf("Debug: WHO_AM_I read returned %d, value: 0x%02X\n", ret, data);
    if (ret != ICM20948_RET_OK) return ret;
    if (data != ICM20948_WHO_AM_I_VAL) {
        printf("ERROR: ICM20948 WHO_AM_I mismatch! Read: 0x%02X, Expected: 0x%02X\n", data, ICM20948_WHO_AM_I_VAL);
        return ICM20948_RET_GEN_FAIL;
    }
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 1ms starting\n");
        dev->intf.delay_us(1000);
        //printf("Debug: Delay of 1ms completed\n");
    }

    printf("Debug: Waking up ICM20948\n");
    // Reset the device
    data = ICM20948_PWR_MGMT_1_DEVICE_RESET;
    ret = _write_reg(dev, 0, REG_PWR_MGMT_1, data);
    if (ret != ICM20948_RET_OK) return ret;
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 100ms starting\n");
        dev->intf.delay_us(100000);  // 100ms reset delay
        //printf("Debug: Delay of 100ms completed\n");
    }
    dev->current_bank = 0xFF; // Reset bank cache after device reset

    printf("Debug: Resetting ICM20948\n");
    // Clear sleep bit, set clock source to Auto Select
    data = ICM20948_PWR_MGMT_1_CLKSEL_AUTO;
    ret = _write_reg(dev, 0, REG_PWR_MGMT_1, data);
    if (ret != ICM20948_RET_OK) return ret;
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 10ms starting\n");
        dev->intf.delay_us(10000);
        //printf("Debug: Delay of 10ms completed\n");
    }

    printf("Debug: Waking ICM20948 and setting clock source\n");
    // Wake up device
    data = 0x01; // Wake up device, clock = auto select
    ret = _write_reg(dev, 0, REG_PWR_MGMT_1, data);
    if (ret != ICM20948_RET_OK) return ret;
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 50ms starting\n");
        dev->intf.delay_us(50000); // Add 50ms stabilization delay
        //printf("Debug: Delay of 50ms completed\n");
    }

    printf("Debug: Configuring I2C master and AK09916 initialization\n");
    uint8_t user_ctrl_write = ICM20948_USERCTRL_I2C_IF_DIS | ICM20948_USERCTRL_I2C_MST_EN;
    printf("Debug: Attempting to write 0x%02X to REG_USER_CTRL (Bank 0, Addr 0x03)\n", user_ctrl_write);
    ret = _write_reg(dev, 0, REG_USER_CTRL, user_ctrl_write);
    if (ret != ICM20948_RET_OK) {
        //printf("Error: Failed to write USER_CTRL (ret=%d)\n", ret);
        return ret;
    }
    // Add a small delay before reading back
    if (dev->intf.delay_us) { dev->intf.delay_us(1000); }

    // Verify USER_CTRL by reading it back
    uint8_t user_ctrl_read = 0;
    ret = _read_regs(dev, 0, REG_USER_CTRL, &user_ctrl_read, 1);
    if (ret != ICM20948_RET_OK) {
        //printf("Error: Failed to read back USER_CTRL (ret=%d)\n", ret);
        // Attempt to disable I2C Master if readback fails? Might not be necessary.
        return ret;
    }
    printf("Debug: Read back USER_CTRL = 0x%02X\n", user_ctrl_read);

    // Check if the required bits are set
    if ((user_ctrl_read & user_ctrl_write) != user_ctrl_write) {
         //printf("ERROR: USER_CTRL verification failed! Wrote 0x%02X, Read 0x%02X. Required bits not set.\n",
        //        user_ctrl_write, user_ctrl_read);
         return ICM20948_RET_GEN_FAIL;
    }
    printf("Debug: USER_CTRL verified successfully (I2C Master Enabled, I2C Interface Disabled)\n");

    // Set I2C Master clock speed (~400kHz)
    ret = _select_bank(dev, 3);
    if (ret != ICM20948_RET_OK) return ret;
    printf("Debug: Setting I2C master clock speed\n");
    uint8_t i2c_mst_ctrl = 0x07;  // 400kHz clock speed
    ret = _write_reg(dev, 3, REG_I2C_MST_CTRL, i2c_mst_ctrl);
    if (ret != ICM20948_RET_OK) {
        //printf("Error: Failed to set I2C_MST_CTRL\n");
        return ret;
    }
    ret = _select_bank(dev, 0);
    if (ret != ICM20948_RET_OK) return ret;
    //printf("Debug: Bank switched back to 0\n");

    // Initialize magnetometer (AK09916) using I2C Master, with retries
    const int max_ak_init_retries = 3;
    for (int i = 0; i < max_ak_init_retries; ++i) {
        ret = _ak09916_init(dev);
        if (ret == ICM20948_RET_OK) {
            break; // Success
        }
        //printf("WARN: _ak09916_init attempt %d/%d failed (ret=%d). Retrying...\n", i + 1, max_ak_init_retries, ret);
        if (dev->intf.delay_us) {
            //printf("Debug: Delay of 100ms starting\n");
            dev->intf.delay_us(100 * 1000); // 100ms delay before retry
            //printf("Debug: Delay of 100ms completed\n");
        }
    }

    // Check final result after retries
    if (ret != ICM20948_RET_OK) {
        //printf("ERROR: _ak09916_init failed after %d attempts.\n", max_ak_init_retries);
        return ret;
    }

    printf("ICM20948 initialized successfully.\n");
    return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_read_accel(icm20948_dev_t *dev, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    if (!dev || !accel_x || !accel_y || !accel_z) return ICM20948_RET_NULL_PTR;

    uint8_t data[6];
    icm20948_return_code_t ret = _read_regs(dev, 0, REG_ACCEL_XOUT_H, data, 6);
    if (ret != ICM20948_RET_OK) return ret;

    *accel_x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *accel_y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *accel_z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_read_gyro(icm20948_dev_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    if (!dev || !gyro_x || !gyro_y || !gyro_z) {
        return ICM20948_RET_NULL_PTR;
    }

    uint8_t data[6];
    icm20948_return_code_t ret = _read_regs(dev, 0, REG_GYRO_XOUT_H, data, 6);
    if (ret != ICM20948_RET_OK) return ret;

    *gyro_x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *gyro_y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *gyro_z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    return ICM20948_RET_OK;
}

icm20948_return_code_t icm20948_device_reset(icm20948_dev_t *dev) {
    if (!dev) return ICM20948_RET_NULL_PTR;

    uint8_t data = 0;
    icm20948_return_code_t ret;

    // --- Attempt to explicitly disable I2C Master first ---
    ret = _read_regs(dev, 0, REG_USER_CTRL, &data, 1);
    if (ret == ICM20948_RET_OK) { // Proceed even if read fails, reset is main goal
        data &= ~ICM20948_USERCTRL_I2C_MST_EN;
        ret = _write_reg(dev, 0, REG_USER_CTRL, data);
        // Ignore write error here, prioritize the device reset
    }
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 10ms starting\n");
        dev->intf.delay_us(10 * 1000); // 10ms delay after disabling master
        //printf("Debug: Delay of 10ms completed\n");
    }

    // Read PWR_MGMT_1 register (Bank 0)
    ret = _read_regs(dev, 0, REG_PWR_MGMT_1, &data, 1);
    if (ret != ICM20948_RET_OK) return ret;

    // Set the device reset bit
    data |= ICM20948_PWR_MGMT_1_DEVICE_RESET;

    // Write back to trigger reset
    ret = _write_reg(dev, 0, REG_PWR_MGMT_1, data);
    if (ret != ICM20948_RET_OK) return ret;

    // Wait for reset to complete (datasheet recommends >100ms)
    if (dev->intf.delay_us) {
        //printf("Debug: Delay of 250ms starting\n");
        dev->intf.delay_us(250 * 1000); // 250ms delay
        //printf("Debug: Delay of 250ms completed\n");
    }

    return ICM20948_RET_OK;
}