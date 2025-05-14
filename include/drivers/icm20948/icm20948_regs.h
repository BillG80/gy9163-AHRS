#ifndef ICM20948_REGS_H
#define ICM20948_REGS_H

// This header defines register addresses and constants specific to the ICM-20948
// It is intended for internal use by the driver (icm20948.c)
// Public constants (like expected WHO_AM_I) should be in icm20948.h

// --- SPI Flags ---
#define SPI_READ_FLAG  0x80
#define SPI_WRITE_FLAG 0x00 // Mask to ensure write bit is 0 (often implicitly 0x7F & reg)

// --- Special Registers ---
#define REG_BANK_SEL 0x7F // Register Bank Select

// --- User Bank 0 Registers ---
#define REG_WHO_AM_I        0x00 // Should return 0xEA
#define REG_USER_CTRL       0x03
#define REG_LP_CONFIG       0x05
#define REG_PWR_MGMT_1      0x06
#define REG_PWR_MGMT_2      0x07
#define REG_INT_PIN_CFG     0x0F
#define REG_INT_ENABLE      0x10
#define REG_INT_ENABLE_1    0x11
#define REG_INT_ENABLE_2    0x12
#define REG_INT_ENABLE_3    0x13
#define REG_I2C_MST_STATUS  0x17
#define REG_INT_STATUS      0x19
#define REG_INT_STATUS_1    0x1A
#define REG_INT_STATUS_2    0x1B
#define REG_INT_STATUS_3    0x1C
#define REG_DELAY_TIMEH     0x28
#define REG_DELAY_TIMEL     0x29
#define REG_ACCEL_XOUT_H    0x2D
#define REG_ACCEL_XOUT_L    0x2E
#define REG_ACCEL_YOUT_H    0x2F
#define REG_ACCEL_YOUT_L    0x30
#define REG_ACCEL_ZOUT_H    0x31
#define REG_ACCEL_ZOUT_L    0x32
#define REG_GYRO_XOUT_H     0x33
#define REG_GYRO_XOUT_L     0x34
#define REG_GYRO_YOUT_H     0x35
#define REG_GYRO_YOUT_L     0x36
#define REG_GYRO_ZOUT_H     0x37
#define REG_GYRO_ZOUT_L     0x38
#define REG_TEMP_OUT_H      0x39
#define REG_TEMP_OUT_L      0x3A
#define REG_EXT_SLV_SENS_DATA_00 0x3B // Start of data read from external sensors (e.g., magnetometer)
// ... up to REG_EXT_SLV_SENS_DATA_23 0x52
#define REG_FIFO_EN_1       0x66
#define REG_FIFO_EN_2       0x67
#define REG_FIFO_RST        0x68
#define REG_FIFO_MODE       0x69
#define REG_FIFO_COUNTH     0x70
#define REG_FIFO_COUNTL     0x71
#define REG_FIFO_R_W        0x72
#define REG_DATA_RDY_STATUS 0x74
#define REG_FIFO_CFG        0x76

// --- User Bank 1 Registers ---
// (Add if needed)
#define REG_SELF_TEST_X_GYRO 0x02
#define REG_SELF_TEST_Y_GYRO 0x03
#define REG_SELF_TEST_Z_GYRO 0x04
#define REG_SELF_TEST_X_ACCEL 0x0E
#define REG_SELF_TEST_Y_ACCEL 0x0F
#define REG_SELF_TEST_Z_ACCEL 0x10
#define REG_XA_OFFS_H       0x14
#define REG_XA_OFFS_L       0x15
#define REG_YA_OFFS_H       0x17
#define REG_YA_OFFS_L       0x18
#define REG_ZA_OFFS_H       0x1A
#define REG_ZA_OFFS_L       0x1B
#define REG_TIMEBASE_CORRECTION_PLL 0x28

// --- User Bank 2 Registers ---
#define REG_GYRO_SMPLRT_DIV 0x00
#define REG_GYRO_CONFIG_1   0x01
#define REG_GYRO_CONFIG_2   0x02
#define REG_XG_OFFS_USRH    0x03
#define REG_XG_OFFS_USRL    0x04
#define REG_YG_OFFS_USRH    0x05
#define REG_YG_OFFS_USRL    0x06
#define REG_ZG_OFFS_USRH    0x07
#define REG_ZG_OFFS_USRL    0x08
#define REG_ODR_ALIGN_EN    0x09
#define REG_ACCEL_SMPLRT_DIV_1 0x10
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ACCEL_INTEL_CTRL 0x12
#define REG_ACCEL_WOM_THR   0x13
#define REG_ACCEL_CONFIG    0x14
#define REG_ACCEL_CONFIG_2  0x15
#define REG_FSYNC_CONFIG    0x52
#define REG_TEMP_CONFIG     0x53
#define REG_MOD_CTRL_USR    0x54

// --- User Bank 3 Registers ---
#define REG_I2C_MST_CTRL    0x01
#define REG_I2C_MST_ODR_CONFIG 0x00
#define REG_I2C_MST_DELAY_CTRL 0x02
#define REG_I2C_SLV0_ADDR   0x03
#define REG_I2C_SLV0_REG    0x04
#define REG_I2C_SLV0_CTRL   0x05
#define REG_I2C_SLV0_DO     0x06
#define REG_I2C_SLV1_ADDR   0x07
#define REG_I2C_SLV1_REG    0x08
#define REG_I2C_SLV1_CTRL   0x09
#define REG_I2C_SLV1_DO     0x0A
#define REG_I2C_SLV2_ADDR   0x0B
#define REG_I2C_SLV2_REG    0x0C
#define REG_I2C_SLV2_CTRL   0x0D
#define REG_I2C_SLV2_DO     0x0E
#define REG_I2C_SLV3_ADDR   0x0F
#define REG_I2C_SLV3_REG    0x10
#define REG_I2C_SLV3_CTRL   0x11
#define REG_I2C_SLV3_DO     0x12
#define REG_I2C_SLV4_ADDR   0x13
#define REG_I2C_SLV4_REG    0x14
#define REG_I2C_SLV4_CTRL   0x15
#define REG_I2C_SLV4_DO     0x16
#define REG_I2C_SLV4_DI     0x17

// --- Register Bit Definitions ---

// REG_PWR_MGMT_1 (Bank 0)
#define ICM20948_PWR_MGMT_1_DEVICE_RESET (1 << 7)
#define ICM20948_PWR_MGMT_1_SLEEP        (1 << 6)
#define ICM20948_PWR_MGMT_1_LP_EN        (1 << 5)
#define ICM20948_PWR_MGMT_1_TEMP_DIS     (1 << 3)
#define ICM20948_PWR_MGMT_1_CLKSEL_MASK  0x07
#define ICM20948_PWR_MGMT_1_CLKSEL_AUTO  0x01

// REG_PWR_MGMT_2 (Bank 0)
#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL (0x38) // Bits 5:3 = 111 to disable Accel (all axes)
#define ICM20948_PWR_MGMT_2_DISABLE_GYRO  (0x07) // Bits 2:0 = 111 to disable Gyro (all axes)

// REG_GYRO_CONFIG_1 (Bank 2)
#define ICM20948_GYRO_CONFIG_1_FS_SEL_MASK  (3 << 1) // Bits 2:1
#define ICM20948_GYRO_FS_SEL_250DPS         (0 << 1)
#define ICM20948_GYRO_FS_SEL_500DPS         (1 << 1)
#define ICM20948_GYRO_FS_SEL_1000DPS        (2 << 1)
#define ICM20948_GYRO_FS_SEL_2000DPS        (3 << 1)
#define ICM20948_GYRO_CONFIG_1_FCHOICE_DLPF_EN (1 << 0) // Bit 0: 1 = Use DLPF, 0 = Bypass DLPF
#define ICM20948_GYRO_CONFIG_1_DLPFCFG_MASK (7 << 3) // Bits 5:3
// Gyro DLPF Cutoffs (Examples - see datasheet table)
#define ICM20948_GYRO_DLPFCFG_196_6HZ       (0 << 3)
#define ICM20948_GYRO_DLPFCFG_151_8HZ       (1 << 3)
#define ICM20948_GYRO_DLPFCFG_119_5HZ       (2 << 3)
#define ICM20948_GYRO_DLPFCFG_51_2HZ        (3 << 3) // Used in example
#define ICM20948_GYRO_DLPFCFG_23_9HZ        (4 << 3)
#define ICM20948_GYRO_DLPFCFG_11_6HZ        (5 << 3)
#define ICM20948_GYRO_DLPFCFG_5_7HZ         (6 << 3)
#define ICM20948_GYRO_DLPFCFG_361_4HZ       (7 << 3)

// REG_ACCEL_CONFIG (Bank 2)
#define ICM20948_ACCEL_CONFIG_FS_SEL_MASK (3 << 1) // Bits 2:1
#define ICM20948_ACCEL_FS_SEL_2G          (0 << 1)
#define ICM20948_ACCEL_FS_SEL_4G          (1 << 1)
#define ICM20948_ACCEL_FS_SEL_8G          (2 << 1)
#define ICM20948_ACCEL_FS_SEL_16G         (3 << 1)
#define ICM20948_ACCEL_CONFIG_FCHOICE_DLPF_EN (1 << 0) // Bit 0: 1 = Use DLPF, 0 = Bypass DLPF
#define ICM20948_ACCEL_CONFIG_DLPFCFG_MASK (7 << 3) // Bits 5:3
// Accel DLPF Cutoffs (Examples - see datasheet table)
#define ICM20948_ACCEL_DLPFCFG_246HZ        (1 << 3)
#define ICM20948_ACCEL_DLPFCFG_111_4HZ      (2 << 3)
#define ICM20948_ACCEL_DLPFCFG_50_4HZ       (3 << 3) // Used in example
#define ICM20948_ACCEL_DLPFCFG_23_9HZ       (4 << 3)
#define ICM20948_ACCEL_DLPFCFG_11_5HZ       (5 << 3)
#define ICM20948_ACCEL_DLPFCFG_5_7HZ        (6 << 3)
#define ICM20948_ACCEL_DLPFCFG_473HZ        (7 << 3)

// --- USER BANK 3 Registers ---
// I2C Master Config
#define REG_I2C_MST_ODR_CONFIG  0x00 // Output data rate for aux sensors
#define REG_I2C_MST_CTRL        0x01 // I2C Master Control
#define REG_I2C_MST_DELAY_CTRL  0x02 // Delay controls for master
#define REG_I2C_SLV0_ADDR       0x03 // Slave 0 Address
#define REG_I2C_SLV0_REG        0x04 // Slave 0 Register
#define REG_I2C_SLV0_CTRL       0x05 // Slave 0 Control
#define REG_I2C_SLV0_DO         0x06 // Slave 0 Data Out
#define REG_I2C_SLV1_ADDR       0x07 // Slave 1 Address
#define REG_I2C_SLV1_REG        0x08 // Slave 1 Register
#define REG_I2C_SLV1_CTRL       0x09 // Slave 1 Control
#define REG_I2C_SLV1_DO         0x0A // Slave 1 Data Out
#define REG_I2C_SLV2_ADDR       0x0B // ...
#define REG_I2C_SLV2_REG        0x0C
#define REG_I2C_SLV2_CTRL       0x0D
#define REG_I2C_SLV2_DO         0x0E
#define REG_I2C_SLV3_ADDR       0x0F // ...
#define REG_I2C_SLV3_REG        0x10
#define REG_I2C_SLV3_CTRL       0x11
#define REG_I2C_SLV3_DO         0x12
#define REG_I2C_SLV4_ADDR       0x13 // Slave 4 Address (for single transactions)
#define REG_I2C_SLV4_REG        0x14 // Slave 4 Register
#define REG_I2C_SLV4_CTRL       0x15 // Slave 4 Control
#define REG_I2C_SLV4_DO         0x16 // Slave 4 Data Out
#define REG_I2C_SLV4_DI         0x17 // Slave 4 Data In

// I2C Master Status (Also in Bank 0, REG_I2C_MST_STATUS 0x17)
// #define REG_I2C_MST_STATUS      0x18 // In Bank 3 (duplicate name, use Bank 0 one)


// --- USER BANK 0 Registers (Additions) ---
#define REG_USER_CTRL           0x03 // User Control (I2C_MST_EN is here)
#define REG_I2C_MST_STATUS      0x17 // I2C Master Status (use this one)
// External Sensor Data registers
#define REG_EXT_SLV_SENS_DATA_00 0x3B
#define REG_EXT_SLV_SENS_DATA_01 0x3C
#define REG_EXT_SLV_SENS_DATA_02 0x3D
#define REG_EXT_SLV_SENS_DATA_03 0x3E
#define REG_EXT_SLV_SENS_DATA_04 0x3F
#define REG_EXT_SLV_SENS_DATA_05 0x40
#define REG_EXT_SLV_SENS_DATA_06 0x41
#define REG_EXT_SLV_SENS_DATA_07 0x42
#define REG_EXT_SLV_SENS_DATA_08 0x43
// ... up to 23 (0x52)


// --- Bit Definitions ---
// REG_USER_CTRL (Bank 0)
#define ICM20948_USERCTRL_DMP_EN        (1 << 7)
#define ICM20948_USERCTRL_FIFO_EN       (1 << 6)
#define ICM20948_USERCTRL_I2C_MST_EN    (1 << 5)
#define ICM20948_USERCTRL_I2C_IF_DIS    (1 << 4) // Disable SPI, enable I2C for primary interface
#define ICM20948_USERCTRL_DMP_RST       (1 << 3)
#define ICM20948_USERCTRL_SRAM_RST      (1 << 2)
#define ICM20948_USERCTRL_I2C_MST_RST   (1 << 1)

// REG_I2C_MST_CTRL (Bank 3)
#define ICM20948_I2C_MST_CTRL_CLK_400KHZ 0x07 // Example clock speed
#define ICM20948_I2C_MST_CTRL_P_NSR      (1 << 4) // Stop condition control
#define ICM20948_I2C_MST_CTRL_MULT_MST_EN (1 << 7) // Multi-master enable

// REG_I2C_SLV0_ADDR / REG_I2C_SLV4_ADDR (Bank 3)
#define ICM20948_I2C_SLV_RNW             (1 << 7) // Read=1, Write=0

// REG_I2C_SLV0_CTRL / REG_I2C_SLV4_CTRL (Bank 3)
#define ICM20948_I2C_SLV_EN              (1 << 7) // Enable this slave for transactions
#define ICM20948_I2C_SLV_BYTE_SW         (1 << 6) // Byte swapping
#define ICM20948_I2C_SLV_REG_DIS         (1 << 5) // Disable writing the register address phase
#define ICM20948_I2C_SLV_GRP             (1 << 4) // Grouping for reads
#define ICM20948_I2C_SLV_LENG_MASK       0x0F     // Mask for transaction length (bytes)

// REG_I2C_SLV4_CTRL (Bank 3) - Specific to SLV4
#define ICM20948_I2C_SLV4_DONE_INT_EN    (1 << 6) // Enable SLV4 done interrupt
#define ICM20948_I2C_SLV4_REG_DIS        (1 << 5) // Disable writing the register address phase
#define ICM20948_I2C_SLV4_CTRL_DLY_EN   (1 << 5)
#define ICM20948_I2C_SLV4_CTRL_REG_DIS (1 << 4)
#define ICM20948_I2C_SLV4_CTRL_EN      (1 << 7)

// REG_I2C_MST_STATUS (Bank 0)
#define ICM20948_I2C_MST_STATUS_SLV4_DONE (1 << 6) // SLV4 transaction complete
#define ICM20948_I2C_MST_STATUS_LOST_ARB (1 << 5) // Lost arbitration
#define ICM20948_I2C_MST_STATUS_SLV4_NACK (1 << 4) // SLV4 NACK received
#define ICM20948_I2C_MST_STATUS_SLV0_NACK (1 << 0) // SLV0 NACK received
#define ICM20948_I2C_MST_STATUS_PASS_THROUGH (1 << 7)

// REG_INT_PIN_CFG (Bank 0, 0x0F)
#define ICM20948_INT_PIN_CFG_BYPASS_EN  (1 << 1)

// --- AK09916 Magnetometer Specific ---
#define AK09916_I2C_ADDR        0x0C // Default I2C Address
#define AK09916_REG_WIA1        0x00 // Company ID (Should be 0x48)
#define AK09916_REG_WIA2        0x01 // Device ID (Should be 0x09)
#define AK09916_REG_ST1         0x10 // Status 1 (DRDY)
#define AK09916_REG_HXL         0x11 // Mag Data X Low Byte
#define AK09916_REG_HXH         0x12 // Mag Data X High Byte
#define AK09916_REG_HYL         0x13 // Mag Data Y Low Byte
#define AK09916_REG_HYH         0x14 // Mag Data Y High Byte
#define AK09916_REG_HZL         0x15 // Mag Data Z Low Byte
#define AK09916_REG_HZH         0x16 // Mag Data Z High Byte
#define AK09916_REG_ST2         0x18 // Status 2 (HOFL, Overrun)
#define AK09916_REG_CNTL2       0x31 // Control 2 (Mode setting)
#define AK09916_REG_CNTL3       0x32 // Control 3 (Soft Reset)

// AK09916 CNTL2 Modes
#define AK09916_MODE_POWER_DOWN 0x00
#define AK09916_MODE_SINGLE     0x01
#define AK09916_MODE_CONT_10HZ  0x02
#define AK09916_MODE_CONT_20HZ  0x04
#define AK09916_MODE_CONT_50HZ  0x06
#define AK09916_MODE_CONT_100HZ 0x08 // Use this one
#define AK09916_MODE_SELF_TEST  0x10

// AK09916 ST1 Bits
#define AK09916_ST1_DRDY        (1 << 0) // Data Ready

// AK09916 ST2 Bits
#define AK09916_ST2_HOFL        (1 << 3) // Magnetic sensor overflow

// AK09916 Expected Values
#define AK09916_WIA1_EXPECTED 0x48 // <<< ADD THIS (Value for WIA1)
#define AK09916_WIA2_EXPECTED 0x09 // <<< ADD THIS (Value for WIA2)

// AK09916 Control Register Values
#define AK09916_CNTL2_MODE_POWER_DOWN    0x00
#define AK09916_CNTL2_MODE_SINGLE   0x01
#define AK09916_CNTL2_MODE_CONT_10HZ 0x02 // Continuous mode 1 (10 Hz)
#define AK09916_CNTL2_MODE_CONT_20HZ 0x04 // Continuous mode 2 (20 Hz)
#define AK09916_CNTL2_MODE_CONT_50HZ 0x06 // Continuous mode 3 (50 Hz) // <<< ADD THIS (or ensure it exists)
#define AK09916_CNTL2_MODE_CONT_100HZ 0x08 // Continuous mode 4 (100 Hz)
#define AK09916_CNTL2_SELF_TEST     0x10
#define AK09916_CNTL3_SOFT_RESET    0x01
#define I2C_READ_FLAG 0x80
#define I2C_WRITE_FLAG 0x00

// ICM20948 I2C Master Status bits (REG_I2C_MST_STATUS, Bank 0, 0x17)
#define ICM20948_I2C_MST_STATUS_SLV4_DONE (1 << 6)
#define ICM20948_I2C_MST_STATUS_SLV4_NACK (1 << 4)

// --- SPI Read/Write Flags ---
#define READ_FLAG_MASK  0x80 // Set MSB to 1 for read
#define WRITE_FLAG_MASK 0x7F // Set MSB to 0 for write (by ANDing with this mask)

// --- AK09916 (Magnetometer) Registers & Constants ---
#define AK09916_I2C_ADDR    0x0C // Default I2C Address

#define AK09916_REG_WIA1    0x00 // Company ID (Should be 0x48)
#define AK09916_REG_WIA2    0x01 // Device ID (Should be 0x09)
#define AK09916_REG_RSV1    0x02 // Reserved
#define AK09916_REG_ST1     0x10 // Status 1
#define AK09916_REG_HXL     0x11 // Mag Data X LSB
#define AK09916_REG_HXH     0x12 // Mag Data X MSB
#define AK09916_REG_HYL     0x13 // Mag Data Y LSB
#define AK09916_REG_HYH     0x14 // Mag Data Y MSB
#define AK09916_REG_HZL     0x15 // Mag Data Z LSB
#define AK09916_REG_HZH     0x16 // Mag Data Z MSB
#define AK09916_REG_TMPS    0x17 // Dummy register? Not specified in datasheet
#define AK09916_REG_ST2     0x18 // Status 2
#define AK09916_REG_CNTL1   0x30 // Control 1 (Reserved, do not write)
#define AK09916_REG_CNTL2   0x31 // Control 2 (Mode setting)
#define AK09916_REG_CNTL3   0x32 // Control 3 (Soft Reset)
#define AK09916_REG_TS1     0x33 // Test Reg 1 (Reserved)
#define AK09916_REG_TS2     0x34 // Test Reg 2 (Reserved)
#define AK09916_REG_I2CDIS  0x36 // I2C Disable (Not typically used)

// AK09916 CNTL2 Modes
#define AK09916_MODE_POWER_DOWN     0x00
#define AK09916_MODE_SINGLE         0x01
#define AK09916_MODE_CONT_10HZ      0x02
#define AK09916_MODE_CONT_20HZ      0x04
#define AK09916_MODE_CONT_50HZ      0x06
#define AK09916_MODE_CONT_100HZ     0x08 // Used in init
#define AK09916_MODE_SELF_TEST      0x10

// AK09916 CNTL3 Commands
#define AK09916_SOFT_RESET          0x01

// AK09916 ST1 Status Bits
#define AK09916_ST1_DRDY_MASK       0x01 // Data Ready
#define AK09916_ST1_DOR_MASK        0x02 // Data Overrun

// AK09916 ST2 Status Bits
#define AK09916_ST2_DRDY_MASK       0x01 // Data Ready (Mirrored from ST1?) - Check datasheet
#define AK09916_ST2_HOFL_MASK       0x08 // Magnetic Sensor Overflow (Bit 3)

// --- I2C Master Related (Bank 3) ---
// ... I2C Master registers ...
#define I2C_SLV_EN          0x80 // Enable I2C slave
#define I2C_SLV_BYTE_SW     0x40 // Enable byte swapping
#define I2C_SLV_REG_DIS     0x20 // Disable writing transaction register address
#define I2C_SLV_GRP         0x10 // Group slaves for odd/even sample rate
#define I2C_SLV_DLY_EN      0x01 // Enable delay for this slave

// --- General ---
#define I2C_READ_FLAG       0x80 // Flag to OR with I2C slave address for reads

// I2C_MST_CTRL Bits (Bank 3, Reg 0x01)
#define I2C_MST_CTRL_MULT_MST_EN 0x80
#define I2C_MST_CTRL_WAIT_FOR_ES 0x40
#define I2C_MST_CTRL_SLV_3_FIFO_EN 0x20
#define I2C_MST_CTRL_P_NSR       0x10 // Controls Stop condition generation
#define I2C_MST_CTRL_CLK_MASK    0x0F // Bits 3:0 for clock speed selection

// I2C_MST_STATUS Bits (Bank 3, Reg 0x17)
#define I2C_MST_STAT_PASS_THROUGH 0x80
#define I2C_MST_STAT_SLV4_DONE    0x40
#define I2C_MST_STAT_LOST_ARB     0x20
#define I2C_MST_STAT_SLV4_NACK    0x10
#define I2C_MST_STAT_SLV3_NACK    0x08
#define I2C_MST_STAT_SLV2_NACK    0x04
#define I2C_MST_STAT_SLV1_NACK    0x02
#define I2C_MST_STAT_SLV0_NACK    0x01 // NACK received from SLV0

// I2C_MST_DELAY_CTRL Bits (Bank 3, Reg 0x02)
#define I2C_MST_DELAY_SLV4_EN   0x10
#define I2C_MST_DELAY_SLV3_EN   0x08
#define I2C_MST_DELAY_SLV2_EN   0x04
#define I2C_MST_DELAY_SLV1_EN   0x02
#define I2C_MST_DELAY_SLV0_EN   0x01 // Enable delay for SLV0 sampling

// I2C_SLV*_CTRL Bits (Bank 3, Reg 0x05, 0x08, 0x0B, 0x0E)
#define I2C_SLV_EN          0x80 // Enable I2C slave
#define I2C_SLV_BYTE_SW     0x40 // Enable byte swapping
#define I2C_SLV_REG_DIS     0x20 // Disable writing transaction register address
#define I2C_SLV_GRP         0x10 // Group slaves for odd/even sample rate
#define I2C_SLV_DLY_EN      0x01 // Enable delay for this slave (Needs I2C_MST_DELAY_CTRL bit set too)

#define ICM20948_ADDR_EXT_SLV_SENS_DATA_00 0x00

#endif // ICM20948_REGS_H 