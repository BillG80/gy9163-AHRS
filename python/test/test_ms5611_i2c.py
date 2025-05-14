import smbus2
import time

# --- Configuration ---
I2C_BUS = 0  # <<<--- UPDATED FOR I2C BUS 0
I2C_ADDR = 0x77  # <<<--- CHANGE THIS to 0x76 or 0x77 based on i2cdetect -y 0

# --- MS5611 Commands ---
CMD_RESET = 0x1E
CMD_PROM_RD_BASE = 0xA0  # Base for PROM read commands (0xA0, 0xA2, ..., 0xAE)
CMD_ADC_READ = 0x00
CMD_CONV_D1_OSR4096 = 0x48 # Convert D1 (Pressure) OSR=4096
CMD_CONV_D2_OSR4096 = 0x58 # Convert D2 (Temperature) OSR=4096
CMD_CONV_D1_OSR2048 = 0x44  # Pressure OSR=2048
CMD_CONV_D2_OSR2048 = 0x54  # Temperature OSR=2048

# --- Conversion Timing ---
CONV_TIME_OSR4096 = 0.00822  # ~8.22 ms conversion time for OSR=4096
CONV_TIME_OSR2048 = 0.00562  # ~5.62 ms conversion time for OSR=2048

# --- Calibration & Compensation ---
def calculate_pressure_temperature(d1, d2, prom):
    # Unpack calibration coefficients from PROM list (skip index 0)
    C1 = prom[1]
    C2 = prom[2]
    C3 = prom[3]
    C4 = prom[4]
    C5 = prom[5]
    C6 = prom[6]
    # Calculate temperature difference
    dT = d2 - C5 * 256
    # First order temperature (in hundredths of °C)
    TEMP = 2000 + dT * C6 / 8388608.0
    # Offset and Sensitivity
    OFF = C2 * 65536 + (C4 * dT) / 128.0
    SENS = C1 * 32768 + (C3 * dT) / 256.0
    # Second order compensation
    T2 = 0.0
    OFF2 = 0.0
    SENS2 = 0.0
    if TEMP < 2000:
        T2 = (dT * dT) / 2147483648.0
        OFF2 = 5 * ((TEMP - 2000)**2) / 2.0
        SENS2 = 5 * ((TEMP - 2000)**2) / 4.0
        if TEMP < -1500:
            OFF2 += 7 * ((TEMP + 1500)**2)
            SENS2 += 11 * ((TEMP + 1500)**2) / 2.0
    # Apply second order
    TEMP = TEMP - T2
    OFF = OFF - OFF2
    SENS = SENS - SENS2
    # Calculate pressure (in Pa)
    P = (((d1 * SENS) / 2097152.0) - OFF) / 32768.0
    # Convert to human units
    temp_c = TEMP / 100.0
    press_mbar = P / 100.0
    return temp_c, press_mbar

# --- Helper Functions ---
def read_word_data_swapped(bus, address, command):
    """Reads a 16-bit word, swapping bytes for MS5611."""
    try:
        block = bus.read_i2c_block_data(address, command, 2)
        print(f"DEBUG: PROM raw block for cmd 0x{command:02X}: {block}")
        # Print raw bytes for comparison with C code
        print(f"    Raw bytes read for cmd 0x{command:02X}: MSB=0x{block[0]:02X}, LSB=0x{block[1]:02X}")
        # Swap bytes (MSB comes first from sensor)
        return (block[0] << 8) | block[1]
    except OSError as e:
        print(f"Error reading word data for command 0x{command:02X}: {e}")
        return None
    except IndexError:
         print(f"Error reading word data for command 0x{command:02X}: Received less than 2 bytes.")
         return None


def read_adc(bus, address):
    """Reads the 24-bit ADC result after conversion."""
    try:
        block = bus.read_i2c_block_data(address, CMD_ADC_READ, 3)
        print(f"DEBUG: ADC raw block: {block}")
        # Combine bytes into 24-bit value
        value = (block[0] << 16) | (block[1] << 8) | block[2]
        return value
    except OSError as e:
        print(f"Error reading ADC: {e}")
        return None
    except IndexError:
        print(f"Error reading ADC: Received less than 3 bytes.")
        return None

# CRC-4 validation function (per datasheet)
def crc4(prom):
    n_prom = prom.copy()
    n_rem = 0
    crc_read = n_prom[7] & 0xF
    n_prom[7] &= 0xFF00
    for cnt in range(16):
        if cnt & 1:
            n_rem ^= n_prom[cnt >> 1] & 0x00FF
        else:
            n_rem ^= (n_prom[cnt >> 1] >> 8)
        for _ in range(8):
            if n_rem & 0x8000:
                n_rem = (n_rem << 1) ^ 0x3000
            else:
                n_rem <<= 1
            n_rem &= 0xFFFF
    return (n_rem >> 12) & 0xF

# --- Main Test ---
try:
    print(f"Attempting to open I2C bus {I2C_BUS}...")
    bus = smbus2.SMBus(I2C_BUS)
    print(f"Successfully opened I2C bus {I2C_BUS}.")
    print(f"Using device address 0x{I2C_ADDR:02X}")

    # 1. Reset Sensor
    print(f"\nSending RESET command (0x{CMD_RESET:02X})...")
    try:
        bus.write_byte(I2C_ADDR, CMD_RESET)
        print("✓ Reset command sent.")
        time.sleep(0.01) # Wait a bit after reset
    except OSError as e:
        print(f"❌ Error sending RESET command: {e}")
        print("   Check wiring (SDA=GPIO2, SCL=GPIO3 for I2C0), I2C address, and ensure PS is HIGH (3.3V).")
        bus.close()
        exit()

    # 2. Read PROM Coefficients
    print("\nReading PROM coefficients (Commands 0xA0 to 0xAE):")
    # Read all 8 PROM entries into a list (index 0 is reserved, 1..6 = C1..C6, 7 = CRC)
    prom = [0]*8
    read_ok = True
    for i in range(8):
        cmd = CMD_PROM_RD_BASE + (i * 2)
        print(f"  Reading PROM word {i} (Cmd 0x{cmd:02X})...")
        time.sleep(0.005)
        val = read_word_data_swapped(bus, I2C_ADDR, cmd)
        if val is None:
            print(f"  ❌ Failed to read PROM word {i}.")
            read_ok = False
        else:
            prom[i] = val
            if i == 0:
                print(f"  ✓ Factory data: {val} (0x{val:04X})")
            elif 1 <= i <= 6:
                print(f"  ✓ C{i}: {val} (0x{val:04X})")
            else:
                print(f"  ✓ CRC: {val} (0x{val:04X})")
    if not read_ok:
         print("   One or more PROM words failed to read. Check connections/address.")

    # 3. Read Temperature (Optional basic check)
    if read_ok: # Only proceed if PROM read seemed okay
        print("\nAttempting Temperature Read:")
        try:
            # Trigger D2 conversion
            print(f"  Sending Temp Conversion command (0x{CMD_CONV_D2_OSR4096:02X})...")
            bus.write_byte(I2C_ADDR, CMD_CONV_D2_OSR4096)
            print("  ✓ Temp Conversion command sent.")
            time.sleep(CONV_TIME_OSR4096) # Wait for conversion (datasheet: ~8.22ms for OSR 4096)

            # Read ADC result
            print("  Reading ADC result...")
            d2 = read_adc(bus, I2C_ADDR)
            if d2 is not None:
                print(f"  ✓ Raw Temperature (D2): {d2} (0x{d2:06X})")
            else:
                print("  ❌ Failed to read temperature ADC.")
        except OSError as e:
            print(f"  ❌ Error during temperature read sequence: {e}")

    # 4. Read Pressure (Optional basic check)
    if read_ok: # Only proceed if PROM read seemed okay
        print("\nAttempting Pressure Read:")
        try:
            # Trigger D1 conversion
            print(f"  Sending Pressure Conversion command (0x{CMD_CONV_D1_OSR4096:02X})...")
            bus.write_byte(I2C_ADDR, CMD_CONV_D1_OSR4096)
            print("  ✓ Pressure Conversion command sent.")
            time.sleep(CONV_TIME_OSR4096) # Wait for conversion

            # Read ADC result
            print("  Reading ADC result...")
            d1 = read_adc(bus, I2C_ADDR)
            if d1 is not None:
                print(f"  ✓ Raw Pressure (D1): {d1} (0x{d1:06X})")
                # Compute compensated values
                temp_c, press_mbar = calculate_pressure_temperature(d1, d2, prom)
                print(f"  ✓ Compensated: Temp={temp_c:.2f} C, Press={press_mbar:.2f} mbar")

                # Validate PROM CRC before continuous mode
                stored_crc = prom[7] & 0xF
                calc_crc = crc4(prom)
                print(f"\nCRC Read={stored_crc}, CRC Calc={calc_crc}")
                if calc_crc != stored_crc:
                    print("❌ CRC mismatch! Aborting.")
                    bus.close()
                    exit()
                print("✓ CRC OK\n")

                # Continuous measurement at 50 Hz using OSR=2048
                print("Starting continuous measurement at 50Hz (OSR=2048)...")
                period = 0.02  # 50 Hz sample period
                while True:
                    t0 = time.time()
                    # Temp
                    bus.write_byte(I2C_ADDR, CMD_CONV_D2_OSR2048)
                    time.sleep(CONV_TIME_OSR2048)
                    d2 = read_adc(bus, I2C_ADDR)
                    # Press
                    bus.write_byte(I2C_ADDR, CMD_CONV_D1_OSR2048)
                    time.sleep(CONV_TIME_OSR2048)
                    d1 = read_adc(bus, I2C_ADDR)
                    if d1 is not None and d2 is not None:
                        temp_c, press_mbar = calculate_pressure_temperature(d1, d2, prom)
                        # Compute elapsed time and update rate
                        dt = time.time() - t0
                        rate_hz = 1.0 / dt if dt > 0 else float('inf')
                        print(f"Temp={temp_c:.2f}°C, Press={press_mbar:.2f} mbar, Rate={rate_hz:.1f} Hz")
                    else:
                        print("❌ ADC read error in loop")
                        dt = time.time() - t0
                    if dt < period:
                        time.sleep(period - dt)
        except OSError as e:
            print(f"  ❌ Error during pressure read sequence: {e}")


except FileNotFoundError:
    print(f"❌ Error: I2C bus {I2C_BUS} not found. Ensure I2C is enabled on your Pi.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    if 'bus' in locals() and bus is not None:
        bus.close()
        print("\nI2C bus closed.")
