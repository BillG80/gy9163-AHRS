import spidev
import RPi.GPIO as GPIO
import time

# --- Pin Configuration (BCM numbering) ---
PS_PIN = 25  # PS pin connected to GPIO 25 (Set LOW for SPI)
CSB_PIN = 7  # CSB pin connected to GPIO 7 (Manual Chip Select)

# --- SPI Configuration ---
SPI_BUS = 0
SPI_DEVICE = 1 # Corresponds to CE1 (GPIO 7), but we control manually
SPI_SPEED_HZ = 1000000 # 1 MHz clock speed
SPI_MODE = 0b00 # CPOL=0, CPHA=0 (Mode 0)

# --- MS5611 Commands ---
CMD_RESET = 0x1E
CMD_PROM_RD_BASE = 0xA0 # Base for PROM read commands (0xA0, 0xA2, ..., 0xAE)
CMD_ADC_READ = 0x00
CMD_CONV_D1_OSR4096 = 0x48 # Convert D1 (Pressure) OSR=4096
CMD_CONV_D2_OSR4096 = 0x58 # Convert D2 (Temperature) OSR=4096

# --- Globals ---
spi = None

# --- Helper Functions ---
def setup_gpio_spi():
    """Initializes GPIO and SPI"""
    global spi
    print("Setting up GPIO...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup PS pin as Output LOW
    GPIO.setup(PS_PIN, GPIO.OUT)
    GPIO.output(PS_PIN, GPIO.LOW)
    print(f"✓ Set PS Pin (GPIO {PS_PIN}) to OUTPUT LOW (SPI Mode)")

    # Setup CSB pin as Output HIGH (inactive)
    GPIO.setup(CSB_PIN, GPIO.OUT)
    GPIO.output(CSB_PIN, GPIO.HIGH)
    print(f"✓ Set CSB Pin (GPIO {CSB_PIN}) to OUTPUT HIGH (Inactive)")

    print("\nSetting up SPI...")
    spi = spidev.SpiDev()
    try:
        spi.open(SPI_BUS, SPI_DEVICE) # Use CE1 device as placeholder
        spi.max_speed_hz = SPI_SPEED_HZ
        spi.mode = SPI_MODE
        # Explicitly set no_cs=True if available (might depend on spidev version)
        # This ensures the kernel doesn't toggle CE1 automatically
        try:
            spi.no_cs = True
        except AttributeError:
            print("  (Note: spidev version might not support no_cs attribute)")

        print(f"✓ SPI Bus {SPI_BUS}, Device {SPI_DEVICE} opened.")
        print(f"✓ Mode: {spi.mode}, Speed: {spi.max_speed_hz} Hz")
    except FileNotFoundError:
        print(f"❌ Error: SPI device /dev/spidev{SPI_BUS}.{SPI_DEVICE} not found.")
        print("   Ensure SPI is enabled in raspi-config.")
        cleanup()
        exit()
    except Exception as e:
        print(f"❌ Error opening/configuring SPI: {e}")
        cleanup()
        exit()

def cs_activate():
    """Activates Chip Select (Pulls LOW)"""
    GPIO.output(CSB_PIN, GPIO.LOW)
    # time.sleep(0.000001) # Optional small delay

def cs_deactivate():
    """Deactivates Chip Select (Pulls HIGH)"""
    # time.sleep(0.000001) # Optional small delay
    GPIO.output(CSB_PIN, GPIO.HIGH)

def spi_write_command(command):
    """Sends a single command byte via SPI"""
    try:
        cs_activate()
        spi.writebytes([command])
        cs_deactivate()
        print(f"  ✓ Sent command: 0x{command:02X}")
        return True
    except Exception as e:
        cs_deactivate() # Ensure CS goes high on error
        print(f"  ❌ Error sending command 0x{command:02X}: {e}")
        return False

def spi_read_prom(prom_cmd):
    """Reads a 16-bit PROM value for a given command"""
    try:
        cs_activate()
        # Send command, then read 2 bytes. Need 3 bytes total in transfer.
        # Send [prom_cmd, 0x00, 0x00], receive [dummy, byte1, byte2]
        read_bytes = spi.xfer2([prom_cmd, 0x00, 0x00])
        cs_deactivate()
        if len(read_bytes) == 3:
            value = (read_bytes[1] << 8) | read_bytes[2]
            print(f"  ✓ Read PROM (Cmd 0x{prom_cmd:02X}): Raw Bytes={read_bytes}, Value={value} (0x{value:04X})")
            return value
        else:
             print(f"  ❌ Error reading PROM (Cmd 0x{prom_cmd:02X}): Received {len(read_bytes)} bytes, expected 3.")
             return None
    except Exception as e:
        cs_deactivate() # Ensure CS goes high on error
        print(f"  ❌ Error during PROM read (Cmd 0x{prom_cmd:02X}): {e}")
        return None

def spi_read_adc():
    """Reads the 24-bit ADC result"""
    try:
        cs_activate()
        # Send ADC Read command (0x00), then read 3 bytes. Need 4 bytes total.
        # Send [0x00, 0x00, 0x00, 0x00], receive [dummy, byte1, byte2, byte3]
        read_bytes = spi.xfer2([CMD_ADC_READ, 0x00, 0x00, 0x00])
        cs_deactivate()
        if len(read_bytes) == 4:
            value = (read_bytes[1] << 16) | (read_bytes[2] << 8) | read_bytes[3]
            print(f"  ✓ Read ADC: Raw Bytes={read_bytes}, Value={value} (0x{value:06X})")
            return value
        else:
            print(f"  ❌ Error reading ADC: Received {len(read_bytes)} bytes, expected 4.")
            return None
    except Exception as e:
        cs_deactivate() # Ensure CS goes high on error
        print(f"  ❌ Error during ADC read: {e}")
        return None

def cleanup():
    """Closes SPI and cleans up GPIO"""
    global spi
    print("\nCleaning up...")
    if spi:
        spi.close()
        print("✓ SPI closed.")
    GPIO.cleanup([PS_PIN, CSB_PIN]) # Cleanup only pins we used
    print("✓ GPIO cleaned up.")
    print("✓ Cleanup complete.")

# --- Main Test ---
if __name__ == "__main__":
    try:
        setup_gpio_spi()

        # 1. Reset Sensor
        print(f"\nSending RESET command (0x{CMD_RESET:02X})...")
        if not spi_write_command(CMD_RESET):
            raise RuntimeError("Failed to send RESET command.")
        time.sleep(0.01) # Wait for reset to complete

        # 2. Read PROM Coefficients
        print("\nReading PROM coefficients (Commands 0xA0 to 0xAE):")
        coeffs = {}
        read_ok = True
        for i in range(8):
            prom_cmd = CMD_PROM_RD_BASE + (i * 2)
            value = spi_read_prom(prom_cmd)
            if value is not None:
                coeffs[i] = value
            else:
                read_ok = False
        if not read_ok:
            print("   One or more coefficients failed to read. Check connections/wiring.")

        # 3. Read Temperature (Optional basic check)
        if read_ok:
            print("\nAttempting Temperature Read:")
            print(f"  Sending Temp Conversion command (0x{CMD_CONV_D2_OSR4096:02X})...")
            if spi_write_command(CMD_CONV_D2_OSR4096):
                time.sleep(0.01) # Wait for conversion (~8.22ms for OSR 4096)
                print("  Reading ADC result...")
                d2 = spi_read_adc()
                if d2 is not None:
                     print(f"  -> Raw Temperature (D2): {d2}")
                else:
                    print("  ❌ Failed to read temperature ADC.")
            else:
                print("  ❌ Failed to send temperature conversion command.")

        # 4. Read Pressure (Optional basic check)
        if read_ok:
            print("\nAttempting Pressure Read:")
            print(f"  Sending Pressure Conversion command (0x{CMD_CONV_D1_OSR4096:02X})...")
            if spi_write_command(CMD_CONV_D1_OSR4096):
                time.sleep(0.01) # Wait for conversion
                print("  Reading ADC result...")
                d1 = spi_read_adc()
                if d1 is not None:
                    print(f"  -> Raw Pressure (D1): {d1}")
                else:
                    print("  ❌ Failed to read pressure ADC.")
            else:
                print("  ❌ Failed to send pressure conversion command.")

    except KeyboardInterrupt:
        print("\nUser interrupted.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        cleanup()
