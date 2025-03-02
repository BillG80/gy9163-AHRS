import time
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1305, sh1106  # Import multiple types

# --- Try different devices ---
devices = [ssd1306, ssd1305, sh1106]
bus = 0  # You said bus 0
address = 0x3C  # You said address 0x3C

for device_class in devices:
    print(f"Trying device: {device_class.__name__}")
    try:
        serial = i2c(port=bus, address=address)
        device = device_class(serial)

        for _ in range(5):  # Display multiple times
            with canvas(device) as draw:
                draw.text((0, 0), "Hello!", fill="white")
            time.sleep(1)  # Wait a second

        device.clear()
        print(f"  Device {device_class.__name__} initialized and tested successfully (but check display!).")
        break  # If successful, exit the loop

    except Exception as e:
        print(f"  Error with {device_class.__name__}: {e}") 