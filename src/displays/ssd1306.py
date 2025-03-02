from smbus2 import SMBus
from PIL import Image, ImageDraw, ImageFont
import os

class SSD1306:
    """Driver for SSD1306 OLED display."""
    
    def __init__(self, bus):
        """Initialize the SSD1306 OLED display.
        
        Args:
            bus (int): I2C bus number (0 or 1)
        """
        self.bus = SMBus(bus)
        self.address = 0x3C  # Standard I2C address for SSD1306
        self.width = 128  # OLED display width
        self.height = 64  # OLED display height
        self.pages = self.height // 8
        
        # Create image buffer
        self.image = Image.new('1', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)
        # Load a font with precise sizing for 15px height
        font_size = 14  # Starting with 14pt to achieve ~15px height
        try:
            self.font = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", font_size)
        except:
            try:
                self.font = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf", font_size)
            except:
                try:
                    self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", font_size)
                except:
                    self.font = ImageFont.load_default()
        
        # Calculate font dimensions using a test string
        test_text = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
        bbox = self.font.getbbox(test_text)
        self.font_height = bbox[3] - bbox[1]  # bottom - top
        self.font_width = (bbox[2] - bbox[0]) / len(test_text)  # average character width
        
        # Verify the height and adjust if needed
        while self.font_height != 15 and isinstance(self.font, ImageFont.FreeTypeFont):
            if self.font_height < 15:
                font_size += 1
            else:
                font_size -= 1
            self.font = ImageFont.truetype(self.font.path, font_size)
            bbox = self.font.getbbox(test_text)
            self.font_height = bbox[3] - bbox[1]
        
        # Calculate line spacing
        self.line_spacing = self.font_height + 1  # Add 1 pixel vertical interval
        
        self.initialize_display()
    
    def initialize_display(self):
        """Initialize the display with required settings."""
        commands = [
            0xAE,        # Display off
            0xD5, 0x80,  # Set display clock div
            0xA8, 0x1F,  # Set multiplex to 32 (0x1F + 1)
            0xD3, 0x00,  # Set display offset
            0x40,        # Set start line to 0
            0x8D, 0x14,  # Charge pump setting
            0x20, 0x00,  # Memory mode
            0xA1,        # Segment remap
            0xC8,        # COM scan direction
            0xDA, 0x02,  # Set COM pins hardware configuration
            0x81, 0xCF,  # Set contrast
            0xD9, 0xF1,  # Set precharge
            0xDB, 0x40,  # Set vcom detect
            0xA4,        # Display all on resume
            0xA6,        # Normal display
            0x2E,        # Deactivate scroll
            0xAF         # Display on
        ]
        
        # Send commands properly
        for cmd in commands:
            self._write_command(cmd)
            
    def _write_command(self, command):
        """Write a single command to the display."""
        try:
            self.bus.write_byte_data(self.address, 0x00, command)
        except Exception as e:
            print(f"Error writing command {hex(command)}: {e}")
            
    def _write_data(self, data):
        """Write data to the display."""
        try:
            self.bus.write_byte_data(self.address, 0x40, data)
        except Exception as e:
            print(f"Error writing data: {e}")

    def clear_display(self):
        """Clear the display."""
        self.image.paste(0, (0, 0, self.width, self.height))
        self._display_image()
    
    def display_measurements(self, pressure, altitude, temperature, acceleration):
        """Display the four measurements on the OLED screen in two rows.
        Each row is 15 pixels high with 1 pixel interval between rows.
        First row: 0-14 pixels
        Interval: 16th pixel
        Second row: 17-31 pixels
        
        Args:
            pressure (float): Pressure value in hPa
            altitude (float): Altitude in meters
            temperature (float): Temperature in Celsius
            acceleration (float): Acceleration in m/s²
        """
        # Clear the image
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        
        # Format the values with proper units (shortened for space)
        p_text = f"P:{pressure:.2f}"
        ac_text = f"AC:{acceleration:.1f}"
        t_text = f"T:{temperature:.1f}"
        a_text = f"A:{altitude:.0f}m"
        
        # Position text with 15px height rows and 1px interval
        y1 = 0    # First row starts at 0
        y2 = 17   # Second row starts at 17 (after 15px height + 1px interval)
        
        self.draw.text((5, y1), p_text, font=self.font, fill=255)
        self.draw.text((self.width//2 + 10, y1), ac_text, font=self.font, fill=255)
        self.draw.text((5, y2), t_text, font=self.font, fill=255)
        self.draw.text((self.width//2 - 5, y2), a_text, font=self.font, fill=255)
        
        self._display_image()
    
    def _display_image(self):
        """Internal method to send the image buffer to the display."""
        # Set column address
        self._write_command(0x21)  # Column address command
        self._write_command(0)     # Start column
        self._write_command(127)   # End column
        
        # Set page address
        self._write_command(0x22)  # Page address command
        self._write_command(0)     # Start page
        self._write_command(7)     # End page
        
        # Convert image to display format
        pixels = self.image.convert("1")
        buffer = bytearray(self.width * self.pages)
        
        # Arrange pixels into pages
        for y in range(self.height):
            for x in range(self.width):
                if pixels.getpixel((x, y)):
                    buffer[x + (y // 8) * self.width] |= (1 << (y % 8))
        
        # Write buffer to display in chunks
        chunk_size = 32  # Write 32 bytes at a time
        for i in range(0, len(buffer), chunk_size):
            chunk = buffer[i:i + chunk_size]
            self.bus.write_i2c_block_data(self.address, 0x40, list(chunk)) 