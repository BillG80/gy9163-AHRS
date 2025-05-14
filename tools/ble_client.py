import asyncio
from bleak import BleakClient, BleakScanner
import struct
import time
import logging
import sys
from queue import Queue
from threading import Event

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class BLEClient:
    # UUIDs matching the server
    SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
    SENSOR_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
    CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
    
    def __init__(self):
        self.client = None
        self.connected = False
        self.running = False
        self.device_address = None
        self.data_queue = Queue()
        self.reconnect_event = Event()
        self.latest_data = None
    
    def notification_handler(self, sender, data):
        """Handle incoming sensor data."""
        try:
            unpacked = struct.unpack('9f', data)
            sensor_data = {
                'timestamp': unpacked[0],
                'accelerometer': {'x': unpacked[1], 'y': unpacked[2], 'z': unpacked[3]},
                'gyroscope': {'x': unpacked[4], 'y': unpacked[5], 'z': unpacked[6]},
                'temperature': unpacked[7],
                'pressure': unpacked[8]
            }
            self.data_queue.put(sensor_data)
            self.latest_data = sensor_data
            
        except Exception as e:
            logger.error(f"Error handling notification: {e}")
    
    async def scan_for_device(self):
        """Scan for the GY91 device."""
        try:
            logger.info("Scanning for GY91 device...")
            devices = await BleakScanner.discover()
            
            for device in devices:
                if device.name and "GY91" in device.name:
                    logger.info(f"Found GY91 device: {device.address}")
                    return device.address
            
            return None
            
        except Exception as e:
            logger.error(f"Scan error: {e}")
            return None
    
    async def connect(self):
        """Connect to the device with auto-reconnect."""
        while self.running:
            try:
                if not self.device_address:
                    self.device_address = await self.scan_for_device()
                    if not self.device_address:
                        logger.error("Device not found. Retrying in 5 seconds...")
                        await asyncio.sleep(5)
                        continue
                
                logger.info(f"Connecting to {self.device_address}...")
                self.client = BleakClient(
                    self.device_address,
                    disconnected_callback=self._handle_disconnect
                )
                
                await self.client.connect()
                self.connected = True
                logger.info("Connected successfully")
                
                # Set up notifications
                await self.client.start_notify(
                    self.SENSOR_CHAR_UUID,
                    self.notification_handler
                )
                
                # Wait for disconnect or stop
                self.reconnect_event.clear()
                await self._wait_for_disconnect()
                
            except Exception as e:
                logger.error(f"Connection error: {e}")
                self.connected = False
                await asyncio.sleep(5)
    
    def _handle_disconnect(self, client):
        """Handle disconnection and trigger reconnect."""
        logger.info("Disconnected from device")
        self.connected = False
        self.reconnect_event.set()
    
    async def _wait_for_disconnect(self):
        """Wait for disconnection."""
        while self.connected and self.running:
            await asyncio.sleep(1)
    
    async def start(self):
        """Start the client."""
        self.running = True
        await self.connect()
    
    async def stop(self):
        """Stop the client."""
        self.running = False
        if self.client:
            await self.client.disconnect()
    
    def get_latest_data(self):
        """Get the latest sensor data."""
        return self.latest_data

async def main():
    client = BLEClient()
    
    try:
        # Start client in background
        client_task = asyncio.create_task(client.start())
        
        # Main display loop
        while True:
            data = client.get_latest_data()
            if data:
                # Clear screen
                print("\033[2J\033[H")
                
                # Print data
                print("=== Sensor Data ===")
                print(f"Connection Status: {'Connected' if client.connected else 'Disconnected'}")
                print(f"\nAccelerometer (g):")
                print(f"  X: {data['accelerometer']['x']:7.3f}")
                print(f"  Y: {data['accelerometer']['y']:7.3f}")
                print(f"  Z: {data['accelerometer']['z']:7.3f}")
                print(f"\nGyroscope (deg/s):")
                print(f"  X: {data['gyroscope']['x']:7.3f}")
                print(f"  Y: {data['gyroscope']['y']:7.3f}")
                print(f"  Z: {data['gyroscope']['z']:7.3f}")
                print(f"\nTemperature: {data['temperature']:.1f}°C")
                print(f"Pressure: {data['pressure']:.1f} hPa")
            
            await asyncio.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nStopping client...")
        await client.stop()
        await client_task

if __name__ == "__main__":
    asyncio.run(main()) 