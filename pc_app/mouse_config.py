#!/usr/bin/env python3
"""
PC Application for configuring PAW3395 BLE Mouse

This application connects to the mouse via BLE and allows configuration of settings.
It uses bleak library for BLE communication.

Install dependencies:
    pip install bleak

Usage:
    python mouse_config.py
"""

import asyncio
import struct
from typing import Dict, List, Optional, Tuple

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Please install bleak: pip install bleak")
    exit(1)

# Settings service UUID (must match firmware)
SETTINGS_SERVICE_UUID = "5f4e3d2c-1b8a-3f9e-9d4c-a6b5-0000-712d"
SETTINGS_CHAR_UUID = "5f4e3d2c-1b8a-3f9e-9d4c-a6b5-0100-712d"

# Protocol commands
CMD_GET_SETTINGS_LIST = 0x01
CMD_GET_SETTING = 0x02
CMD_SET_SETTING = 0x03
CMD_APPLY_SETTINGS = 0x04


class SettingDefinition:
    """Represents a setting definition from the device."""
    
    def __init__(self, name: str, value: int, min_val: int, max_val: int):
        self.name = name
        self.value = value
        self.min = min_val
        self.max = max_val
    
    @property
    def is_boolean(self) -> bool:
        """Check if this setting is a boolean (switch)."""
        return self.min == 0 and self.max == 1
    
    def __repr__(self):
        setting_type = "switch" if self.is_boolean else "range"
        return f"Setting({self.name}, value={self.value}, min={self.min}, max={self.max}, type={setting_type})"


class MouseConfigApp:
    """Main application class for mouse configuration."""
    
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.settings: Dict[str, SettingDefinition] = {}
        self.device_address: Optional[str] = None
        
    async def find_device(self) -> Optional[str]:
        """Find and return the address of the PAW3395 mouse."""
        print("Scanning for devices...")
        
        devices = await BleakScanner.discover(timeout=5.0)
        
        for device in devices:
            # Look for our device by name or service UUID
            if device.name and "PAW3395" in device.name:
                print(f"Found device: {device.name} ({device.address})")
                return device.address
            
            # Also check for our service UUID
            if hasattr(device, 'metadata') and device.metadata:
                pass  # Could check services here
        
        # If not found by name, show available devices
        print("\nAvailable BLE devices:")
        for i, device in enumerate(devices[:20]):  # Show first 20
            print(f"  {i+1}. {device.name or 'Unknown'} ({device.address})")
        
        return None
    
    async def connect(self, address: str):
        """Connect to the mouse device."""
        print(f"Connecting to {address}...")
        
        self.client = BleakClient(address)
        await self.client.connect()
        
        if self.client.is_connected:
            print("Connected successfully!")
            self.device_address = address
        else:
            raise Exception("Failed to connect")
    
    async def disconnect(self):
        """Disconnect from the mouse."""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("Disconnected")
    
    async def send_command(self, cmd: int, data: bytes = b'') -> bytes:
        """Send a command to the device and wait for response."""
        if not self.client or not self.client.is_connected:
            raise Exception("Not connected")
        
        # Prepare command packet
        packet = bytes([cmd]) + data
        
        # Send command
        await self.client.write_gatt_char(SETTINGS_CHAR_UUID, packet, response=True)
        
        # Wait for notification with response
        # In a real implementation, you'd set up a notification handler
        # For now, we'll just return empty and handle responses asynchronously
        return b''
    
    async def request_settings_list(self) -> List[SettingDefinition]:
        """Request the list of all settings from the device."""
        if not self.client or not self.client.is_connected:
            return []
        
        # Set up notification handler
        responses = []
        
        def notification_handler(sender, data):
            responses.append(data)
            print(f"Received notification: {data.hex()}")
        
        self.client.start_notify(SETTINGS_CHAR_UUID, notification_handler)
        
        try:
            # Send request
            await self.client.write_gatt_char(SETTINGS_CHAR_UUID, bytes([CMD_GET_SETTINGS_LIST]), response=True)
            
            # Wait for response
            await asyncio.sleep(0.5)
            
            if responses:
                return self._parse_settings_list(responses[0])
            
        finally:
            self.client.stop_notify(SETTINGS_CHAR_UUID)
        
        return []
    
    def _parse_settings_list(self, data: bytes) -> List[SettingDefinition]:
        """Parse the settings list response."""
        settings = []
        
        if len(data) < 1:
            return settings
        
        idx = 0
        count = data[idx]
        idx += 1
        
        print(f"Received {count} settings")
        
        for _ in range(count):
            if idx >= len(data):
                break
            
            # Read name length
            name_len = data[idx]
            idx += 1
            
            if idx + name_len > len(data):
                break
            
            # Read name
            name = data[idx:idx + name_len].decode('utf-8')
            idx += name_len
            
            # Read value (4 bytes, little endian)
            if idx + 4 > len(data):
                break
            value = struct.unpack('<i', data[idx:idx + 4])[0]
            idx += 4
            
            # Read min (4 bytes, little endian)
            if idx + 4 > len(data):
                break
            min_val = struct.unpack('<i', data[idx:idx + 4])[0]
            idx += 4
            
            # Read max (4 bytes, little endian)
            if idx + 4 > len(data):
                break
            max_val = struct.unpack('<i', data[idx:idx + 4])[0]
            idx += 4
            
            setting = SettingDefinition(name, value, min_val, max_val)
            settings.append(setting)
            print(f"  {setting}")
        
        return settings
    
    async def set_setting(self, name: str, value: int) -> bool:
        """Set a setting value on the device."""
        if not self.client or not self.client.is_connected:
            return False
        
        # Encode name length and name
        name_bytes = name.encode('utf-8')
        data = bytes([len(name_bytes)]) + name_bytes
        
        # Encode value (4 bytes, little endian)
        data += struct.pack('<i', value)
        
        # Send command
        await self.client.write_gatt_char(SETTINGS_CHAR_UUID, 
                                          bytes([CMD_SET_SETTING]) + data, 
                                          response=True)
        
        print(f"Set {name} = {value}")
        return True
    
    def create_ui_element(self, setting: SettingDefinition):
        """Create appropriate UI element based on setting type."""
        if setting.is_boolean:
            # Create switch/checkbox for boolean settings
            return self._create_switch(setting)
        else:
            # Create range/slider for numeric settings
            return self._create_range(setting)
    
    def _create_switch(self, setting: SettingDefinition) -> str:
        """Create a switch/checkbox UI element."""
        state = "ON" if setting.value else "OFF"
        return f"[{state}] {setting.name}"
    
    def _create_range(self, setting: SettingDefinition) -> str:
        """Create a range/slider UI element."""
        bar_width = 40
        range_size = setting.max - setting.min
        
        if range_size == 0:
            filled = bar_width
        else:
            filled = int((setting.value - setting.min) / range_size * bar_width)
        
        bar = '█' * filled + '░' * (bar_width - filled)
        return f"[{bar}] {setting.name}: {setting.value} ({setting.min}-{setting.max})"
    
    def display_settings_form(self, settings: List[SettingDefinition]):
        """Display the settings form in the terminal."""
        print("\n" + "=" * 60)
        print("MOUSE SETTINGS")
        print("=" * 60)
        
        for i, setting in enumerate(settings):
            ui_element = self.create_ui_element(setting)
            print(f"{i+1}. {ui_element}")
        
        print("\n" + "=" * 60)
        print("Commands:")
        print("  <number> <value> - Change setting (e.g., '1 1' or '3 150')")
        print("  r - Refresh settings")
        print("  q - Quit")
        print("=" * 60)
    
    async def run_interactive(self):
        """Run the interactive configuration interface."""
        # Find device
        address = await self.find_device()
        
        if not address:
            # Allow manual entry
            address = input("\nEnter device address manually: ").strip()
        
        if not address:
            print("No device specified. Exiting.")
            return
        
        # Connect
        try:
            await self.connect(address)
        except Exception as e:
            print(f"Connection failed: {e}")
            return
        
        # Get settings
        print("\nRequesting settings...")
        settings = await self.request_settings_list()
        
        if not settings:
            print("Failed to retrieve settings. Make sure the device supports the settings service.")
            await self.disconnect()
            return
        
        # Store settings
        self.settings = {s.name: s for s in settings}
        
        # Interactive loop
        while True:
            self.display_settings_form(settings)
            
            try:
                user_input = input("\nCommand: ").strip()
                
                if user_input.lower() == 'q':
                    break
                
                if user_input.lower() == 'r':
                    settings = await self.request_settings_list()
                    self.settings = {s.name: s for s in settings}
                    continue
                
                # Parse "<number> <value>" command
                parts = user_input.split()
                if len(parts) == 2:
                    try:
                        setting_num = int(parts[0])
                        new_value = int(parts[1])
                        
                        if 1 <= setting_num <= len(settings):
                            setting = settings[setting_num - 1]
                            
                            # Validate range
                            if new_value < setting.min or new_value > setting.max:
                                print(f"Value must be between {setting.min} and {setting.max}")
                                continue
                            
                            # Apply setting
                            success = await self.set_setting(setting.name, new_value)
                            
                            if success:
                                # Update local copy
                                setting.value = new_value
                                print(f"✓ Setting updated successfully")
                        else:
                            print("Invalid setting number")
                    except ValueError:
                        print("Invalid input. Use format: <number> <value>")
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        await self.disconnect()


async def main():
    """Main entry point."""
    print("=" * 60)
    print("PAW3395 BLE Mouse Configuration Tool")
    print("=" * 60)
    
    app = MouseConfigApp()
    await app.run_interactive()
    
    print("\nGoodbye!")


if __name__ == "__main__":
    asyncio.run(main())
