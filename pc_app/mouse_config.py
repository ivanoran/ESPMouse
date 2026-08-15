#!/usr/bin/env python3
"""
PC Application for configuring PAW3395 BLE Mouse
Uses bleak library for BLE communication
"""

import asyncio
import struct
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Please install bleak: pip install bleak")
    exit(1)

# Protocol commands
CMD_GET_SETTINGS_LIST = 0x01
CMD_GET_SETTING = 0x02
CMD_SET_SETTING = 0x03
CMD_APPLY_SETTINGS = 0x04

# Report IDs
REPORT_ID_FEATURE = 0x02

@dataclass
class Setting:
    name: str
    value: int
    min_val: int
    max_val: int
    
    @property
    def is_switch(self) -> bool:
        """Check if setting should be displayed as switch (min=0, max=1)"""
        return self.min_val == 0 and self.max_val == 1

class Paw3395MouseConfig:
    def __init__(self):
        self.device_address: Optional[str] = None
        self.client: Optional[BleakClient] = None
        self.settings: Dict[str, Setting] = {}
        self._settings_service_uuid = "5f4e3d2c-1b8a-3f9e-9d4c-a6b50000712d"
        self._settings_char_uuid = "5f4e3d2c-1b8a-3f9e-9d4c-a6b50100712d"
        
    async def find_device(self) -> Optional[str]:
        """Find the mouse device"""
        print("Scanning for PAW3395 Mouse...")
        devices = await BleakScanner.discover(timeout=5.0)
        
        for device in devices:
            if "PAW3395" in device.name or "Mouse" in (device.name or ""):
                print(f"Found device: {device.name} at {device.address}")
                return device.address
        
        return None
    
    async def connect(self, address: str):
        """Connect to the mouse"""
        self.device_address = address
        self.client = BleakClient(address)
        await self.client.connect()
        print(f"Connected to {address}")
        
    async def disconnect(self):
        """Disconnect from the mouse"""
        if self.client:
            await self.client.disconnect()
            self.client = None
            print("Disconnected")
    
    def _encode_command(self, cmd: int, data: bytes = b'') -> bytes:
        """Encode command for feature report"""
        result = bytearray(64)
        result[0] = cmd
        if data:
            result[1:1+len(data)] = data
        return bytes(result)
    
    def _decode_settings_list(self, data: bytes) -> List[Setting]:
        """Decode settings list response"""
        settings = []
        idx = 1  # Skip count byte
        count = data[0]
        
        for _ in range(count):
            if idx >= len(data):
                break
                
            name_len = data[idx]
            idx += 1
            
            if idx + name_len > len(data):
                break
                
            name = data[idx:idx+name_len].decode('utf-8')
            idx += name_len
            
            if idx + 12 > len(data):
                break
                
            value = struct.unpack('<i', data[idx:idx+4])[0]
            idx += 4
            
            min_val = struct.unpack('<i', data[idx:idx+4])[0]
            idx += 4
            
            max_val = struct.unpack('<i', data[idx:idx+4])[0]
            idx += 4
            
            settings.append(Setting(name=name, value=value, min_val=min_val, max_val=max_val))
        
        return settings
    
    async def get_settings_list(self) -> List[Setting]:
        """Get all settings from device"""
        if not self.client or not self.client.is_connected:
            raise Exception("Not connected")
        
        # Send GET_SETTINGS_LIST command via feature report
        cmd_data = self._encode_command(CMD_GET_SETTINGS_LIST)
        await self.client.write_gatt_char(
            self._settings_char_uuid,
            cmd_data,
            response=True
        )
        
        # Wait for notification with response
        await asyncio.sleep(0.1)
        
        # For now, we'll read settings from a simplified approach
        # In real implementation, you'd wait for notifications
        # This is a placeholder - actual implementation depends on HID feature reports
        
        # Using HID feature report approach
        # Note: bleak doesn't directly support HID feature reports
        # You may need to use hidraw on Linux or special HID library
        
        return []
    
    async def set_setting(self, name: str, value: int):
        """Set a setting value"""
        if not self.client:
            raise Exception("Not connected")
        
        name_bytes = name.encode('utf-8')
        data = bytearray([len(name_bytes)])
        data.extend(name_bytes)
        data.extend(struct.pack('<i', value))
        
        cmd_data = self._encode_command(CMD_SET_SETTING, bytes(data))
        await self.client.write_gatt_char(self._settings_char_uuid, cmd_data)
        
        print(f"Set {name} = {value}")


def create_ui_element(setting: Setting) -> str:
    """Create UI element representation for a setting"""
    if setting.is_switch:
        state = "ON" if setting.value else "OFF"
        return f"[{state}] {setting.name}"
    else:
        bar_width = 20
        range_size = setting.max_val - setting.min_val
        if range_size == 0:
            filled = bar_width
        else:
            filled = int((setting.value - setting.min_val) / range_size * bar_width)
        bar = "█" * filled + "░" * (bar_width - filled)
        return f"[{bar}] {setting.name} ({setting.value})"


async def main():
    config = Paw3395MouseConfig()
    
    try:
        # Find and connect to device
        address = await config.find_device()
        if not address:
            print("Device not found. Make sure the mouse is in pairing mode.")
            return
        
        await config.connect(address)
        
        print("\n=== PAW3395 Mouse Configuration ===\n")
        print("Commands:")
        print("  [number] - Toggle switch or change value")
        print("  q - Quit")
        print("  r - Refresh settings")
        print()
        
        while True:
            # Get settings (placeholder - needs proper HID implementation)
            print("Current settings (placeholder - implement HID feature reports):")
            print("  Note: Full implementation requires hidraw or pywin32 for HID")
            print()
            
            cmd = input("Command: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'r':
                print("Refreshing...")
            else:
                try:
                    idx = int(cmd)
                    print(f"Setting {idx} selected (implement value change)")
                except ValueError:
                    print("Invalid command")
    
    except KeyboardInterrupt:
        pass
    finally:
        await config.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
