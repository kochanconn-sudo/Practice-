# device_detection.py
# coding:utf-8
"""
Device detection utilities for platform-specific configuration
"""
import os
import platform
from dataclasses import dataclass
from typing import Optional


@dataclass
class DeviceInfo:
    """Device information with all platform-specific settings"""
    device_type: str
    platform_name: str
    gpio_backend: str
    i2c_bus: int


def detect_device() -> DeviceInfo:
    """
    Detect device type and return all platform-specific configuration.
    
    Returns:
        DeviceInfo: Complete device information including GPIO backend and I2C bus
    """
    try:
        # Check for Jetson using multiple methods
        jetson_detected = False
        jetson_model = ""
        
        # Method 1: Check /etc/nv_tegra_release
        if os.path.exists('/etc/nv_tegra_release'):
            jetson_detected = True
            
        # Method 2: Check /proc/device-tree/model for Jetson Orin
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model_content = f.read().strip('\x00').lower()
                if 'jetson orin' in model_content:
                    jetson_model = "Orin"
                elif 'jetson' in model_content:
                    jetson_detected = True
                    jetson_model = "Unknown"
        except FileNotFoundError:
            pass
        
        if jetson_detected:
            if jetson_model == "Orin" or "orin" in jetson_model.lower():
                return DeviceInfo(
                    device_type='JETSON_ORIN_NANO',
                    platform_name='Jetson Orin Nano',
                    gpio_backend='Jetson.GPIO',
                    i2c_bus=7
                )
            else:
                return DeviceInfo(
                    device_type='JETSON_OTHER', 
                    platform_name='Jetson (Other)',
                    gpio_backend='Jetson.GPIO',
                    i2c_bus=7
                )
        
        # Check for Raspberry Pi using /proc/device-tree/model
        model = ""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip('\x00').lower()
        except FileNotFoundError:
            pass
        
        if 'raspberry pi 5' in model:
            return DeviceInfo(
                device_type='RPI5',
                platform_name='Raspberry Pi 5',
                gpio_backend='gpiozero',
                i2c_bus=1
            )
        elif 'raspberry pi 4' in model:
            return DeviceInfo(
                device_type='RPI4',
                platform_name='Raspberry Pi 4',
                gpio_backend='RPi.GPIO',
                i2c_bus=1
            )
        elif 'raspberry' in model:
            return DeviceInfo(
                device_type='RPI_OTHER',
                platform_name='Raspberry Pi (Other)',
                gpio_backend='RPi.GPIO',
                i2c_bus=1
            )
        
        # Fallback check using platform.machine()
        machine = platform.machine()
        if machine.startswith('arm'):
            return DeviceInfo(
                device_type='RPI_OTHER',
                platform_name='ARM Device (assumed Raspberry Pi)',
                gpio_backend='RPi.GPIO',
                i2c_bus=1
            )
        
        # Default fallback to Jetson settings
        return DeviceInfo(
            device_type='UNKNOWN',
            platform_name='Unknown (defaulting to Jetson)',
            gpio_backend='gpiozero',
            i2c_bus=7
        )
        
    except Exception as e:
        print(f"Device detection error: {e}")
        # Safe fallback
        return DeviceInfo(
            device_type='UNKNOWN',
            platform_name='Unknown (error occurred)',
            gpio_backend='gpiozero',
            i2c_bus=7
        )


# Convenience functions for backward compatibility
def detect_device_type() -> str:
    """Legacy function for backward compatibility"""
    return detect_device().device_type


def detect_platform() -> str:
    """Legacy function for backward compatibility"""
    return detect_device().platform_name