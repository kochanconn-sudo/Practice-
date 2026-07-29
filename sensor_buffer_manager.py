"""
sensor_buffer_manager.py
マルチレートセンサー対応：センサーバッファリングと統計値計算
"""

from collections import defaultdict
import statistics
import numpy as np


class SensorBufferManager:
    def __init__(self):
        self.ultrasonic_buffer = defaultdict(list)
        self.imu_buffer = defaultdict(list)
        self.rpm_buffer = []
        
    def add_ultrasonic_values(self, ultrasonic_data):
        if ultrasonic_data is None:
            return
        for zone_name, value in ultrasonic_data.items():
            if value is not None and isinstance(value, (int, float)):
                self.ultrasonic_buffer[zone_name].append(float(value))
    
    def add_imu_values(self, imu_acl_x, imu_acl_y, imu_acl_z, imu_gyr_x, imu_gyr_y, imu_gyr_z):
        for name, value in [
            ('acl_x', imu_acl_x), ('acl_y', imu_acl_y), ('acl_z', imu_acl_z),
            ('gyr_x', imu_gyr_x), ('gyr_y', imu_gyr_y), ('gyr_z', imu_gyr_z)
        ]:
            if value is not None and isinstance(value, (int, float)):
                self.imu_buffer[f'imu_{name}'].append(float(value))
    
    def add_rpm_value(self, rpm_value):
        if rpm_value is not None and isinstance(rpm_value, (int, float)):
            self.rpm_buffer.append(float(rpm_value))
    
    def compute_statistics(self, latest_sensor_data):
        extended_data = dict(latest_sensor_data)
        
        for zone_name, buffer_values in self.ultrasonic_buffer.items():
            if buffer_values:
                latest_val = buffer_values[-1]
                avg_val = statistics.mean(buffer_values)
                min_val = min(buffer_values)
                max_val = max(buffer_values)
                
                extended_data[f'ultrasonic/{zone_name}_latest'] = round(latest_val, 1)
                extended_data[f'ultrasonic/{zone_name}_avg'] = round(avg_val, 1)
                extended_data[f'ultrasonic/{zone_name}_min'] = round(min_val, 1)
                extended_data[f'ultrasonic/{zone_name}_max'] = round(max_val, 1)
        
        for imu_name, buffer_values in self.imu_buffer.items():
            if buffer_values:
                latest_val = buffer_values[-1]
                avg_val = statistics.mean(buffer_values)
                std_val = statistics.stdev(buffer_values) if len(buffer_values) > 1 else 0.0
                
                extended_data[f'{imu_name}_latest'] = round(latest_val, 4)
                extended_data[f'{imu_name}_avg'] = round(avg_val, 4)
                extended_data[f'{imu_name}_std'] = round(std_val, 4)
        
        if self.rpm_buffer:
            rpm_latest = self.rpm_buffer[-1]
            rpm_avg = statistics.mean(self.rpm_buffer)
            
            extended_data['rpm_latest'] = round(rpm_latest, 1)
            extended_data['rpm_avg'] = round(rpm_avg, 1)
        
        return extended_data
    
    def reset(self):
        self.ultrasonic_buffer = defaultdict(list)
        self.imu_buffer = defaultdict(list)
        self.rpm_buffer = []


_buffer_manager = None

def init_buffer_manager():
    global _buffer_manager
    _buffer_manager = SensorBufferManager()
    return _buffer_manager

def get_buffer_manager():
    global _buffer_manager
    if _buffer_manager is None:
        init_buffer_manager()
    return _buffer_manager

def add_sensor_to_buffer(sensor_data):
    manager = get_buffer_manager()
    
    ultrasonic_data = {}
    for i in range(5):
        zone_name = f'zone_{i}'
        key_in_sensor_data = f'ultrasonic/{zone_name}'
        if key_in_sensor_data in sensor_data:
            ultrasonic_data[zone_name] = sensor_data[key_in_sensor_data]
    if ultrasonic_data:
        manager.add_ultrasonic_values(ultrasonic_data)
    
    imu_acl_x = sensor_data.get('imu_acl_x')
    imu_acl_y = sensor_data.get('imu_acl_y')
    imu_acl_z = sensor_data.get('imu_acl_z')
    imu_gyr_x = sensor_data.get('imu_gyr_x')
    imu_gyr_y = sensor_data.get('imu_gyr_y')
    imu_gyr_z = sensor_data.get('imu_gyr_z')
    manager.add_imu_values(imu_acl_x, imu_acl_y, imu_acl_z, imu_gyr_x, imu_gyr_y, imu_gyr_z)
    
    rpm_value = sensor_data.get('rpm_value')
    if rpm_value:
        manager.add_rpm_value(rpm_value)

def get_sensor_data_with_statistics(latest_sensor_data):
    manager = get_buffer_manager()
    extended_data = manager.compute_statistics(latest_sensor_data)
    manager.reset()
    return extended_data
