import config
import ultrasonic
import time

sensors = {}
for name in config.ULTRASONIC_SENSOR_LIST:
    sensors[name] = ultrasonic.Ultrasonic(name)

print("各センサーの前に順番に手をかざしてください")
print("Ctrl+C で終了\n")

while True:
    distances = {}
    for name, sensor in sensors.items():
        distances[name] = sensor.get_data()
    
    print(f"RrLH:{distances['RrLH']:>5.0f}, FrLH:{distances['FrLH']:>5.0f}, "
          f"FrFR:{distances['FrFR']:>5.0f}, FrRH:{distances['FrRH']:>5.0f}, RrRH:{distances['RrRH']:>5.0f}")
    time.sleep(0.3)
