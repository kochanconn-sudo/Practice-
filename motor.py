# coding:utf-8
import Adafruit_PCA9685
import config
import time

# カスタムI2Cクラス：設定可能なI2Cバスを使用
class CustomI2C:
    """カスタムI2Cクラス：config.I2C_BUSを使用"""
    
    def get_i2c_device(self, address, busnum=None, i2c_interface=None, **kwargs):
        """設定可能なI2Cバスを使用するget_i2c_device関数"""
        import Adafruit_GPIO.I2C as I2C
        
        # busnumが指定されていない場合、config.I2C_BUSを使用
        if busnum is None:
            busnum = config.I2C_BUS
            
        return I2C.Device(address, busnum, i2c_interface, **kwargs)

class Motor:
    def __init__(self):
        # Initialize PCA9685 with custom I2C that uses configurable bus
        custom_i2c = CustomI2C()
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, i2c=custom_i2c)
        self.pwm.set_pwm_freq(60)

        # Configuration parameters
        self.CHANNEL_STEERING = config.CHANNEL_STEERING
        self.CHANNEL_THROTTLE = config.CHANNEL_THROTTLE
        self.STEERING_CENTER_PWM = config.STEERING_CENTER_PWM
        self.STEERING_WIDTH_PWM = config.STEERING_WIDTH_PWM
        self.STEERING_RIGHT_PWM = config.STEERING_RIGHT_PWM
        self.STEERING_LEFT_PWM = config.STEERING_LEFT_PWM
        self.THROTTLE_STOPPED_PWM = config.THROTTLE_STOPPED_PWM
        self.THROTTLE_WIDTH_PWM = config.THROTTLE_WIDTH_PWM
        self.THROTTLE_FORWARD_PWM = config.THROTTLE_FORWARD_PWM
        self.THROTTLE_REVERSE_PWM = config.THROTTLE_REVERSE_PWM
        
        self.JOYSTICK_STEERING_SCALE = config.JOYSTICK_STEERING_SCALE
        self.JOYSTICK_THROTTLE_SCALE = config.JOYSTICK_THROTTLE_SCALE
        

    def set_steering_pwm_value(self, steering_value):
        # steering_pwm_value = int(self.STEERING_CENTER_PWM + abs(self.STEERING_RIGHT_PWM - self.STEERING_CENTER_PWM) * steering_value)
        if steering_value >= 0: # right
            steering_pwm_value = int(self.STEERING_CENTER_PWM + (self.STEERING_RIGHT_PWM - self.STEERING_CENTER_PWM) * steering_value)
        else: # left
            steering_pwm_value = int(self.STEERING_CENTER_PWM + (self.STEERING_LEFT_PWM - self.STEERING_CENTER_PWM) * steering_value * -1)
        #print(steering_pwm_value)        
        steering_pwm_value = self.limit_steering_pwm(steering_pwm_value)
        self.pwm.set_pwm(self.CHANNEL_STEERING, 0, steering_pwm_value)
        #print(f"Steering PWM set to: {steering_pwm_value}")

    def set_throttle_pwm_value(self, throttle_value):
        # RCによっては調整が必要、前提はTHROTTLE_FORWARD_PWM < THROTTLE_REVERSE_PWM
        if throttle_value >= 0: # up
            throttle_pwm_value = int(self.THROTTLE_STOPPED_PWM + (self.THROTTLE_FORWARD_PWM - self.THROTTLE_STOPPED_PWM) * throttle_value)
        else: #down
            throttle_pwm_value = int(self.THROTTLE_STOPPED_PWM + (self.THROTTLE_REVERSE_PWM - self.THROTTLE_STOPPED_PWM) * throttle_value * -1)
        
        self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, throttle_pwm_value)
        # print(f"Throttle PWM set to: {throttle_pwm_value}")

    def limit_steering_pwm(self, steering_pwm_value):
        if steering_pwm_value > config.STEERING_HI_LIMIT:
            #print ("\n!!!警告!!! 壊さないように最大値:{}で設定ください!\n".format(config.STEERING_HI_LIMIT))
            return config.STEERING_HI_LIMIT
        elif steering_pwm_value < config.STEERING_LO_LIMIT:
            #print ("\n!!!警告!!! 壊さないように最小値:{}で設定ください!\n".format(config.STEERING_LO_LIMIT))
            return config.STEERING_LO_LIMIT
        else:
            return steering_pwm_value
        

    def adjust_steering(self):
        print('========================================')
        print(' ステアリング調整、ステアの中心位置を決める')
        print('========================================')
        while True:
            print('PWM の値を入力, 例 390')
            print('中心値が決まればEnter')
            print('ジジっとノイズがし続けたら注意、壊れる、、、')
            ad = input()
            if ad == 'e' or ad =='':
                self.STEERING_RIGHT_PWM = self.STEERING_CENTER_PWM + self.STEERING_WIDTH_PWM
                self.STEERING_LEFT_PWM = self.STEERING_CENTER_PWM - self.STEERING_WIDTH_PWM
                break
            self.STEERING_CENTER_PWM = int(ad)
            self.limit_steering_pwm(self.STEERING_CENTER_PWM)
            self.pwm.set_pwm(self.CHANNEL_STEERING, 0, self.STEERING_CENTER_PWM)
        print('')
        return self.STEERING_RIGHT_PWM,self.STEERING_CENTER_PWM,self.STEERING_LEFT_PWM

    def adjust_throttle(self):
        print('========================================')
        print(' スロットル調整、ニュートラル位置を決める')
        print('========================================')
        while True:
            print('PWM の値を入力, 例 390')
            print('中心値が決まればEnter')
            ad = input()
            if ad == 'e' or ad =='':
                self.THROTTLE_FORWARD_PWM = self.THROTTLE_STOPPED_PWM + self.THROTTLE_WIDTH_PWM
                self.THROTTLE_REVERSE_PWM = self.THROTTLE_STOPPED_PWM - self.THROTTLE_WIDTH_PWM
                break
            self.THROTTLE_STOPPED_PWM = int(ad)
            self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, self.THROTTLE_STOPPED_PWM)
        print('')
        return self.THROTTLE_FORWARD_PWM,self.THROTTLE_STOPPED_PWM,self.THROTTLE_REVERSE_PWM

    def breaking(self):
            print(" breaking!!!")
            self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, self.THROTTLE_STOPPED_PWM)
            time.sleep(0.05)
            self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, self.THROTTLE_REVERSE_PWM)
            time.sleep(0.05)
            self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, self.THROTTLE_STOPPED_PWM)
            time.sleep(0.05)
            self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, self.THROTTLE_REVERSE_PWM)
            time.sleep(0.05)
            self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, self.THROTTLE_STOPPED_PWM)

    def cleanup(self):
            # 停止処理
            self.set_throttle_pwm_value(0)
            self.set_steering_pwm_value(0)
            print("Motor cleanup complete.")

# ROS2の有無を判定してインポート
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32
    from geometry_msgs.msg import Twist

    class MotorROSNode(Node):
        def __init__(self, motor):
            super().__init__('motor_node')
            self.motor = motor

            # /cmd_vel (Twist) — 統一インターフェース（推奨）
            self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            # 互換用: 個別Float32トピック
            self.create_subscription(Float32, '/steering', self.steering_callback, 10)
            self.create_subscription(Float32, '/throttle', self.throttle_callback, 10)

        def cmd_vel_callback(self, msg):
            steering = max(-1.0, min(1.0, msg.angular.z))
            throttle = max(-1.0, min(1.0, msg.linear.x))
            self.motor.set_steering_pwm_value(steering)
            self.motor.set_throttle_pwm_value(throttle)

        def steering_callback(self, msg):
            self.motor.set_steering_pwm_value(msg.data)

        def throttle_callback(self, msg):
            self.motor.set_throttle_pwm_value(msg.data)

    def main_ros(args=None):
        rclpy.init(args=args)
        motor = Motor()
        node = MotorROSNode(motor)
        try:
            node.get_logger().info("Motor node started")
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            motor.cleanup()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

except ImportError:
    rclpy = None

def main_manual():
    from config import STEERING_RIGHT_PWM,STEERING_CENTER_PWM,STEERING_LEFT_PWM,THROTTLE_FORWARD_PWM,THROTTLE_STOPPED_PWM,THROTTLE_REVERSE_PWM
    motor = None
    try:
        motor = Motor()
        motor.set_steering_pwm_value(0)
        motor.set_throttle_pwm_value(0)
        STEERING_RIGHT_PWM,STEERING_CENTER_PWM,STEERING_LEFT_PWM = motor.adjust_steering()
        THROTTLE_FORWARD_PWM,THROTTLE_STOPPED_PWM,THROTTLE_REVERSE_PWM = motor.adjust_throttle()

    except KeyboardInterrupt:
        print("\n中断しました。")

    finally:
        print("---下記をconfig.pyの値に入力。\n値の微調整は走りながら決定。")
        print(f'STEERING_RIGHT_PWM = {STEERING_RIGHT_PWM}')
        print(f'STEERING_CENTER_PWM = {STEERING_CENTER_PWM}')
        print(f'STEERING_LEFT_PWM = {STEERING_LEFT_PWM}')
        print(f'THROTTLE_FORWARD_PWM = {THROTTLE_FORWARD_PWM}')
        print(f'THROTTLE_STOPPED_PWM = {THROTTLE_STOPPED_PWM}')
        print(f'THROTTLE_REVERSE_PWM = {THROTTLE_REVERSE_PWM}')
        print("---上記をconfig.pyの値に入力。\n値の微調整は走りながら決定。")
        if motor:
            motor.cleanup()

if __name__ == "__main__":
    import argparse

    # デバイス検出とプラットフォーム設定
    from device_detection import detect_device
    device_info = detect_device()
    config.DEVICE_TYPE = device_info.device_type
    config.PLATFORM_NAME = device_info.platform_name
    config.GPIO_BACKEND = device_info.gpio_backend
    config.I2C_BUS = device_info.i2c_bus
    print(f"Platform detected: {config.PLATFORM_NAME}, I2C Bus: {config.I2C_BUS}")

    parser = argparse.ArgumentParser(description="Motor control with or without ROS2")
    parser.add_argument('--ros', action='store_true', help="Run with ROS2 node")
    args = parser.parse_args()

    if args.ros and rclpy:
        print('''
        別のターミナルを開き下記のコマンド例を実行して確認。
        ステアリングを右に操作する例 (0.5 右)
            ros2 topic pub /steering std_msgs/msg/Float32 "{data: 0.5}"
        ステアリングを左に操作する例 (-0.5 左)
            ros2 topic pub /steering std_msgs/msg/Float32 "{data: -0.5}"
        ステアリングを中心に戻す例
            ros2 topic pub /steering std_msgs/msg/Float32 "{data: 0.0}"
        ''')
        main_ros()
    else:
        if args.ros and not rclpy:
            print("Warning: ROS2 is not available. Switching to manual mode.")
        main_manual()
