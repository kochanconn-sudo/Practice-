# ultrasonic.py
# coding:utf-8
import os
import time
import config
import numpy as np

# GPIO backend will be determined at runtime
# Default values - will be overridden by init_gpio()
USE_GPIOZERO = False
GPIO = None
factory = None
DigitalOutputDevice = None
DigitalInputDevice = None

def init_gpio():
    """Initialize GPIO backend based on detected platform"""
    global USE_GPIOZERO, GPIO, factory
    
    if config.GPIO_BACKEND == 'RPi.GPIO':
        # RPi.GPIO for Raspberry Pi 4 and older
        import RPi.GPIO as GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(config.ULTRASONIC_ECHO_PIN_NUMBER, GPIO.IN)
        GPIO.setup(config.ULTRASONIC_TRIGER_PIN_NUMBER, GPIO.OUT, initial=GPIO.LOW)
        USE_GPIOZERO = False
        
    elif config.GPIO_BACKEND == 'Jetson.GPIO':
        # Jetson.GPIO for NVIDIA Jetson boards
        import Jetson.GPIO as GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(config.ULTRASONIC_ECHO_PIN_NUMBER, GPIO.IN)
        GPIO.setup(config.ULTRASONIC_TRIGER_PIN_NUMBER, GPIO.OUT, initial=GPIO.LOW)
        USE_GPIOZERO = False
        
    elif config.GPIO_BACKEND == 'gpiozero':
        # gpiozero for Raspberry Pi 5
        try:
            global DigitalOutputDevice, DigitalInputDevice
            from gpiozero import DigitalOutputDevice, DigitalInputDevice
            from gpiozero.pins.lgpio import LGPIOFactory
            factory = LGPIOFactory()
            USE_GPIOZERO = True
        except ImportError as e:
            print(f"gpiozero not available: {e}")
            # Fallback to Jetson.GPIO if gpiozero is not available
            import Jetson.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(config.ULTRASONIC_ECHO_PIN_NUMBER, GPIO.IN)
            GPIO.setup(config.ULTRASONIC_TRIGER_PIN_NUMBER, GPIO.OUT, initial=GPIO.LOW)
            USE_GPIOZERO = False
    else:
        raise ValueError(f"Unsupported GPIO backend: {config.GPIO_BACKEND}")
# BOARD to BCM pin mapping for Raspberry Pi
BOARD_TO_BCM = {
    11: 17, 13: 27, 15: 22, 29: 5, 31: 6, 33: 13, 35: 19, 37: 26,
    12: 18, 16: 23, 18: 24, 22: 25, 32: 12, 36: 16, 38: 20, 40: 21
}

# GPIO will be initialized when first Ultrasonic instance is created
gpio_initialized = False

class Ultrasonic:
    def __init__(self, sensor_name):
        # 超音波センサ(HC-SR04)のクラス作成
        # データシート参考：https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
        # 超音波発信/受信用のGPiOピン番号
        global gpio_initialized
        
        # Initialize GPIO backend if not already done
        if not gpio_initialized:
            init_gpio()
            gpio_initialized = True
            
        self.sensor_name = sensor_name
        self.records = np.zeros(config.RIGHT_LEFT_RECORD_NUMBER)
        self.sound_speed_mps = 343 # Speed of sound in m/s
        self.cutoff = config.CUTOFF_RANGE #
        self.cutofftime = self.cutoff*2/1000 / (self.sound_speed_mps) # Max time for echo
        self.distance = 0
        
        if USE_GPIOZERO:
            # gpiozero version (RPi5 compatible)
            echo_board = config.ULTRASONIC_ECHO_PINS[sensor_name]
            trig_board = config.ULTRASONIC_TRIG_PINS[sensor_name]
            
            echo_bcm = BOARD_TO_BCM.get(echo_board, echo_board)
            trig_bcm = BOARD_TO_BCM.get(trig_board, trig_board)
            
            self.echo_pin = DigitalInputDevice(echo_bcm, pin_factory=factory)
            self.trig_pin = DigitalOutputDevice(trig_bcm, pin_factory=factory)
        else:
            # RPi.GPIO/Jetson.GPIO version
            self.echo_pin = config.ULTRASONIC_ECHO_PINS[sensor_name]
            self.trig_pin = config.ULTRASONIC_TRIG_PINS[sensor_name]
        
    # 障害物センサ測定関数
    def measure(self):
        """Measure the distance using the ultrasonic sensor."""
        self.distance, sigoff, sigon = 0, 0, 0
        
        # 10usのトリガー信号を送信
        if USE_GPIOZERO:
            self.trig_pin.on()
            time.sleep(0.00001)
            self.trig_pin.off()
        else:
            GPIO.output(self.trig_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, GPIO.LOW)

        # エコー信号の立ち下がりと立ち上がりの時間を記録
        starttime = time.perf_counter()
        if USE_GPIOZERO:
            while not self.echo_pin.is_active:
                sigoff = time.perf_counter()
                if sigoff - starttime > 0.02: 
                    break
        else:
            while(GPIO.input(self.echo_pin) == GPIO.LOW):
                sigoff = time.perf_counter()
                if sigoff - starttime > 0.02: 
                    break

        # エコー信号の立ち上がり時間が音速の往復時間
        if USE_GPIOZERO:
            while self.echo_pin.is_active:
                sigon = time.perf_counter()
                if sigon - sigoff > self.cutofftime: 
                    break
        else:
            while(GPIO.input(self.echo_pin) == GPIO.HIGH):
                sigon = time.perf_counter()
                if sigon - sigoff > self.cutofftime: 
                    break

        # time * sound speed / 2(round trip)
        measured_distance = int((sigon - sigoff) * self.sound_speed_mps / 2 *1000)
        measured_distance = min(measured_distance, self.cutoff)
        # 負値のノイズの場合は一つ前のデータに置き換え
        if measured_distance < 0:
            print("@",self.sensor_name,", a noise occureed, use the last value")
            self.distance = self.records[0]
            print(self.records)
        else:
            self.distance = measured_distance

        # 過去の超音波センサの値を記録の一番前に挿入し、最後を消す
        self.records = np.insert(self.records, 0, self.distance)
        self.records = np.delete(self.records,-1)
        return self.distance
    
    def get_data(self):
        return self.measure()
    
    def cleanup(self):
        print(f"Ultrasonic:{self.sensor_name} cleanup complete.")
        if USE_GPIOZERO:
            self.echo_pin.close()
            self.trig_pin.close()

def plot_csv_data(file_path):
    import matplotlib.pyplot as plt
    import pandas as pd
    # CSVファイルを読み取る
    data = pd.read_csv(file_path)

    # グラフの設定
    plt.figure(figsize=(10, 6))
    for column in data.columns[1:]:  # Timestamp以外の列をプロット
        plt.plot(data["Timestamp"], data[column], label=column)

    # グラフのタイトルとラベル
    plt.title("Ultrasonic Sensor Measurements")
    plt.xlabel("Timestamp (s)")
    plt.ylabel("Distance (mm)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

   # 保存先ディレクトリとファイル名
    output_dir = os.path.dirname(file_path)
    output_file = os.path.join(output_dir, "ultrasonic_measurements.png")

    # グラフを保存
    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"グラフが保存されました: {output_file}")

    # グラフの表示
    #plt.show()

def main():
    import config
    if not USE_GPIOZERO and config.GPIO_BACKEND == 'RPi.GPIO':
        import RPi.GPIO as GPIO
    elif not USE_GPIOZERO and config.GPIO_BACKEND == 'Jetson.GPIO':
        import Jetson.GPIO as GPIO
    
    # 超音波センサを設定、使う分だけリストにultrasonicインスタンスを入れる
    ultrasonic_instances = [] 
    # 一つだけ使う場合、複数使う場合はコメントアウト外す、通常はconfigからそのままimport
    #config.ULTRASONIC_SENSOR_LIST = ["FrFR"]
    # config.ULTRASONIC_SENSOR_LIST = ["FrLH","FrFR","FrRH"]
    config.ULTRASONIC_SENSOR_LIST = ["RrLH", "FrLH", "FrFR", "FrRH","RrRH"] #,"RrRH"

    # Initialize ultrasonic sensors
    ultrasonic_instances = [Ultrasonic(sensor_name) for sensor_name in config.ULTRASONIC_SENSOR_LIST]
    print("使用する超音波センサ:", [sensor.sensor_name for sensor in ultrasonic_instances])

    # データ記録用配列作成
    measured_distance_list = np.zeros(len(ultrasonic_instances))
    #distance_stack = np.zeros(len(ultrasonic_instances)+1)
    num_sensors = len(ultrasonic_instances)
    distance_stack = np.zeros((0, num_sensors + 1))  # 時間列 + センサー数列

    # 計測回数
    try:
        user_input = input(f"計測回数を入力 (Enter でデフォルト {config.SAMPLING_TIMES} 回): ")
        sampling_times = int(user_input) if user_input.strip() else config.SAMPLING_TIMES
    except ValueError:
        print(f"無効な入力です。デフォルト値 {config.SAMPLING_TIMES} 回を使用します。")
        sampling_times = config.SAMPLING_TIMES    
        time.sleep(0.5)
        print(f'Enterで計測開始、計測回数：{sampling_times} ')

    # 計測    
    start_time = time.perf_counter()
    try:
        for i in range(sampling_times):
            timestamp = time.perf_counter() - start_time
            measured_distances = [sensor.measure() for sensor in ultrasonic_instances]

            # Add timestamp and measurements to the stack
            distance_stack = np.vstack((distance_stack, [timestamp] + measured_distances))

            # Display measurements
            message = f"Timestamp: {timestamp:.2f}, " + ", ".join(
                f"{name}: {dist}" for name, dist in zip(config.ULTRASONIC_SENSOR_LIST, measured_distances)
            )
            print(message)

    except KeyboardInterrupt:
        sampling_times = i
        print("計測が中断されました。")

    finally:
        # recordsディレクトリが存在しない場合は作成
        output_dir = os.path.dirname(config.RECORDS_DIRECTORY_ULTRASONIC_TEST)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"ディレクトリを作成しました: {output_dir}")
        
        np.savetxt(config.RECORDS_DIRECTORY_ULTRASONIC_TEST, 
                   distance_stack,
                   fmt=['%.3f'] + ['%d'] * len(config.ULTRASONIC_SENSOR_LIST), delimiter=",",
                   header="Timestamp," + ",".join(config.ULTRASONIC_SENSOR_LIST),comments=""
                   )
        ## 列方向に時間平均: np.round axis=0、スライスで時間の列は取得しない[:,1:]
        print('測定回数： ',sampling_times)
        print('平均距離(mm)：', np.round(np.mean(distance_stack[:,1:], axis=0),0))
        print("平均測定時間/センサ(秒):",round((time.perf_counter()-start_time)/sampling_times/len(ultrasonic_instances),2))
        print("記録保存--> ",config.RECORDS_DIRECTORY_ULTRASONIC_TEST)
        plot_csv_data(config.RECORDS_DIRECTORY_ULTRASONIC_TEST)
        
        # Cleanup
        if USE_GPIOZERO:
            for sensor in ultrasonic_instances:
                sensor.cleanup()
        else:
            GPIO.cleanup()
        
# ROSパート（ROS2の有無を判定してインポート）
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Range
    from std_msgs.msg import Header

    class UltrasonicNode(Node):
        def __init__(self, queue_size, timer_period, sensor_names):
            super().__init__('ultrasonic_node')

            # Initialize the ultrasonic sensors
            self.pubs  = {}
            self.ultrasonic_instances = []
            for sensor_name in sensor_names:
                self.ultrasonic_instances.append(Ultrasonic(sensor_name))
                topic_name = f'/sonner/{sensor_name}/range'
                self.pubs[sensor_name] = self.create_publisher(Range, topic_name, queue_size)

            # タイマーの設定 (例えば0.1秒ごとにデータを送信)
            self.timer = self.create_timer(timer_period, self.publish_data)

            # センサーの名前リスト
            self.sensor_names = sensor_names

        def publish_data(self):
            # センサーごとの距離を計測
            for sensor_name in self.sensor_names:
                distance = self.ultrasonic_instances[self.sensor_names.index(sensor_name)].measure()

                # Range メッセージの作成
                msg = Range()

                # ヘッダーの設定
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()  # 現在のタイムスタンプ
                msg.header.frame_id = 'base_link'  # センサーが取り付けられているフレーム（例）

                # メッセージの各フィールドの設定
                msg.radiation_type = 0  # 超音波センサーの場合は0（レーザーセンサーは1）
                msg.field_of_view = 3.14/12  # センサーの視野角（ラジアン）
                msg.min_range = 0.2  # 最小測定距離
                msg.max_range = 4.0  # 最大測定距離
                msg.range = distance  # 測定された距離

                # センサーごとにデータをパブリッシュ
                self.pubs[sensor_name].publish(msg)

                self.get_logger().info(f"Published {sensor_name} range: {distance} meters")

                # Publish the distances as a single message
                self.pubs[sensor_name].publish(msg)

        def spin(self):
            try:
                self.get_logger().info(f"Start Spin")
                rclpy.spin(self)
            except KeyboardInterrupt:
                    print("\nROS2 node stopped by user.")
            finally:
                if rclpy.ok():
                    self.destroy_node()  # Clean up resources
                    rclpy.shutdown()  # Shutdown ROS 2 system

    def main_ros(queue_size=1, timer_period=0.1):
        rclpy.init()
        sensor_names = config.ULTRASONIC_SENSOR_LIST
        ultrasonic_node = UltrasonicNode(queue_size=10, timer_period=0.1, sensor_names=sensor_names)
        try:
            ultrasonic_node.spin()
        except KeyboardInterrupt:
            print("\nShutting down ROS node...")



except ImportError:
    # print("ROS2関連ライブラリがインストールされていません。ROS2モードは無効です。")
    rclpy = None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Ultrasonic measurement with or without ROS2")
    parser.add_argument('--ros', action='store_true', help="Run with ROS2 node")
    args = parser.parse_args()

    if args.ros and rclpy:
        print("Start with ROS2")
        main_ros()
    else:
        main()

    