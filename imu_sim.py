import config
import zmq
import json
import time
from collections import deque

class IMU:
    def __init__(self, memory_size=3):
        self.sensor_name = "imu"
        self.latest_sim_time = 0.0
        self.before_sim_time = 0.0
        self.is_initialized = False

        self.memory_size = memory_size
        self.imu_acceleration = {
            axis: deque([0.0] * memory_size, maxlen=memory_size) for axis in "xyz"
        }
        self.imu_angular_velocity = {
            axis: deque([0.0] * memory_size, maxlen=memory_size) for axis in "xyz"
        }
        self.imu_jerk = {
            axis: deque([0.0] * memory_size, maxlen=memory_size) for axis in "xyz"
        }
        self.imu_angle = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.imu_velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.imu_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.bias = {"x": 0.0, "y": 0.0, "z": 0.0}

        # ZeroMQ subscriber
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, self.sensor_name)
        self.subscriber.setsockopt(zmq.RCVTIMEO, 100)
        self.subscriber.connect("tcp://localhost:5555")

    def __del__(self):
        self.subscriber.close()
        self.context.term()

    def calculate_jerk(self, dt):
        """加加速度（ジャーク）の計算"""
        for axis in self.imu_acceleration:
            self.imu_jerk[axis].append(
                (self.imu_acceleration[axis][-1] - self.imu_acceleration[axis][-2]) / dt
            )

    def calculate_angle_from_gyro(self, dt):
        """角速度から角度を積算"""
        for axis in "xyz":
            self.imu_angle[axis] += self.imu_angular_velocity[axis][-1] * dt

    def calculate_velocity_and_position(self, dt):
        """加速度から速度と位置を積算"""
        for axis in "xyz":
            # バイアス補正を適用
            corrected_acceleration = self.imu_acceleration[axis][-1] - self.bias[axis]

            # フィルタリング（移動平均）
            filtered_acceleration = (
                sum(list(self.imu_acceleration[axis])[-self.memory_size:]) / self.memory_size
            )
            
            # バイアス補正後の加速度にフィルタリングを適用（必要に応じて選択）
            acceleration_to_use = corrected_acceleration  
            
            # 速度を更新
            self.imu_velocity[axis] += acceleration_to_use * dt

            # 位置を更新
            self.imu_position[axis] += self.imu_velocity[axis] * dt

    def get_raw_data(self):
        try:
            [topic, msg] = self.subscriber.recv_multipart()
            topic_name = topic.decode('utf-8')
            if topic_name == self.sensor_name:
                data = json.loads(msg.decode('utf-8'))
                sim_time = data["sim_time"]
                acc = {}
                gyro = {}
                for axis in "xyz":
                    acc[axis] = data["linear_acceleration"][axis]
                    gyro[axis] = data["angular_velocity"][axis]

                return True, (sim_time, acc, gyro)

        except json.JSONDecodeError as e:
            # print("json decoding error")
            pass
        except zmq.Again as e:
            # print("No message received within the timeout period.")
            pass

        return False, None


    def initialize(self, duration=8.0, discard_samples=50):
        """IMUの初期化（バイアス推定）"""
        print("Initializing IMU...")
        start_time = time.perf_counter()
        samples = {"x": [], "y": [], "z": []}
        discarded_count = 0

        while time.perf_counter() - start_time < duration:
            while True:
                ret, raw_data = self.get_raw_data()
                if ret:
                    break

            sim_time = raw_data[0]
            imu_raw_acceleration = raw_data[1]
            imu_raw_angular_velocity = raw_data[2]

            # 最初のdiscard_samples個のサンプルをスキップ
            if discarded_count < discard_samples:
                discarded_count += 1
            else:
                for i, axis in enumerate("xyz"):
                    samples[axis].append(imu_raw_acceleration[axis])

            time.sleep(0.01)  # サンプリング間隔

        # バイアスを計算
        for axis in "xyz":
            if samples[axis]:  # サンプルが存在する場合に計算
                self.bias[axis] = sum(samples[axis]) / len(samples[axis])
            else:
                self.bias[axis] = 0.0  # サンプルがない場合のデフォルト値
        self.is_initialized = True
        print(f"IMU initialized with bias: {self.bias}")

    def measure(self):
        raw_data = None
        while True:
            ret, raw_data = self.get_raw_data()
            if ret:
                break

        sim_time = raw_data[0]
        imu_raw_acceleration = raw_data[1]
        imu_raw_angular_velocity = raw_data[2]

        self.before_sim_time = self.latest_sim_time
        self.latest_sim_time = sim_time
        dt = self.latest_sim_time - self.before_sim_time
        for axis in "xyz":
            self.imu_acceleration[axis].append(
                imu_raw_acceleration[axis]
            )
            self.imu_angular_velocity[axis].append(
                imu_raw_angular_velocity[axis]
            )

        if not self.is_initialized:
            self.is_initialized = True
        else:
            self.calculate_jerk(dt)
            self.calculate_angle_from_gyro(dt)
            self.calculate_velocity_and_position(dt)

        return (
            self.imu_acceleration, 
            self.imu_angular_velocity,
            self.imu_angle,
            self.imu_jerk,
            self.imu_velocity,
            self.imu_position
        )

if __name__ == "__main__":

    imu = IMU()
    try:
        imu.initialize()

        while True:
            value = imu.measure()
            print(value)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


