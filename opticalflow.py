import time
from multiprocessing import Process, Value, Array
import sys
import os

# Jetson Orin Nanoのモデル認識問題の回避
# gpio_pin_dataモジュールを完全にファイルパスベースでロード
try:
    import importlib.util

    # Jetson.GPIOのインストールパスを探す
    jetson_gpio_paths = [
        '/home/jetson/venv/lib/python3.10/site-packages/Jetson/GPIO',
        '/usr/local/lib/python3.10/dist-packages/Jetson/GPIO',
        '/usr/lib/python3/dist-packages/Jetson/GPIO',
    ]

    gpio_pin_data_path = None
    for base_path in jetson_gpio_paths:
        candidate = os.path.join(base_path, 'gpio_pin_data.py')
        if os.path.exists(candidate):
            gpio_pin_data_path = candidate
            break

    if gpio_pin_data_path:
        # ファイルから直接モジュールをロード
        spec = importlib.util.spec_from_file_location('Jetson.GPIO.gpio_pin_data', gpio_pin_data_path)
        gpio_pin_data = importlib.util.module_from_spec(spec)

        # sys.modulesに先に登録
        sys.modules['Jetson.GPIO.gpio_pin_data'] = gpio_pin_data

        # モジュールを実行
        spec.loader.exec_module(gpio_pin_data)

        # 実行後にパッチを適用
        original_get_model = gpio_pin_data.get_model

        def patched_get_model():
            """Jetson Orin Nanoのモデル認識をフォールバック"""
            try:
                return original_get_model()
            except Exception:
                # モデル認識に失敗した場合、Jetson Orin Nanoとして扱う
                print("Warning: Could not detect Jetson model, assuming JETSON_ORIN_NANO")
                return "JETSON_ORIN_NANO"

        gpio_pin_data.get_model = patched_get_model
        print("Jetson GPIO model detection patched successfully")
except (ImportError, AttributeError, Exception) as e:
    # Jetson.GPIOがない環境では何もしない
    print(f"Note: Could not patch Jetson.GPIO: {e}")
    pass

from pmw3901 import BG_CS_BACK_BCM, BG_CS_FRONT_BCM, PMW3901 as PMW3901_Base
import config

# Jetson互換のPMW3901ラッパー (no_cs属性の問題を回避)
class PMW3901(PMW3901_Base):
    def __init__(self, spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM):
        import time
        import spidev
        try:
            import RPi.GPIO as GPIO
        except:
            import Jetson.GPIO as GPIO

        self.spi_cs_gpio = spi_cs_gpio
        self.spi_dev = spidev.SpiDev()
        self.spi_dev.open(spi_port, spi_cs)
        self.spi_dev.max_speed_hz = 400000

        # Jetsonでno_cs属性がサポートされていない場合はスキップ
        try:
            self.spi_dev.no_cs = True
        except (OSError, AttributeError):
            print("Warning: SPI no_cs not supported, using GPIO CS control only")

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.spi_cs_gpio, GPIO.OUT)

        GPIO.output(self.spi_cs_gpio, 0)
        time.sleep(0.05)
        GPIO.output(self.spi_cs_gpio, 1)

        self._write(0x3a, 0x5a)  # REG_POWER_UP_RESET
        time.sleep(0.02)
        for offset in range(5):
            self._read(0x02 + offset)  # REG_DATA_READY

        self._secret_sauce()

        product_id, revision = self.get_id()
        if product_id != 0x49 or revision != 0x00:
            raise RuntimeError("Invalid Product ID or Revision for PMW3901: 0x{:02x}/0x{:02x}".format(product_id, revision))

class OpticalFlowSensor:
    def __init__(self, polling_interval=0.1, timeout_s=0.005, sampling_count=6):
        # Sensor state
        self.is_active = Value('b', 1)
        self.pixel_motion = Array('f', 2)  # Pixel change detected by the sensor
        self.cumulative_pixel_motion = Array('f', 2)  # Cumulative pixel change for interval
        self.position = Array('f', 2)  # Absolute position in m
        self.velocity = Array('f', 2)  # Velocity in m/s
        self.previous_velocity = Array('f', 2)
        self.acceleration = Array('f', 2)

        # Configuration parameters
        self.polling_interval = polling_interval
        self.timeout_s = timeout_s
        self.sampling_count = Value('i', sampling_count)
        self.position_scaling_factor = config.POSITION_SCALING_FACTOR

        # Process management
        self.process = Process(target=self._opticalflow_process)
        self.process.start()

    def _opticalflow_process(self):
        # 子プロセスでもJetson GPIOパッチを適用
        try:
            import importlib.util
            import sys
            import os

            jetson_gpio_paths = [
                '/home/jetson/venv/lib/python3.10/site-packages/Jetson/GPIO',
                '/usr/local/lib/python3.10/dist-packages/Jetson/GPIO',
                '/usr/lib/python3/dist-packages/Jetson/GPIO',
            ]

            gpio_pin_data_path = None
            for base_path in jetson_gpio_paths:
                candidate = os.path.join(base_path, 'gpio_pin_data.py')
                if os.path.exists(candidate):
                    gpio_pin_data_path = candidate
                    break

            if gpio_pin_data_path:
                spec = importlib.util.spec_from_file_location('Jetson.GPIO.gpio_pin_data', gpio_pin_data_path)
                gpio_pin_data = importlib.util.module_from_spec(spec)
                sys.modules['Jetson.GPIO.gpio_pin_data'] = gpio_pin_data
                spec.loader.exec_module(gpio_pin_data)

                original_get_model = gpio_pin_data.get_model

                def patched_get_model():
                    try:
                        return original_get_model()
                    except Exception:
                        return "JETSON_ORIN_NANO"

                gpio_pin_data.get_model = patched_get_model
        except Exception:
            pass

        # 子プロセスでもJetson互換PMW3901を使用
        import time
        import spidev
        from pmw3901 import BG_CS_FRONT_BCM, PMW3901 as PMW3901_Base
        try:
            import RPi.GPIO as GPIO
        except:
            import Jetson.GPIO as GPIO

        class PMW3901_Jetson(PMW3901_Base):
            def __init__(self, spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM):
                self.spi_cs_gpio = spi_cs_gpio
                self.spi_dev = spidev.SpiDev()
                self.spi_dev.open(spi_port, spi_cs)
                self.spi_dev.max_speed_hz = 400000

                # Jetsonでno_cs属性がサポートされていない場合はスキップ
                try:
                    self.spi_dev.no_cs = True
                except (OSError, AttributeError):
                    print("Warning: SPI no_cs not supported, using GPIO CS control only")

                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.spi_cs_gpio, GPIO.OUT)

                GPIO.output(self.spi_cs_gpio, 0)
                time.sleep(0.05)
                GPIO.output(self.spi_cs_gpio, 1)

                self._write(0x3a, 0x5a)  # REG_POWER_UP_RESET
                time.sleep(0.02)
                for offset in range(5):
                    self._read(0x02 + offset)  # REG_DATA_READY

                self._secret_sauce()

                product_id, revision = self.get_id()
                if product_id != 0x49 or revision != 0x00:
                    raise RuntimeError("Invalid Product ID or Revision for PMW3901: 0x{:02x}/0x{:02x}".format(product_id, revision))

        try:
            # spi_cs: SPIチップセレクト番号 (0 or 1, /dev/spidev0.X)
            # spi_cs_gpio: GPIOピン番号 (ソフトウェアCS制御用)
            sensor = PMW3901_Jetson(spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM)
            sensor.set_rotation(0)
            print("Optical Flow Sensor initialized.")
        except Exception as e:
            import traceback
            print(f"Failed to initialize sensor: {e}")
            print("Full traceback:")
            traceback.print_exc()
            self.is_active.value = 0
            return

        while self.is_active.value:
            try:
                self.cumulative_pixel_motion[:] = [0, 0]
                start_time = time.perf_counter()

                for _ in range(self.sampling_count.value):
                    try:
                        motion = sensor.get_motion(self.timeout_s)
                        self.cumulative_pixel_motion[0] += motion[0]
                        self.cumulative_pixel_motion[1] += motion[1]
                    except RuntimeError:
                        continue

                # Calculate elapsed time
                end_time = time.perf_counter()
                elapsed_time = end_time - start_time

                # Convert cumulative pixel motion to m
                delta_x = self.cumulative_pixel_motion[0] * self.position_scaling_factor
                delta_y = self.cumulative_pixel_motion[1] * self.position_scaling_factor

                # Update position (m)
                self.position[0] += delta_x
                self.position[1] += delta_y

                # Update velocity (m/s)
                self.velocity[0] = delta_x / elapsed_time
                self.velocity[1] = delta_y / elapsed_time

                # Update acceleration
                self.acceleration[0] = (self.velocity[0] - self.previous_velocity[0]) / elapsed_time
                self.acceleration[1] = (self.velocity[1] - self.previous_velocity[1]) / elapsed_time

                # Store current velocity as previous velocity for next loop
                self.previous_velocity[:] = self.velocity[:]

            except Exception as e:
                print(f"Error in process loop: {e}")

    def update(self):
        while self.is_active.value:
            self.poll()
            time.sleep(self.polling_interval)

    def poll(self):
        if self.is_active.value:
            return self.velocity[0], self.velocity[1]
        pass

    def run(self):
        self.poll()
        return self.velocity[0], self.velocity[1]

    def shutdown(self):
        self.is_active.value = 0
        self.process.join()
        print("Optical Flow Sensor process terminated.")

    def calibration_check(self, move_distance_mm=50, move_duration_s=5):
        """Check sensor calibration by moving a set distance and duration."""
        move_distance_m = move_distance_mm / 1000.0
        print("現在のPOSITION_SCALING_FACTOR: ",config.POSITION_SCALING_FACTOR)
        message = f"Enter：キャリブレーション開始。{move_duration_s}秒以内に{move_distance_mm}mm マシンを前進。"
        input(message)
        print("Starting calibration check...")
        initial_position = list(self.position[:])
        start_time = time.time()

        while time.time() - start_time < move_duration_s:
            pos = self.position[:]
            print(f"Current Position: x={pos[0]*1000:.1f}mm y={pos[1]*1000:.1f}mm")
            time.sleep(self.polling_interval)

        final_position = list(self.position[:])
        moved_distance_m = final_position[1] - initial_position[1]
        adjusted_scaling_factor = moved_distance_m / move_distance_m * self.position_scaling_factor
        print(f"移動量(y): {moved_distance_m*1000:.1f} mm (Expected: {move_distance_mm}mm)")
        print(f"必要に応じ config.py の POSITION_SCALING_FACTOR を修正: {abs(adjusted_scaling_factor)}")

# ============================================================================
# MTF01 MSPv2 protocol parser
# ============================================================================
_MSP2_SENSOR_RANGEFINDER = 0x1F01
_MSP2_SENSOR_OPTIC_FLOW = 0x1F02


def _crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class _MSPv2Parser:
    _IDLE = 0
    _HEADER2 = 1
    _DIRECTION = 2
    _FLAG = 3
    _FUNC_LO = 4
    _FUNC_HI = 5
    _SIZE_LO = 6
    _SIZE_HI = 7
    _PAYLOAD = 8
    _CRC = 9

    def __init__(self):
        self._reset()

    def _reset(self):
        self.state = self._IDLE
        self.flag = 0
        self.function = 0
        self.payload_size = 0
        self.payload = bytearray()
        self.payload_cnt = 0

    def parse_byte(self, b):
        if self.state == self._IDLE:
            if b == 0x24:
                self.state = self._HEADER2
        elif self.state == self._HEADER2:
            if b == 0x58:
                self.state = self._DIRECTION
            else:
                self._reset()
        elif self.state == self._DIRECTION:
            self.state = self._FLAG
        elif self.state == self._FLAG:
            self.flag = b
            self.state = self._FUNC_LO
        elif self.state == self._FUNC_LO:
            self.function = b
            self.state = self._FUNC_HI
        elif self.state == self._FUNC_HI:
            self.function |= (b << 8)
            self.state = self._SIZE_LO
        elif self.state == self._SIZE_LO:
            self.payload_size = b
            self.state = self._SIZE_HI
        elif self.state == self._SIZE_HI:
            self.payload_size |= (b << 8)
            self.payload = bytearray()
            self.payload_cnt = 0
            if self.payload_size == 0:
                self.state = self._CRC
            elif self.payload_size > 1024:
                self._reset()
            else:
                self.state = self._PAYLOAD
        elif self.state == self._PAYLOAD:
            self.payload.append(b)
            self.payload_cnt += 1
            if self.payload_cnt == self.payload_size:
                self.state = self._CRC
        elif self.state == self._CRC:
            crc_data = bytearray()
            crc_data.append(self.flag)
            crc_data.append(self.function & 0xFF)
            crc_data.append((self.function >> 8) & 0xFF)
            crc_data.append(self.payload_size & 0xFF)
            crc_data.append((self.payload_size >> 8) & 0xFF)
            crc_data.extend(self.payload)
            expected_crc = _crc8_dvb_s2(bytes(crc_data))
            func = self.function
            payload = bytes(self.payload)
            self._reset()
            if expected_crc == b:
                return self._decode(func, payload)
        return None

    def _decode(self, function, payload):
        import struct
        if function == _MSP2_SENSOR_RANGEFINDER and len(payload) >= 5:
            quality = payload[0]
            distance_mm = struct.unpack_from("<i", payload, 1)[0]
            return ("range", quality, distance_mm)
        elif function == _MSP2_SENSOR_OPTIC_FLOW and len(payload) >= 9:
            quality = payload[0]
            motion_x = struct.unpack_from("<i", payload, 1)[0]
            motion_y = struct.unpack_from("<i", payload, 5)[0]
            return ("flow", quality, motion_x, motion_y)
        return None


class MTF01OpticalFlowSensor:
    """MTF-01 optical flow + rangefinder sensor (serial MSPv2 protocol)"""

    def __init__(self, port="/dev/ttyTHS1", baud=115200, flow_scale=4200.0,
                 polling_interval=0.1):
        # Shared state (same interface as OpticalFlowSensor)
        self.is_active = Value('b', 1)
        self.pixel_motion = Array('f', 2)
        self.cumulative_pixel_motion = Array('f', 2)
        self.position = Array('f', 2)       # m
        self.velocity = Array('f', 2)       # m/s
        self.previous_velocity = Array('f', 2)
        self.acceleration = Array('f', 2)
        self.height_mm = Value('f', 0.0)    # rangefinder height (mm)
        self._ready = Value('b', 0)         # subprocess ready flag

        self.polling_interval = polling_interval
        self.position_scaling_factor = config.POSITION_SCALING_FACTOR

        self._port = port
        self._baud = baud
        self._flow_scale = flow_scale

        self.process = Process(target=self._mtf01_process)
        self.process.start()
        # Wait for subprocess to finish initialization
        while self.is_active.value and not self._ready.value:
            time.sleep(0.05)

    def _mtf01_process(self):
        import serial
        import math

        try:
            ser = serial.Serial(self._port, self._baud, timeout=1)
            print(f"MTF-01 initialized on {self._port} @ {self._baud}")
        except Exception as e:
            print(f"Failed to initialize MTF-01: {e}")
            self.is_active.value = 0
            return

        self._ready.value = 1
        parser = _MSPv2Parser()
        height_m = 0.0
        last_flow_time = None

        try:
            while self.is_active.value:
                data = ser.read(ser.in_waiting or 1)
                for byte in data:
                    result = parser.parse_byte(byte)
                    if result is None:
                        continue

                    if result[0] == "range":
                        _, quality, distance_mm = result
                        if quality > 0 and distance_mm > 0:
                            height_m = distance_mm / 1000.0
                            self.height_mm.value = float(distance_mm)

                    elif result[0] == "flow":
                        _, quality, motion_x, motion_y = result
                        now = time.monotonic()

                        self.pixel_motion[0] = float(motion_x)
                        self.pixel_motion[1] = float(motion_y)

                        if last_flow_time is not None and quality > 0 and height_m > 0:
                            dt = now - last_flow_time
                            if 0 < dt < 1.0:
                                # flow counts -> radians -> meters
                                dx_m = (motion_x / self._flow_scale) * height_m
                                dy_m = (motion_y / self._flow_scale) * height_m

                                self.cumulative_pixel_motion[0] += motion_x
                                self.cumulative_pixel_motion[1] += motion_y

                                # Update position (m)
                                self.position[0] += dx_m
                                self.position[1] += dy_m

                                # Update velocity (m/s)
                                vx = dx_m / dt
                                vy = dy_m / dt
                                self.velocity[0] = vx
                                self.velocity[1] = vy

                                # Update acceleration
                                self.acceleration[0] = (vx - self.previous_velocity[0]) / dt
                                self.acceleration[1] = (vy - self.previous_velocity[1]) / dt
                                self.previous_velocity[0] = vx
                                self.previous_velocity[1] = vy

                        last_flow_time = now
        except KeyboardInterrupt:
            pass

        ser.close()

    def poll(self):
        if self.is_active.value:
            return self.velocity[0], self.velocity[1]
        pass

    def run(self):
        self.poll()
        return self.velocity[0], self.velocity[1]

    def shutdown(self):
        self.is_active.value = 0
        self.process.join()
        print("MTF-01 Sensor process terminated.")

    def calibration_check(self, move_distance_mm=50, move_duration_s=5):
        move_distance_m = move_distance_mm / 1000.0
        print(f"--- MTF-01 キャリブレーション ---")
        print(f"  現在の変換係数 (flow_scale): {self._flow_scale} counts/rad")
        print(f"  地面までの高さ: {self.height_mm.value:.0f} mm")
        print(f"  ※ 移動距離 = (センサーカウント / flow_scale) × 高さ")
        print(f"    flow_scale↑ → 計測距離↓ / flow_scale↓ → 計測距離↑")
        message = f"\nEnter：キャリブレーション開始。{move_duration_s}秒以内に{move_distance_mm}mm マシンを前進。"
        input(message)
        print("計測中...")
        initial_position = list(self.position[:])
        start_time = time.time()

        while time.time() - start_time < move_duration_s:
            pos = self.position[:]
            print(f"  位置: x={pos[0]*1000:.1f}mm y={pos[1]*1000:.1f}mm  高さ: {self.height_mm.value:.0f} mm")
            time.sleep(self.polling_interval)

        final_position = list(self.position[:])
        moved_distance_m = final_position[1] - initial_position[1]
        print(f"\n--- 結果 ---")
        print(f"  センサー計測値(y): {moved_distance_m*1000:.1f} mm")
        print(f"  実際の移動距離:    {move_distance_mm} mm")
        if moved_distance_m != 0:
            ratio = moved_distance_m / move_distance_m
            adjusted_scale = self._flow_scale / ratio
            print(f"  誤差: {(ratio - 1) * 100:+.1f}%")
            print(f"  推奨 flow_scale: {abs(adjusted_scale):.0f} (現在: {self._flow_scale:.0f})")
        else:
            print("  移動が検出されませんでした。センサーと地面の距離を確認してください。")


def _detect_mtf01(port="/dev/ttyTHS1", baud=115200, timeout=2.0):
    """MTF01がシリアルポートに接続されているか確認"""
    try:
        import serial
        ser = serial.Serial(port, baud, timeout=1)
        parser = _MSPv2Parser()
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            data = ser.read(ser.in_waiting or 1)
            for byte in data:
                result = parser.parse_byte(byte)
                if result is not None:
                    ser.close()
                    return True
        ser.close()
    except Exception:
        pass
    return False


def detect_opticalflow():
    """接続されているオプティカルフローセンサーを自動検出して返す"""
    print("Scanning for optical flow sensors...")

    # MTF01 (serial) を確認
    mtf01_port = "/dev/ttyTHS1"
    if os.path.exists(mtf01_port):
        print(f"  Checking MTF-01 on {mtf01_port}...")
        if _detect_mtf01(mtf01_port):
            print(f"  Found MTF-01 on {mtf01_port}")
            return MTF01OpticalFlowSensor(port=mtf01_port)
        else:
            print(f"  {mtf01_port}: no MTF-01 response")
    else:
        print(f"  {mtf01_port}: port not found")

    # PMW3901 (SPI) を試行
    print("  Trying PMW3901 (SPI)...")
    try:
        sensor = OpticalFlowSensor()
        time.sleep(1)
        if sensor.is_active.value:
            print("  Found PMW3901")
            return sensor
        else:
            sensor.shutdown()
            print("  PMW3901: initialization failed")
    except Exception as e:
        print(f"  PMW3901: {e}")

    print("Error: No optical flow sensor detected.")
    exit(1)


# ROS2の有無を判定してインポート
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped, PointStamped, AccelStamped

    class OpticalFlowNode(Node):
        def __init__(self, sensor: OpticalFlowSensor):
            super().__init__('optical_flow_node')

            # OpticalFlowSensor インスタンスを受け取る
            self.sensor = sensor
            self.frame_id = 'optical_flow'

            # パブリッシャーの設定（Stamped型: タイムスタンプ + frame_id付き）
            self.velocity_pub = self.create_publisher(TwistStamped, '/optical_flow/velocity', 10)
            self.position_pub = self.create_publisher(PointStamped, '/optical_flow/position', 10)
            self.acceleration_pub = self.create_publisher(AccelStamped, '/optical_flow/acceleration', 10)

            # タイマー設定（データ送信間隔）
            self.timer_period = self.sensor.polling_interval
            self.timer = self.create_timer(self.timer_period, self.publish_data)

            self.get_logger().info("OpticalFlowNode initialized.")

        def publish_data(self):
            try:
                now = self.get_clock().now().to_msg()

                # 速度データのパブリッシュ (m/s)
                velocity_msg = TwistStamped()
                velocity_msg.header.stamp = now
                velocity_msg.header.frame_id = self.frame_id
                velocity_msg.twist.linear.x = float(self.sensor.velocity[0])
                velocity_msg.twist.linear.y = float(self.sensor.velocity[1])
                self.velocity_pub.publish(velocity_msg)

                # 位置データのパブリッシュ (m)
                position_msg = PointStamped()
                position_msg.header.stamp = now
                position_msg.header.frame_id = self.frame_id
                position_msg.point.x = float(self.sensor.position[0])
                position_msg.point.y = float(self.sensor.position[1])
                self.position_pub.publish(position_msg)

                # 加速度データのパブリッシュ (m/s²)
                acceleration_msg = AccelStamped()
                acceleration_msg.header.stamp = now
                acceleration_msg.header.frame_id = self.frame_id
                acceleration_msg.accel.linear.x = float(self.sensor.acceleration[0])
                acceleration_msg.accel.linear.y = float(self.sensor.acceleration[1])
                self.acceleration_pub.publish(acceleration_msg)

                self.get_logger().debug(
                    f"Published - Vel: ({velocity_msg.twist.linear.x:.4f}, {velocity_msg.twist.linear.y:.4f}) m/s, "
                    f"Pos: ({position_msg.point.x:.4f}, {position_msg.point.y:.4f}) m"
                )
            except Exception as e:
                self.get_logger().error(f"Error in publish_data: {e}")

        def shutdown(self):
            self.sensor.shutdown()
            self.get_logger().info("OpticalFlowNode shutting down.")

    def main_ros():
        sensor = detect_opticalflow()
        try:
            # ROS2初期化
            rclpy.init()
            # ROSノード起動
            node = OpticalFlowNode(sensor)
            rclpy.spin(node)

        except KeyboardInterrupt:
            print("Shutting down OpticalFlowNode...")
        finally:
            # シャットダウン処理
            if rclpy.ok():
                node.shutdown()
                rclpy.shutdown()

except ImportError:
    # print("ROS2関連ライブラリがインストールされていません。ROS2モードは無効です。")
    rclpy = None

def main_manual():
    sensor = detect_opticalflow()
    try:
        answer = ""
        while (answer == ""):
            sensor.calibration_check()
            answer = input("Enter：再キャリブレーション / 任意のキー：速度測定\n")
        while True:
            print(f"Velocity: {sensor.run()} m/s")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down...")
        sensor.shutdown()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Oplicalflow with or without ROS2")
    parser.add_argument('--ros', action='store_true', help="Run with ROS2 node")
    args = parser.parse_args()

    if args.ros and rclpy:
        print("Open another terminal and check the velocity values by typing:\n ros2 topic echo /optical_flow/velocity geometry_msgs/msg/TwistStamped")
        main_ros()
    else:
        if args.ros and not rclpy:
            print("Warning: ROS2 is not available. Switching to manual mode.")
        main_manual()        
