import multiprocessing
import ctypes
import os
import numpy as np
from time import perf_counter, sleep
from typing import cast
import config
import platform
import signal
import logging
import time
import gc
import subprocess

# Conditional cv2 import based on device type to avoid bus errors on RPi5
if config.DEVICE_TYPE == 'RPI5':
    # Skip cv2 import on RPi5 to avoid bus error
    CV2_AVAILABLE = False
    cv2 = None
    # print("OpenCV import skipped on RPi5 to avoid bus error")
else:
    try:
        import cv2
        CV2_AVAILABLE = True
    except ImportError as e:
        CV2_AVAILABLE = False
        cv2 = None
        print(f"Warning: OpenCV not available: {e}")

# ROS2の有無を判定してインポート
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from rclpy.executors import MultiThreadedExecutor

    class CameraNode(Node):
        def __init__(self, node_name: str, topic_name: str, frame_id: str, queue_size: int):
            super().__init__(node_name)
            self.publisher = self.create_publisher(Image, topic_name, queue_size)
            self.frame_id = frame_id

        def publish_frame(self, frame: np.ndarray):
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height, msg.width, channels = frame.shape
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = msg.width * channels
            msg.data = frame.tobytes()
            self.publisher.publish(msg)

    class RVizSubscriber(Node):
        def __init__(self, topic_name: str):
            super().__init__('rviz_subscriber')
            self.subscription = self.create_subscription(
                Image,
                topic_name,
                self.listener_callback,
                qos_profile_sensor_data=None
            )

        def listener_callback(self, msg):
            self.get_logger().info(f"Receiving frame on {msg.header.frame_id}...")

except ImportError:
    rclpy = None

# 既存のカメララッパー
class BaseCameraWrapper:
    def read(self):
        raise NotImplementedError("Subclasses must implement 'read'.")

    def release(self):
        raise NotImplementedError("Subclasses must implement 'release'.")

    def get_data(self):
        return self.read()[1] # [1]only return image data
    
    def cleanup(self):
        self.release()
        time.sleep(0.05)  # GStreamerリソース解放の最小待機（短縮）
        gc.collect()
        logging.getLogger(__name__).info(f"Camera cleanup complete.")

class LidarCameraWrapper(BaseCameraWrapper):
    """LiDARの生成画像をカメラインターフェースとして公開するラッパー"""
    def __init__(self, lidar_instance):
        self.lidar_instance = lidar_instance

    def read(self):
        image = getattr(self.lidar_instance, 'latest_image', None)
        if image is not None:
            return True, image
        return False, None

    def release(self):
        pass  # LiDARのライフサイクルは別管理

class PiCameraWrapper(BaseCameraWrapper):
    def __init__(self, device_id):
        # picamera2のログレベルを抑制
        import logging
        logging.getLogger('picamera2').setLevel(logging.ERROR)

        from picamera2 import Picamera2
        from libcamera import Transform
        import threading

        # Tuningファイルの読み込み（config.pyで指定、未指定ならデフォルト）
        tuning_file = getattr(config, 'CAMERA_TUNING_FILE', None)
        if tuning_file and os.path.exists(tuning_file):
            tuning = Picamera2.load_tuning_file(tuning_file)
            self.picam2 = Picamera2(camera_num=device_id, tuning=tuning)
            logging.getLogger(__name__).info(f"Tuning file loaded: {tuning_file}")
        else:
            self.picam2 = Picamera2(camera_num=device_id)
            
        # Determine flip settings based on camera ID
        if device_id == 0:
            vflip = config.CAMERA_0_VFLIP
            hflip = config.CAMERA_0_HFLIP
        else:
            vflip = config.CAMERA_1_VFLIP
            hflip = config.CAMERA_1_HFLIP

        # Transform設定（libcameraのTransformを使用）
        transform = Transform()
        if vflip and hflip:
            # 180度回転
            transform = Transform(vflip=True, hflip=True)
        elif vflip:
            # 垂直反転のみ
            transform = Transform(vflip=True)
        elif hflip:
            # 水平反転のみ
            transform = Transform(hflip=True)

        # カメラ設定
        picamera_config = self.picam2.create_preview_configuration(
            main={"format": "BGR888", "size": (config.IMAGE_W, config.IMAGE_H)},
            transform=transform  # transformパラメータで反転を設定
        )
        self.picam2.configure(picamera_config)

        # フレームレートのみ設定（VerticalFlip/HorizontalFlipは削除）
        controls = {"FrameRate": config.CAMERA_FRAMERATE}

        # 利用可能なコントロールを確認してから設定
        available_controls = self.picam2.camera_controls
        if "FrameRate" in available_controls:
            try:
                self.picam2.set_controls(controls)
            except RuntimeError as e:
                print(f"Warning: Could not set FrameRate control: {e}")
                # フレームレートが設定できない場合は続行

        self.picam2.start()

        # センサー名を取得
        try:
            sensor_name = "Unknown"
            # カメラのプロパティからセンサー名を取得
            camera_properties = self.picam2.camera_properties
            if 'Model' in camera_properties:
                sensor_name = camera_properties['Model']
            logging.getLogger(__name__).info(f"Camera {device_id}: OK ({sensor_name})")
        except Exception:
            logging.getLogger(__name__).info(f"Camera {device_id}: OK")

        sleep(0.1)

        # 非同期キャプチャスレッド：バックグラウンドで常に最新フレームを取得
        self._latest_frame = self.picam2.capture_array()
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while self._running:
            self._latest_frame = self.picam2.capture_array()

    def read(self):
        return True, self._latest_frame

    def release(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.picam2.stop()

class UnitreeSV125Wrapper(BaseCameraWrapper):
    """Unitree SV1-25 カメラ用ラッパー（C++ネイティブSDK使用で60fps対応）"""
    def __init__(self, device_id=None, use_native=True):
        if not CV2_AVAILABLE:
            raise RuntimeError("OpenCV is required for UnitreeSV125Wrapper but not available")

        # device_id=None の場合は自動検出
        if device_id is None:
            resolved = resolve_device_path("sv125")
            if resolved is None:
                raise RuntimeError("SV1-25 camera not detected. Run 'python3 camera.py' to list devices.")
            self.device_path = resolved
            self.device_id = int(resolved.replace("/dev/video", ""))
            logging.getLogger(__name__).info(f"SV1-25 auto-detected at {resolved}")
        else:
            self.device_id = device_id
            self.device_path = f"/dev/video{device_id}"

        self.use_native = use_native
        self.process = None
        self.frame_buffer = None

        if use_native:
            # C++ネイティブSDK経由（60fps）
            self._init_native()
        else:
            # OpenCV V4L2経由（約30fps）
            self._init_opencv()

    def _init_native(self):
        """C++ネイティブSDKで初期化（60fps対応）"""
        # stream_framesプログラムのパスを取得
        script_dir = os.path.dirname(os.path.abspath(__file__))
        stream_frames_path = os.path.join(script_dir, 'stream_frames')

        if not os.path.exists(stream_frames_path):
            logging.getLogger(__name__).warning(
                f"stream_frames not found at {stream_frames_path}, falling back to OpenCV"
            )
            self.use_native = False
            self._init_opencv()
            return

        # C++プログラムをサブプロセスとして起動
        # stream_frames_xu は XU初期化でカメラを自動検出し、FPSのみ引数に取る
        self.process = subprocess.Popen(
            [stream_frames_path, '30'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0  # バッファなし
        )

        # 起動確認（SDKの初期化完了まで待機してから最初のフレームを読む）
        import select
        initialized = False
        # stdoutにデータが届くまで最大5秒待機
        ready = select.select([self.process.stdout], [], [], 5.0)[0]
        if ready:
            ret, frame = self._read_frame_native()
            if ret:
                initialized = True
                logging.getLogger(__name__).info(
                    f"Unitree SV1-25 Camera {self.device_path}: OK (Native SDK, 60fps)"
                )

        if not initialized:
            logging.getLogger(__name__).warning(
                f"Native SDK failed for SV1-25, falling back to OpenCV"
            )
            self._cleanup_native()
            self.use_native = False
            self._init_opencv()

    def _init_opencv(self):
        """OpenCV V4L2で初期化（約30fps）"""
        self._set_exposure(self.device_path)

        # V4L2キャプチャでカメラを開く
        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open Unitree SV1-25 camera at {self.device_path}")

        # MJPEG形式、928x400に設定（60fps対応）
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 928)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
        self.cap.set(cv2.CAP_PROP_FPS, 60)

        # 設定確認
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))

        logging.getLogger(__name__).info(
            f"Unitree SV1-25 Camera {self.device_id}: OK "
            f"({actual_width}x{actual_height} @ {actual_fps}fps, OpenCV)"
        )

    def _set_exposure(self, dev_path):
        """V4L2経由で露出設定を適用"""
        try:
            subprocess.run(['v4l2-ctl', '-d', dev_path,
                          '--set-ctrl', 'exposure_auto=1'],
                         stderr=subprocess.DEVNULL, timeout=1)
            subprocess.run(['v4l2-ctl', '-d', dev_path,
                          '--set-ctrl', 'exposure_absolute=8000'],
                         stderr=subprocess.DEVNULL, timeout=1)
            subprocess.run(['v4l2-ctl', '-d', dev_path,
                          '--set-ctrl', 'gain=150'],
                         stderr=subprocess.DEVNULL, timeout=1)
            subprocess.run(['v4l2-ctl', '-d', dev_path,
                          '--set-ctrl', 'brightness=20'],
                         stderr=subprocess.DEVNULL, timeout=1)
            logging.getLogger(__name__).info(f"SV1-25 Camera {dev_path}: Exposure settings applied")
        except Exception as e:
            logging.getLogger(__name__).warning(f"Failed to set exposure for camera {dev_path}: {e}")

    def _sync_to_frame_header(self):
        """パイプから 'FRAM' マジックバイトを探して同期する（チャンク読み取り）"""
        magic = b'FRAM'
        buf = b''
        max_scan = 2 * 1024 * 1024  # 最大2MBスキャン
        while len(buf) < max_scan:
            chunk = self.process.stdout.read(4096)
            if len(chunk) == 0:
                return None
            buf += chunk
            pos = buf.find(magic)
            if pos >= 0:
                remaining = buf[pos:]
                while len(remaining) < 8:
                    extra = self.process.stdout.read(8 - len(remaining))
                    if len(extra) == 0:
                        return None
                    remaining += extra
                return remaining[:8]
        return None

    def _read_frame_native(self):
        """C++プロセスからフレームを読み取り"""
        if self.process is None or self.process.poll() is not None:
            return False, None

        try:
            # ヘッダー読み取り: "FRAM" + 4バイトサイズ
            header = self.process.stdout.read(8)
            if len(header) != 8:
                return False, None

            if header[0:4] != b'FRAM':
                # 同期ずれ — FRAMマジックを探してリカバリ
                header = self._sync_to_frame_header()
                if header is None:
                    return False, None

            # フレームサイズを取得（リトルエンディアン）
            frame_size = (header[4] |
                         (header[5] << 8) |
                         (header[6] << 16) |
                         (header[7] << 24))

            # サイズ妥当性チェック（MJPEGフレームは通常10KB〜500KB）
            if frame_size <= 0 or frame_size > 2 * 1024 * 1024:
                return False, None

            # フレームデータを読み取り
            frame_data = self.process.stdout.read(frame_size)
            if len(frame_data) != frame_size:
                return False, None

            # MJPEGデータをデコード（BGR）→ RGB変換
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is None:
                return False, None

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return True, frame

        except Exception as e:
            logging.getLogger(__name__).error(f"Error reading native frame: {e}")
            return False, None

    def read(self):
        """フレームを読み取り（RGB形式、IMAGE_W x IMAGE_H にリサイズ）"""
        if self.use_native:
            ret, frame = self._read_frame_native()
        else:
            if not hasattr(self, 'cap') or self.cap is None:
                return False, None
            ret, frame = self.cap.read()
            if ret and frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if ret and frame is not None:
            h, w = frame.shape[:2]
            if h != config.IMAGE_H or w != config.IMAGE_W:
                frame = cv2.resize(frame, (config.IMAGE_W, config.IMAGE_H),
                                   interpolation=cv2.INTER_AREA)
        return ret, frame

    def _cleanup_native(self):
        """C++プロセスを終了"""
        if self.process is not None:
            try:
                self.process.terminate()
                self.process.wait(timeout=2)
            except:
                self.process.kill()
            self.process = None

    def release(self):
        """カメラを解放"""
        if self.use_native:
            self._cleanup_native()
        else:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                self.cap = None
        time.sleep(0.05)
        gc.collect()

class JetsonCameraWrapper(BaseCameraWrapper):
    def __init__(self, device_id=0):
        if not CV2_AVAILABLE:
            raise RuntimeError("OpenCV is required for JetsonCameraWrapper but not available")

        self.device_id = device_id
        
        # Determine flip method based on camera ID and config
        if device_id == 0:
            vflip = config.CAMERA_0_VFLIP
            hflip = config.CAMERA_0_HFLIP
        else:
            vflip = config.CAMERA_1_VFLIP
            hflip = config.CAMERA_1_HFLIP
        
        # GStreamer flip-method values:
        # 0: none, 1: counterclockwise, 2: rotate-180, 3: clockwise
        # 4: horizontal-flip, 5: upper-right-diagonal, 6: vertical-flip, 7: upper-left-diagonal
        if vflip and hflip:
            flip_method = 2  # rotate-180 (both flips)
        elif vflip:
            flip_method = 6  # vertical-flip
        elif hflip:
            flip_method = 4  # horizontal-flip
        else:
            flip_method = 0  # none
        
        # nvvideoconvertが利用可能かチェック
        use_nvvideoconvert = self._check_gstreamer_element('nvvideoconvert')
        
        if use_nvvideoconvert:
            # パフォーマンス最適化版: nvvideoconvert + VIC
            # RGBA形式: [:,:,:3]でRGBを直接取得（PiCameraWrapper RGB888と統一）
            self.pipeline = (
                f"nvarguscamerasrc sensor-id={device_id} "
                f"exposuretimerange=\"13000 16666666\" "  # 露光上限1/60s: AEによるFPS低下を防止
                f"bufapi-version=1 ! "  # 低遅延バッファAPI
                f"video/x-raw(memory:NVMM), width={config.IMAGE_W}, height={config.IMAGE_H}, "
                f"format=(string)NV12, framerate={config.CAMERA_FRAMERATE}/1 ! "
                f"nvvideoconvert "
                f"flip-method={flip_method} "
                f"interpolation-method=0 "  # Nearest（最速）
                f"compute-hw=2 "  # VIC使用（省電力・高速）
                f"nvbuf-memory-type=0 ! "  # Device memory
                "video/x-raw, format=(string)RGBA ! "
                "appsink drop=true max-buffers=1 sync=false emit-signals=false"
            )
            print(f"Jetson camera@id:{device_id}, flip-method:{flip_method} [Optimized: nvvideoconvert+VIC, RGB]")

        else:
            # フォールバック: 従来のnvvidconv版
            # RGBA形式: videoconvert不要で高速化、かつRGB出力を統一
            self.pipeline = (
                f"nvarguscamerasrc sensor-id={device_id} "
                f"exposuretimerange=\"13000 16666666\" ! "  # 露光上限1/60s: AEによるFPS低下を防止
                f"video/x-raw(memory:NVMM), width={config.IMAGE_W}, height={config.IMAGE_H}, "
                f"format=(string)NV12, framerate={config.CAMERA_FRAMERATE}/1 ! "
                f"nvvidconv flip-method={flip_method} ! "
                "video/x-raw, format=(string)RGBA ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            print(f"Jetson camera@id:{device_id}, flip-method:{flip_method} [Legacy: nvvidconv, RGB]")
        
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open Jetson CSI camera with ID {device_id}.")
    
    def _check_gstreamer_element(self, element_name):
        """GStreamerエレメントの利用可能性をチェック"""
        try:
            result = subprocess.run(
                ['gst-inspect-1.0', element_name],
                capture_output=True,
                stderr=subprocess.DEVNULL,
                timeout=2
            )
            return result.returncode == 0
        except:
            return False
    
    def read(self):
        """
        RGB形式でフレームを返す（PiCameraWrapperと統一）
        Returns:
            tuple: (ret, frame) - frameはRGB形式（3チャンネル）
        """
        # カメラが解放されている場合はエラーを回避
        if not hasattr(self, 'cap') or self.cap is None:
            return False, None

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return ret, None

        # RGBA→RGB変換（最初の3チャンネルを抽出）
        if frame.shape[2] == 4:
            frame = frame[:, :, :3]

        return ret, frame
    
    def release(self):
        """リソースを解放"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            self.cap = None
        # 最小限の待機とクリーンアップ
        time.sleep(0.1)  # 短縮: 最小待機時間
        gc.collect()
        # pkillを非ブロッキングで実行（タイムアウト短縮）
        try:
            subprocess.run(['pkill', '-f', 'nvarguscamerasrc'],
                         stderr=subprocess.DEVNULL, timeout=0.5)
        except:
            pass
            

class USBCameraWrapper(BaseCameraWrapper):
    """汎用USBカメラ用ラッパー（OpenCV V4L2）"""
    def __init__(self, device_id=0):
        if not CV2_AVAILABLE:
            raise RuntimeError("OpenCV is required for USBCameraWrapper but not available")

        self.device_id = device_id
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera with device_id={device_id}")

        # MJPEGフォーマットで高速転送（USB帯域削減）
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.IMAGE_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.IMAGE_H)
        self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FRAMERATE)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"USB camera@id:{device_id}, resolution:{actual_w}x{actual_h}, fps:{actual_fps}")

    def read(self):
        if not hasattr(self, 'cap') or self.cap is None:
            return False, None
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return ret, None
        # BGR→RGB変換（numpy slice: cvtColorより高速）
        frame = frame[:, :, ::-1]
        # モデル入力サイズにリサイズ
        if frame.shape[0] != config.IMAGE_H or frame.shape[1] != config.IMAGE_W:
            frame = cv2.resize(frame, (config.IMAGE_W, config.IMAGE_H), interpolation=cv2.INTER_LINEAR)
        return ret, frame

    def release(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            self.cap = None


class MultiprocessCameraWrapper(BaseCameraWrapper):
    def __init__(self, base_camera_type: type, device_id: int):
        self.base_camera_type = base_camera_type
        self.device_id = device_id
        self.__buffer = None
        self.__ready = None
        self.__cancel = None
        self.__shape = None
        self.__process = None
        self.__released = False
        self._initialize_shared_memory()

    ###
    def _initialize_shared_memory(self):
        # 仮のカメラインスタンスを使用してフレームサイズを取得
        #temp_camera = self.base_camera_type(device_id=self.device_id)
        #ret, frame = temp_camera.read()
        #if not ret:
        #    raise RuntimeError(f"Failed to capture initial frame for camera {self.device_id}.")
        #height, width, channels = frame.shape
        #self.__shape = (height, width, channels)
        #temp_camera.release()
        height, width, channels= config.IMAGE_H,config.IMAGE_W,config.IMAGE_DEPTH
        print("shape is:",height, width, channels)

        # 共有メモリと同期用イベントの初期化
        self.__buffer = multiprocessing.Array(
            ctypes.c_uint8, height * width * channels)
        #self.__buffer = multiprocessing.sharedctypes.RawArray(
        #    ctypes.c_uint8, height * width * channels)
        self.__ready = multiprocessing.Event()
        self.__cancel = multiprocessing.Event()

        # バックグラウンドプロセスの開始
        self.__process = multiprocessing.Process(
            target=self._capture_loop,
            args=(self.base_camera_type, self.device_id, self.__buffer, self.__ready, self.__cancel),
            daemon=True
        )
        self.__process.start()

    ### printをコメントアウトのこと
    def _capture_loop(self, camera_type: type, device_id: int, buffer: ctypes.Array[ctypes.c_uint8],
                        ready: multiprocessing.Event, cancel: multiprocessing.Event):
            """
            子プロセス内でカメラを初期化し、フレームを共有メモリに書き込む。
            """
            import signal
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            print("id",device_id)
            # 子プロセス内でカメラインスタンスを初期化
            camera = camera_type(device_id=device_id)
            try:
                while not cancel.is_set():
                    start_time = perf_counter()
                    ret, frame = camera.read()
                    if ret:
                        ready.clear()
                        np.copyto(np.ctypeslib.as_array(buffer), frame.ravel())
                        ready.set()
                        # FPS計算
                        fps = round(1 / (perf_counter() - start_time), 2)
                        print(f"id:{device_id} - fps: {fps}")
            finally:
                camera.release()
                logging.getLogger(__name__).debug(f"Camera {device_id}: Released.")

    #def read(self):
    #    """共有メモリからフレームを読み取る。"""
    #    self.__ready.wait()
    #    frame = np.frombuffer(self.__buffer, dtype=np.uint8).reshape(self.__shape)
    #    return True, frame.copy()

    def release(self):
        """カメラとプロセスを終了する。"""
        if self.__released:
            return
        self.__cancel.set()
        self.__process.join()
        self.base_camera.release()
        self.__released = True
        logging.getLogger(__name__).debug(f"Camera {self.device_id}: Process terminated.")

def detect_video_devices():
    """
    v4l2-ctl --list-devices の出力をパースし、検出されたデバイス一覧を返す。

    Returns:
        list of dict: [{"name": str, "paths": [str], "type": "CSI"|"USB"}, ...]
    """
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True, text=True, timeout=5
        )
        output = result.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []

    devices = []
    current_name = None
    current_paths = []

    for line in output.splitlines():
        if not line.startswith('\t') and line.strip():
            # 新しいデバイスヘッダ行 — 前のデバイスを保存
            if current_name is not None and current_paths:
                dev_type = "CSI" if "vi-output" in current_name or "imx" in current_name.lower() else "USB"
                devices.append({"name": current_name.strip().rstrip(':'), "paths": current_paths, "type": dev_type})
            current_name = line.rstrip(':')
            current_paths = []
        elif line.startswith('\t'):
            path = line.strip()
            if path.startswith('/dev/video'):
                current_paths.append(path)

    # 最後のデバイスを保存
    if current_name is not None and current_paths:
        dev_type = "CSI" if "vi-output" in current_name or "imx" in current_name.lower() else "USB"
        devices.append({"name": current_name.strip().rstrip(':'), "paths": current_paths, "type": dev_type})

    return devices


def resolve_device_path(camera_type):
    """
    config設定または自動検出でデバイスパスを解決する。

    Args:
        camera_type: "sv125" or "usb_generic"

    Returns:
        str: デバイスパス (例: "/dev/video4") or None
    """
    # config に固定IDが設定されている場合はそれを使う
    config_key = {
        "sv125": "CAMERA_ID_SV125",
        "unitree": "CAMERA_ID_SV125",
        "usb_generic": "CAMERA_ID_USB_GENERIC",
    }.get(camera_type)

    if config_key:
        fixed_id = getattr(config, config_key, None)
        if fixed_id is not None:
            return f"/dev/video{fixed_id}"

    # 自動検出: キーワードマッチ
    lookup_type = "sv125" if camera_type in ("sv125", "unitree") else camera_type
    keyword = config.CAMERA_DETECT_KEYWORDS.get(lookup_type)
    if keyword is None:
        return None

    devices = detect_video_devices()
    for dev in devices:
        if keyword in dev["name"]:
            # 最初のパスを返す（通常 video capture ノード）
            return dev["paths"][0] if dev["paths"] else None

    return None


def check_cameras():
    """全カメラ一覧を表示（CSI + USB + config設定状態）"""
    devices = detect_video_devices()

    print("=== 検出されたカメラデバイス ===")
    if not devices:
        print("  (デバイスが見つかりません)")
    else:
        for dev in devices:
            # キーワードマッチでタイプラベルを付与
            matched_type = ""
            for cam_type, keyword in config.CAMERA_DETECT_KEYWORDS.items():
                if keyword in dev["name"]:
                    matched_type = f" → {cam_type}"
                    break

            for path in dev["paths"]:
                print(f"  [{dev['type']}]  {path:<16s} {dev['name']}{matched_type}")

    # config 設定表示
    print()
    print("=== config.py 設定 ===")
    for cam_type, config_key in [("sv125", "CAMERA_ID_SV125"), ("usb_generic", "CAMERA_ID_USB_GENERIC")]:
        fixed_id = getattr(config, config_key, None)
        if fixed_id is not None:
            print(f"  {config_key:<25s} = {fixed_id} (固定)")
        else:
            auto_path = resolve_device_path(cam_type)
            if auto_path:
                print(f"  {config_key:<25s} = None (自動検出: {auto_path})")
            else:
                print(f"  {config_key:<25s} = None (未検出)")

    print()
    print("使用例:")
    print("  python3 camera.py --camera-type sv125")
    print("  python3 camera.py --camera-type sv125 --device-id 4")


def create_camera(device_id=None, use_multiprocess=False, camera_type=None, lidar_instance=None):
    """
    カメラインスタンスを作成

    Args:
        device_id: カメラデバイスID (None=自動検出)
        use_multiprocess: マルチプロセスを使用するか
        camera_type: カメラタイプを指定 ('sv125', 'jetson', 'pi', 'lidar', None=自動検出)
        lidar_instance: LiDARインスタンス (camera_type='lidar' の場合に必須)
    """
    # LiDARカメラの場合
    if camera_type == 'lidar':
        if lidar_instance is None:
            raise ValueError("lidar_instance is required when camera_type='lidar'")
        return LidarCameraWrapper(lidar_instance)

    # カメラタイプが明示的に指定されている場合
    if camera_type == 'sv125' or camera_type == 'unitree':
        # device_id=None → UnitreeSV125Wrapper 内で自動検出
        print(f"Using Unitree SV1-25 camera (device_id={device_id if device_id is not None else 'auto'})")
        if use_multiprocess:
            return MultiprocessCameraWrapper(base_camera_type=UnitreeSV125Wrapper, device_id=device_id)
        return UnitreeSV125Wrapper(device_id=device_id)
    elif camera_type == 'usb' or camera_type == 'usb_generic':
        print(f"Using USB camera (device_id={device_id if device_id is not None else 'auto'})")
        if use_multiprocess:
            return MultiprocessCameraWrapper(base_camera_type=USBCameraWrapper, device_id=device_id or 0)
        return USBCameraWrapper(device_id=device_id or 0)
    elif camera_type == 'jetson':
        base_camera_type = JetsonCameraWrapper
        print(f"Using Jetson CSI camera (device_id={device_id})")
    elif camera_type == 'pi' or camera_type == 'raspberry':
        base_camera_type = PiCameraWrapper
        print(f"Using Raspberry Pi camera (device_id={device_id})")
    else:
        # 自動検出
        node_name = platform.uname().node.lower()
        machine = platform.uname().machine.lower()

        # Check for device-tree model (more reliable for ARM devices)
        model = ""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip().lower()
        except FileNotFoundError:
            pass

        # Platform detection logic
        is_raspberry_pi = "raspberrypi" in node_name or "raspberry" in model
        is_jetson = ("jetson" in node_name or "orin" in node_name or "tegra" in model or
                    "jetson" in model or "orin" in model)

        if is_raspberry_pi:
            base_camera_type = PiCameraWrapper
        elif is_jetson:
            base_camera_type = JetsonCameraWrapper
        else:
            raise RuntimeError("Unsupported platform for camera. Only Raspberry Pi and Jetson devices are supported.")

    # CSI/Piカメラはdevice_id必須、Noneなら0をデフォルトにする
    if device_id is None:
        device_id = 0

    if use_multiprocess:
        camera = MultiprocessCameraWrapper(base_camera_type=base_camera_type, device_id=device_id)
    else:
        camera = base_camera_type(device_id=device_id)

    return camera

def create_cameras(use_multiprocess=False):
    # Improved platform detection (same as create_camera)
    node_name = platform.uname().node.lower()
    machine = platform.uname().machine.lower()
    
    # Check for device-tree model (more reliable for ARM devices)
    model = ""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip().lower()
    except FileNotFoundError:
        pass
    
    # Platform detection logic
    is_raspberry_pi = "raspberrypi" in node_name or "raspberry" in model
    is_jetson = ("jetson" in node_name or "orin" in node_name or "tegra" in model or 
                "jetson" in model or "orin" in model)

    cameras = []
    if is_raspberry_pi:
        base_camera_type = PiCameraWrapper
    elif is_jetson:
        base_camera_type = JetsonCameraWrapper
    else:
        raise RuntimeError("Unsupported platform for camera. Only Raspberry Pi and Jetson devices are supported.")

    if use_multiprocess:
        cameras = [MultiprocessCameraWrapper(base_camera_type=base_camera_type, device_id=i) for i in range(2)]
    else:
        cameras = [base_camera_type(device_id=i) for i in range(2)]

    return cameras


if __name__ == "__main__":
    import time
    import argparse

    parser = argparse.ArgumentParser(description="Camera wrapper with multiprocess and ROS2 support")
    parser.add_argument("--multiprocess", action="store_true", help="Use multiprocessing for camera access")
    parser.add_argument("--ros", action="store_true", help="Run with ROS2 node")
    parser.add_argument("--vis", action="store_true", help="Run with RViz visualization")
    parser.add_argument("--nogui", action="store_true", help="Disable GUI display")
    parser.add_argument("--camera-type", type=str, choices=['sv125', 'unitree', 'jetson', 'pi', 'usb'],
                       help="Camera type: sv125/unitree (Unitree SV1-25), jetson (CSI), pi (Raspberry Pi), usb (generic USB)")
    parser.add_argument("--device-id", type=int, default=None, help="Camera device ID (default: auto-detect)")
    parser.add_argument("--camera-index", type=int, default=None, help="Camera topic index (e.g. 0 → /camera0/image_raw)")
    parser.add_argument("--params-file", type=str, default=None, help="Path to vehicle_params.yaml (for launch file)")
    args = parser.parse_args()

    # --ros モード: 1カメラ = 1プロセスとして動作
    # 複数カメラはlaunchファイルで別プロセスとして起動（ROS2の分散処理）
    if args.ros:
        if not rclpy:
            print("ROS2が利用できません")
            import sys
            sys.exit(1)

        cam_type = args.camera_type
        cam_dev = args.device_id
        cam_idx = args.camera_index if args.camera_index is not None else (cam_dev if cam_dev is not None else 0)

        try:
            cam = create_camera(device_id=cam_dev, camera_type=cam_type)
            ret, frame = cam.read()
            if not ret or frame is None:
                print(f"camera_{cam_idx}: test read failed")
                import sys
                sys.exit(1)
            print(f"camera_{cam_idx}: type={cam_type}, device_id={cam_dev}, "
                  f"frame={frame.shape} OK")
        except Exception as e:
            print(f"camera_{cam_idx}: init failed: {e}")
            import sys
            sys.exit(1)

        rclpy.init()
        node = CameraNode(
            node_name=f"camera_node_{cam_idx}",
            topic_name=f"/camera{cam_idx}/image_raw",
            frame_id=f"camera{cam_idx}_link",
            queue_size=2
        )

        print(f"ROS2 camera node started: /camera{cam_idx}/image_raw")
        try:
            while rclpy.ok():
                ret, frame = cam.read()
                if ret and frame is not None:
                    node.publish_frame(frame)
                rclpy.spin_once(node, timeout_sec=0.001)
        except KeyboardInterrupt:
            print(f"\nStopping camera_{cam_idx}.")
        finally:
            cam.release()
            node.destroy_node()
            rclpy.shutdown()
        import sys
        sys.exit(0)

    # カメラタイプ未指定（非ROS） → カメラ一覧を表示して終了
    if not args.camera_type:
        check_cameras()
        import sys
        sys.exit(0)

    # カメラタイプ指定時 → キャプチャモード
    cameras = [create_camera(device_id=args.device_id,
                             use_multiprocess=args.multiprocess,
                             camera_type=args.camera_type)]

    if args.vis and rclpy:
        rclpy.init()
        rviz_nodes = [
            RVizSubscriber(topic_name=f"/camera{i+1}/image_raw") for i in range(len(cameras))
        ]
        executor = MultiThreadedExecutor()
        for node in rviz_nodes:
            executor.add_node(node)

        try:
            print("Starting RViz visualization...")
            executor.spin()
        except KeyboardInterrupt:
            print("\nStopping RViz visualization.")
        finally:
            for node in rviz_nodes:
                node.destroy_node()
            rclpy.shutdown()

    elif args.ros and rclpy:
        rclpy.init()
        nodes = [
            CameraNode(
                node_name=f"camera_node_{i}",
                topic_name=f"/camera{i}/image_raw",
                frame_id=f"camera{i}_link",
                queue_size=10
            ) for i in range(len(cameras))
        ]
        executor = MultiThreadedExecutor()
        for node in nodes:
            executor.add_node(node)

        try:
            while rclpy.ok():
                for i, camera in enumerate(cameras):
                    ret, frame = camera.read()
                    if ret:
                        nodes[i].publish_frame(frame)
                executor.spin_once(timeout_sec=0.01)
        except KeyboardInterrupt:
            print("\nStopping ROS2 nodes.")
        finally:
            for camera in cameras:
                camera.release()
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()

    else:
        try:
            while True:
                if args.multiprocess:
                    sleep(0.1)  # メインループの待機間隔
                else:
                    start_time = perf_counter()
                    for i, camera in enumerate(cameras):
                        ret, frame = camera.read()
                        if ret and not args.nogui and CV2_AVAILABLE:
                            cv2.imshow(f"Camera {i+1}", frame[:, :, ::-1])  # RGB→BGR for cv2.imshow
                        elif ret:
                            logging.getLogger(__name__).debug(f"Camera {i+1}: Frame captured.")
                    print("FPS:", 1 / (perf_counter() - start_time))
                    if not args.nogui and CV2_AVAILABLE and cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except KeyboardInterrupt:
            print("\nStopping cameras.")
        finally:
            for camera in cameras:
                camera.release()
            if not args.nogui and CV2_AVAILABLE:
                cv2.destroyAllWindows()