import os
import datetime
import json
import cv2
import numpy as np
import threading
import queue
import config

class _AsyncImageWriter:
    """画像保存を非同期で行うバックグラウンドスレッド"""
    def __init__(self, max_queue_size=100):
        self._queue = queue.Queue(maxsize=max_queue_size)
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        while True:
            task = self._queue.get()
            if task is None:
                break
            try:
                func, args = task
                func(*args)
            except Exception as e:
                print(f"AsyncImageWriter error: {e}")
            finally:
                self._queue.task_done()

    def submit_imwrite(self, path, image_bgr):
        """cv2.imwrite を非同期キューに投入"""
        try:
            self._queue.put_nowait((cv2.imwrite, (path, image_bgr)))
        except queue.Full:
            # キューが満杯の場合は同期的に書き込み（データ欠落防止）
            cv2.imwrite(path, image_bgr)

    def submit_npy_save(self, path, array):
        """np.save を非同期キューに投入"""
        try:
            self._queue.put_nowait((np.save, (path, array)))
        except queue.Full:
            np.save(path, array)

    def flush(self):
        """キュー内の全タスク完了を待機"""
        self._queue.join()

    def shutdown(self):
        """ワーカースレッドを終了"""
        self._queue.put(None)
        self._thread.join(timeout=5)

# モジュールレベルのシングルトン（全RecordManagerインスタンスで共有）
_async_writer = _AsyncImageWriter()

class RecordManager:
    _current_session_dir = None  # Donkeycar形式用のセッションディレクトリ
    _current_images_dir = None  # CSV/NDJSON形式用のimagesディレクトリ

    def __init__(self):
        self.records = []
        self.headers = None
        self.record_directory = self._initialize_directory(config.RECORDS_DIRECTORY, "records")
        self.image_directory = self._initialize_directory(config.IMAGES_DIRECTORY, "images")
        self.lidar_directory = None
        self.file_path = ""
        
        # Donkeycar形式用の初期化
        if config.SAVE_FORMAT == "donkeycar":
            self.donkey_index = 0
            self.donkey_session_id = datetime.datetime.now().strftime('%y-%m-%d_%H')
            self.donkey_catalog_index = 0
            self.donkey_catalog_path = None
            self.donkey_manifest_data = None
            self.current_catalog_records = []  # 現在のカタログのレコードリスト
            self._initialize_donkeycar()

    def _initialize_directory(self, base_dir, prefix):
        # Donkeycar形式の場合は特別な命名規則とディレクトリ構造
        if config.SAVE_FORMAT == "donkeycar":
            # 既存のdata_フォルダを探す（最新のものを使用）
            data_dirs = [d for d in os.listdir("data") if d.startswith("data_") and os.path.isdir(os.path.join("data", d))] if os.path.exists("data") else []
            
            # セッション継続中は既存のディレクトリを使用
            # 新規セッションまたは既存ディレクトリがない場合のみ新規作成
            if hasattr(RecordManager, '_current_session_dir') and RecordManager._current_session_dir and os.path.exists(RecordManager._current_session_dir):
                dir_path = RecordManager._current_session_dir
                print(f"既存のDonkeycarディレクトリを使用: {dir_path}")
            else:
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                dir_path = os.path.join("data", f"data_{timestamp}")
                os.makedirs(dir_path, exist_ok=True)
                print(f"新規Donkeycarディレクトリ作成: {dir_path}")
                RecordManager._current_session_dir = dir_path
            return dir_path
        else:
            # CSV/NDJSON形式もセッション管理を実装
            if prefix == "records":
                # recordsディレクトリはファイル保存時に作成
                return base_dir
            elif prefix == "images":
                # imagesディレクトリはセッションごとに管理
                session_key = f"_current_{prefix}_dir"
                if hasattr(RecordManager, session_key) and getattr(RecordManager, session_key) and os.path.exists(getattr(RecordManager, session_key)):
                    dir_path = getattr(RecordManager, session_key)
                    print(f"既存の{prefix}ディレクトリを使用: {dir_path}")
                else:
                    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                    dir_path = os.path.join(base_dir, f"{prefix}_{timestamp}")
                    os.makedirs(dir_path, exist_ok=True)
                    print(f"新規{prefix}ディレクトリ作成: {dir_path}")
                    setattr(RecordManager, session_key, dir_path)
                return dir_path
            else:
                return base_dir


    def record_data(self, timestamp, mode, sensor_data, steering_value, throttle_value):
        # Donkeycar形式の場合は画像保存をスキップ（_record_donkeycar内で処理）
        if config.SAVE_FORMAT != "donkeycar":
            # カメラデータがある場合だけ画像保存
            camera_images = {}
            for key in list(sensor_data.keys()):
                if key.startswith("camera_") and sensor_data[key] is not None:
                    camera_images[key] = sensor_data[key]
            
            steering_px = self.map_range(steering_value, config.IMAGE_W)
            throttle_px = self.map_range(throttle_value, config.IMAGE_H)
            
            if len(camera_images) == 1:
                # カメラが1台の場合は "cam" として保存
                key, image = list(camera_images.items())[0]
                cam_file_name = self._save_image_with_name(image, timestamp, steering_px, throttle_px, self.image_directory, "cam")
                sensor_data["cam_path"] = cam_file_name
                del sensor_data[key] #画像データはデータ容量的に外す
            elif len(camera_images) > 1:
                # 複数カメラがある場合は結合画像を保存（設定に基づく）
                cam0 = camera_images.get("camera_0")
                cam1 = camera_images.get("camera_1")
                if cam0 is not None and cam1 is not None and config.SAVE_CONCATENATED_IMAGE:
                    # 画像を結合（設定に基づいて横または縦）
                    if config.IMAGE_CONCAT_DIRECTION == "vertical":
                        concatenated = np.vstack([cam0, cam1])
                    else:
                        concatenated = np.hstack([cam0, cam1])

                    # 結合画像のリサイズ（設定に基づく）
                    if config.RESIZE_CONCATENATED_IMAGE:
                        concatenated = cv2.resize(concatenated, (config.IMAGE_W, config.IMAGE_H))

                    # 結合画像を保存
                    cam_file_name = self._save_image_with_name(concatenated, timestamp, steering_px, throttle_px, self.image_directory, "cam")
                    sensor_data["cam_path"] = cam_file_name
                
                # 個別のカメラ画像も保存
                for key, image in camera_images.items():
                    # camera_0 -> cam0, camera_1 -> cam1
                    cam_name = f"cam{key.split('_')[1]}"
                    image_file_name = self._save_image_with_name(image, timestamp, steering_px, throttle_px, self.image_directory, cam_name)
                    # 画像ファイルパスを記録
                    sensor_data[f"{cam_name}_path"] = image_file_name
                    del sensor_data[key] #画像データはデータ容量的に外す

            # LiDAR BEV画像の保存（カメラと同じimages配下）
            lidar_image = sensor_data.pop("lidar_image", None)
            if lidar_image is not None:
                lidar_file_name = self._save_image_with_name(lidar_image, timestamp, steering_px, throttle_px, self.image_directory, "lidar")
                sensor_data["lidar_image_path"] = lidar_file_name

        # LiDAR点群データの抽出（sensor_dataからpopして別処理）
        lidar_distance_array = sensor_data.pop("lidar_distance_array", None)

        # テーブルフォーマットで保存
        if config.SAVE_FORMAT == "csv":
            if lidar_distance_array is not None:
                lidar_npy_path = self._save_lidar_npy(timestamp, lidar_distance_array)
                sensor_data["lidar_distance_array_path"] = lidar_npy_path
            self._record_csv(timestamp, mode, sensor_data, steering_value, throttle_value)
        elif config.SAVE_FORMAT == "ndjson":
            if lidar_distance_array is not None:
                lidar_npy_path = self._save_lidar_npy(timestamp, lidar_distance_array)
                sensor_data["lidar_distance_array_path"] = lidar_npy_path
            self._record_ndjson(timestamp, mode, sensor_data, steering_value, throttle_value)
        elif config.SAVE_FORMAT == "donkeycar":
            self._record_donkeycar(timestamp, mode, sensor_data, steering_value, throttle_value, lidar_distance_array=lidar_distance_array)
    
    def map_range(self, value, target_max):
        # Clamp the value to the range [-1, 1]
        value = max(-1, min(value, 1))
        
        # Map the clamped value from [-1, 1] to [0, target_max]
        return (value + 1) * (target_max / 2)

    def _record_csv(self, timestamp, mode, sensor_data, steering_value, throttle_value):
        # ヘッダー初期化（初回のみ）
        if not self.headers:
            headers = [
                "timestamp", 
                "mode",
                "steering", 
                "throttle",
            ]
            
            # 超音波センサー（最新値 + 統計値）
            for i in range(5):  # zone_0 ~ zone_4
                zone = f'zone_{i}'
                headers.extend([
                    f'ultrasonic/{zone}_latest',
                    f'ultrasonic/{zone}_avg',
                    f'ultrasonic/{zone}_min',
                    f'ultrasonic/{zone}_max',
                ])
            
            # IMU（最新値 + 統計値）
            for axis in ['x', 'y', 'z']:
                for prefix in ['acl', 'gyr']:
                    headers.extend([
                        f'imu_{prefix}_{axis}_latest',
                        f'imu_{prefix}_{axis}_avg',
                        f'imu_{prefix}_{axis}_std',
                    ])
            
            # RPM（最新値 + 統計値）
            headers.extend(['rpm_latest', 'rpm_avg'])
            
            # その他のセンサー（カメラパスなど）
            other_keys = [k for k in sensor_data.keys() 
                         if not k.startswith('ultrasonic/') 
                         and not k.startswith('imu_') 
                         and not k.startswith('rpm')]
            headers.extend(other_keys)
            
            # ルール実行情報（拡張項目）
            headers.extend([
                "plan_rule",
                "adjustment_method",
                "pid_error",
                "pid_p_value",
                "pid_d_value",
                "failure_flag",
                "failure_reason",
            ])
            
            self.headers = headers

        # データ行を構築
        row = [
            timestamp,
            mode,
            steering_value,
            throttle_value,
        ]
        
        # 各ヘッダーに対応する値を追加
        for header in self.headers[4:]:  # timestamp, mode, steering, throttle はスキップ
            row.append(sensor_data.get(header, ""))
        
        self.records.append(row)

    def _record_ndjson(self, timestamp, mode, sensor_data, steering_value, throttle_value):
        entry = {
            "timestamp": timestamp,
            "mode": mode,
            "steering": steering_value,
            "throttle": throttle_value,
        }

        for key, value in sensor_data.items():
            entry[f"sensor/{key}"] = value

        self.records.append(entry)

    def save_data(self):
        # 非同期画像保存の完了を待機
        _async_writer.flush()
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M')
        print(self.record_directory,f"{timestamp}_{config.RECORD_FILE_NAME}")
        self.file_path = os.path.join(self.record_directory,f"{timestamp}_{config.RECORD_FILE_NAME}")
        if config.SAVE_FORMAT == "csv":
            self._save_csv()
        elif config.SAVE_FORMAT == "ndjson":
            self._save_ndjson()
        elif config.SAVE_FORMAT == "donkeycar":
            self._save_donkeycar()
        
    def _save_csv(self):
        # 拡張子を確認し、必要に応じて追加
        if not self.file_path.endswith(".csv"):
            self.file_path += ".csv"

        # ディレクトリが存在しない場合は作成
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

        # CSVとして保存
        with open(self.file_path, "w") as file:
            file.write(",".join(self.headers) + "\n")
            for row in self.records:
                file.write(",".join(map(str, row)) + "\n")

        print(f"CSV保存: {self.file_path}")
        # セッション終了時にセッションディレクトリをリセット
        RecordManager._current_images_dir = None

    def _save_ndjson(self):
        # 拡張子を確認し、必要に応じて追加
        if not self.file_path.endswith(".ndjson"):
            self.file_path += ".ndjson"

        # ディレクトリが存在しない場合は作成
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

        # NDJSONとして保存
        with open(self.file_path, "w") as file:
            for entry in self.records:
                file.write(json.dumps(entry) + "\n")

        print(f"NDJSON保存: {self.file_path}")
        # セッション終了時にセッションディレクトリをリセット
        RecordManager._current_images_dir = None

    def _save_image(self, image,  ts, steer, throttle,  image_dir):
        try:
            #image = cv2.resize(image, (image_size_w, image_size_h))
            image_file_name = image_dir +'/' + ts +'_'+ str(int(steer)) +'_'+ str(int(throttle)) +'.jpg'
            # RGB→BGR変換（cv2.imwriteはBGR形式を期待）
            cv2.imwrite(image_file_name, image[:, :, ::-1])
            #image_sh[:] = image.flatten()
            return image_file_name
            #return image
        except:
            print("Cannot save image!")
            pass

    def _save_image_with_name(self, image, ts, steer, throttle, image_dir, name_prefix):
        try:
            # ディレクトリが存在しない場合のみ作成
            if not os.path.exists(image_dir):
                os.makedirs(image_dir)
                print(f"ディレクトリ作成: {image_dir}")

            image_file_name = image_dir +'/' + ts +'_'+ name_prefix +'_'+ str(int(steer)) +'_'+ str(int(throttle)) +'.jpg'
            # RGB→BGR変換して非同期保存（メインループをブロックしない）
            _async_writer.submit_imwrite(image_file_name, image[:, :, ::-1].copy())
            return image_file_name
        except:
            print(f"Cannot save {name_prefix} image!")
            pass


    def generate_terminal_output(self, elapsed_time, record_count, mode, steering_value, throttle_value, sensor_data):
        sensor_values = ", ".join(
            [f"{key}:{value:>5}" for key, value in sensor_data.items()]
        )
        return f"[REC:{record_count}{elapsed_time}] Mode:{mode}, St:{steering_value:>6.2f}, Th:{throttle_value:>5.2f}, {sensor_values}"

    def generate_plotter_output(self, timestamp, mode, steering_value, throttle_value, sensor_data):
        timestamp_formated = timestamp[8:10] + ":" + timestamp[10:12]+ ":" + timestamp[12:14]+ "." + timestamp[14:17]
        values = [
            f"{timestamp_formated}",
            mode,
            str(steering_value),
            str(throttle_value),
        ] + [f"{value:.2f}" for value in sensor_data.values()]
        return ", ".join(values)
    
    def _get_lidar_directory(self):
        """CSV/NDJSON形式用のlidarディレクトリを取得・作成"""
        if self.lidar_directory is None:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            lidar_base = getattr(config, 'LIDAR_DIRECTORY', 'lidar')
            self.lidar_directory = os.path.join(lidar_base, f"lidar_{timestamp}")
            os.makedirs(self.lidar_directory, exist_ok=True)
        return self.lidar_directory

    def _save_lidar_npy(self, timestamp, lidar_distance_array):
        """CSV/NDJSON形式用: LiDAR点群データをnpyファイルとして保存"""
        lidar_dir = self._get_lidar_directory()
        npy_filename = f"{timestamp}_lidar_distance_array.npy"
        npy_path = os.path.join(lidar_dir, npy_filename)
        _async_writer.submit_npy_save(npy_path, np.array(lidar_distance_array, dtype=np.int16))
        return npy_path

    def _initialize_donkeycar(self):
        """Donkeycar形式の初期化"""
        # カメラの数を確認
        camera_count = sum(1 for key in config.ACTIVE_SENSORS if key.startswith("camera_"))
        
        # ヘッダーの作成（Donkeycar形式）
        headers = []
        types = []
        
        # カメラヘッダー（cam/image_arrayを必ず含む）
        headers.append("cam/image_array")
        types.append("image_array")
        
        # 複数カメラの場合は個別のカメラヘッダーも追加
        if camera_count > 1:
            for i in range(camera_count):
                headers.append(f"cam{i}/image_array")
                types.append("image_array")
        
        # 基本的な制御データ
        headers.extend(["user/angle", "user/throttle", "user/mode"])
        types.extend(["float", "float", "str"])

        # speed ラベル（NUM_OUTPUTS=3 時の学習用、センサー計測速度 m/s）
        if getattr(config, 'NUM_OUTPUTS', 2) >= 3:
            headers.append("user/speed")
            types.append("float")
        
        # manifest.jsonの作成
        self.donkey_manifest_data = {
            "headers": headers,
            "types": types,
            "metadata": {},
            "session_info": {
                "created_at": datetime.datetime.now().timestamp(),
                "sessions": {
                    "all_full_ids": [self.donkey_session_id],
                    "last_id": 0,
                    "last_full_id": self.donkey_session_id
                }
            },
            "catalog_info": {
                "paths": [],
                "current_index": 0,
                "max_len": 1000,
                "deleted_indexes": []
            }
        }
        
        # センサーデータのヘッダーを追加
        sensor_headers = []
        sensor_types = []
        
        # 超音波センサー
        if "ultrasonic" in config.ACTIVE_SENSORS:
            for us_name in config.ULTRASONIC_SENSOR_LIST:
                sensor_headers.append(f"ultrasonic/{us_name}")
                sensor_types.append("int")
        
        # IMUセンサー
        if "imu" in config.ACTIVE_SENSORS:
            imu_headers = ["imu/acl_x", "imu/acl_y", "imu/acl_z", "imu/gyr_x", "imu/gyr_y", "imu/gyr_z"]
            sensor_headers.extend(imu_headers)
            sensor_types.extend(["float"] * len(imu_headers))

        # RPMセンサー
        if "rpm" in config.ACTIVE_SENSORS:
            sensor_headers.extend(["rpm/value", "rpm/speed"])
            sensor_types.extend(["int", "float"])

        # オプティカルフローセンサー (m/s)
        if "optical_flow" in config.ACTIVE_SENSORS:
            sensor_headers.extend(["of/vx", "of/vy"])
            sensor_types.extend(["float", "float"])

        # LiDAR点群データ
        if "lidar" in config.ACTIVE_SENSORS and getattr(config, 'SAVE_LIDAR_DATA', False):
            sensor_headers.append("lidar/distance_array")
            sensor_types.append("nparray")

        # LiDAR BEV画像
        if "lidar" in config.ACTIVE_SENSORS and getattr(config, 'SAVE_LIDAR_IMAGES', False):
            sensor_headers.append("lidar/image_array")
            sensor_types.append("image_array")

        self.donkey_manifest_data["headers"].extend(sensor_headers)
        self.donkey_manifest_data["types"].extend(sensor_types)

        # LiDARメタデータをmanifestに記録
        if "lidar" in config.ACTIVE_SENSORS and getattr(config, 'SAVE_LIDAR_DATA', False):
            self.donkey_manifest_data["metadata"]["lidar_type"] = getattr(config, 'LIDAR_TYPE', 'UNKNOWN')
            self.donkey_manifest_data["metadata"]["lidar_angle_start"] = getattr(config, 'LIDAR_ANGLE_START', 0)
            self.donkey_manifest_data["metadata"]["lidar_angle_end"] = getattr(config, 'LIDAR_ANGLE_END', 0)
            self.donkey_manifest_data["metadata"]["lidar_clockwise"] = getattr(config, 'LIDAR_CLOCKWISE', True)
            self.donkey_manifest_data["metadata"]["lidar_data_points"] = getattr(config, 'LIDAR_DATA_POINTS', 0)

        # imagesディレクトリの作成
        donkey_images_dir = os.path.join(self.record_directory, "images")
        if not os.path.exists(donkey_images_dir):
            os.makedirs(donkey_images_dir)

        # lidarディレクトリの作成
        if "lidar" in config.ACTIVE_SENSORS and getattr(config, 'SAVE_LIDAR_DATA', False):
            self.lidar_directory = os.path.join(self.record_directory, "lidar")
            if not os.path.exists(self.lidar_directory):
                os.makedirs(self.lidar_directory)
    
    def _save_current_catalog(self):
        """現在のカタログファイルを保存（1000件毎の中間保存）"""
        if not self.current_catalog_records or not self.donkey_catalog_path:
            return

        # カタログファイルに書き込み
        with open(self.donkey_catalog_path, "w") as f:
            for record in self.current_catalog_records:
                f.write(json.dumps(record) + "\n")

        # manifest.jsonを更新保存
        manifest_lines = []
        manifest_lines.append(json.dumps(self.donkey_manifest_data["headers"]))
        manifest_lines.append(json.dumps(self.donkey_manifest_data["types"]))
        manifest_lines.append(json.dumps(self.donkey_manifest_data["metadata"]))
        manifest_lines.append(json.dumps(self.donkey_manifest_data["session_info"]))
        manifest_lines.append(json.dumps(self.donkey_manifest_data["catalog_info"]))

        manifest_path = os.path.join(self.record_directory, "manifest.json")
        with open(manifest_path, "w") as f:
            f.write("\n".join(manifest_lines) + "\n")

        print(f"中間保存完了: {self.donkey_catalog_path} ({len(self.current_catalog_records)}件)")

    def _record_donkeycar(self, timestamp, mode, sensor_data, steering_value, throttle_value, lidar_distance_array=None):
        """Donkeycar形式でデータを記録"""
        # カタログファイルの確認・作成
        if self.donkey_index % 1000 == 0:
            if self.donkey_catalog_path and self.donkey_index > 0:
                # 現在のカタログファイルを保存（1000件毎に中間保存）
                self._save_current_catalog()

            # 新しいカタログファイルを作成
            catalog_filename = f"catalog_{self.donkey_catalog_index}.catalog"
            self.donkey_catalog_path = os.path.join(self.record_directory, catalog_filename)
            self.donkey_manifest_data["catalog_info"]["paths"].append(catalog_filename)
            self.donkey_catalog_index += 1

            # 新しいカタログ用のレコードリストを初期化
            self.current_catalog_records = []
        
        # レコードの作成
        record = {
            "_index": self.donkey_index,
            "_session_id": self.donkey_session_id,
            "_timestamp_ms": int(datetime.datetime.now().timestamp() * 1000),
            "user/angle": steering_value,
            "user/throttle": throttle_value,
            "user/mode": mode
        }

        # speed ラベル（センサー計測速度 m/s）
        if getattr(config, 'NUM_OUTPUTS', 2) >= 3:
            record["user/speed"] = sensor_data.get("speed", 0.0)
        
        # カメラ画像の保存（Donkeycar形式）
        camera_keys = sorted([k for k in sensor_data.keys() if k.startswith("camera_") and "_path" not in k])
        
        if len(camera_keys) == 1:
            # カメラが1台の場合は "cam" として保存
            key = camera_keys[0]
            if sensor_data.get(key) is not None:
                image_filename = f"{self.donkey_index}_cam_image_array_.jpg"
                image_path = os.path.join(self.record_directory, "images", image_filename)
                # RGB→BGR変換して非同期保存
                _async_writer.submit_imwrite(image_path, sensor_data[key][:, :, ::-1].copy())
                record["cam/image_array"] = image_filename
        elif len(camera_keys) > 1:
            # 複数カメラがある場合は結合画像を作成（設定に基づく）
            cam0 = sensor_data.get("camera_0")
            cam1 = sensor_data.get("camera_1")
            if cam0 is not None and cam1 is not None and config.SAVE_CONCATENATED_IMAGE:
                # 画像を結合（設定に基づいて横または縦）
                if config.IMAGE_CONCAT_DIRECTION == "vertical":
                    concatenated = np.vstack([cam0, cam1])
                else:
                    concatenated = np.hstack([cam0, cam1])

                # 結合画像のリサイズ（設定に基づく）
                if config.RESIZE_CONCATENATED_IMAGE:
                    concatenated = cv2.resize(concatenated, (config.IMAGE_W, config.IMAGE_H))

                image_filename = f"{self.donkey_index}_cam_image_array_.jpg"
                image_path = os.path.join(self.record_directory, "images", image_filename)
                # RGB→BGR変換して非同期保存
                _async_writer.submit_imwrite(image_path, concatenated[:, :, ::-1].copy())
                record["cam/image_array"] = image_filename

            # 個別のカメラ画像も保存
            for idx, key in enumerate(camera_keys):
                if sensor_data.get(key) is not None:
                    # カメラインデックスの取得（camera_0 -> 0）
                    cam_idx = int(key.split("_")[1])

                    # Donkeycar形式のヘッダー名とファイル名
                    header_name = f"cam{cam_idx}/image_array"
                    image_filename = f"{self.donkey_index}_cam{cam_idx}_image_array_.jpg"

                    image_path = os.path.join(self.record_directory, "images", image_filename)
                    # RGB→BGR変換して非同期保存
                    _async_writer.submit_imwrite(image_path, sensor_data[key][:, :, ::-1].copy())
                    record[header_name] = image_filename

        # LiDAR点群データの保存（npyファイル）
        if lidar_distance_array is not None and self.lidar_directory:
            lidar_filename = f"{self.donkey_index}_lidar_distance_array_.npy"
            lidar_path = os.path.join(self.lidar_directory, lidar_filename)
            # 非同期保存
            _async_writer.submit_npy_save(lidar_path, np.array(lidar_distance_array, dtype=np.int16))
            record["lidar/distance_array"] = lidar_filename

        # LiDAR BEV画像の保存（カメラと同じimages/配下）
        lidar_image = sensor_data.pop("lidar_image", None)
        if lidar_image is not None:
            lidar_img_filename = f"{self.donkey_index}_lidar_image_array_.jpg"
            lidar_img_path = os.path.join(self.record_directory, "images", lidar_img_filename)
            # RGB→BGR変換して非同期保存
            _async_writer.submit_imwrite(lidar_img_path, lidar_image[:, :, ::-1].copy())
            record["lidar/image_array"] = lidar_img_filename

        # センサーデータの追加
        for key, value in sensor_data.items():
            if not key.startswith("camera_") and "_path" not in key:
                # 超音波センサーとLiDARゾーンデータ（同じゾーン名を使用）
                if key in config.ULTRASONIC_SENSOR_LIST:
                    # ultrasonicが有効な場合はultrasonic/として保存
                    if "ultrasonic" in config.ACTIVE_SENSORS:
                        record[f"ultrasonic/{key}"] = int(value) if value is not None else 0
                    # lidarが有効な場合はlidar/として保存
                    elif "lidar" in config.ACTIVE_SENSORS:
                        record[f"lidar/{key}"] = float(value) if value is not None else 0.0
                # IMUセンサー (imu_acl_x, imu_gyr_z 等)
                elif key.startswith("imu_"):
                    imu_key = key.replace("imu_", "imu/")
                    record[imu_key] = float(value) if value is not None else 0.0
                # RPMセンサー (rpm_value, rpm_speed)
                elif key.startswith("rpm_"):
                    rpm_key = key.replace("rpm_", "rpm/")
                    record[rpm_key] = value if value is not None else 0
                # オプティカルフローセンサー (of_vx, of_vy) — m/s
                elif key.startswith("of_"):
                    of_key = key.replace("of_", "of/")
                    record[of_key] = float(value) if value is not None else 0.0
        
        # レコードを保存
        self.records.append(record)
        self.current_catalog_records.append(record)  # 現在のカタログにも追加
        self.donkey_index += 1
        self.donkey_manifest_data["catalog_info"]["current_index"] = self.donkey_index
    
    def _save_donkeycar(self):
        """Donkeycar形式でデータを保存（終了時の最終保存）"""
        # 最後の残りのレコードを保存
        if self.current_catalog_records and self.donkey_catalog_path:
            self._save_current_catalog()

        print(f"Donkeycar形式で最終保存完了: {self.record_directory} (総レコード数: {self.donkey_index}件)")

        # 記録セッション終了時にセッションディレクトリをリセット
        RecordManager._current_session_dir = None

        # 【既存コード - コメントアウト】
        # # manifest.jsonを保存
        # manifest_lines = []
        # manifest_lines.append(json.dumps(self.donkey_manifest_data["headers"]))
        # manifest_lines.append(json.dumps(self.donkey_manifest_data["types"]))
        # manifest_lines.append(json.dumps(self.donkey_manifest_data["metadata"]))
        # manifest_lines.append(json.dumps(self.donkey_manifest_data["session_info"]))
        # manifest_lines.append(json.dumps(self.donkey_manifest_data["catalog_info"]))
        #
        # manifest_path = os.path.join(self.record_directory, "manifest.json")
        # with open(manifest_path, "w") as f:
        #     f.write("\n".join(manifest_lines) + "\n")
        #
        # # カタログファイルを保存
        # current_catalog_records = []
        # for i, record in enumerate(self.records):
        #     catalog_index = i // 1000
        #     if catalog_index >= len(self.donkey_manifest_data["catalog_info"]["paths"]):
        #         break
        #
        #     catalog_path = os.path.join(self.record_directory,
        #                                self.donkey_manifest_data["catalog_info"]["paths"][catalog_index])
        #
        #     if i % 1000 == 0 and current_catalog_records:
        #         # 前のカタログを保存
        #         with open(prev_catalog_path, "w") as f:
        #             for rec in current_catalog_records:
        #                 f.write(json.dumps(rec) + "\n")
        #         current_catalog_records = []
        #
        #     current_catalog_records.append(record)
        #     prev_catalog_path = catalog_path
        #
        # # 最後のカタログを保存
        # if current_catalog_records and 'prev_catalog_path' in locals():
        #     with open(prev_catalog_path, "w") as f:
        #         for rec in current_catalog_records:
        #             f.write(json.dumps(rec) + "\n")
        #
        # print(f"Donkeycar形式で保存: {self.record_directory}")
        #
        # # 記録セッション終了時にセッションディレクトリをリセット
        # RecordManager._current_session_dir = None