# coding:utf-8
import os
import time
import sys
import numpy as np
import threading
import socket
from logging import getLogger, INFO, Logger
import json
from flask import Flask, render_template, jsonify, Response, request
import io
import cv2
from PIL import Image
from multiprocessing import Process, Value, Array
from abc import ABC, abstractmethod
import signal as sig
# WallDetectorをインポート
from lidar_detector import WallDetector, DetectionMethod

logger: Logger = getLogger("donkeycar.parts.lidar")

import config as cfg

class LidarBase(ABC):
    """LiDAR関連の共通処理を提供する基底クラス"""
    
    def __init__(self, image_w=None, image_h=None):
        """基底クラスの初期化
        
        Args:
            image_w: 画像幅（Noneの場合は設定から取得）
            image_h: 画像高さ（Noneの場合は設定から取得）
        """        
        # 画像サイズ設定
        self.image_w = image_w if image_w is not None else getattr(cfg, 'IMAGE_W', 224)
        self.image_h = image_h if image_h is not None else getattr(cfg, 'IMAGE_H', 224)
        
        # ゾーンインデックスを設定ファイルから読み込む
        self.zone_index = getattr(cfg, 'ZONE_INDEX', [])
        
        # 共通パラメータの初期化（サブクラスでオーバーライド可能）
        self.min_distance = getattr(cfg, 'LIDAR_MIN_DISTANCE', 100)      # unit:mm
        self.max_distance = getattr(cfg, 'LIDAR_MAX_DISTANCE', 20000)    # unit:mm
        self.wall_distance = getattr(cfg, 'LIDAR_WALL_DISTANCE', 300)    # unit:mm
        self.ignore_distance = getattr(cfg, 'LIDAR_IGNORE_DISTANCE', 150) # unit:mm
        
        # 角度関連パラメータ（全LiDAR共通）
        self.angle_start = getattr(cfg, 'LIDAR_ANGLE_START', -135)  # 度
        self.angle_end = getattr(cfg, 'LIDAR_ANGLE_END', 135)       # 度
        self.angle_offset = getattr(cfg, 'LIDAR_ANGLE_OFFSET', 0)   # 度
        self.clockwise = getattr(cfg, 'LIDAR_CLOCKWISE', False)     # スキャン方向
        
        # 画像変換関連パラメータ
        self.binary_mode = getattr(cfg, 'LIDAR_BINARY_IMAGE', False)
        
        # 単位系設定
        self.unit_type = getattr(cfg, 'LIDAR_UNIT_TYPE', 'mm')        # センサーのネイティブ単位
        self.target_unit = getattr(cfg, 'LIDAR_TARGET_UNIT', 'mm')    # システム内部単位
        
        self.measurements = []
        self.running = True
        
        # 検出関連
        self.detect_points_threshold = getattr(cfg, 'LIDAR_DETECT_POINTS_THRESHOLD', 15)
        self.zone_thresholds = getattr(cfg, 'LIDAR_DETECT_POINTS_THRESHOLD_ZONE', 
                                     [self.detect_points_threshold] * len(self.zone_index))
        self.detect_distance_threshold = getattr(cfg, 'LIDAR_DETECT_DISTANCE_THRESHOLD', 300)
        self.distance_thresholds = getattr(cfg, 'LIDAR_DETECT_DISTANCE_THRESHOLD_ZONE', 
                                         [self.detect_distance_threshold] * len(self.zone_index))
        
        # 壁検出の有効/無効
        self.detect_walls_enabled = getattr(cfg, 'LIDAR_DETECT_WALLS', False)
        self.wall_segments = []
        self.wall_detection_time = 0.0
        
        # 画像変換器を初期化
        self.image_converter = LidarImageConverter(
            image_w=self.image_w,
            image_h=self.image_h,
            max_distance=self.max_distance,
            min_distance=self.min_distance,
            angle_start=self.angle_start,
            angle_end=self.angle_end,
            angle_offset=self.angle_offset,
            clockwise=self.clockwise,
            binary_mode=self.binary_mode
        )
        
        # 画像変換器の詳細パラメータを設定
        self._setup_image_converter_parameters()
        
        # image_converterが作成されたので、デフォルトの空画像で初期化
        # latest_imageが既に初期化されていない場合のみ設定
        if not hasattr(self, 'latest_image') or self.latest_image is None:
            self.latest_image = np.zeros((self.image_h, self.image_w, 3), dtype=np.uint8)

        # バックグラウンド画像生成スレッド
        import threading
        self._image_gen_enabled = False  # run.pyから制御
        self._image_gen_event = threading.Event()
        self._image_gen_thread = threading.Thread(target=self._image_gen_loop, daemon=True)
        self._image_gen_thread.start()
        
        # WallDetectorインスタンスを作成
        global current_detection_method
        initial_method = getattr(cfg, 'LIDAR_DETECTION_METHOD', 'distance_based')
        try:
            method_enum = DetectionMethod(initial_method)
        except ValueError:
            method_enum = DetectionMethod.DISTANCE_BASED
            initial_method = 'distance_based'
        self.wall_detector = WallDetector(method=method_enum)
        current_detection_method = initial_method  # グローバル変数を初期化

        # デフォルトのWallDetectorパラメータを設定（サブクラスで上書き可能）
        self._setup_wall_detector_parameters()
    
    def _convert_units(self, points):
        """
        LiDARデータの単位変換を行う共通メソッド
        
        Args:
            points: 生データのリスト
            
        Returns:
            list: ターゲット単位に変換されたデータ
        """
        if self.unit_type == self.target_unit:
            # 単位変換不要
            return points
        elif self.unit_type == "m" and self.target_unit == "mm":
            # m -> mm (x1000)
            return [point * 1000.0 if point > 0 else point for point in points]
        elif self.unit_type == "mm" and self.target_unit == "m":
            # mm -> m (x0.001)
            return [point / 1000.0 if point > 0 else point for point in points]
        else:
            logger.warning(f"Unsupported unit conversion: {self.unit_type} -> {self.target_unit}")
            return points
        
        # 画像保存関連の設定
        self.save_images = getattr(cfg, 'SAVE_LIDAR_IMAGES', False)
        self.tub_path = None
        self.counter = 0
        self.lidar_dir = None
        
        # LiDARデータ保存用
        self.save_lidar_binary = getattr(cfg, 'SAVE_LIDAR_DATA', False)
        self.lidar_data_dir = None
        
        # 最新のLiDAR画像（後で初期化）
        self.latest_image = None
        
        # 検出詳細情報
        self.detection_details = []
        
        # マルチプロセス用の共有メモリ（サブクラスで初期化）
        self.on = None
        self.points = None
        self.p = None
    
    def _setup_image_converter_parameters(self):
        """画像変換器の詳細パラメータを設定"""

        # 画像変換器のパラメータを設定（configから縮尺パラメータを読み込み）
        scale_factor = getattr(cfg, 'LIDAR_IMAGE_SCALE_FACTOR', 0.8)
        meters_per_pixel = getattr(cfg, 'LIDAR_IMAGE_METERS_PER_PIXEL', 0.045)
        
        # 車両サイズ設定を読み込み
        vehicle_width = getattr(cfg, 'VEHICLE_WIDTH', 200)
        vehicle_length = getattr(cfg, 'VEHICLE_LENGTH', 450)
        vehicle_color = getattr(cfg, 'VEHICLE_DISPLAY_COLOR', (255, 255, 255))
        vehicle_thickness = getattr(cfg, 'VEHICLE_DISPLAY_THICKNESS', 2)
        
        # LiDAR搭載位置オフセット設定を読み込み
        lidar_offset_x = getattr(cfg, 'LIDAR_OFFSET_X', 0)
        lidar_offset_y = getattr(cfg, 'LIDAR_OFFSET_Y', 0)
        
        # 画像変換器を更新
        self.image_converter.update_parameters(
            scale_factor=scale_factor,
            meters_per_pixel=meters_per_pixel,
            vehicle_width=vehicle_width,
            vehicle_length=vehicle_length,
            vehicle_color=vehicle_color,
            vehicle_thickness=vehicle_thickness,
            lidar_offset_x=lidar_offset_x,
            lidar_offset_y=lidar_offset_y
        )
    
    def _setup_wall_detector_parameters(self):
        """基本的なWallDetectorパラメータを設定（サブクラスでオーバーライド可能）"""
        self.wall_detector.set_parameters(
            min_distance=self.ignore_distance,  # 壁検出ではignore_distanceを使用
            max_distance=self.max_distance,
            wall_distance=self.wall_distance,
            # 角度関連パラメータ
            angle_start=self.angle_start,
            angle_end=self.angle_end,
            angle_offset=self.angle_offset,
            clockwise=self.clockwise,
            # Distance-Based用パラメータ
            max_gap=getattr(cfg, 'LIDAR_WALL_MAX_GAP', 500),
            # 共通パラメータ
            min_wall_points=getattr(cfg, 'LIDAR_WALL_MIN_POINTS', 10),
            max_linearity=getattr(cfg, 'LIDAR_WALL_MAX_LINEARITY', 0.08),
            # Split-Merge用パラメータ
            split_epsilon=getattr(cfg, 'LIDAR_SPLIT_EPSILON', 60),
            min_segment_length=getattr(cfg, 'LIDAR_MIN_SEGMENT_LENGTH', 400),
            use_adaptive=getattr(cfg, 'LIDAR_USE_ADAPTIVE', True),
            use_2d_optimization=getattr(cfg, 'LIDAR_USE_2D_OPTIMIZATION', True),
            # RANSAC用パラメータ
            ransac_threshold=getattr(cfg, 'LIDAR_RANSAC_THRESHOLD', 60),
            min_inlier_ratio=getattr(cfg, 'LIDAR_MIN_INLIER_RATIO', 0.6),
            max_trials=getattr(cfg, 'LIDAR_RANSAC_MAX_TRIALS', 150),
            early_stop_ratio=getattr(cfg, 'LIDAR_EARLY_STOP_RATIO', 0.9),
            # Sliding Window用パラメータ
            window_size=getattr(cfg, 'LIDAR_WINDOW_SIZE', 20),
            window_stride=getattr(cfg, 'LIDAR_WINDOW_STRIDE', 5),
            overlap_threshold=getattr(cfg, 'LIDAR_OVERLAP_THRESHOLD', 700),
            # Hybrid用パラメータ
            confidence_threshold=getattr(cfg, 'LIDAR_CONFIDENCE_THRESHOLD', 0.8),
            # セグメント統合用パラメータ
            merge_angle_threshold=getattr(cfg, 'LIDAR_MERGE_ANGLE_THRESHOLD', 10),
            merge_distance_threshold=getattr(cfg, 'LIDAR_MERGE_DISTANCE_THRESHOLD', 100)
        )
    
    def poll(self):
        """LiDARデータを取得する（共通実装）"""
        global current_lidar_data

        if self.running:
            # 共有メモリから高速コピー（frombuffer + astype）
            self.measurements = np.frombuffer(self.points.get_obj(), dtype=np.float32).astype(np.int32)
            # グローバル変数を更新（Web表示用）
            current_lidar_data = self.measurements

            # 壁検出を実行（設定で有効な場合のみ）
            if self.detect_walls_enabled:
                self.detect_walls(self.measurements)
    
    @abstractmethod
    def multiprocess(self, points):
        """マルチプロセスでLiDAR通信を実行（各実装で定義）"""
        pass
    
    def update(self):
        """更新ループ（共通実装）"""
        while self.running:
            self.poll()
            
            # Web表示用に画像を生成
            if hasattr(self, 'image_converter') and self.measurements is not None and len(self.measurements) > 0:
                self.latest_image = self.image_converter.points_to_image(
                    self.measurements, 
                    draw_wall_segments=self.detect_walls_enabled, 
                    wall_segments=self.wall_segments,
                )
            
            time.sleep(0.02) #早すぎても遅すぎてもだめ,0.02sがちょうどよい
    
    def run_threaded(self):
        """スレッド実行（共通実装）"""
        if self.running:
            detection_distances, detection_binary, zone_distances = self.detect(self.measurements)
            
            # 画像生成
            if hasattr(self, 'image_converter') and self.measurements is not None and len(self.measurements) > 0:
                try:
                    image = self.image_converter.points_to_image(
                        self.measurements, 
                        draw_wall_segments=self.detect_walls_enabled, 
                        wall_segments=self.wall_segments,
                    )
                    # 画像の形状を検証
                    if image is not None and hasattr(image, 'shape') and len(image.shape) == 3:
                        self.latest_image = image
                    else:
                        logger.warning(f"Invalid image generated: shape={getattr(image, 'shape', 'None')}")
                        # デフォルトの空画像を生成
                        self.latest_image = np.zeros((self.image_converter.image_h, self.image_converter.image_w, 3), dtype=np.uint8)
                except Exception as e:
                    logger.error(f"Error generating lidar image: {e}")
                    # エラー時はデフォルトの空画像を生成
                    self.latest_image = np.zeros((self.image_converter.image_h, self.image_converter.image_w, 3), dtype=np.uint8)
            
            # latest_imageがNoneの場合は空画像を返す
            if self.latest_image is None:
                self.latest_image = np.zeros((self.image_h, self.image_w, 3), dtype=np.uint8)
            return detection_distances, detection_binary, self.wall_segments, self.latest_image, self.measurements
        # デフォルトの空画像を返す
        return [], [], [], np.zeros((self.image_h, self.image_w, 3), dtype=np.uint8), []
    
    def _image_gen_loop(self):
        """バックグラウンドで画像生成を行うスレッド"""
        while self.running:
            # イベント待ち（generate_imageリクエストが来るまでブロック）
            self._image_gen_event.wait()
            self._image_gen_event.clear()

            if not self.running:
                break

            if hasattr(self, 'image_converter') and self.measurements is not None and len(self.measurements) > 0:
                try:
                    image = self.image_converter.points_to_image(
                        self.measurements,
                        draw_wall_segments=self.detect_walls_enabled,
                        wall_segments=self.wall_segments,
                    )
                    if image is not None and hasattr(image, 'shape') and len(image.shape) == 3:
                        self.latest_image = image
                except Exception as e:
                    logger.error(f"Error generating lidar image in background: {e}")

    def run(self, generate_image=False):
        """同期実行（共通実装）

        Args:
            generate_image: Trueの場合、バックグラウンドスレッドに画像生成を依頼
        """
        if not self.running:
            return [], [], [], np.zeros((self.image_h, self.image_w, 3), dtype=np.uint8), []
        self.poll()
        time.sleep(0)  # yield time to other threads
        detection_distances, detection_binary, zone_distances = self.detect(self.measurements)

        # 画像生成をバックグラウンドスレッドに依頼（ノンブロッキング）
        if generate_image:
            self._image_gen_event.set()

        # latest_imageがNoneの場合は空画像を返す
        if self.latest_image is None:
            self.latest_image = np.zeros((self.image_h, self.image_w, 3), dtype=np.uint8)
        return detection_distances, detection_binary, self.wall_segments, self.latest_image, self.measurements
    
    def cleanup(self):
        """クリーンアップ（run.pyのcleanup_systemから呼ばれる）"""
        logger.info("LiDAR cleanup started...")
        self.shutdown()
        logger.info("LiDAR cleanup completed.")

    def shutdown(self):
        """シャットダウン（共通実装）"""
        self.running = False
        # 画像生成スレッドを終了
        if hasattr(self, '_image_gen_event'):
            self._image_gen_event.set()  # ブロック解除
        if hasattr(self, '_image_gen_thread'):
            self._image_gen_thread.join(timeout=1.0)
        if hasattr(self, 'on') and self.on is not None:
            self.on.value = False
        time.sleep(0.5)
        if hasattr(self, 'p') and self.p is not None:
            self.p.terminate()
            self.p.join(timeout=1.0)
        if hasattr(self, 'update_thread') and self.update_thread is not None:
            self.update_thread.join(timeout=1.0)
    
    def set_tub_path(self, tub_path):
        """記録用のtubパスを設定"""
        self.tub_path = tub_path
        
        if tub_path:
            # LiDAR画像保存用ディレクトリ
            if self.save_images:
                self.lidar_dir = os.path.join(tub_path, 'lidar_img')
                if not os.path.exists(self.lidar_dir):
                    os.makedirs(self.lidar_dir)
                    logger.info(f"Created lidar image directory: {self.lidar_dir}")
            
            # LiDARデータ保存用ディレクトリ
            if self.save_lidar_binary:
                self.lidar_data_dir = os.path.join(tub_path, 'lidar')
                if not os.path.exists(self.lidar_data_dir):
                    os.makedirs(self.lidar_data_dir)
                    logger.info(f"Created lidar data directory: {self.lidar_data_dir}")
    
    def save_image(self, recording=False):
        """LiDAR画像を保存する"""
        if not self.save_images or not recording or self.latest_image is None or self.lidar_dir is None:
            return self.counter
            
        filename = f"{self.counter}_lidar_image_array_.jpg"
        filepath = os.path.join(self.lidar_dir, filename)
        
        try:
            img = Image.fromarray(self.latest_image)
            img.save(filepath)
            logger.debug(f"Saved lidar image to {filepath}")
            self.counter += 1
        except Exception as e:
            logger.error(f"Error saving lidar image: {e}")
            
        return self.counter
    
    def save_lidar_data(self, recording=False):
        """LiDARの点群データをnumpy binary形式で保存する"""
        if not self.save_lidar_binary or not recording or not self.measurements or self.lidar_data_dir is None:
            return self.counter
            
        filename = f"{self.counter}_lidar_dist_.npy"
        filepath = os.path.join(self.lidar_data_dir, filename)
        
        try:
            # measurementsをnumpy配列に変換して保存
            lidar_array = np.array(self.measurements, dtype=np.float32)
            np.save(filepath, lidar_array)
            logger.debug(f"Saved lidar data to {filepath}")
        except Exception as e:
            logger.error(f"Error saving lidar data: {e}")
            
        return self.counter
    
    def detect_walls(self, points):
        """壁セグメントの検出（WallDetectorを使用）"""
        if not self.detect_walls_enabled:
            self.wall_segments = []
            self.wall_detection_time = 0.0
            return []
        
        # 処理時間計測開始
        start_time = time.time()
        
        # WallDetectorを使用して壁を検出
        self.wall_segments = self.wall_detector.detect(points)
        
        # 処理時間を記録（ミリ秒）
        self.wall_detection_time = (time.time() - start_time) * 1000
        
        logger.debug(f"Detected {len(self.wall_segments)} wall segments in {self.wall_detection_time:.1f}ms")
        
        return self.wall_segments
    
    def set_detection_method(self, method_name: str):
        """検出手法を変更"""
        global current_detection_method
        try:
            method = DetectionMethod(method_name)
            self.wall_detector.set_method(method)
            current_detection_method = method_name  # グローバル変数を更新
            logger.info(f"Detection method changed to: {method_name}")
            return True
        except ValueError:
            logger.error(f"Invalid detection method: {method_name}")
            return False
    
    def update_detector_parameters(self, params: dict):
        """検出器のパラメータを更新"""
        self.wall_detector.set_parameters(**params)
        logger.info(f"Detector parameters updated: {params}")
    
    def get_image(self):
        """最新のLiDAR画像を取得"""
        return self.latest_image
            
    def _calculate_median(self, points, zone_indices):
        """
        指定されたゾーンの有効な距離の中央値を計算
        
        Args:
            points: LiDARの測定値配列
            zone_indices: 対象ゾーンのインデックス配列
            
        Returns:
            float: 中央値（有効な値がない場合は0.0）
        """
        valid_distances = []
        for i in zone_indices:
            if i < len(points) and points[i] > self.min_distance:
                valid_distances.append(points[i])
        
        if valid_distances:
            valid_distances.sort()
            n = len(valid_distances)
            if n % 2 == 0:
                return (valid_distances[n//2-1] + valid_distances[n//2]) / 2
            else:
                return valid_distances[n//2]
        return 0.0
        
    def get_zone_all_distances(self, points, zone_index):
        """
        各ゾーンの全ての有効な距離データを取得
        
        Args:
            points: LiDARの測定値配列
            zone_index: ゾーンインデックス配列
            
        Returns:
            list: 各ゾーンの距離配列のリスト [zone0_distances, zone1_distances, ...]
        """
        zone_distances = []
        points_array = np.array(points)
        
        for zone in range(len(zone_index)):
            if zone < len(zone_index) and zone_index[zone]:
                # ゾーン内のインデックスを一度に処理
                zone_indices = np.array(zone_index[zone])
                valid_mask = (zone_indices >= 0) & (zone_indices < len(points_array))
                
                if np.any(valid_mask):
                    valid_indices = zone_indices[valid_mask]
                    zone_points = points_array[valid_indices]
                    
                    # min_distance以上の有効な距離のみを抽出
                    valid_distances = zone_points[zone_points >= self.min_distance]
                    zone_distances.append(valid_distances.tolist())
                else:
                    zone_distances.append([])
            else:
                zone_distances.append([])
        
        return zone_distances

    def calculate_zone_detection_distances(self, points, zone_index, detect_points_threshold, max_distance, zone_thresholds=None, distance_thresholds=None):
        """
        各ゾーンの検知距離・バイナリベクトル・全点中央値を一括計算する統合処理

        Args:
            points: LiDARの測定値配列
            zone_index: ゾーンインデックス配列
            detect_points_threshold: 検知閾値（zone_thresholdsが指定されていない場合のデフォルト）
            max_distance: 最大距離
            zone_thresholds: ゾーン別検知閾値のリスト（オプション）
            distance_thresholds: ゾーン別距離閾値のリスト（オプション、mm単位）

        Returns:
            tuple: (detection_distances, detection_binary, zone_median_distances)
        """
        detection_distances = []
        detection_binary = []
        zone_median_distances = []

        points_array = np.array(points)

        for zone in range(len(zone_index)):
            if zone < len(zone_index) and zone_index[zone]:
                # ゾーン内のインデックスを一度に処理
                zone_indices = np.array(zone_index[zone])
                valid_mask = (zone_indices >= 0) & (zone_indices < len(points_array))

                if np.any(valid_mask):
                    valid_indices = zone_indices[valid_mask]
                    zone_distances = points_array[valid_indices]

                    # 検知用: wall_distance以内の点でmedian（障害物検出用）
                    detection_mask = (zone_distances > self.min_distance) & (zone_distances < self.wall_distance)
                    detection_count = np.sum(detection_mask)
                    if detection_count > 0:
                        median_distance = int(np.median(zone_distances[detection_mask]))
                    else:
                        median_distance = max_distance

                    # 全点median: min_distance以上の全点でmedian（ゾーン距離用）
                    all_valid = zone_distances[zone_distances >= self.min_distance]
                    zone_median = int(np.median(all_valid)) if len(all_valid) > 0 else 0
                else:
                    detection_count = 0
                    median_distance = max_distance
                    zone_median = 0
            else:
                detection_count = 0
                median_distance = max_distance
                zone_median = 0

            detection_distances.append(median_distance)
            zone_median_distances.append(zone_median)

            # ゾーン別点数閾値を使用（指定されていない場合はデフォルトのdetect_points_thresholdを使用）
            if zone_thresholds and zone < len(zone_thresholds):
                points_threshold = zone_thresholds[zone]
            else:
                points_threshold = detect_points_threshold

            # ゾーン別距離閾値を使用
            if distance_thresholds and isinstance(distance_thresholds, (list, tuple)) and zone < len(distance_thresholds):
                distance_threshold = distance_thresholds[zone]
            elif distance_thresholds and isinstance(distance_thresholds, (int, float)):
                distance_threshold = distance_thresholds  # 単一値の場合
            else:
                distance_threshold = 300  # デフォルト値

            # 検出判定：点数閾値以上 AND 距離が閾値以内
            points_detected = detection_count >= points_threshold
            distance_detected = median_distance > 0 and median_distance <= distance_threshold
            detected = points_detected and distance_detected

            detection_binary.append(1 if detected else 0)

        # 検出があった場合のみログ出力
        #if any(detection_binary):
        #    print(f"detection={detection_binary} | {detection_distances}")

        return detection_distances, detection_binary, zone_median_distances
    
    def detect(self, points):
        """
        障害物検出の共通処理 - サブクラスで単位変換が必要な場合はオーバーライド
        
        Args:
            points: LiDARの測定値配列
            
        Returns:
            tuple: (各ゾーンの検知距離, バイナリベクトル, 各ゾーンの中央値距離)
        """
        # 統合処理で検知距離・バイナリベクトル・全点中央値を一括計算
        detection_distances, detection_binary, zone_distances = self.calculate_zone_detection_distances(
            points, self.zone_index, self.detect_points_threshold, self.max_distance,
            self.zone_thresholds, self.distance_thresholds
        )

        # インスタンス変数として保存（is_zone_clearメソッドで使用）
        self.detection_distances = detection_distances
        self.detection_binary = detection_binary
        self.zone_distances = zone_distances
        
        # detection_detailsを更新
        self.update_detection_details(points, detection_distances, detection_binary)
        
        return detection_distances, detection_binary, zone_distances
    
    def calculate_zone_median_distances(self, points):
        """
        各ゾーン内の全点の中央値を計算

        Args:
            points: LiDARの測定値配列

        Returns:
            list: 各ゾーンの中央値距離のリスト
        """
        zone_medians = []

        # pointsが空またはNoneの場合は全ゾーンを0.0で返す
        if points is None or len(points) == 0:
            return [0] * len(self.zone_index)

        points_array = np.array(points)

        for zone in range(len(self.zone_index)):
            if zone < len(self.zone_index) and self.zone_index[zone]:
                zone_indices = np.array(self.zone_index[zone])
                valid_mask = (zone_indices >= 0) & (zone_indices < len(points_array))

                if np.any(valid_mask):
                    valid_indices = zone_indices[valid_mask]
                    zone_points = points_array[valid_indices]

                    # min_distance以上の有効な距離のみで中央値を計算
                    valid_distances = zone_points[zone_points >= self.min_distance]
                    if len(valid_distances) > 0:
                        try:
                            median_distance = int(np.median(valid_distances))
                        except Exception as e:
                            logger.warning(f"Error calculating median for zone {zone}: {e}")
                            median_distance = 0
                    else:
                        median_distance = 0
                else:
                    median_distance = 0
            else:
                median_distance = 0

            zone_medians.append(median_distance)

        #print(zone_medians)
        return zone_medians
    
    def is_zone_clear(self, zone_index, distance_threshold=None):
        """
        指定ゾーンに指定距離内の障害物があるかチェック
        
        Args:
            zone_index: チェックするゾーンのインデックス（0-based）
            distance_threshold: 障害物検出の距離閾値（mm）。Noneの場合は設定値を使用
            
        Returns:
            bool: 障害物がない場合True（通行可能）
        """
        # zone_distancesが存在しない場合はTrueを返す
        if not hasattr(self, 'zone_distances') or not self.zone_distances:
            return True
        
        # 指定ゾーンの中央値距離を取得
        if zone_index < len(self.zone_distances):
            zone_distance = self.zone_distances[zone_index]
        else:
            return True
        
        # 距離閾値が指定されていない場合は、設定値を使用
        if distance_threshold is None:
            if hasattr(self, 'distance_thresholds') and zone_index < len(self.distance_thresholds):
                distance_threshold = self.distance_thresholds[zone_index]
            else:
                distance_threshold = self.detect_distance_threshold
        
        print(f"Zone {zone_index}: distance={zone_distance}, threshold={distance_threshold}")
        
        # 障害物がない、または閾値より遠い場合はTrue
        return zone_distance == 0 or zone_distance > distance_threshold
    
    def get_zone_status(self, distance_threshold_override=None):
        """
        全ゾーンの障害物状態を取得
        
        Args:
            distance_threshold_override: 全ゾーン共通の距離閾値オーバーライド（mm）
            
        Returns:
            dict: ゾーンインデックスをキー、クリア状態（bool）を値とする辞書
        """
        zone_status = {}
        
        if not hasattr(self, 'zone_index') or not self.zone_index:
            return zone_status
        
        for zone_idx in range(len(self.zone_index)):
            zone_status[zone_idx] = self.is_zone_clear(zone_idx, distance_threshold_override)
        
        return zone_status

    def update_detection_details(self, points, detection_distances, detection_binary):
        """
        detection_detailsを更新する
        
        Args:
            points: LiDARの測定値配列
            detection_distances: 各ゾーンの検知距離
            detection_binary: バイナリ検知ベクトル
        """
        try:
            zone_names = getattr(cfg, 'ZONE_NAMES', [f'Zone{i}' for i in range(len(self.zone_index))])
        except ImportError:
            zone_names = [f'Zone{i}' for i in range(len(self.zone_index))]
        
        # detection_detailsを初期化
        if not hasattr(self, 'detection_details'):
            self.detection_details = []
        
        self.detection_details = []
        
        for zone in range(len(self.zone_index)):
            zone_name = zone_names[zone] if zone < len(zone_names) else f'Zone{zone}'
            median_distance = detection_distances[zone] if zone < len(detection_distances) else 0.0
            detected = detection_binary[zone] if zone < len(detection_binary) else 0
            
            # ゾーン内の距離データを収集
            zone_points = []
            zone_distances = []
            detection_range_distances = []  # 検出範囲内の距離のみ
            
            if zone < len(self.zone_index) and self.zone_index[zone]:
                for idx in self.zone_index[zone]:
                    if 0 <= idx < len(points) and points[idx] > 0:
                        distance = points[idx]
                        if self.min_distance < distance < self.max_distance:
                            zone_points.append(idx)
                            zone_distances.append(distance)
                            # 検出範囲内の距離も別途保存
                            if distance < self.wall_distance:
                                detection_range_distances.append(distance)
            
            zone_detail = {
                'name': zone_name,
                'median_distance': median_distance,
                'detected': bool(detected),
                'total_points': len(zone_points),
                'detection_range_points': len(detection_range_distances),  # 検出範囲内点数を追加
                'distances': zone_distances[:20] if zone_distances else [],  # より多くの距離データを送信
                'detection_distances': detection_range_distances[:20] if detection_range_distances else []  # 検出範囲内の距離を送信
            }
            ### numpy計算修正前
            
            self.detection_details.append(zone_detail)

# グローバル変数
current_lidar_data = []
current_detection_binary = []
current_detection_method = DetectionMethod.DISTANCE_BASED.value

class WallSegment:
    """壁セグメントのデータクラス"""
    def __init__(self, points, start, end, angle, length, linearity):
        self.points = points  # セグメントを構成する点
        self.start = start    # 開始点 (x, y)
        self.end = end        # 終了点 (x, y)
        self.angle = angle    # 線分の角度（ラジアン）
        self.length = length  # 線分の長さ
        self.linearity = linearity  # 直線性（低いほど良い直線）
        
    def to_dict(self):
        """JSONシリアライズ用の辞書に変換"""
        return {
            'start': {'x': float(self.start[0]), 'y': float(self.start[1])},
            'end': {'x': float(self.end[0]), 'y': float(self.end[1])},
            'angle': float(self.angle),
            'length': float(self.length),
            'linearity': float(self.linearity),
            'points': [{'x': float(p[0]), 'y': float(p[1])} for p in self.points]
        }

class LidarImageConverter:
    """LiDARポイントクラウドから画像を生成するクラス"""
    
    def __init__(self, image_w=224, image_h=224, max_distance=20000, min_distance=20, 
                 angle_start=-135, angle_end=135, angle_offset=0, clockwise=False, binary_mode=False,
                 scale_factor=0.8, meters_per_pixel=0.045, vehicle_width=200, vehicle_length=450,
                 vehicle_color=(255, 255, 255), vehicle_thickness=2, lidar_offset_x=0, lidar_offset_y=0):
        """
        初期化
        
        Args:
            image_w: 出力画像の幅
            image_h: 出力画像の高さ
            max_distance: LiDAR測定の最大距離 (mm)
            min_distance: LiDAR測定の最小距離 (mm)
            angle_start: 開始角度 (度)
            angle_end: 終了角度 (度)
            angle_offset: 角度オフセット (度)
            clockwise: スキャン方向 (True:時計回り、False:反時計回り)
            binary_mode: 白黒2値画像モード (True:有効、False:無効)
            scale_factor: 画像サイズに対するスケール係数 (0.0-1.0)
            meters_per_pixel: 1ピクセルあたりの実際の距離（メートル）
            vehicle_width: 車両の幅 (mm)
            vehicle_length: 車両の長さ (mm)
            vehicle_color: 車両表示色 (RGB)
            vehicle_thickness: 車両枠線の太さ (ピクセル)
            lidar_offset_x: LiDAR搭載位置のX軸オフセット (mm、右が正)
            lidar_offset_y: LiDAR搭載位置のY軸オフセット (mm、前が正)
        """
        self.image_w = image_w
        self.image_h = image_h
        self.max_distance = max_distance
        self.min_distance = min_distance
        self.angle_start = angle_start
        self.angle_end = angle_end
        self.angle_offset = angle_offset
        self.clockwise = clockwise
        self.binary_mode = binary_mode
        self.scale_factor = scale_factor
        self.meters_per_pixel = meters_per_pixel
        self.vehicle_width = vehicle_width
        self.vehicle_length = vehicle_length
        self.vehicle_color = vehicle_color
        self.vehicle_thickness = vehicle_thickness
        
        # LiDAR搭載位置オフセット（mm単位）
        # 車両中心を原点として、前方が正のY、右が正のX
        self.lidar_offset_x = lidar_offset_x
        self.lidar_offset_y = lidar_offset_y
        
        # 画像の中心座標（LiDARセンサーの位置）
        self.center_x = image_w // 2
        self.center_y = image_h // 2
        
        # スケーリング係数（LiDAR距離をピクセルに変換）
        # meters_per_pixelは1ピクセルあたりのメートル数
        # 1ピクセルあたり0.045mなら、1m = 1/0.045 = 22.22ピクセル
        # scale_factorで調整（0.8なら画像端まで使わずに余白を残す）
        base_scale = 1.0 / (self.meters_per_pixel * 1000)  # mm単位に変換
        self.scale = base_scale * self.scale_factor
    
    def update_parameters(self, **kwargs):
        """パラメータを動的に更新
        
        Args:
            **kwargs: 更新するパラメータ（キーワード引数）
        """
        # 更新可能なパラメータリスト
        updatable_params = [
            'angle_start', 'angle_end', 'angle_offset', 'clockwise',
            'binary_mode', 'scale_factor', 'meters_per_pixel',
            'vehicle_width', 'vehicle_length', 'vehicle_color', 
            'vehicle_thickness', 'lidar_offset_x', 'lidar_offset_y'
        ]
        
        # パラメータを更新
        for param, value in kwargs.items():
            if param in updatable_params and hasattr(self, param):
                setattr(self, param, value)
        
        # スケールの再計算が必要な場合
        if 'scale_factor' in kwargs or 'meters_per_pixel' in kwargs:
            base_scale = 1.0 / (self.meters_per_pixel * 1000)  # mm単位に変換
            self.scale = base_scale * self.scale_factor
        
    def points_to_image(self, points, draw_wall_segments=False, wall_segments=None):
        """
        LiDARポイントクラウドをRGB画像に変換（高速化版）
        wall_segmentsは辞書形式のリストに対応
        """
        # RGB画像を初期化 (黒背景)
        image = np.zeros((self.image_h, self.image_w, 3), dtype=np.uint8)
        
        # 点がなければ、空の画像を返す
        if len(points) == 0:
            return image
        
        # LiDARの角度設定を使用
        angle_start = self.angle_start * np.pi / 180
        angle_end = self.angle_end * np.pi / 180
        
        # clockwise設定に基づいて角度配列を生成（get_lidar_dataと同じロジック）
        if self.clockwise:
            # 時計回り：角度を降順で生成
            angles = np.linspace(angle_end, angle_start, len(points))
        else:
            # 反時計回り：角度を昇順で生成
            angles = np.linspace(angle_start, angle_end, len(points))
        
        # 角度オフセットを適用
        angle_offset_rad = self.angle_offset * np.pi / 180
        angles = angles + angle_offset_rad
        
        # ベクトル化による高速処理
        points_array = np.array(points)
        
        # 有効距離の点のみフィルタリング
        valid_mask = (points_array > self.min_distance) & (points_array < self.max_distance)
        
        if np.any(valid_mask):
            valid_distances = points_array[valid_mask]
            valid_angles = angles[valid_mask]
            
            # 座標変換をベクトル化
            x_coords = (self.center_x + valid_distances * np.cos(valid_angles) * self.scale).astype(int)
            y_coords = (self.center_y - valid_distances * np.sin(valid_angles) * self.scale).astype(int)
            
            # 画像範囲内チェック
            in_bounds = (x_coords >= 0) & (x_coords < self.image_w) & (y_coords >= 0) & (y_coords < self.image_h)
            
            if np.any(in_bounds):
                x_final = x_coords[in_bounds]
                y_final = y_coords[in_bounds]
                distances_final = valid_distances[in_bounds]
                
                if self.binary_mode:
                    # 2値画像モード：全て白
                    image[y_final, x_final] = [255, 255, 255]
                else:
                    # 距離に応じた色をベクトル化で計算
                    colors = self._get_colors_by_distance_vectorized(distances_final)
                    image[y_final, x_final] = colors
        
        # 壁セグメントを描画（辞書形式に対応）
        if draw_wall_segments and wall_segments:
            for segment in wall_segments:
                # 辞書形式のセグメントから座標を取得（mm単位）
                # get_lidar_dataと同じ座標変換
                start_x = int(self.center_x + segment['start']['x'] * self.scale)
                end_x = int(self.center_x + segment['end']['x'] * self.scale)
                start_y = int(self.center_y - segment['start']['y'] * self.scale)
                end_y = int(self.center_y - segment['end']['y'] * self.scale)
                
                # 線を描画
                cv2.line(image, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
        
        # 車両を描画
        self._draw_vehicle(image)
        
        # LiDARセンサーの位置を描画
        # cv2.circle(image, (self.center_x, self.center_y), 3, (0, 0, 255), -1)
        
        return image
    
    def _draw_vehicle(self, image):
        """車両を長方形で描画（LiDARオフセットを考慮）"""
        # 車両のサイズをピクセルに変換
        vehicle_width_px = int(self.vehicle_width * self.scale)
        vehicle_length_px = int(self.vehicle_length * self.scale)
        
        # LiDARオフセットをピクセルに変換
        offset_x_px = int(self.lidar_offset_x * self.scale)
        offset_y_px = int(self.lidar_offset_y * self.scale)
        
        # 車両中心の座標を計算（LiDARセンサーからのオフセットを考慮）
        # 画像座標系では上が負のY方向なので、オフセットのY座標を反転
        vehicle_center_x = self.center_x - offset_x_px  # LiDARから見た車両中心のX座標
        vehicle_center_y = self.center_y + offset_y_px  # LiDARから見た車両中心のY座標（画像座標系）
        
        # 車両の角の座標を計算
        left = vehicle_center_x - vehicle_width_px // 2
        right = vehicle_center_x + vehicle_width_px // 2
        top = vehicle_center_y - vehicle_length_px // 2
        bottom = vehicle_center_y + vehicle_length_px // 2
        
        # 長方形を塗りつぶしで描画
        cv2.rectangle(image, (left, top), (right, bottom), 
                     self.vehicle_color, -1)
        
        # 車両の前方を示す線を描画（車両中心から前方へ）
        front_center_x = vehicle_center_x
        front_center_y = top
        cv2.line(image, (vehicle_center_x, vehicle_center_y), 
                (front_center_x, front_center_y), 
                self.vehicle_color, max(1, self.vehicle_thickness // 2))
    
    def _draw_grid(self, image):
        """グリッドと距離マーカーを描画"""
        # グリッドの色
        grid_color = (20, 20, 20)  # 暗めのグレー
        
        # グリッド線を描画（1000mmごと）
        grid_step = int(1000 * self.scale)  # 1000mm = 1m
        
        # 水平線と垂直線
        for i in range(1000, int(self.max_distance) + 1, 1000):  # 1000mmずつ
            offset = int(i * self.scale)
            
            # 水平線
            cv2.line(image, 
                    (self.center_x - offset, 0), 
                    (self.center_x - offset, self.image_h-1), 
                    grid_color, 1)
            cv2.line(image, 
                    (self.center_x + offset, 0), 
                    (self.center_x + offset, self.image_h-1), 
                    grid_color, 1)
            
            # 垂直線
            cv2.line(image, 
                    (0, self.center_y - offset), 
                    (self.image_w-1, self.center_y - offset), 
                    grid_color, 1)
            cv2.line(image, 
                    (0, self.center_y + offset), 
                    (self.image_w-1, self.center_y + offset), 
                    grid_color, 1)
            
            # 距離マーカー（同心円）
            cv2.circle(image, (self.center_x, self.center_y), offset, grid_color, 1)
        
        return image
    
    def _get_colors_by_distance_vectorized(self, distances_mm):
        """
        距離配列に応じた色配列を返す（ベクトル化版）
        """
        colors = np.zeros((len(distances_mm), 3), dtype=np.uint8)
        
        # 近い距離 (< 1000mm) - 赤色
        near_mask = distances_mm < 1000
        if np.any(near_mask):
            green_vals = (distances_mm[near_mask] / 1000 * 200).astype(int)
            colors[near_mask] = np.column_stack([np.full_like(green_vals, 255), green_vals, np.zeros_like(green_vals)])
        
        # 中間距離 (1000-5000mm) - 黄色から緑へ
        mid_mask = (distances_mm >= 1000) & (distances_mm < 5000)
        if np.any(mid_mask):
            ratio = (distances_mm[mid_mask] - 1000) / 4000
            red_vals = (255 * (1 - ratio)).astype(int)
            blue_vals = (200 * ratio).astype(int)
            colors[mid_mask] = np.column_stack([red_vals, np.full_like(red_vals, 255), blue_vals])
        
        # 遠い距離 (>= 5000mm) - 青色
        far_mask = distances_mm >= 5000
        if np.any(far_mask):
            intensity = np.minimum(1, (self.max_distance - distances_mm[far_mask]) / 10000.0)
            blue_vals = (255 * intensity).astype(int)
            colors[far_mask] = np.column_stack([np.zeros_like(blue_vals), np.zeros_like(blue_vals), blue_vals])
        
        return colors

    def _get_color_by_distance(self, distance_mm):
        """
        距離に応じた色を返す（mm単位）
        近い：赤色
        中間：黄色から緑へ
        遠い：青色のグラデーション
        """
        if distance_mm < 1000:  # 1000mm = 1m
            # 近い：赤色
            green = int(distance_mm / 1000 * 200)
            return (0, green, 255)  # BGR形式
        elif distance_mm < 5000:  # 5000mm = 5m
            # 中間：黄色から緑へ
            ratio = (distance_mm - 1000) / 4000
            red = int(255 * (1 - ratio))
            blue = int(200 * ratio)
            return (blue, 255, red)  # BGR形式
        else:
            # 遠い：青色のグラデーション
            intensity = min(1, (self.max_distance - distance_mm) / 10000.0)
            green = int(100 * intensity)
            red = 0
            blue = int(200 * intensity + 55)
            return (blue, green, red)  # BGR形式

class TMINI(LidarBase):
    '''
    YDLidar TMINI driver for DonkeyCar
    Serial connection, 400 points, 360 degree scan
    Range: 20-12000mm
    '''
    
    def __init__(self, zone_index=None, image_w=None, image_h=None, **kwargs):
        super().__init__(image_w, image_h)
        
        logger.info(f"TMINI initialized with image size: {self.image_w}x{self.image_h}")
        
        # TMINI固有のパラメータ
        if zone_index is None:
            zone_index = getattr(cfg, 'ZONE_INDEX', [])
        self.zone_index = zone_index
        
        # TMINI固有のデータ点数
        self.no_points = 400 - 3  # no scan point bug対応
        self.angle_range = 360  # 度
        
        # マルチプロセス用の共有メモリ初期化
        self.on = Value('b', 1)
        self.points = Array('f', self.no_points)
        
        # マルチプロセス開始
        self.p = Process(target=self.multiprocess, args=(self.points,))
        self.p.start()
        
        # update用スレッドを開始（Web表示用）
        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.daemon = True
        self.update_thread.start()
    
    def multiprocess(self, points):
        """マルチプロセスでTMINI通信を実行"""
        # 子プロセスでシグナルハンドラをデフォルトに戻す（親プロセスとの競合を回避）
        sig.signal(sig.SIGINT, sig.SIG_DFL)
        sig.signal(sig.SIGTERM, sig.SIG_DFL)

        try:
            import ydlidar
            ydlidar.os_init()
            port = "/dev/ttyAMA0"
            # port = "/dev/ydlidar"
            # for key, value in ports.items():
            #     port = value
                #print(port)
                            
            self.laser = ydlidar.CYdLidar()
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, float(getattr(cfg, 'LIDAR_SCAN_RATE', 10)))
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
            self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
            self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, getattr(cfg, 'LIDAR_MAX_DISTANCE', 4000) / 1000.0)
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, getattr(cfg, 'LIDAR_MIN_DISTANCE', 20) / 1000.0)
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

            if self.on.value:
                self.on.value = self.laser.initialize()
                if self.on.value:
                    self.on.value = self.laser.turnOn()
                    scan = ydlidar.LaserScan()
                    
                    while self.on.value and ydlidar.os_isOk():
                        try:
                            r = self.laser.doProcessSimple(scan)
                            if r:
                                # 生データを取得
                                raw_points = []
                                for i in range(min(len(scan.points), self.no_points)):
                                    try:
                                        raw_points.append(scan.points[i].range)
                                    except:
                                        raw_points.append(4.0)  # max_distance
                                
                                # 単位変換を適用
                                converted_points = self._convert_units(raw_points)

                                # 共有メモリに格納
                                for i, point in enumerate(converted_points):
                                    points[i] = point
                            else:
                                logger.debug("Failed to get Lidar Data.")

                        except KeyboardInterrupt:
                            self.on.value = False
                            break
                        except Exception as e:
                            logger.error(f"TMINI data acquisition error: {e}")
                            time.sleep(0.1)

                    # LiDARをシャットダウン
                    try:
                        self.laser.turnOff()
                    except OSError as e:
                        # シグナルハンドラとの競合によるエラーを無視
                        if "Signal 15 ignored" in str(e) or "race condition" in str(e):
                            logger.debug(f"Ignoring expected shutdown error: {e}")
                        else:
                            logger.error(f"TMINI turnOff error: {e}")
                    except Exception as e:
                        logger.error(f"TMINI turnOff error: {e}")

            # 接続を切断
            try:
                self.laser.disconnecting()
            except Exception as e:
                logger.debug(f"TMINI disconnecting error (ignored): {e}")
            
        except ImportError:
            logger.error("ydlidar module not found. Please install YDLidar SDK.")
            self.on.value = False
        except Exception as e:
            logger.error(f"TMINI connection error: {e}")
            self.on.value = False


class HokuyoUST20(LidarBase):
    '''
    Hokuyo UST-20 LiDAR driver for DonkeyCar
    Ethernet connection, 1081 points, 270 degree scan
    Range: 20-20000mm
    '''
    
    def __init__(self, ip_address=None, port=None, image_w=None, image_h=None):
        # 2値画像モードの設定を先に行う
        self.binary_mode = getattr(cfg, 'LIDAR_BINARY_IMAGE', False)
        super().__init__(image_w, image_h)  # 基底クラスの初期化を呼び出す
        
        logger.info(f"HokuyoUST20 initialized with image size: {self.image_w}x{self.image_h}")
        
        # Hokuyo UST-20固有のパラメータ（設定ファイルから読み込む）
        self.ip_address = ip_address if ip_address is not None else getattr(cfg, 'LIDAR_IP_ADDRESS', '192.168.0.10')
        self.port = port if port is not None else getattr(cfg, 'LIDAR_PORT', 10940)
        
        # Hokuyo UST-20固有のデフォルト値で上書き（必要な場合）
        # 基底クラスで既に設定ファイルから読み込んでいるので、
        # 特別な値が必要な場合のみ上書き
        
        # Hokuyo UST-20固有のデータ点数
        self.no_points = getattr(cfg, 'LIDAR_DATA_POINTS', 1081)
        self.angle_range = getattr(cfg, 'LIDAR_ANGLE_RANGE', 270)  # 度
        
        
        
        # マルチプロセス用の共有メモリ初期化
        self.on = Value('b', 1)
        self.points = Array('f', self.no_points)
        
        # マルチプロセス開始
        self.p = Process(target=self.multiprocess, args=(self.points,))
        self.p.start()
        
        # update用スレッドを開始（Web表示用）
        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.daemon = True
        self.update_thread.start()
    



    

    def multiprocess(self, points):
        """マルチプロセスでUST-20通信を実行"""
        # 子プロセスでシグナルハンドラをデフォルトに戻す（親プロセスとの競合を回避）
        sig.signal(sig.SIGINT, sig.SIG_DFL)
        sig.signal(sig.SIGTERM, sig.SIG_DFL)

        socket_obj = None

        try:
            # ソケット接続
            socket_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            socket_obj.settimeout(10.0)
            socket_obj.connect((self.ip_address, self.port))
            logger.info(f"Connected to UST-20 at {self.ip_address}:{self.port}")
            
            # 測定開始
            self._send_command(socket_obj, 'BM')
            
            while self.on.value:
                try:
                    # GDコマンドでデータ取得
                    distances = self._get_gd_data(socket_obj)
                    
                    if distances and len(distances) > 0:
                        # 共有メモリに格納
                        for i in range(min(len(distances), self.no_points)):
                            points[i] = distances[i] if 10 <= distances[i] <= 20000 else 0
                        
                        # 残りの点を0で埋める
                        for i in range(len(distances), self.no_points):
                            points[i] = 0
                    
                    # スキャンレートに応じたスリープ時間
                    scan_rate = getattr(cfg, 'LIDAR_SCAN_RATE', 40)
                    if scan_rate > 0:
                        time.sleep(1.0 / scan_rate)
                    
                except Exception as e:
                    logger.error(f"Data acquisition error: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            logger.error(f"Connection error: {e}")
            self.on.value = False
            
        finally:
            if socket_obj:
                try:
                    self._send_command(socket_obj, 'QT')  # 測定停止
                    socket_obj.close()
                except:
                    pass
            logger.info("UST-20 connection closed")
    
    def _send_command(self, socket_obj, command):
        """コマンド送信"""
        try:
            cmd = command + '\n'
            socket_obj.send(cmd.encode('ascii'))
            
            # エコーバック読み込み
            echo = ""
            while True:
                char = socket_obj.recv(1).decode('ascii')
                if char == '\n':
                    break
                echo += char
            
            # レスポンス読み込み
            response = ""
            while True:
                char = socket_obj.recv(1).decode('ascii')
                response += char
                if response.endswith('\n\n'):
                    break
                if len(response) > 50000:
                    break
            
            return response.strip()
        except Exception as e:
            logger.error(f"Command send error: {e}")
            return None
    
    def _extract_data_from_response(self, raw_response):
        """レスポンスからデータ部分を抽出"""
        lines = raw_response.split('\n')
        
        # ステータス行をスキップ
        data_lines = []
        timestamp = None
        
        for i, line in enumerate(lines[1:], 1):
            if not line:
                continue
            
            # タイムスタンプ行の検出
            if i == 1 and (len(line) <= 10 or any(c in line for c in ['?', '<', "'"])):
                timestamp = line
                continue
            
            # データ行
            data_lines.append(line)
        
        if not data_lines:
            return "", timestamp
        
        # 64バイト + チェックサム処理
        extracted_data = ""
        
        for line in data_lines:
            if len(line) >= 64:
                # データ部分（最初の64文字）を抽出
                data_part = line[:64]
                extracted_data += data_part
            else:
                # 64文字未満の場合はそのまま追加
                extracted_data += line
        
        return extracted_data, timestamp
    
    def _decode_distances(self, clean_data):
        """距離データをデコード"""
        distances = []
        
        # 3文字ずつ処理
        for i in range(0, len(clean_data), 3):
            if i + 2 >= len(clean_data):
                break
            
            try:
                char1 = clean_data[i]
                char2 = clean_data[i + 1]
                char3 = clean_data[i + 2]
                
                # ASCIIコードから数値変換（30hを引く）
                val1 = ord(char1) - 0x30
                val2 = ord(char2) - 0x30
                val3 = ord(char3) - 0x30
                
                # 6ビット値の範囲チェック
                if 0 <= val1 <= 63 and 0 <= val2 <= 63 and 0 <= val3 <= 63:
                    # 18ビット値に復元
                    distance = (val1 << 12) | (val2 << 6) | val3
                    distances.append(distance)
                else:
                    distances.append(0)
            except Exception:
                distances.append(0)
        
        return distances
    
    def _get_gd_data(self, socket_obj):
        """GD0000108000コマンドでデータ取得"""
        response = self._send_command(socket_obj, 'GD0000108000')
        if not response:
            return None
        
        # レスポンス解析
        lines = response.split('\n')
        status_line = lines[0] if lines else ""
        
        if not status_line.startswith('00'):
            return None
        
        # データ部分抽出
        clean_data, timestamp = self._extract_data_from_response(response)
        
        if not clean_data:
            return None
        
        # 距離データのデコード
        distances = self._decode_distances(clean_data)
        
        return distances

def convert_numpy_to_json(obj):
    """numpy配列をJSON対応形式に変換するヘルパー関数"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.bool_):
        return bool(obj)
    elif isinstance(obj, dict):
        return {key: convert_numpy_to_json(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_to_json(item) for item in obj]
    else:
        return obj

# Flask関連の関数（tmini.pyと同じ）
def create_flask_app(lidar_instance):
    """Flaskアプリを作成する関数"""
    app = Flask(__name__)
    
    @app.route('/')
    def index():
        # config値をテンプレートに渡す
        lidar_config = {
            'type': cfg.LIDAR_TYPE,
            'min_distance': cfg.LIDAR_MIN_DISTANCE,
            'max_distance': cfg.LIDAR_MAX_DISTANCE,
            'angle_range': getattr(cfg, 'LIDAR_ANGLE_RANGE', 270),
            'data_points': getattr(cfg, 'LIDAR_DATA_POINTS', 1081),
            'scan_rate': getattr(cfg, 'LIDAR_SCAN_RATE', 40),
            'zone_distance_thresholds': getattr(cfg, 'LIDAR_DETECT_DISTANCE_THRESHOLD_ZONE', [300, 300, 400, 300, 300])
        }
        return render_template('lidar_visualizer.html', lidar_config=lidar_config)
        
    @app.route('/lidar_data')
    def get_lidar_data():
        """現在のLiDARデータをJSON形式で返す"""
        global current_lidar_data, current_detection_binary, current_detection_method
        
        points = []
        
        if current_lidar_data is not None and len(current_lidar_data) > 0:
            angle_start = lidar_instance.angle_start * np.pi / 180
            angle_end = lidar_instance.angle_end * np.pi / 180
            
            # clockwise設定に基づいて角度配列を生成
            if lidar_instance.clockwise:
                # 時計回り：角度を降順で生成
                angles = np.linspace(angle_end, angle_start, len(current_lidar_data))
            else:
                # 反時計回り：角度を昇順で生成
                angles = np.linspace(angle_start, angle_end, len(current_lidar_data))
            
            angle_offset_rad = lidar_instance.angle_offset * np.pi / 180
            angles = angles + angle_offset_rad
            
            for i, (angle, range_mm) in enumerate(zip(angles, current_lidar_data)):
                # range_mmはすでにmm単位
                
                if range_mm > 20 and range_mm < 20000:  # 20mm to 20000mm
                    is_near_lidar = range_mm <= lidar_instance.min_distance
                    is_ignored = range_mm <= lidar_instance.ignore_distance

                    points.append({
                        'x': float(range_mm * np.cos(angle)),
                        'y': float(range_mm * np.sin(angle)),
                        'range': float(range_mm),
                        'angle': float(angle),
                        'is_near_lidar': is_near_lidar,
                        'is_ignored': is_ignored
                    })
        
        # 検出詳細情報を含める
        detection_details = []
        if hasattr(lidar_instance, 'detection_details'):
            detection_details = convert_numpy_to_json(lidar_instance.detection_details)

        # configからゾーン情報を取得して角度に変換
        zone_angles = []
        zone_names = getattr(cfg, 'ZONE_NAMES', ['Zone0', 'Zone1', 'Zone2', 'Zone3'])
        
        if hasattr(cfg, 'ZONE_INDEX') and cfg.ZONE_INDEX:
            # LiDARの設定を取得
            angle_start = getattr(cfg, 'LIDAR_ANGLE_START', -135)
            angle_end = getattr(cfg, 'LIDAR_ANGLE_END', 135)
            total_points = getattr(cfg, 'LIDAR_DATA_POINTS', 1081)
            angle_range = abs(angle_end - angle_start)
            
            # 1点あたりの角度を計算
            angle_per_point = angle_range / (total_points - 1)
            
            for zone_idx in range(len(cfg.ZONE_INDEX)):
                zone_indices = cfg.ZONE_INDEX[zone_idx]
                if zone_indices:
                    # インデックスから実際の角度を計算
                    # ラップアラウンド（配列の最初と最後をまたぐ）ゾーンを検出
                    sorted_indices = sorted(zone_indices)
                    
                    # 隣接するインデックス間の最大ギャップを見つける
                    max_gap = 0
                    gap_start_idx = 0
                    
                    for i in range(len(sorted_indices)):
                        current_idx = sorted_indices[i]
                        next_idx = sorted_indices[(i + 1) % len(sorted_indices)]
                        
                        # ラップアラウンドを考慮したギャップ計算
                        if next_idx < current_idx:  # ラップアラウンド
                            gap = (cfg.LIDAR_DATA_POINTS - current_idx) + next_idx
                        else:
                            gap = next_idx - current_idx
                        
                        if gap > max_gap:
                            max_gap = gap
                            gap_start_idx = (i + 1) % len(sorted_indices)
                    
                    # 最大ギャップがあるかチェック（ラップアラウンドゾーンの判定）
                    if max_gap > len(sorted_indices):
                        # ラップアラウンドゾーン: ギャップの終端から開始端まで
                        end_idx = sorted_indices[gap_start_idx - 1] if gap_start_idx > 0 else sorted_indices[-1]
                        start_idx = sorted_indices[gap_start_idx]
                        
                        start_angle = angle_start + start_idx * angle_per_point
                        end_angle = angle_start + end_idx * angle_per_point
                        
                        # 時計回りスキャンの場合は角度の正負を反転
                        if cfg.LIDAR_CLOCKWISE:
                            start_angle = -start_angle
                            end_angle = -end_angle
                            # 時計回りでは開始と終了の角度が逆になる
                            start_angle, end_angle = end_angle, start_angle
                        
                        # 角度がラップアラウンドする場合の調整
                        if start_angle > end_angle:
                            end_angle += 360.0
                    else:
                        # 通常のゾーン
                        min_idx = min(zone_indices)
                        max_idx = max(zone_indices)
                        
                        start_angle = angle_start + min_idx * angle_per_point
                        end_angle = angle_start + max_idx * angle_per_point
                        
                        # 時計回りスキャンの場合は角度の正負を反転
                        if cfg.LIDAR_CLOCKWISE:
                            start_angle = -start_angle
                            end_angle = -end_angle
                            # 時計回りでは開始と終了の角度が逆になる
                            start_angle, end_angle = end_angle, start_angle
                    
                    zone_angles.append({
                        'zone_id': zone_idx,
                        'start_angle': start_angle,
                        'end_angle': end_angle,
                        'name': zone_names[zone_idx] if zone_idx < len(zone_names) else f'Zone{zone_idx}'
                    })
                    #print(zone_angles)
        
        json_data = {
            'points': points,
            'wall_segments': convert_numpy_to_json(lidar_instance.wall_segments),
            'angle_offset': lidar_instance.angle_offset,
            'detection_method': current_detection_method,
            'walls_enabled': lidar_instance.detect_walls_enabled,
            'detection_binary': convert_numpy_to_json(current_detection_binary),
            'detection_details': convert_numpy_to_json(detection_details),
            'zone_angles': zone_angles,
            'ignore_distance': lidar_instance.ignore_distance,  # 除外距離を追加
            'timestamp': time.time(),  # タイムスタンプを追加してキャッシュ回避
            'data_valid': len(points) > 0  # データの有効性フラグ
        }
        
        # Apply convert_numpy_to_json to the entire dictionary to catch any remaining numpy types
        json_data = convert_numpy_to_json(json_data)
        
        response = jsonify(json_data)
        # キャッシュ無効化ヘッダーを追加
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        
        return response

    @app.route('/set_detection_method', methods=['POST'])
    def set_detection_method():
        """検出手法を変更"""
        data = request.json
        method = data.get('method')
        
        if method:
            success = lidar_instance.set_detection_method(method)
            return jsonify({'success': success, 'method': method})
        
        return jsonify({'success': False, 'error': 'No method specified'})
    
    @app.route('/toggle_wall_detection', methods=['POST'])
    def toggle_wall_detection():
        """壁検出の有効/無効を切り替え"""
        lidar_instance.detect_walls_enabled = not lidar_instance.detect_walls_enabled
        return jsonify({
            'enabled': lidar_instance.detect_walls_enabled
        })


    @app.route('/update_parameters', methods=['POST'])
    def update_parameters():
        """検出パラメータを更新"""
        params = request.json
        
        if params:
            lidar_instance.update_detector_parameters(params)
            return jsonify({'success': True, 'parameters': params})
        
        return jsonify({'success': False, 'error': 'No parameters provided'})
    
    @app.route('/get_detector_info')
    def get_detector_info():
        """現在の検出器情報を取得"""
        try:
            detector = lidar_instance.wall_detector
            detector_info = detector.get_detector_info()
            
            return jsonify({
                'method': detector.method.value,
                'parameters': detector_info['detector_parameters']
            })
        except Exception as e:
            return jsonify({
                'error': str(e),
                'method': 'unknown',
                'parameters': {}
            }), 500

    @app.route('/debug_detection')
    def debug_detection():
        """検出デバッグ情報を取得"""
        global current_lidar_data
        
        if current_lidar_data is None or len(current_lidar_data) == 0 or not lidar_instance.detect_walls_enabled:
            return jsonify({
                'enabled': False,
                'message': 'Wall detection is disabled or no data available'
            })
        
        try:
            # WallDetectorのデバッグ情報を取得
            detector = lidar_instance.wall_detector
            debug_info = {
                'enabled': True,
                'method': detector.method.value,
                'input_points': len(current_lidar_data),
                'valid_points': 0,
                'segments_found': len(lidar_instance.wall_segments),
                'processing_time_ms': lidar_instance.wall_detection_time,
                'parameters': detector.get_detector_info()['detector_parameters']
            }
            
            # 有効な点数をカウント
            ranges = np.array(current_lidar_data)  # すでにmm単位
            valid_indices = np.where((ranges > lidar_instance.min_distance) & 
                                   (ranges < lidar_instance.max_distance))[0]
            debug_info['valid_points'] = len(valid_indices)
            
            return jsonify(debug_info)
            
        except Exception as e:
            return jsonify({
                'enabled': False,
                'error': str(e)
            })

    @app.route('/lidar_stream.mjpeg')
    def lidar_mjpeg_stream():
        """LiDAR画像をMJPEGストリームとして提供"""
        def generate():
            while True:
                if lidar_instance.latest_image is None:
                    # 設定された画像サイズを使用
                    img_w = getattr(lidar_instance.image_converter, 'image_w', 224) if hasattr(lidar_instance, 'image_converter') else 224
                    img_h = getattr(lidar_instance.image_converter, 'image_h', 224) if hasattr(lidar_instance, 'image_converter') else 224
                    img_rgb = np.zeros((img_h, img_w, 3), dtype=np.uint8)
                    cv2.putText(img_rgb, "No LiDAR Data", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    # LidarImageConverterは既にRGB形式で画像を生成している
                    img_rgb = lidar_instance.latest_image
                img_pil = Image.fromarray(img_rgb)
                img_io = io.BytesIO()
                img_pil.save(img_io, 'JPEG', quality=70)
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + img_io.getvalue() + b'\r\n')
                
                time.sleep(0.05)
        
        return Response(generate(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    
    @app.route('/lidar_debug')
    def lidar_debug():
        """LiDARデバッグ情報を取得"""
        debug_info = {
            'running': lidar_instance.running,
            'measurements_count': len(lidar_instance.measurements) if lidar_instance.measurements else 0,
            'latest_image_exists': lidar_instance.latest_image is not None,
            'latest_image_shape': lidar_instance.latest_image.shape if lidar_instance.latest_image is not None else None,
            'image_converter_exists': hasattr(lidar_instance, 'image_converter'),
            'image_w': getattr(lidar_instance, 'image_w', 'not set'),
            'image_h': getattr(lidar_instance, 'image_h', 'not set'),
            'min_distance': lidar_instance.min_distance,
            'max_distance': lidar_instance.max_distance,
            'angle_start': lidar_instance.angle_start,
            'angle_end': lidar_instance.angle_end,
            'process_alive': lidar_instance.p.is_alive() if hasattr(lidar_instance, 'p') else False,
            'current_lidar_data_count': len(current_lidar_data) if current_lidar_data else 0,
            'sample_measurements': lidar_instance.measurements[:5] if lidar_instance.measurements else []
        }
        return jsonify(debug_info)

    return app

        
# メイン関数
def main():
    # templates/index.htmlのJinja2テンプレートを使用（Flask render_templateで処理）

    # ファクトリー関数を使ってLiDARインスタンスを作成
    ip_address = '192.168.0.139' #10
    print(f'ip address:{ip_address}でlidarとの通信を開始します...')
    lidar = create_lidar(lidar_type=cfg.LIDAR_TYPE, ip_address=ip_address, image_w=224, image_h=224)
    
    # Flaskアプリを初期化
    app = create_flask_app(lidar)
    
    # Flaskサーバーを別スレッドで起動（ログを無効化）
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8080, debug=False, threaded=True))
    flask_thread.daemon = True
    flask_thread.start()

    print("LiDAR ウェブインターフェイスが起動しました。")
    print("ブラウザで http://localhost:8080 にアクセスしてください。")
    print("LiDAR画像ストリーミング: http://localhost:8080/lidar_stream.mjpeg")

    # ブラウザを自動的に開く
    import webbrowser
    def open_browser():
        time.sleep(1.5)  # サーバー起動を待つ
        webbrowser.open('http://localhost:8080')
        print("ブラウザを自動的に開きました。")

    browser_thread = threading.Thread(target=open_browser)
    browser_thread.daemon = True
    browser_thread.start()
    
    # グローバル変数を更新するためのポーリングループ
    def update_globals():
        global current_lidar_data
        while True:
            if lidar.running:
                current_lidar_data = lidar.measurements
            time.sleep(0.1)
    
    # グローバル変数更新用スレッドを開始
    globals_thread = threading.Thread(target=update_globals)
    globals_thread.daemon = True
    globals_thread.start()
    
    try:
        while True:
            # 新しい返り値形式に対応
            result = lidar.run()
            if len(result) == 5:
                detection, detection_binary, wall_info, image, dist_array = result
            elif len(result) == 4:
                detection, detection_binary, wall_info, image = result
                dist_array = []
            else:
                # 互換性のためのフォールバック
                detection, wall_info, image = result[:3]
                detection_binary = []
                dist_array = []
                
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        lidar.shutdown()
        lidar.on.value = False
        time.sleep(1)
        print(' Stopping Lidar.')
        time.sleep(1)

class LidarScanSaver:
    """
    DonkeyCar part for saving detailed LiDAR scan data with angle, distance, and quality
    """
    def __init__(self, tub_path):
        self.tub_path = tub_path
        self.lidar_scan_dir = None
        self.counter = 0
        self.setup_directories()
        
    def setup_directories(self):
        """Create lidar/scan data directory"""
        if self.tub_path:
            self.lidar_scan_dir = os.path.join(self.tub_path, 'lidar', 'scan')
            if not os.path.exists(self.lidar_scan_dir):
                os.makedirs(self.lidar_scan_dir)
                logger.info(f"Created lidar scan directory: {self.lidar_scan_dir}")
    
    def run(self, recording, dist_array, num_records):
        """
        Save detailed lidar scan data when recording
        
        Args:
            recording: Whether we are currently recording
            dist_array: The raw lidar point measurements (all points)
            num_records: Current record number from TubWriter
        
        Returns:
            The filename of the saved lidar scan data (or None if not recording)
        """
        if not recording or not dist_array or self.lidar_scan_dir is None:
            return None
            
        # Use the TubWriter's record number to keep data synchronized
        self.counter = num_records - 1  # num_records is 1-based
        
        filename = f"{self.counter}_lidar_scan.npz"
        filepath = os.path.join(self.lidar_scan_dir, filename)
        
        try:
            # Create detailed scan data with angle, distance, and quality
            scan_data = self._create_scan_data(dist_array)
            
            # Save as compressed numpy array with multiple fields
            np.savez_compressed(filepath, **scan_data)
            logger.debug(f"Saved lidar scan data to {filepath}")
            return filename
        except Exception as e:
            logger.error(f"Error saving lidar scan data: {e}")
            return None
    
    def _create_scan_data(self, raw_measurements):
        """Create detailed scan data structure from raw measurements"""
        # LiDAR specifications from config
        angle_start = getattr(cfg, 'LIDAR_ANGLE_START', -135.0)
        angle_end = getattr(cfg, 'LIDAR_ANGLE_END', 135.0)
        
        # 生の測定データから距離を取得
        distances = raw_measurements
        
        # 角度の増分を計算
        if len(distances) > 1:
            angle_increment = (angle_end - angle_start) / (len(distances) - 1)
        else:
            angle_increment = 0
        
        # clockwise設定を取得
        clockwise = getattr(cfg, 'LIDAR_CLOCKWISE', False)
        
        # 各ポイントの角度配列を作成
        if clockwise:
            # 時計回り：角度を降順で生成
            angles = np.linspace(angle_end, angle_start, len(distances), dtype=np.float32)
        else:
            # 反時計回り：角度を昇順で生成
            angles = np.linspace(angle_start, angle_end, len(distances), dtype=np.float32)
        
        # 距離配列を作成（mm単位のまま）
        distances_array = np.array(distances, dtype=np.float32)  # mm単位
        
        # 品質配列を作成 (0=無効, 1=有効)
        quality = np.where(distances_array > 20, 1, 0).astype(np.uint8)  # 20mm以上で有効
        
        # 強度配列を作成 (プレースホルダー - UST-20は強度を提供しない)
        intensity = np.full_like(distances_array, 0.0, dtype=np.float32)
        
        # タイムスタンプを作成
        import time
        timestamp = time.time()
        
        return {
            'header': {
                'timestamp': timestamp,
                'frame_id': 'lidar',
                'seq': self.counter
            },
            'angle_min': min(angle_start, angle_end),
            'angle_max': max(angle_start, angle_end),
            'angle_increment': angle_increment,
            'time_increment': 0.0,  # UST-20 takes snapshot, not scanning
            'scan_time': 1.0 / getattr(cfg, 'LIDAR_SCAN_RATE', 40) if getattr(cfg, 'LIDAR_SCAN_RATE', 40) > 0 else 0.025,
            'range_min': getattr(cfg, 'LIDAR_MIN_DISTANCE', 100),  # min range in mm
            'range_max': getattr(cfg, 'LIDAR_MAX_DISTANCE', 20000),  # max range in mm
            'angles': angles,
            'ranges': distances_array,
            'intensities': intensity,
            'quality': quality
        }


class LidarDataSaver:
    """
    Legacy LidarDataSaver for backward compatibility
    """
    def __init__(self, tub_path):
        self.tub_path = tub_path
        self.lidar_data_dir = None
        self.counter = 0
        self.setup_directories()
        
    def setup_directories(self):
        """Create lidar data directory"""
        if self.tub_path:
            self.lidar_data_dir = os.path.join(self.tub_path, 'lidar')
            if not os.path.exists(self.lidar_data_dir):
                os.makedirs(self.lidar_data_dir)
                logger.info(f"Created lidar data directory: {self.lidar_data_dir}")
    
    def run(self, recording, lidar_dist, num_records):
        """
        Save lidar data when recording
        
        Args:
            recording: Whether we are currently recording
            lidar_dist: The lidar distance measurements list
            num_records: Current record number from TubWriter
        
        Returns:
            The filename of the saved lidar data (or None if not recording)
        """
        if not recording or not lidar_dist or self.lidar_data_dir is None:
            return None
            
        # Use the TubWriter's record number to keep data synchronized
        self.counter = num_records - 1  # num_records is 1-based
        
        filename = f"{self.counter}_lidar_dist_.npy"
        filepath = os.path.join(self.lidar_data_dir, filename)
        
        try:
            # Convert to numpy array and save
            lidar_array = np.array(lidar_dist, dtype=np.float32)
            np.save(filepath, lidar_array)
            logger.debug(f"Saved lidar data to {filepath}")
            return filename
        except Exception as e:
            logger.error(f"Error saving lidar data: {e}")
            return None


def _try_tcp_connect(ip: str, port: int, timeout: float = 2.0) -> bool:
    """TCP接続を試行し、成功すればTrueを返す"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.close()
        return True
    except (socket.timeout, socket.error, OSError):
        return False


def _apply_tmini_config(config_module):
    """TMINI用の設定値をconfigモジュールに上書きする"""
    setattr(config_module, 'LIDAR_TYPE', 'TMINI')
    setattr(config_module, 'LIDAR_SCAN_RATE', 10)
    scan_rate = 10
    setattr(config_module, 'LIDAR_DATA_POINTS', int(4000 / scan_rate))
    setattr(config_module, 'LIDAR_ANGLE_RANGE', 360)
    setattr(config_module, 'LIDAR_ANGLE_START', 0)
    setattr(config_module, 'LIDAR_ANGLE_END', 360)
    setattr(config_module, 'LIDAR_ANGLE_OFFSET', 90)
    setattr(config_module, 'LIDAR_CLOCKWISE', True)

    setattr(config_module, 'LIDAR_COMM_TYPE', 'serial')
    if not hasattr(config_module, 'LIDAR_SERIAL_PORT') or config_module.LIDAR_SERIAL_PORT is None:
        setattr(config_module, 'LIDAR_SERIAL_PORT', '/dev/ttyAMA0')
    setattr(config_module, 'LIDAR_SERIAL_BAUDRATE', 230400)

    setattr(config_module, 'LIDAR_UNIT_TYPE', 'm')
    setattr(config_module, 'LIDAR_TARGET_UNIT', 'mm')

    setattr(config_module, 'LIDAR_MIN_DISTANCE', 20)
    setattr(config_module, 'LIDAR_MAX_DISTANCE', 4000)
    setattr(config_module, 'LIDAR_IGNORE_DISTANCE', 150)

    # config.pyにZONE_INDEXが定義済み（非空）ならそちらを優先
    existing = getattr(config_module, 'ZONE_INDEX', None)
    if not existing or all(len(z) == 0 for z in existing):
        setattr(config_module, 'ZONE_INDEX', [
            [x for x in range(274, 324)],                              # RrLH: 真左（180°）中心299
            [x for x in range(324, 374)],                              # FrLH: 左斜め45°（135°）中心349
            [x for x in range(374, 400)] + [x for x in range(0, 24)], # FrFR: 真正面（90°）中心399/0
            [x for x in range(24, 74)],                                # FrRH: 右斜め45°（45°）中心49
            [x for x in range(74, 124)],                               # RrRH: 真右（0°）中心99
        ])


def _apply_ust20_config(config_module):
    """UST20用の設定値をconfigモジュールに上書きする"""
    setattr(config_module, 'LIDAR_TYPE', 'UST20')
    setattr(config_module, 'LIDAR_SCAN_RATE', 40)
    setattr(config_module, 'LIDAR_DATA_POINTS', 1081)
    setattr(config_module, 'LIDAR_CLOCKWISE', False)
    setattr(config_module, 'LIDAR_ANGLE_RANGE', 270)
    setattr(config_module, 'LIDAR_ANGLE_START', -135)
    setattr(config_module, 'LIDAR_ANGLE_END', 135)
    angle_step = 4
    setattr(config_module, 'LIDAR_ANGLE_STEP', angle_step)
    setattr(config_module, 'LIDAR_ANGLE_OFFSET', 90)

    setattr(config_module, 'LIDAR_COMM_TYPE', 'ethernet')
    if not hasattr(config_module, 'LIDAR_IP_ADDRESS') or config_module.LIDAR_IP_ADDRESS is None:
        setattr(config_module, 'LIDAR_IP_ADDRESS', '192.168.0.139')
    if not hasattr(config_module, 'LIDAR_PORT') or config_module.LIDAR_PORT is None:
        setattr(config_module, 'LIDAR_PORT', 10940)

    setattr(config_module, 'LIDAR_UNIT_TYPE', 'mm')
    setattr(config_module, 'LIDAR_TARGET_UNIT', 'mm')

    setattr(config_module, 'LIDAR_MIN_DISTANCE', 100)
    setattr(config_module, 'LIDAR_MAX_DISTANCE', 20000)
    setattr(config_module, 'LIDAR_IGNORE_DISTANCE', 100)

    # config.pyにZONE_INDEXが定義済み（非空）ならそちらを優先
    existing = getattr(config_module, 'ZONE_INDEX', None)
    if not existing or all(len(z) == 0 for z in existing):
        setattr(config_module, 'ZONE_INDEX', [
            [x for x in range(180 * angle_step, 240 * angle_step)],
            [x for x in range(150 * angle_step, 180 * angle_step)],
            [x for x in range(120 * angle_step, 150 * angle_step)],
            [x for x in range(90 * angle_step, 120 * angle_step)],
            [x for x in range(30 * angle_step, 90 * angle_step)],
        ])


def _apply_none_config(config_module):
    """LiDAR未検出時のデフォルト設定を上書きする"""
    setattr(config_module, 'LIDAR_TYPE', 'NONE')
    setattr(config_module, 'LIDAR_DATA_POINTS', 0)
    setattr(config_module, 'LIDAR_ANGLE_RANGE', 0)
    setattr(config_module, 'LIDAR_ANGLE_START', 0)
    setattr(config_module, 'LIDAR_ANGLE_END', 0)
    setattr(config_module, 'LIDAR_ANGLE_OFFSET', 0)
    setattr(config_module, 'LIDAR_CLOCKWISE', True)
    setattr(config_module, 'LIDAR_SCAN_RATE', 1)
    setattr(config_module, 'LIDAR_MIN_DISTANCE', 20)
    setattr(config_module, 'LIDAR_MAX_DISTANCE', 4000)
    setattr(config_module, 'LIDAR_IGNORE_DISTANCE', 150)
    setattr(config_module, 'ZONE_INDEX', [[], [], [], [], []])


def detect_lidar(config_module) -> str | None:
    """接続されたLiDARを自動検出し、config値を上書きする。

    LIDAR_TYPE が "AUTO" の場合のみ実行。明示指定時は None を返す。

    Returns:
        検出されたLiDARタイプ ("TMINI", "UST20", "NONE") または None (明示指定時)
    """
    lidar_type = getattr(config_module, 'LIDAR_TYPE', 'AUTO')
    if lidar_type not in ("AUTO", "auto"):
        return None  # 明示指定 → スキップ

    # 1. TMINI検出: シリアルポートの存在チェック
    serial_port = getattr(config_module, 'LIDAR_SERIAL_PORT', '/dev/ttyAMA0')
    if os.path.exists(serial_port):
        _apply_tmini_config(config_module)
        logger.info(f"TMINI detected: serial port {serial_port} found")
        return "TMINI"

    # 2. UST20検出: TCP接続試行
    ip = getattr(config_module, 'LIDAR_IP_ADDRESS', '192.168.0.139')
    port = getattr(config_module, 'LIDAR_PORT', 10940)
    if _try_tcp_connect(ip, port, timeout=2.0):
        _apply_ust20_config(config_module)
        logger.info(f"UST20 detected: TCP connection to {ip}:{port} succeeded")
        return "UST20"

    # 見つからない
    _apply_none_config(config_module)
    logger.warning("No LiDAR detected. Set LIDAR_TYPE='NONE'")
    return "NONE"


def create_lidar(lidar_type: str = None, **kwargs):
    """
    設定に基づいて適切なLiDARインスタンスを生成するファクトリー関数
    
    Args:
        lidar_type: LiDARの種類 (None の場合は設定から読み込み)
        **kwargs: 各LiDAR固有のパラメータをオーバーライド
    
    Returns:
        LidarBase: 指定されたタイプのLiDARインスタンス
    
    Raises:
        ValueError: 未知のLiDARタイプが指定された場合
    """    
    # AUTO の場合は自動検出を実行
    if lidar_type in ("AUTO", "auto", None):
        detected = detect_lidar(cfg)
        if detected:
            lidar_type = detected
        else:
            lidar_type = getattr(cfg, 'LIDAR_TYPE', 'NONE')

    if lidar_type == "UST20":
        return HokuyoUST20(**kwargs)
    elif lidar_type == "TMINI":
        return TMINI(**kwargs)
    else:
        raise ValueError(f"Unknown lidar type: {lidar_type}. Supported types: UST20, TMINI")


# ── ROS2 ノード実装 ────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    import math

    class LidarNode(Node):
        def __init__(self):
            super().__init__('lidar_node')
            # LiDARインスタンスを作成
            self.lidar = create_lidar(lidar_type=cfg.LIDAR_TYPE, image_w=224, image_h=224)
            # パブリッシャー（slam_toolbox標準: /scan）
            self.publisher = self.create_publisher(LaserScan, '/scan', 10)

            # 静的TF: base_link → lidar_link
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_tf()

            # スキャンレートに合わせたタイマー
            scan_rate = getattr(cfg, 'LIDAR_SCAN_RATE', 40)
            self.timer = self.create_timer(1.0 / scan_rate, self.publish_scan)
            self.get_logger().info(f"Lidar node started (rate={scan_rate}Hz)")

        def _publish_static_tf(self):
            """base_link → lidar_link の静的TFを発行"""
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'lidar_link'
            # config.pyのオフセット値 (mm → m)
            t.transform.translation.x = getattr(cfg, 'LIDAR_OFFSET_Y', 0) / 1000.0  # 前後→x
            t.transform.translation.y = getattr(cfg, 'LIDAR_OFFSET_X', 0) / 1000.0   # 左右→y
            t.transform.translation.z = 0.05  # LiDAR搭載高さ（概算）
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

        def publish_scan(self):
            try:
                if not self.lidar.running or self.lidar.measurements is None:
                    return
                measurements = self.lidar.measurements
                if len(measurements) == 0:
                    return

                angle_start = getattr(cfg, 'LIDAR_ANGLE_START', -135.0)
                angle_end = getattr(cfg, 'LIDAR_ANGLE_END', 135.0)
                scan_rate = getattr(cfg, 'LIDAR_SCAN_RATE', 40)
                min_distance = getattr(cfg, 'LIDAR_MIN_DISTANCE', 100)
                max_distance = getattr(cfg, 'LIDAR_MAX_DISTANCE', 20000)

                n = len(measurements)
                angle_increment = (angle_end - angle_start) / (n - 1) if n > 1 else 0.0

                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'lidar_link'
                msg.angle_min = math.radians(angle_start)
                msg.angle_max = math.radians(angle_end)
                msg.angle_increment = math.radians(angle_increment)
                msg.time_increment = 0.0
                msg.scan_time = 1.0 / scan_rate if scan_rate > 0 else 0.025
                msg.range_min = min_distance / 1000.0
                msg.range_max = max_distance / 1000.0
                msg.ranges = [float(d) / 1000.0 for d in measurements]
                msg.intensities = []

                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Scan publish error: {e}")

    def main_ros(args=None):
        rclpy.init(args=args)
        node = LidarNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            if hasattr(node, 'lidar'):
                node.lidar.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

except ImportError:
    rclpy = None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run LiDAR as ROS2 node or standalone")
    parser.add_argument('--ros', action='store_true', help="Run as ROS2 node")
    args = parser.parse_args()

    if args.ros and rclpy:
        print("Starting in ROS2 mode...")
        main_ros()
    else:
        print("Starting in standalone mode...")
        main()
