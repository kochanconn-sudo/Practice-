# coding:utf-8
import collections
import numpy as np
import time
import torch
from torchvision import transforms
from PIL import Image
import config
from train_pytorch import normalize_ultrasonics
from position_inference import infer_position
from yolo_detection import detect_objects, apply_detection_control_modification, select_model_by_detection, calculate_object_tracking_steering, calculate_obstacle_avoidance_steering
from follow_the_gap import follow_the_gap
import logging

logger = logging.getLogger(__name__)


class Planner:
    def __init__(self):
        #　判断フラグ
        self.in_recovery = False
        self.recovery_phase = None       # "brake", "neutral", "back"
        self.recovery_phase_count = 0    # 現在のブレーキセット番号（1〜RECOVERY_BRAKING）
        self.recovery_phase_start = 0
        self.recovery_steering = 0.0     # リカバリー中のステアリング値
        self.before_recovery_detection_times = 3 ## 目前に前壁をtimes回検知
        self.recovery_seconds_remaining = 0
        self.recovery_time_start = time.perf_counter()
        self.recovery_time_end = time.perf_counter()
        self.recovery_time_duration = config.RECOVERY_TIME_DURATION
        self.recovery_frames_remaining = 0

        # 操作値出力
        self.message = ""

        # クラス内で操作値保持
        self.steering = 0.0
        self.throttle = 0.0
        self.speed = 0.0
        
        # 過去の操作値記録回数
        self.records_steering = np.zeros(config.RIGHT_LEFT_RECORD_NUMBER)
        self.records_throttle = np.zeros(config.RIGHT_LEFT_RECORD_NUMBER)
        
        # pid用のタイマー
        self.time_current = time.perf_counter()
        self.time_before = time.perf_counter()
        
        # pid用の最小距離
        self.minimum_distance_current = config.TARGET_RANGE
        self.minimum_distance_before = config.TARGET_RANGE
        self.integral_delta_distance = 0.0

        # 位置推論関連の状態管理
        self.position_inference_counter = 0
        self.current_position_id = None
        self.current_driving_model = None

        # Follow the Gap用LiDARデータ
        self._lidar_data = None

        # YOLO検知関連の状態管理
        self.yolo_detection_counter = 0
        self.current_detections = []
        self.yolo_active_model = None

        # 時系列モデル推論用の状態
        self._seq_frame_buffer = collections.deque(maxlen=50)  # 前処理済みGPUテンソルをキャッシュ
        self._seq_transform = None  # 遅延初期化

    def _select_model_by_position(self, mode, position_model, position_models_dict, camera_images, default_model):
        """
        位置推論によるモデル選択

        Args:
            mode: 走行モード
            position_model: 位置推論モデル
            position_models_dict: 位置別モデル辞書
            camera_images: カメラ画像辞書 {'camera_0': image, 'camera_1': image}
            default_model: デフォルトモデル

        Returns:
            selected_model: 選択されたモデル
        """
        if not config.USE_POSITION_SWITCHING or mode == "user" or position_model is None:
            return default_model

        self.position_inference_counter += 1

        # 指定フレーム間隔で位置推論を実行
        if self.position_inference_counter >= config.POSITION_INFERENCE_INTERVAL:
            self.position_inference_counter = 0

            # 位置推論用の画像を選択
            position_image = None
            if hasattr(config, 'POSITION_MODEL_INPUT_IMAGE') and camera_images:
                for ci in range(4):
                    if f"cam{ci}" in config.POSITION_MODEL_INPUT_IMAGE:
                        position_image = camera_images.get(f'camera_{ci}')
                        break

            # デフォルトはcamera_0
            if position_image is None and camera_images:
                position_image = camera_images.get('camera_0')

            if position_image is not None:
                # 位置を推論
                inferred_position, confidence = infer_position(position_model, position_image)

                if inferred_position is not None:
                    # 位置が変わった場合、ログ出力
                    if inferred_position != self.current_position_id:
                        position_name = config.POSITION_CLASS_NAMES[inferred_position] if inferred_position < len(config.POSITION_CLASS_NAMES) else f"Position{inferred_position}"
                        logger.info(f"位置が変更されました: {position_name} (信頼度: {confidence:.2f})")
                        self.current_position_id = inferred_position

                        # 位置に応じたモデルを選択
                        if inferred_position in position_models_dict:
                            self.current_driving_model = position_models_dict[inferred_position]
                            logger.info(f"モデルを切り替え: 位置{inferred_position}用モデル")
                        elif 'default' in position_models_dict:
                            self.current_driving_model = position_models_dict['default']
                            logger.info(f"デフォルトモデルを使用（位置{inferred_position}用モデルなし）")
                        else:
                            self.current_driving_model = default_model
                            logger.info(f"通常モデルを使用（位置{inferred_position}用モデルなし）")

        # 現在のモデルを返す（位置推論していない場合はデフォルト）
        return self.current_driving_model if self.current_driving_model is not None else default_model

    def _select_model_by_yolo(self, mode, yolo_model, yolo_models_dict, inference_camera_image, default_model):
        """
        YOLO物体検知によるモデル選択

        Args:
            mode: 走行モード
            yolo_model: YOLOモデル
            yolo_models_dict: クラス別モデル辞書
            inference_camera_image: 推論用カメラ画像
            default_model: デフォルトモデル

        Returns:
            selected_model: 選択されたモデル
        """
        if not config.USE_YOLO_DETECTION or mode == "user" or yolo_model is None:
            return default_model

        self.yolo_detection_counter += 1

        # 指定フレーム間隔で物体検知を実行
        if self.yolo_detection_counter >= config.YOLO_DETECTION_INTERVAL:
            self.yolo_detection_counter = 0

            if inference_camera_image is not None:
                # 物体検知を実行
                self.current_detections = detect_objects(yolo_model, inference_camera_image)

                # 検知結果の表示
                if config.YOLO_DISPLAY_DETECTIONS and self.current_detections:
                    detection_summary = ", ".join([
                        f"{d['class_name']}({d['confidence']:.2f})"
                        for d in self.current_detections
                    ])
                    logger.info(f"物体検知: {detection_summary}")

                # モデル切り替え（YOLO_MODEL_SWITCHINGが設定されている場合）
                if config.YOLO_MODEL_SWITCHING and yolo_models_dict:
                    self.yolo_active_model, detected_class = select_model_by_detection(
                        self.current_detections, yolo_models_dict, default_model
                    )
                    if detected_class:
                        logger.info(f"検知によるモデル切り替え: {detected_class['class_name']} (信頼度: {detected_class['confidence']:.2f})")
                        return self.yolo_active_model

        # YOLO検知モデルがある場合はそれを返す、なければデフォルト
        return self.yolo_active_model if self.yolo_active_model is not None else default_model

    def compute_motor_commands(self, mode, plan, ranges, model=None, camera_image=None, data_aggregator=None):
        """
        plan: str (go_straight, right_left_3, nn, donkeycar, resnet18, mobilevit_xxs, edgenext_xx_small, gru, tcn, causal_cnn など)
        ranges: dict {"FrFR": xx, "FrLH": xx, ...}
        model: ニューラルネット用のモデル
        camera_image: 画像ベースモデル用の画像 (numpy配列など) - MODEL_INPUT_IMAGEで指定されたカメラの画像
        data_aggregator: データ集約器（時系列モデルのフレーム履歴用）

        Returns:
            steering_value, throttle_value
        """
        if plan == "go_straight":
            if ranges["FrFR"] < config.STOP_RANGE:
                return 0.0, 0.0  # 停止
            return 0.0, config.FORWARD_STRAIGHT

        elif plan == "right_left_3":
            inputs = (ranges["FrLH"], ranges["FrFR"], ranges["FrRH"])
            return self.right_left_3(*inputs)

        elif plan == "right_left_3_records":
            inputs = (ranges["FrLH"], ranges["FrFR"], ranges["FrRH"])
            return self.right_left_3_records(*inputs)

        elif plan == "wall_follow":
            side = config.HAND_SIDE
            range_front = ranges["FrFR"]
            range_front_side = ranges["FrRH"] if side == "right" else ranges["FrLH"]
            range_rear_side = ranges.get("RrRH", range_front_side) if side == "right" else ranges.get("RrLH", range_front_side)
            return self.wall_follow(range_front, range_front_side, range_rear_side, side)

        elif plan == "wall_follow_pid":
            side = config.HAND_SIDE
            range_front = ranges["FrFR"]
            range_front_side = ranges["FrRH"] if side == "right" else ranges["FrLH"]
            range_rear_side = ranges.get("RrRH", range_front_side) if side == "right" else ranges.get("RrLH", range_front_side)
            return self.wall_follow_pid(range_front, range_front_side, range_rear_side, side)

        elif plan == "nn":
            if not model:
                logger.warning(f"PLAN='{plan}' ですがモデルが未ロードです。MODEL_PATH を確認してください: {getattr(config, 'MODEL_PATH', '未設定')}")
                return 0.0, 0.0
            inputs = [ranges[key] for key in ranges]
            return self.nn(model, *inputs)

        elif plan in ["donkeycar", "resnet18", "mobilevit_xxs", "edgenext_xx_small"]:
            if not model or camera_image is None:
                logger.warning(f"PLAN='{plan}' ですが{'モデルが未ロードです。MODEL_PATH を確認してください: ' + str(getattr(config, 'MODEL_PATH', '未設定')) if not model else 'カメラ画像がありません'}")
                return 0.0, 0.0
            return self.model_catalog_inference(model, camera_image)

        elif plan in ["gru", "tcn", "causal_cnn"]:
            if not model or camera_image is None:
                logger.warning(f"PLAN='{plan}' ですが{'モデルが未ロードです。MODEL_PATH を確認してください: ' + str(getattr(config, 'MODEL_PATH', '未設定')) if not model else 'カメラ画像がありません'}")
                return 0.0, 0.0
            return self.sequence_model_inference(model, camera_image, data_aggregator)

        elif plan == "center_follow_pid":
            return self.center_follow_pid(
                ranges["FrFR"],
                ranges["FrLH"], ranges.get("RrLH", ranges["FrLH"]),
                ranges["FrRH"], ranges.get("RrRH", ranges["FrRH"]),
            )

        elif plan == "gap_follow":
            inputs = (ranges["FrLH"], ranges["FrFR"], ranges["FrRH"])
            return self.gap_follow(*inputs)

        elif plan == "racer":
            inputs = (ranges["FrLH"], ranges["FrFR"], ranges["FrRH"])
            return self.racer(*inputs)

        elif plan == "follow_the_gap":
            lidar_data = self._lidar_data
            if lidar_data is not None:
                return follow_the_gap(lidar_data)
            else:
                logger.warning("follow_the_gap: LiDARデータなし")
                return 0.0, 0.0

        else:
            logger.warning(f"不明なプラン: '{plan}'。PLAN_LIST から選択してください: {getattr(config, 'PLAN_LIST', [])}")
            return 0.0, 0.0
        
    # 前側１センサーを用いた停止
    def recovery_stop(self, ultrasonic_Fr):
        ## 目前に前壁をtimes回検知
        times = 3
        if max(ultrasonic_Fr.records[0:self.before_recovery_detection_times-1]) < config.STOP_RANGE:
                self.in_recovery = True                
                print("停止")

    # 前側3センサーを用いた後退
    def recovery_back(self, data_aggregator):
        """
        直近の超音波値を取得して後退リカバリを判定する。
        ESCのバック動作はブレーキ→ニュートラル(0)→バックの順で行う。
        """
        BRAKE_DURATION = config.RECOVERY_BRAKE_DURATION
        NEUTRAL_DURATION = config.RECOVERY_NEUTRAL_DURATION

        # リカバリー中なら残り時間を更新
        if self.in_recovery:
            now = time.perf_counter()
            phase_elapsed = now - self.recovery_phase_start

            if self.recovery_phase == "brake":
                if phase_elapsed >= BRAKE_DURATION:
                    self.recovery_phase = "neutral"
                    self.recovery_phase_start = now
                    print(f"RECOVERY PHASE: neutral ({self.recovery_phase_count}/{config.RECOVERY_BRAKING})")
            elif self.recovery_phase == "neutral":
                if phase_elapsed >= NEUTRAL_DURATION:
                    if self.recovery_phase_count < config.RECOVERY_BRAKING:
                        # 次のブレーキセットへ
                        self.recovery_phase_count += 1
                        self.recovery_phase = "brake"
                        self.recovery_phase_start = now
                        print(f"RECOVERY PHASE: brake ({self.recovery_phase_count}/{config.RECOVERY_BRAKING})")
                    else:
                        # 全セット完了、バックへ
                        self.recovery_phase = "back"
                        self.recovery_phase_start = now
                        self.recovery_time_end = now + config.RECOVERY_TIME_DURATION
                        print("RECOVERY PHASE: back")
            elif self.recovery_phase == "back":
                self.recovery_time_remaining = self.recovery_time_end - now
                if self.recovery_time_remaining <= 0:
                    # 終了時間に達したらリカバリ解除
                    self.in_recovery = False
                    self.recovery_phase = None
                    print("RECOVERY END")
                else:
                    print(f"RECOVERY TIME REMAINING: {self.recovery_time_remaining:.2f}")

        else:
            # 1) 過去 self.before_recovery_detection_times フレーム分のセンサー履歴を取得
            n = self.before_recovery_detection_times

            FrFR_history   = data_aggregator.get_sensor_history("FrFR")    # [古い, ..., 新しい]
            FrRH_history = data_aggregator.get_sensor_history("FrRH")
            FrLH_history = data_aggregator.get_sensor_history("FrLH")

            # 2) 直近N件を切り出し
            recent_FrFR   = FrFR_history[-n:]   # 直近 n 件
            recent_FrRH = FrRH_history[-n:]
            recent_FrLH = FrLH_history[-n:]

            # 値が取得されている
            if len(recent_FrFR) > 0 and len(recent_FrRH) > 0 and len(recent_FrLH) > 0:
                #直近の最大値の利用するセンサーの中で最小値
                min_of_max = min(max(recent_FrFR), max(recent_FrRH), max(recent_FrLH))

                if min_of_max < config.BACKWARD_RANGE:
                    self.in_recovery = True
                    self.recovery_phase = "brake"
                    self.recovery_phase_count = 1
                    self.recovery_phase_start = time.perf_counter()
                    self.recovery_time_start = self.recovery_phase_start
                    # ステアリング決定
                    if config.RECOVERY_STEERING == "auto":
                        # 左右センサーの直近最大値を比較し、空いている方向の逆に切る
                        max_LH = max(recent_FrLH)
                        max_RH = max(recent_FrRH)
                        if max_LH > max_RH:
                            # 左が空いている → 右に切る（逆方向）
                            self.recovery_steering = 1.0
                        else:
                            # 右が空いている → 左に切る（逆方向）
                            self.recovery_steering = -1.0
                        print(f"RECOVERY START - PHASE: brake, auto steering={self.recovery_steering} (LH={max_LH:.0f}, RH={max_RH:.0f})")
                    else:
                        self.recovery_steering = float(config.RECOVERY_STEERING)
                        print(f"RECOVERY START - PHASE: brake, fixed steering={self.recovery_steering}")

        return self.in_recovery

    # 前側３センサーを用いた右左走行
    def right_left_3(self, dis_FrLH, dis_FrFR, dis_FrRH):
        # 検知時の判断
        ## 壁を検知
        if dis_FrFR < config.DETECTION_RANGE or dis_FrLH < config.RIGHT_LEFT_RANGE or dis_FrRH < config.RIGHT_LEFT_RANGE:
            ### 左＜右の距離
            if dis_FrLH < dis_FrRH :
                self.steering =1.0
                self.throttle = config.FORWARD_CORNER
                self.message = "右旋回"
            ### 左＞右の距離
            else:
                self.steering =-1.0
                self.throttle = config.FORWARD_CORNER
                self.message = "左旋回"
        ## 前壁を検知なし
        else:
            self.steering =0.0
            self.throttle = config.FORWARD_STRAIGHT
            self.message = "直進中"

        ## モーターへ出力を返す
        if config.TERMINAL_PRINT:
            print(self.message)
        return self.steering, self.throttle

    # 前側３センサーを用いた右左走行　過去の値でスムージング
    def right_left_3_records(self, dis_FrLH, dis_FrFR, dis_FrRH):
        self.steering, self.throttle  = self.right_left_3(dis_FrLH, dis_FrFR, dis_FrRH)

        # 過去の値を記録の一番前に挿入し、最後を消す
        self.records_steering = np.insert(self.records_steering, 0, self.steering)
        self.records_steering = np.delete(self.records_steering,-1)
        self.records_throttle = np.insert(self.records_throttle, 0, self.throttle)
        self.records_throttle = np.delete(self.records_throttle,-1)

        return round(np.mean(self.records_steering),2), round(np.mean(self.records_throttle),2)

    # === 新規: gap_follow（基本選択肢・混在コースの土俵）===
    def gap_follow(self, dis_FrLH, dis_FrFR, dis_FrRH):
        L, F, R = dis_FrLH, dis_FrFR, dis_FrRH
        g_steer = getattr(config, "GAP_STEER_GAIN", 1.0)
        diff = (L - R) / (L + R + 1)
        steering = max(-1.0, min(1.0, -g_steer * diff))
        if F < getattr(config, 'GAP_FOLLOW_BRAKE_DIST', config.DETECTION_RANGE) or abs(steering) > 0.5:
            throttle = config.FORWARD_CORNER
        else:
            throttle = config.FORWARD_STRAIGHT
        return round(steering, 2), round(throttle, 2)

    # === 新規: racer（やり込み枠・4パラメータ）。基本選択肢には出さない ===
    def racer(self, dis_FrLH, dis_FrFR, dis_FrRH):
        L, F, R = dis_FrLH, dis_FrFR, dis_FrRH
        g_steer = getattr(config, "RACER_STEER_GAIN", 0.8)
        g_speed = getattr(config, "RACER_SPEED_GAIN", 1.0)
        brake   = getattr(config, "RACER_BRAKE_DIST", 500)
        slow    = getattr(config, "RACER_STEER_SLOWDOWN", 0.5)
        ceil    = getattr(config, "RACER_SPEED_CEIL", 0.9)   # 安全上限(固定)
        vmin, vmax = config.FORWARD_CORNER, config.FORWARD_STRAIGHT
        # 操舵: 開いた方へ。左右差を正規化しSTEER_GAINで鋭さ調整
        diff = (L - R) / (L + R + 1)
        steering = max(-1.0, min(1.0, -g_steer * diff))
        # 速度: 前方が brake 以上で全開、未満で線形減速。SPEED_GAINで攻める
        base = 1.0 if F >= brake else max(0.0, F / brake)
        throttle = vmin + (vmax - vmin) * base * g_speed
        # 曲げると減速
        throttle *= (1.0 - slow * abs(steering))
        # 安全上限でクリップ
        throttle = max(vmin, min(ceil, throttle))
        return round(steering, 2), round(throttle, 2)

    def _calc_wall_angle(self, d_front_side, d_rear_side, side):
        """
        2点のセンサー距離から壁角度を算出する。
        Returns: wall_angle (rad) - 0=平行, 正=ノーズが壁から離れている
        """
        import math
        sin45 = math.sin(math.radians(45))
        cos45 = math.cos(math.radians(45))

        if side == "right":
            dx = d_front_side * sin45 - d_rear_side
            dy = d_front_side * cos45
        else:  # left
            dx = -d_front_side * sin45 + d_rear_side
            dy = d_front_side * cos45

        wall_angle = math.atan2(dx, dy)
        return wall_angle

    # 壁を用いた走行（右手法・左手法を選択可能）
    def wall_follow(self, dis_front, dis_front_side, dis_rear_side, side="right"):
        """
        壁を用いた走行（右手法・左手法対応）。
        dis_front: 前方センサーからの距離
        dis_front_side: 壁側前方センサーからの距離 (FrRH or FrLH)
        dis_rear_side: 壁側後方センサーからの距離 (RrRH or RrLH)
        side: 壁の位置 ('right' または 'left')
        """
        if side not in ["right", "left"]:
            raise ValueError("Invalid side. Expected 'right' or 'left'.")

        # 壁の距離に基づいた調整
        target_range = config.TARGET_RANGE
        adjustment = config.TARGET_RANGE_ADJUSTMENT

        # 検知時の判断
        ## 壁が遠い場合
        if (dis_front_side > target_range + adjustment) and (dis_rear_side > target_range + adjustment):
            self.steering = 1.0 if side == "right" else -1.0
            self.throttle = config.FORWARD_CORNER
            self.message = f"{side}手法: 壁が遠い、{side}旋回"

        ## 壁が近い場合
        elif (dis_front_side < target_range - adjustment) or (dis_rear_side < target_range - adjustment):
            self.steering = -1.0 if side == "right" else 1.0
            self.throttle = config.FORWARD_CORNER
            self.message = f"{side}手法: 壁が近い"

        ## 壁が適切な距離にある場合
        else:
            self.steering = 0.0
            self.throttle = config.FORWARD_STRAIGHT
            self.message = f"{side}手法: 壁沿い直進中"

        # 壁角度アライメント補正（WALL_FOLLOW_USE_ALIGNMENT有効時）
        if getattr(config, 'WALL_FOLLOW_USE_ALIGNMENT', False):
            wall_angle = self._calc_wall_angle(dis_front_side, dis_rear_side, side)
            angle_correction = getattr(config, 'WALL_FOLLOW_K_ANGLE', 0.3) * wall_angle
            # 距離判定がNEUTRAL（適切距離）の場合のみ角度補正を適用
            if self.steering == 0.0:
                self.steering = max(-1, min(1, angle_correction))
                if abs(wall_angle) > 0.1:
                    self.throttle = config.FORWARD_CORNER

        # デバッグ用メッセージ出力
        if config.TERMINAL_PRINT:
            print(self.message)

        # モーターへ出力を返す
        return self.steering, self.throttle

    # 壁との距離を一定に保つPID制御走行
    def wall_follow_pid(self, ultrasonic_front, ultrasonic_front_side, ultrasonic_rear_side, side):
        """
        壁との距離を一定に保つPID制御走行。
        side: 壁の位置 ('left' または 'right')
        ultrasonic_front: 前方センサーからの距離データ
        ultrasonic_front_side: 壁側前方センサーからの距離データ (FrRH or FrLH)
        ultrasonic_rear_side: 壁側後方センサーからの距離データ (RrRH or RrLH)
        """

        # 時間更新: 現在の時刻と前回の時刻差を計算
        self.time_before = self.time_current
        self.time_current = time.perf_counter()
        delta_t = self.time_current - self.time_before

        # 壁までの最小距離を計算（壁側の前方センサーと後方センサーの最小値）
        self.minimum_distance_before = self.minimum_distance_current
        self.minimum_distance_current = min(ultrasonic_front_side, ultrasonic_rear_side)

        # 偏差を計算: 現在の最小距離と目標距離（TARGET_RANGE）の差
        delta_dis = self.minimum_distance_current - config.TARGET_RANGE

        # 偏差の積分値を更新: 時間方向に積分することで過去の偏差を考慮
        self.integral_delta_distance += delta_dis

        # 距離変化速度（微分項）を計算
        v = (self.minimum_distance_current - self.minimum_distance_before) / delta_t if delta_t > 0 else 0

        # 壁角度項の追加（WALL_FOLLOW_USE_ALIGNMENT有効時）
        if getattr(config, 'WALL_FOLLOW_USE_ALIGNMENT', False):
            wall_angle = self._calc_wall_angle(ultrasonic_front_side, ultrasonic_rear_side, side)
            angle_term = getattr(config, 'WALL_FOLLOW_K_ANGLE', 0.3) * wall_angle
        else:
            angle_term = 0.0

        # PID制御でステア値を計算
        # - 比例項 (P): 偏差に比例して制御量を計算
        # - 積分項 (I): 偏差の累積を考慮して制御量を補正
        # - 微分項 (D): 変化速度を考慮してスムーズな制御を実現
        # - 壁角度項: 壁との平行度を補正
        steering_gain = config.K_P * delta_dis - config.K_D * v + config.K_I * self.integral_delta_distance + angle_term

        # ステアゲイン値を0 ~ 1に変換
        steering_gain = max(-1, min(1, steering_gain))

        # デバッグ用の出力: PID制御の各項目を出力
        if config.TERMINAL_PRINT:
            self._print_pid_debug(side, steering_gain, delta_dis, self.integral_delta_distance, v)

        # 左右の壁に応じた走行ロジックを実行
        if side == "right":
            self.steering = steering_gain * 1.0
        elif side == "left":
            self.steering = steering_gain * -1.0
        else:
            raise ValueError("Invalid side. Expected 'left' or 'right'.")

        # スロットル値も調整
        if abs(self.steering) > 0.7:
            self.throttle = config.FORWARD_CORNER
        else:
            self.throttle = config.FORWARD_STRAIGHT

        # 計算結果を返す: ステアリング値とスロットル値
        return round(self.steering,2), round(self.throttle,2)

    # 左右壁の中央を維持するPID制御走行
    def center_follow_pid(self, dis_front, dis_front_left, dis_rear_left, dis_front_right, dis_rear_right):
        """
        左右壁の中央を走行するPID制御。
        偏差 = 左壁距離 - 右壁距離（正なら右寄り、負なら左寄り）
        dis_front:       前方センサー距離 (FrFR)
        dis_front_left:  前左センサー距離 (FrLH)
        dis_rear_left:   後左センサー距離 (RrLH)
        dis_front_right: 前右センサー距離 (FrRH)
        dis_rear_right:  後右センサー距離 (RrRH)
        """
        # 時間更新
        self.time_before = self.time_current
        self.time_current = time.perf_counter()
        delta_t = max(self.time_current - self.time_before, 0.01)  # 初回暴れ防止

        # 左右それぞれの代表距離（前後センサの最小値）
        dis_left  = min(dis_front_left,  dis_rear_left)
        dis_right = min(dis_front_right, dis_rear_right)

        # 片壁ロスト検出
        CENTER_FALLBACK = getattr(config, 'CENTER_FALLBACK_RANGE', 800)
        left_lost  = dis_left  > CENTER_FALLBACK
        right_lost = dis_right > CENTER_FALLBACK

        if left_lost and right_lost:
            # 両壁ロスト → 直進
            self.steering = 0.0
            self.throttle = config.FORWARD_STRAIGHT
            return round(self.steering, 2), round(self.throttle, 2)
        elif left_lost:
            # 左壁ロスト → 右壁に従う
            delta_dis = dis_right - config.TARGET_RANGE
            sign = 1.0
        elif right_lost:
            # 右壁ロスト → 左壁に従う
            delta_dis = dis_left - config.TARGET_RANGE
            sign = -1.0
        else:
            # 両壁あり：左右差が偏差（正=右寄り → 左ステア）
            delta_dis = dis_left - dis_right
            sign = -1.0

        # 微分項
        self.minimum_distance_before = self.minimum_distance_current
        self.minimum_distance_current = delta_dis
        v = (self.minimum_distance_current - self.minimum_distance_before) / delta_t

        # 積分項
        self.integral_delta_distance += delta_dis

        # PID計算（center_follow専用ゲイン。未設定時はK_Pにフォールバック）
        kp = getattr(config, 'CENTER_K_P', config.K_P)
        kd = getattr(config, 'CENTER_K_D', config.K_D)
        ki = getattr(config, 'CENTER_K_I', config.K_I)
        steering_gain = kp * delta_dis - kd * v + ki * self.integral_delta_distance
        steering_gain = max(-1.0, min(1.0, steering_gain))
        self.steering = round(steering_gain * sign, 2)

        # スロットル
        if abs(self.steering) > 0.7:
            self.throttle = config.FORWARD_CORNER
        else:
            self.throttle = config.FORWARD_STRAIGHT

        if config.TERMINAL_PRINT:
            print(
                f"CENTER PID: output={self.steering:.2f}, "
                f"[P={kp * delta_dis:.2f}, I={ki * self.integral_delta_distance:.2f}, D={kd * v:.2f}] "
                f"L={dis_left:.0f} R={dis_right:.0f}"
                + (" [L_LOST]" if left_lost else "")
                + (" [R_LOST]" if right_lost else "")
            )

        return self.steering, self.throttle


    # デバッグ用の補助関数
    def _print_pid_debug(self, side, steering, delta_dis, integral_delta_distance, v):
        side_text = "右手法" if side == "right" else "左手法"
        print(
            f"{side_text} PID制御: "
            f"output={steering:.2f}, [P={config.K_P * delta_dis:.2f}, "
            f"I={config.K_I * integral_delta_distance:.2f}, D={config.K_D * v:.2f}]"
        )

    # 右手法のPIDを用いた走行
    ## TODO:wall_followへ移行、削除予定
    def right_hand_pid(self, ultrasonic_FrRH, ultrasonic_RrRH,
        t=0, integral_delta_dis=0, min_dis=config.TARGET_RANGE):
        # 時間更新
        t_before = t
        t = time.perf_counter()
        delta_t = t-t_before
        # 右手法最小距離更新
        min_dis_before = min_dis
        min_dis = min(ultrasonic_FrRH, ultrasonic_RrRH)
        # 目標値までの差更新
        delta_dis = min_dis - self.TARGET_RANGE
        # 目標値までの差積分更新
        integral_delta_dis += delta_dis
         #速度更新
        v = (min_dis - min_dis_before)/delta_t
        # PID制御でステア値更新
        steering = self.K_P*delta_dis - self.K_D*v + self.K_I*integral_delta_dis 
        ### -100~100に収めて正の割合化
        steering = abs(max(-100,min(100,steering))/100)

        ## モーターへ出力を返す
        if config.print_plan_result:
            #print(self.message)
            print("output * PID:{:3.1f}, [P:{:3.1f}, I:{:3.1f}, D:{:3.1f}]".format(steering, self.K_P*delta_dis,self.K_D*v, self.K_I*integral_delta_dis))
        self.steering, self.throttle  = self.right_hand(ultrasonic_FrRH.dis, ultrasonic_RrRH.dis)
        return steering*self.steering, self.throttle

    # 左手法のPIDを用いた走行
    ## TODO:wall_followへ移行、削除予定
    def left_hand_pid(self, ultrasonic_FrLH, ultrasonic_RrLH,
        t=0,integral_delta_dis=0,min_dis=config.TARGET_RANGE):
        # 時間更新
        t_before = t
        t = time.perf_counter()
        delta_t = t-t_before
        # 右手法最小距離更新
        min_dis_before = min_dis
        min_dis = min(ultrasonic_FrLH.dis,ultrasonic_RrLH.dis)
        # 目標値までの差更新
        delta_dis = min_dis - self.TARGET_RANGE
        # 目標値までの差積分更新
        integral_delta_dis += delta_dis
         #速度更新
        v = (min_dis - min_dis_before)/delta_t
        # PID制御でステア値更新
        steering = self.K_P*delta_dis - self.K_D*v + self.K_I*integral_delta_dis 
        ### -100~100に収めて正の割合化
        steering = abs(max(-100,min(100,steering))/100)

        ## モーターへ出力を返す
        if config.print_plan_result:
            #print(self.message)
            print("output * PID:{:3.1f}, [P:{:3.1f}, I:{:3.1f}, D:{:3.1f}]".format(steering, self.K_P*delta_dis,self.K_D*v, self.K_I*integral_delta_dis))
        self.steering, self.throttle  = self.left_hand(ultrasonic_FrLH.dis, ultrasonic_RrLH.dis)
        return steering*self.steering, self.throttle

    # Neural Netを用いた走行
    def nn(self, model, *args):
        ultrasonic_values = args
        model_dtype = next(model.parameters()).dtype
        device = next(model.parameters()).device
        x = torch.tensor(ultrasonic_values, dtype=model_dtype).unsqueeze(0)

        # モデルに正規化パラメータがある場合（data_viewer形式）
        norm_params = getattr(model, '_normalization_params', None)
        if norm_params:
            norm_type = norm_params.get('type', 'zscore')
            if norm_type == 'clip_scale':
                clip_val = norm_params.get('clip_max', 2000.0)
                x = torch.clamp(x, 0, clip_val) / clip_val
            elif 'X_mean' in norm_params and 'X_std' in norm_params:
                mean = torch.tensor(norm_params['X_mean'], dtype=model_dtype)
                std = torch.tensor(norm_params['X_std'], dtype=model_dtype)
                x = (x - mean) / (std + 1e-8)
        else:
            # 従来の正規化（train_pytorch形式）
            x = normalize_ultrasonics(x)

        x = x.to(device)

        # data_viewer形式はforward直接、train_pytorch形式はpredict
        with torch.no_grad():
            if hasattr(model, 'predict') and norm_params is None:
                output = model.predict(model, x).squeeze(0)
            else:
                output = model(x).squeeze(0)

        self.steering = float(output[0])
        self.throttle = float(output[1])

        ## モーターへ出力を返す
        return self.steering, self.throttle
    
    def _preprocess_frame_gpu(self, frame, img_size, device, dtype):
        """1フレームをGPU上で前処理し、テンソル (C, H, W) を返す"""
        # numpy (H, W, C) uint8 → torch (C, H, W) float32、GPU上でリサイズ
        t = torch.from_numpy(frame).permute(2, 0, 1).unsqueeze(0)  # (1, C, H, W)
        t = t.to(device=device, dtype=torch.float32)
        t = torch.nn.functional.interpolate(t, size=img_size, mode='bilinear', align_corners=False)
        t = t.squeeze(0) / 255.0  # (C, H, W), [0, 1]
        return t.to(dtype=dtype)

    def sequence_model_inference(self, model, img, data_aggregator):
        """
        時系列モデル（GRU/TCN/CausalCNN）を使用して推論を行う。
        前処理済みテンソルをバッファにキャッシュし、新フレーム1枚のみ処理する。
        """
        seq_cfg = getattr(model, '_sequence_config', {})
        seq_len = seq_cfg.get('seq_len', 8)
        img_size = seq_cfg.get('img_size', (128, 128))

        try:
            device = next(model.parameters()).device
            model_dtype = next(model.parameters()).dtype

            # 新フレーム1枚のみ前処理してバッファに追加
            frame_tensor = self._preprocess_frame_gpu(img, img_size, device, model_dtype)
            self._seq_frame_buffer.append(frame_tensor)

            # seq_len分のテンソルを取得（不足時は最古で埋める）
            buf = list(self._seq_frame_buffer)
            while len(buf) < seq_len:
                buf.insert(0, buf[0])
            cached_tensors = buf[-seq_len:]

            # (T, C, H, W) → (1, T, S=1, C, H, W)
            images = torch.stack(cached_tensors).unsqueeze(1).unsqueeze(0)

            # ego_states: 制御値履歴から構築 [steering, throttle, 0, 0, 0]
            if data_aggregator is not None:
                control_history = data_aggregator.get_control_history()
            else:
                control_history = []
            while len(control_history) < seq_len:
                control_history.insert(0, (0.0, 0.0))
            control_history = control_history[-seq_len:]

            ego_np = np.array([[s, t, 0.0, 0.0, 0.0] for s, t in control_history], dtype=np.float32)
            ego_states = torch.from_numpy(ego_np).unsqueeze(0).to(device, dtype=model_dtype)

            with torch.no_grad():
                trajectory = model(images, ego_states)

            self.steering = float(trajectory[0, 0, 0].item())
            self.throttle = float(trajectory[0, 0, 1].item())

        except Exception as e:
            logger.error(f"Sequence model inference error: {e}")
            import traceback
            traceback.print_exc()
            self.steering = 0.0
            self.throttle = 0.0

        return self.steering, self.throttle

    def model_catalog_inference(self, model, img):
        """
        model_catalog のモデル（donkey, resnet18, mobilevit_xxs, edgenext_xx_small）を使用して推論を行う
        モデルの型に応じて適切な推論方法を自動選択する
        """
        if not isinstance(img, np.ndarray):
            raise TypeError(f"Input img must be a numpy.ndarray, but got {type(img)}")

        try:
            if hasattr(model, 'run'):
                # PyTorchモデル（model_catalogの.run()メソッド）
                output = model.run(img)
            else:
                # TensorRTModel等（.run()なし → 前処理してtensor入力）
                output = self._direct_tensor_inference(model, img)

            self._process_model_output(output)

        except Exception as e:
            print(f"Model inference error: {e}")
            import traceback
            traceback.print_exc()
            self.steering = 0.0
            self.throttle = 0.0

        return self.steering, self.throttle

    def _direct_tensor_inference(self, model, img):
        """TensorRTModel等の直接tensor入力モデルでの推論（前処理込み）"""
        if self._seq_transform is None:
            self._seq_transform = transforms.Compose([
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                # transforms.Normalize(mean=[0.485, 0.456, 0.406],
                #                      std=[0.229, 0.224, 0.225]),
            ])
        pil_img = Image.fromarray(img)
        tensor_img = self._seq_transform(pil_img).unsqueeze(0).cuda()

        with torch.no_grad():
            result = model(tensor_img)

        if result.device.type != 'cpu':
            result = result.cpu()
        result = result.numpy().reshape(-1)
        return result[0], result[1]

    def _openvino_inference(self, model, img):
        """OpenVINOモデルでの推論"""
        try:
            # OpenVINO推論エンジンが利用可能な場合
            from openvino.runtime import Core
            
            # model_catalogのOpenVINOラッパーを使用
            if hasattr(model, 'run'):
                return model.run(img)
            
            # 直接OpenVINO推論（フォールバック）
            # 実装は必要に応じて追加
            print("Direct OpenVINO inference not implemented, falling back to PyTorch")
            return self._pytorch_inference(model, img)
            
        except ImportError:
            print("OpenVINO not available, falling back to PyTorch")
            return self._pytorch_inference(model, img)
        except Exception as e:
            print(f"OpenVINO inference failed, falling back to PyTorch: {e}")
            return self._pytorch_inference(model, img)

    def _process_model_output(self, output):
        """モデル出力を処理してステアリング・スロットル・速度値を設定

        出力形式:
            NUM_OUTPUTS=2: (steering, throttle)
            NUM_OUTPUTS=3: (steering, throttle, speed)
        """
        def _extract(out, idx, default=0.0):
            """出力からインデックス指定で値を取得"""
            try:
                if torch.is_tensor(out):
                    return float(out[idx].item()) if out.dim() > 0 and len(out) > idx else default
                elif isinstance(out, (tuple, list)):
                    return float(out[idx]) if len(out) > idx else default
                elif isinstance(out, np.ndarray):
                    return float(out[idx]) if out.size > idx else default
                else:
                    return default
            except (IndexError, TypeError):
                return default

        if isinstance(output, (tuple, list, np.ndarray)) or torch.is_tensor(output):
            self.steering = _extract(output, 0)
            self.throttle = _extract(output, 1)
            self.speed = _extract(output, 2)
        else:
            print(f"Unexpected output format: {type(output)}")
            self.steering = 0.0
            self.throttle = 0.0
            self.speed = 0.0

    def cleanup(self):
        print("Planner cleanup complete.")
        pass


class SpeedPIDController:
    """
    速度フィードバックPID制御

    モデルが推論した目標速度(target_speed)と、RPM/オプティカルフローから算出した
    現在速度(current_speed)の差分をPID制御し、throttle値(-1.0〜1.0)を出力する。
    """

    def __init__(self):
        self.kp = config.SPEED_PID_KP
        self.ki = config.SPEED_PID_KI
        self.kd = config.SPEED_PID_KD
        self.integral_limit = config.SPEED_PID_INTEGRAL_LIMIT

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.perf_counter()

    def reset(self):
        """PID状態をリセット"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.perf_counter()

    def compute(self, target_speed, current_speed):
        """
        PID制御でthrottle値を算出する。

        Args:
            target_speed: 目標速度 (m/s) — モデル推論出力
            current_speed: 現在速度 (m/s) — センサー計測値

        Returns:
            throttle: -1.0〜1.0
        """
        now = time.perf_counter()
        dt = now - self._prev_time
        self._prev_time = now

        if dt <= 0:
            dt = 1e-6

        # 偏差
        error = target_speed - current_speed

        # 積分項（ワインドアップ防止）
        self._integral += error * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        # 微分項
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        # PID出力
        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # -1.0〜1.0にクリッピング
        throttle = max(-1.0, min(1.0, output))

        return throttle


def estimate_speed(rpm_speed, of_vy, source='rpm'):
    """
    RPMセンサーとオプティカルフローから現在速度(m/s)を推定する。

    Args:
        rpm_speed: RPMセンサーからの速度 (m/s)
        of_vy: オプティカルフローの前進方向速度 (m/s)
        source: 'rpm', 'optical_flow', 'fused'

    Returns:
        speed: 推定速度 (m/s)
    """
    if source == 'rpm':
        return abs(rpm_speed)
    elif source == 'optical_flow':
        return abs(of_vy)
    elif source == 'fused':
        if rpm_speed > 0:
            return abs(rpm_speed)
        else:
            return abs(of_vy)
    else:
        return abs(rpm_speed)


# imuを用いた走行制御
class DynamicControl:
    def __init__(self, mode=None):
        self.gain_steering = 1.0
        self.gain_throttle = 1.0

    def update_control(self, throttle_gain, steering_gain):
        """動的制御のゲインを更新"""
        self.gain_throttle = throttle_gain
        self.gain_steering = steering_gain

    def counter_steering(self, gyro_data, steering, throttle):
        """
        カウンターステア強度を計算する関数

        Args:
            gyro_data (dict): ジャイロデータを格納した辞書。キー "z" に回転速度のリストが含まれることを想定。
            steering (float): 現在のステアリング値。
            throttle (float): 現在のスロットル値。

        Returns:
            tuple: 調整後のステアリング値、スロットル値。
        """
        # "z"キーが存在しない、またはリストが空の場合に例外を投げる
        if "z" not in gyro_data or not gyro_data["z"]:
            raise ValueError("gyro_data に 'z' キーが存在しないか、リストが空です。")

        # z軸の回転速度の平均を計算
        average_rotation_speed = abs(sum(gyro_data["z"]) / len(gyro_data["z"]))

        # カウンターステア強度を計算し、1を超えないように制限
        counter_steering_strength = min(1, average_rotation_speed / self.rotation_speed)

        # ステアリング値にカウンターステア強度を適用
        adjusted_steering = steering * (1 - counter_steering_strength)

        # スロットル値をそのまま返却（必要に応じて調整可能）
        adjusted_throttle = throttle

        return adjusted_steering, adjusted_throttle

    def lateral_g_throttle(self, acc_data, jerk_data, steering, throttle):
        """
        横Gスロットル制御を計算する関数

        Args:
            acc_data (dict): 加速度データを格納した辞書。キー "y" にy軸方向のデータが含まれることを想定。
            jerk_data (dict): ジャーク（加速度の時間微分）データを格納した辞書。キー "y" にy軸方向のデータが含まれることを想定。
            steering (float): 現在のステアリング値。
            throttle (float): 現在のスロットル値。

        Returns:
            tuple: 調整後のステアリング値とスロットル値 (steering, throttle)。
        """
        # "y"キーが存在しない、またはリストが空の場合に例外を投げる
        if "y" not in acc_data or not acc_data["y"]:
            raise ValueError("acc_data に 'y' キーが存在しないか、リストが空です。")
        if "y" not in jerk_data or not jerk_data["y"]:
            raise ValueError("jerk_data に 'y' キーが存在しないか、リストが空です。")

        # 最新のy軸加速度とジャークの値を取得
        last_acc_y = acc_data["y"][-1]
        last_jerk_y = jerk_data["y"][-1]

        # 横Gスロットル制御量を計算
        lateral_g_control = abs((last_acc_y * last_jerk_y) * self.Cxy / (1 + self.Ts) * abs(last_jerk_y))

        # スロットル値を横Gスロットル制御量に基づいて制限
        adjusted_throttle = min(1, lateral_g_control)

        # ステアリング値はそのまま返す
        adjusted_steering = steering

        return adjusted_steering, adjusted_throttle

class LapCounter:
    def __init__(self):
        self.current_lap = 0
        self.last_checkpoint_time = None

    def increment_lap(self):
        """周回数を1増加させる"""
        self.current_lap += 1
        print(f"Lap incremented: {self.current_lap}")

    def reset_lap(self):
        """周回数をリセット"""
        self.current_lap = 0

    def get_lap_count(self):
        """現在の周回数を取得"""
        return self.current_lap
    
# TODO:CustompPlanの相談
class MyCustomPlanner(Planner):
    pass
        
class DefaultPlanner(Planner):
    def __init__(self):
        super().__init__()
    
    def planning_sequence(self, mode, plan, data_aggregator, model, inference_camera_image=None,
                          position_model=None, position_models_dict=None,
                          yolo_model=None, yolo_models_dict=None,
                          camera_images=None, ranges=None, lidar_data=None):
        """
        判断シーケンス（モデル選択含む）

        Args:
            mode: 走行モード
            plan: プラン名
            data_aggregator: データ集約器
            model: 基本モデル
            inference_camera_image: 推論用カメラ画像
            position_model: 位置推論モデル
            position_models_dict: 位置別モデル辞書
            yolo_model: YOLOモデル
            yolo_models_dict: クラス別モデル辞書
            camera_images: カメラ画像辞書
            ranges: 測距センサーデータ（位置名: 距離値の辞書、ultrasonic/lidar共通）

        Returns:
            steering_value, throttle_value: 制御値
        """
        # 最優先: リカバリー状態に入るか確認（早期リターン）
        if config.RECOVERY_MODE == "back" and mode not in ("user", "auto_str"):
            if self.recovery_back(data_aggregator):
                if self.recovery_phase == "brake":
                    return 0.0, -1.0   # ブレーキ（ESCにブレーキ信号）
                elif self.recovery_phase == "neutral":
                    return 0.0, 0.0    # ニュートラル
                else:  # "back"
                    return self.recovery_steering, -1.0  # バック（ステアリングはセンサーまたは固定値）

        # モデル選択（自動運転モード時のみ）
        active_model = model
        if mode != "user":
            # 位置推論によるモデル選択
            active_model = self._select_model_by_position(
                mode, position_model, position_models_dict, camera_images, active_model
            )

            # YOLO検知によるモデル選択（優先）
            active_model = self._select_model_by_yolo(
                mode, yolo_model, yolo_models_dict, inference_camera_image, active_model
            )

        # Follow the Gap用LiDARデータ
        if lidar_data is not None:
            self._lidar_data = lidar_data
        elif "lidar" in getattr(config, 'ACTIVE_SENSORS', []):
            self._lidar_data = data_aggregator.get_latest_sensor_value("lidar")

        # 測距センサーデータ（run.pyから渡される、既にマッピング済み）
        # rangesがNoneの場合は後方互換のためdata_aggregatorから取得
        if ranges is None:
            ranges = {}
            for sensor_position in config.ULTRASONIC_SENSOR_LIST:
                ranges[sensor_position] = data_aggregator.get_latest_sensor_value(sensor_position)

        # 制御値を計算（選択されたモデルを使用）
        steering_value, throttle_value = self.compute_motor_commands(
            mode, plan, ranges, active_model, inference_camera_image,
            data_aggregator=data_aggregator
        )

        # YOLO検知による制御値修正（自動運転時のみ）
        if config.USE_YOLO_DETECTION and mode != "user" and self.current_detections:
            # 1. 障害物回避制御（最優先：ステアリング補正）
            obstacle_avoidance_applied = False
            if config.USE_YOLO_OBSTACLE_AVOIDANCE:
                avoidance_steering, obstacle_info = calculate_obstacle_avoidance_steering(
                    self.current_detections, config.IMAGE_W, config.IMAGE_H
                )
                if obstacle_info:
                    steering_value += avoidance_steering
                    # 範囲制限
                    steering_value = max(-1.0, min(1.0, steering_value))
                    obstacle_avoidance_applied = True
                    if config.YOLO_DISPLAY_DETECTIONS:
                        logger.info(
                            f"障害物回避: {obstacle_info['class_name']} "
                            f"(信頼度: {obstacle_info['confidence']:.2f}, "
                            f"サイズ比: {obstacle_info['area_ratio']:.2%}, "
                            f"回避方向: {obstacle_info['avoidance_direction']}, "
                            f"補正: {obstacle_info['steering_offset']:.2f})"
                        )

            # 2. 物体追従制御（障害物回避が適用されていない場合のみ）
            if config.USE_YOLO_OBJECT_TRACKING and not obstacle_avoidance_applied:
                steering_offset, tracking_info = calculate_object_tracking_steering(
                    self.current_detections, config.IMAGE_W
                )
                if tracking_info:
                    steering_value += steering_offset
                    # 範囲制限
                    steering_value = max(-1.0, min(1.0, steering_value))
                    if config.YOLO_DISPLAY_DETECTIONS:
                        logger.info(
                            f"物体追従: {tracking_info['class_name']} "
                            f"(信頼度: {tracking_info['confidence']:.2f}, "
                            f"オフセット: {tracking_info['offset']:.2f}, "
                            f"補正: {tracking_info['steering_offset']:.2f})"
                        )

            # 3. YOLO制御ルール適用（スロットル修正等）
            modified_steering, modified_throttle, applied_rule = apply_detection_control_modification(
                self.current_detections, steering_value, throttle_value
            )
            if applied_rule:
                if config.YOLO_DISPLAY_DETECTIONS:
                    logger.info(f"制御修正適用: {applied_rule['description']} ({applied_rule['class_name']}, {applied_rule['confidence']:.2f})")
                steering_value = modified_steering
                throttle_value = modified_throttle

        # 制御値を -1.0〜1.0 にクリッピング
        steering_value = max(-1.0, min(1.0, steering_value))
        throttle_value = max(-1.0, min(1.0, throttle_value))

        return steering_value, throttle_value

# ROS2の有無を判定してインポート
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Bool, String, Float32MultiArray
    from geometry_msgs.msg import Twist

    class PlannerNode(Node):
        def __init__(self):
            super().__init__('planner_node')

            # DefaultPlannerインスタンス化
            self.planner = DefaultPlanner()

            # 状態変数
            self.mode = "user"
            self.joystick_steering = 0.0
            self.joystick_throttle = 0.0
            self.ranges = {}
            self.steering = 0.0
            self.throttle = 0.0

            # サブスクライバー
            self.create_subscription(String, '/joy/mode', self.mode_callback, 10)
            self.create_subscription(Twist, '/cmd_vel_joy', self.joy_cmd_callback, 10)
            self.create_subscription(Float32MultiArray, '/ultrasonic_data', self.ultrasonic_callback, 10)

            # パブリッシャー（統一: /cmd_vel）
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

            # タイマーで定期的にプランニング実行
            self.timer = self.create_timer(0.05, self.planning_loop)

            self.get_logger().info(f"Planner node started (plan={config.PLAN})")

        def mode_callback(self, msg):
            self.mode = msg.data

        def joy_cmd_callback(self, msg):
            self.joystick_steering = msg.angular.z
            self.joystick_throttle = msg.linear.x

        def ultrasonic_callback(self, msg):
            for i, name in enumerate(config.ULTRASONIC_SENSOR_LIST):
                if i < len(msg.data):
                    self.ranges[name] = msg.data[i]

        def planning_loop(self):
            cmd = Twist()

            if self.mode == "user":
                cmd.angular.z = float(self.joystick_steering)
                cmd.linear.x = float(self.joystick_throttle)
            else:
                # 自動モード: DefaultPlannerで計算
                steering, throttle = self.planner.compute_motor_commands(
                    self.mode, config.PLAN, self.ranges)
                cmd.angular.z = float(steering)
                cmd.linear.x = float(throttle)

            self.cmd_pub.publish(cmd)

    def main_ros(args=None):
        rclpy.init(args=args)
        node = PlannerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

except ImportError:
    # print("ROS2関連ライブラリがインストールされていません。ROS2モードは無効です。")
    rclpy = None

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Planning only with ROS2")
    parser.add_argument('--ros', action='store_true', help="Run with ROS2 node")
    args = parser.parse_args()

    if args.ros and rclpy:
        print("Start with ROS2")
        main_ros()
    else:
        # right_left_3のステアリング値計算テスト
        print("=" * 60)
        print("right_left_3 ステアリング値計算テスト")
        print("=" * 60)
        print("3つのセンサー値を入力してステアリング値を計算します")
        print("終了するには Ctrl+C を押してください\n")
        
        planner = Planner()
        
        try:
            while True:
                print("-" * 40)
                # センサー値のテストパターン
                test_cases = [
                    {"FrLH": 300, "FrFR": 500, "FrRH": 600, "name": "左壁接近"},
                    {"FrLH": 600, "FrFR": 500, "FrRH": 300, "name": "右壁接近"},
                    {"FrLH": 400, "FrFR": 200, "FrRH": 400, "name": "前方障害物"},
                    {"FrLH": 800, "FrFR": 800, "FrRH": 800, "name": "障害物なし"},
                    {"FrLH": 200, "FrFR": 600, "FrRH": 700, "name": "左壁非常に接近"},
                ]
                
                print("テストパターンを選択:")
                for i, case in enumerate(test_cases, 1):
                    print(f"{i}. {case['name']} (左:{case['FrLH']}mm, 前:{case['FrFR']}mm, 右:{case['FrRH']}mm)")
                print("6. カスタム値を入力")
                
                choice = input("\n選択 (1-6): ").strip()
                
                if choice in ['1', '2', '3', '4', '5']:
                    idx = int(choice) - 1
                    dis_FrLH = test_cases[idx]["FrLH"]
                    dis_FrFR = test_cases[idx]["FrFR"]
                    dis_FrRH = test_cases[idx]["FrRH"]
                    print(f"\n選択: {test_cases[idx]['name']}")
                elif choice == '6':
                    try:
                        dis_FrLH = float(input("左センサー値 (FrLH) [mm]: "))
                        dis_FrFR = float(input("前センサー値 (FrFR) [mm]: "))
                        dis_FrRH = float(input("右センサー値 (FrRH) [mm]: "))
                    except ValueError:
                        print("無効な入力です。数値を入力してください。")
                        continue
                else:
                    print("無効な選択です。")
                    continue
                
                # right_left_3メソッドを呼び出してステアリング値を計算
                steering, throttle = planner.right_left_3(dis_FrLH, dis_FrFR, dis_FrRH)
                
                # 結果を表示
                print(f"\n【センサー値】")
                print(f"  左(FrLH): {dis_FrLH:6.1f} mm")
                print(f"  前(FrFR): {dis_FrFR:6.1f} mm")
                print(f"  右(FrRH): {dis_FrRH:6.1f} mm")
                print(f"\n【計算結果】")
                print(f"  ステアリング: {steering:6.2f} ", end="")
                if steering < 0:
                    print("(左旋回)")
                elif steering > 0:
                    print("(右旋回)")
                else:
                    print("(直進)")
                print(f"  スロットル:   {throttle:6.2f}")
                
                # 判定ロジックの説明
                print(f"\n【判定理由】")
                if dis_FrFR < config.DETECTION_RANGE:
                    print(f"  前方に障害物検知 (前センサー {dis_FrFR}mm < {config.DETECTION_RANGE}mm)")
                    if dis_FrLH < dis_FrRH:
                        print(f"  左側が近い ({dis_FrLH}mm < 右{dis_FrRH}mm) → 右旋回")
                    else:
                        print(f"  右側が近い ({dis_FrRH}mm <= 左{dis_FrLH}mm) → 左旋回")
                elif dis_FrLH < config.RIGHT_LEFT_RANGE:
                    print(f"  左壁に接近 (左センサー {dis_FrLH}mm < {config.RIGHT_LEFT_RANGE}mm) → 右旋回")
                elif dis_FrRH < config.RIGHT_LEFT_RANGE:
                    print(f"  右壁に接近 (右センサー {dis_FrRH}mm < {config.RIGHT_LEFT_RANGE}mm) → 左旋回")
                else:
                    print(f"  障害物なし → 直進")
                
                input("\nEnterキーを押して続行...")
                
        except KeyboardInterrupt:
            print("\n\nテストを終了します。")

