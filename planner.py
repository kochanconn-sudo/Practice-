# coding:utf-8

import collections
import logging
import time
from typing import Optional, Dict, Any

import numpy as np
import torch
from torchvision import transforms
from PIL import Image

import config
from train_pytorch import normalize_ultrasonics
from position_inference import infer_position
from yolo_detection import (
    detect_objects,
    apply_detection_control_modification,
    select_model_by_detection,
    calculate_object_tracking_steering,
    calculate_obstacle_avoidance_steering,
)
from follow_the_gap import follow_the_gap

# 新しいセンサー認識層
from perception import Perception
from camera_wall_detector import CameraWallDetector

logger = logging.getLogger(__name__)


class Planner:
    def __init__(self):
        # 判断フラグ
        self.in_recovery = False
        self.recovery_phase = None
        self.recovery_phase_count = 0
        self.recovery_phase_start = 0
        self.recovery_steering = 0.0
        self.before_recovery_detection_times = 3
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

        # PID用タイマー
        self.time_current = time.perf_counter()
        self.time_before = time.perf_counter()

        # PID用最小距離
        self.minimum_distance_current = config.TARGET_RANGE
        self.minimum_distance_before = config.TARGET_RANGE
        self.integral_delta_distance = 0.0

        # 位置推論関連
        self.position_inference_counter = 0
        self.current_position_id = None
        self.current_driving_model = None

        # Follow the Gap用LiDARデータ
        self._lidar_data = None

        # YOLO検知関連
        self.yolo_detection_counter = 0
        self.current_detections = []
        self.yolo_active_model = None

        # 時系列モデル推論用
        self._seq_frame_buffer = collections.deque(maxlen=50)
        self._seq_transform = None

        # ====================================================
        # Perception / Camera Wall Detector
        # ====================================================
        self.perception_enabled = getattr(
            config,
            "PERCEPTION_ENABLED",
            True,
        )

        self.perception_apply_correction = getattr(
            config,
            "PERCEPTION_APPLY_CORRECTION",
            True,
        )

        self.perception = None
        self.camera_wall_detector = None

        if self.perception_enabled:
            try:
                self.perception = Perception()
                self.camera_wall_detector = CameraWallDetector()
                logger.info("Perception / CameraWallDetector 起動")
            except Exception as e:
                logger.error(
                    f"Perception初期化失敗: {e}"
                )
                self.perception = None
                self.camera_wall_detector = None
                self.perception_enabled = False

    # ========================================================
    # Perception更新
    # ========================================================

    def update_perception(
        self,
        ranges: Optional[Dict[str, Any]],
        camera_image=None,
        lidar_data=None,
        yolo_detections=None,
    ):
        """
        超音波 + カメラ + LiDAR + YOLOの認識情報をPerceptionへ渡す。

        camera_imageはcamera.pyから取得したnumpy.ndarrayを想定する。
        カメラ画像がない場合でも超音波のみでPerceptionは動作する。
        """
        if not self.perception_enabled or self.perception is None:
            return None

        if ranges is None:
            ranges = {}

        # カメラ画像 -> 壁/コーナー/障害物候補
        camera_data = {}

        if (
            self.camera_wall_detector is not None
            and camera_image is not None
            and isinstance(camera_image, np.ndarray)
        ):
            try:
                camera_data = self.camera_wall_detector.analyze(
                    camera_image
                )
            except Exception as e:
                logger.warning(
                    f"CameraWallDetector解析失敗: {e}"
                )
                camera_data = {}

        # YOLOは既存Plannerの検知結果をそのまま渡す
        if yolo_detections is None:
            yolo_data = {
                "detections": []
            }
        else:
            yolo_data = {
                "detections": yolo_detections
            }

        try:
            result = self.perception.update_normalized(
                ultrasonic_data=ranges,
                camera_data=camera_data,
                lidar_data=lidar_data,
                yolo_data=yolo_data,
            )
            return result
        except Exception as e:
            logger.warning(
                f"Perception更新失敗: {e}"
            )
            return None

    # ========================================================
    # Perception補正
    # ========================================================

    def apply_perception_correction(
        self,
        mode,
        steering_value,
        throttle_value,
    ):
        """
        既存走行モードの計算結果へPerceptionの補正を適用する。

        既存のwall_follow / racer / gap_follow等を壊さず、
        カメラ・超音波の結果を上から補正する構造。
        """
        if (
            not self.perception_enabled
            or not self.perception_apply_correction
            or self.perception is None
        ):
            return steering_value, throttle_value

        try:
            planner_input = self.perception.get_planner_input(
                mode=mode,
                base_steering=steering_value,
                base_throttle=throttle_value,
            )

            steering_value = planner_input["steering"]
            throttle_value = planner_input["throttle"]

            # 緊急停止は最優先
            if planner_input.get("must_stop", False):
                steering_value = 0.0
                throttle_value = 0.0

            return steering_value, throttle_value

        except Exception as e:
            logger.warning(
                f"Perception補正失敗: {e}"
            )
            return steering_value, throttle_value

    # ========================================================
    # 位置によるモデル選択
    # ========================================================

    def _select_model_by_position(
        self,
        mode,
        position_model,
        position_models_dict,
        camera_images,
        default_model,
    ):
        if not config.USE_POSITION_SWITCHING or mode == "user" or position_model is None:
            return default_model

        self.position_inference_counter += 1

        if self.position_inference_counter >= config.POSITION_INFERENCE_INTERVAL:
            self.position_inference_counter = 0

            position_image = None

            if hasattr(config, "POSITION_MODEL_INPUT_IMAGE") and camera_images:
                for ci in range(4):
                    if f"cam{ci}" in config.POSITION_MODEL_INPUT_IMAGE:
                        position_image = camera_images.get(f"camera_{ci}")
                        break

            if position_image is None and camera_images:
                position_image = camera_images.get("camera_0")

            if position_image is not None:
                inferred_position, confidence = infer_position(
                    position_model,
                    position_image,
                )

                if inferred_position is not None:
                    if inferred_position != self.current_position_id:
                        position_name = (
                            config.POSITION_CLASS_NAMES[inferred_position]
                            if inferred_position < len(config.POSITION_CLASS_NAMES)
                            else f"Position{inferred_position}"
                        )

                        logger.info(
                            f"位置が変更されました: {position_name} "
                            f"(信頼度: {confidence:.2f})"
                        )

                        self.current_position_id = inferred_position

                        if inferred_position in position_models_dict:
                            self.current_driving_model = position_models_dict[inferred_position]
                        elif "default" in position_models_dict:
                            self.current_driving_model = position_models_dict["default"]
                        else:
                            self.current_driving_model = default_model

        return (
            self.current_driving_model
            if self.current_driving_model is not None
            else default_model
        )

    # ========================================================
    # YOLOによるモデル選択
    # ========================================================

    def _select_model_by_yolo(
        self,
        mode,
        yolo_model,
        yolo_models_dict,
        inference_camera_image,
        default_model,
    ):
        if not config.USE_YOLO_DETECTION or mode == "user" or yolo_model is None:
            return default_model

        self.yolo_detection_counter += 1

        if self.yolo_detection_counter >= config.YOLO_DETECTION_INTERVAL:
            self.yolo_detection_counter = 0

            if inference_camera_image is not None:
                self.current_detections = detect_objects(
                    yolo_model,
                    inference_camera_image,
                )

                if config.YOLO_DISPLAY_DETECTIONS and self.current_detections:
                    detection_summary = ", ".join(
                        [
                            f"{d['class_name']}({d['confidence']:.2f})"
                            for d in self.current_detections
                        ]
                    )
                    logger.info(
                        f"物体検知: {detection_summary}"
                    )

                if config.YOLO_MODEL_SWITCHING and yolo_models_dict:
                    self.yolo_active_model, detected_class = select_model_by_detection(
                        self.current_detections,
                        yolo_models_dict,
                        default_model,
                    )

                    if detected_class:
                        logger.info(
                            f"検知によるモデル切り替え: "
                            f"{detected_class['class_name']} "
                            f"(信頼度: {detected_class['confidence']:.2f})"
                        )
                        return self.yolo_active_model

        return (
            self.yolo_active_model
            if self.yolo_active_model is not None
            else default_model
        )

    # ========================================================
    # モーター指令計算
    # ========================================================

    def compute_motor_commands(
        self,
        mode,
        plan,
        ranges,
        model=None,
        camera_image=None,
        data_aggregator=None,
    ):
        """
        既存走行モードの制御値計算。

        Perception補正はplanning_sequence()の最後で行う。
        """
        if plan == "go_straight":
            if ranges["FrFR"] < config.STOP_RANGE:
                return 0.0, 0.0
            return 0.0, config.FORWARD_STRAIGHT

        elif plan == "right_left_3":
            inputs = (
                ranges["FrLH"],
                ranges["FrFR"],
                ranges["FrRH"],
            )
            return self.right_left_3(*inputs)

        elif plan == "right_left_3_records":
            inputs = (
                ranges["FrLH"],
                ranges["FrFR"],
                ranges["FrRH"],
            )
            return self.right_left_3_records(*inputs)

        elif plan == "wall_follow":
            side = config.HAND_SIDE
            range_front = ranges["FrFR"]
            range_front_side = (
                ranges["FrRH"]
                if side == "right"
                else ranges["FrLH"]
            )
            range_rear_side = (
                ranges.get("RrRH", range_front_side)
                if side == "right"
                else ranges.get("RrLH", range_front_side)
            )
            return self.wall_follow(
                range_front,
                range_front_side,
                range_rear_side,
                side,
            )

        elif plan == "wall_follow_pid":
            side = config.HAND_SIDE
            range_front = ranges["FrFR"]
            range_front_side = (
                ranges["FrRH"]
                if side == "right"
                else ranges["FrLH"]
            )
            range_rear_side = (
                ranges.get("RrRH", range_front_side)
                if side == "right"
                else ranges.get("RrLH", range_front_side)
            )
            return self.wall_follow_pid(
                range_front,
                range_front_side,
                range_rear_side,
                side,
            )

        elif plan == "nn":
            if not model:
                logger.warning(
                    f"PLAN='{plan}' ですがモデルが未ロードです。"
                    f" MODEL_PATHを確認してください: "
                    f"{getattr(config, 'MODEL_PATH', '未設定')}"
                )
                return 0.0, 0.0

            inputs = [
                ranges[key]
                for key in ranges
            ]
            return self.nn(
                model,
                *inputs,
            )

        elif plan in [
            "donkeycar",
            "resnet18",
            "mobilevit_xxs",
            "edgenext_xx_small",
        ]:
            if not model or camera_image is None:
                logger.warning(
                    f"PLAN='{plan}' ですが"
                    f"{'モデルが未ロードです。MODEL_PATHを確認してください: ' + str(getattr(config, 'MODEL_PATH', '未設定')) if not model else 'カメラ画像がありません'}"
                )
                return 0.0, 0.0

            return self.model_catalog_inference(
                model,
                camera_image,
            )

        elif plan in [
            "gru",
            "tcn",
            "causal_cnn",
        ]:
            if not model or camera_image is None:
                logger.warning(
                    f"PLAN='{plan}' ですが"
                    f"{'モデルが未ロードです。MODEL_PATHを確認してください: ' + str(getattr(config, 'MODEL_PATH', '未設定')) if not model else 'カメラ画像がありません'}"
                )
                return 0.0, 0.0

            return self.sequence_model_inference(
                model,
                camera_image,
                data_aggregator,
            )

        elif plan == "center_follow_pid":
            return self.center_follow_pid(
                ranges["FrFR"],
                ranges["FrLH"],
                ranges.get("RrLH", ranges["FrLH"]),
                ranges["FrRH"],
                ranges.get("RrRH", ranges["FrRH"]),
            )

        elif plan == "gap_follow":
            inputs = (
                ranges["FrLH"],
                ranges["FrFR"],
                ranges["FrRH"],
            )
            return self.gap_follow(*inputs)

        elif plan == "racer":
            inputs = (
                ranges["FrLH"],
                ranges["FrFR"],
                ranges["FrRH"],
            )
            return self.racer(*inputs)

        elif plan == "follow_the_gap":
            lidar_data = self._lidar_data

            if lidar_data is not None:
                return follow_the_gap(
                    lidar_data
                )

            logger.warning(
                "follow_the_gap: LiDARデータなし"
            )
            return 0.0, 0.0

        else:
            logger.warning(
                f"不明なプラン: '{plan}'。"
                f" PLAN_LISTから選択してください: "
                f"{getattr(config, 'PLAN_LIST', [])}"
            )
            return 0.0, 0.0

    # ========================================================
    # Recovery
    # ========================================================

    def recovery_stop(self, ultrasonic_Fr):
        times = 3

        if max(
            ultrasonic_Fr.records[
                0:self.before_recovery_detection_times - 1
            ]
        ) < config.STOP_RANGE:
            self.in_recovery = True
            print("停止")

    def recovery_back(self, data_aggregator):
        BRAKE_DURATION = config.RECOVERY_BRAKE_DURATION
        NEUTRAL_DURATION = config.RECOVERY_NEUTRAL_DURATION

        if self.in_recovery:
            now = time.perf_counter()
            phase_elapsed = (
                now
                - self.recovery_phase_start
            )

            if self.recovery_phase == "brake":
                if phase_elapsed >= BRAKE_DURATION:
                    self.recovery_phase = "neutral"
                    self.recovery_phase_start = now
                    print(
                        f"RECOVERY PHASE: neutral "
                        f"({self.recovery_phase_count}/{config.RECOVERY_BRAKING})"
                    )

            elif self.recovery_phase == "neutral":
                if phase_elapsed >= NEUTRAL_DURATION:
                    if self.recovery_phase_count < config.RECOVERY_BRAKING:
                        self.recovery_phase_count += 1
                        self.recovery_phase = "brake"
                        self.recovery_phase_start = now
                        print(
                            f"RECOVERY PHASE: brake "
                            f"({self.recovery_phase_count}/{config.RECOVERY_BRAKING})"
                        )
                    else:
                        self.recovery_phase = "back"
                        self.recovery_phase_start = now
                        self.recovery_time_end = (
                            now
                            + config.RECOVERY_TIME_DURATION
                        )
                        print(
                            "RECOVERY PHASE: back"
                        )

            elif self.recovery_phase == "back":
                self.recovery_time_remaining = (
                    self.recovery_time_end
                    - now
                )

                if self.recovery_time_remaining <= 0:
                    self.in_recovery = False
                    self.recovery_phase = None
                    print("RECOVERY END")
                else:
                    print(
                        f"RECOVERY TIME REMAINING: "
                        f"{self.recovery_time_remaining:.2f}"
                    )

        else:
            n = self.before_recovery_detection_times

            FrFR_history = data_aggregator.get_sensor_history("FrFR")
            FrRH_history = data_aggregator.get_sensor_history("FrRH")
            FrLH_history = data_aggregator.get_sensor_history("FrLH")

            recent_FrFR = FrFR_history[-n:]
            recent_FrRH = FrRH_history[-n:]
            recent_FrLH = FrLH_history[-n:]

            if (
                len(recent_FrFR) > 0
                and len(recent_FrRH) > 0
                and len(recent_FrLH) > 0
            ):
                min_of_max = min(
                    max(recent_FrFR),
                    max(recent_FrRH),
                    max(recent_FrLH),
                )

                if min_of_max < config.BACKWARD_RANGE:
                    self.in_recovery = True
                    self.recovery_phase = "brake"
                    self.recovery_phase_count = 1
                    self.recovery_phase_start = time.perf_counter()
                    self.recovery_time_start = self.recovery_phase_start

                    if config.RECOVERY_STEERING == "auto":
                        max_LH = max(recent_FrLH)
                        max_RH = max(recent_FrRH)

                        if max_LH > max_RH:
                            self.recovery_steering = 1.0
                        else:
                            self.recovery_steering = -1.0

                        print(
                            f"RECOVERY START - PHASE: brake, "
                            f"auto steering={self.recovery_steering} "
                            f"(LH={max_LH:.0f}, RH={max_RH:.0f})"
                        )
                    else:
                        self.recovery_steering = float(
                            config.RECOVERY_STEERING
                        )
                        print(
                            f"RECOVERY START - PHASE: brake, "
                            f"fixed steering={self.recovery_steering}"
                        )

        return self.in_recovery

    # ========================================================
    # right_left_3
    # ========================================================

    def right_left_3(
        self,
        dis_FrLH,
        dis_FrFR,
        dis_FrRH,
    ):
        if (
            dis_FrFR < config.DETECTION_RANGE
            or dis_FrLH < config.RIGHT_LEFT_RANGE
            or dis_FrRH < config.RIGHT_LEFT_RANGE
        ):
            if dis_FrLH < dis_FrRH:
                self.steering = 1.0
                self.throttle = config.FORWARD_CORNER
                self.message = "右旋回"
            else:
                self.steering = -1.0
                self.throttle = config.FORWARD_CORNER
                self.message = "左旋回"
        else:
            self.steering = 0.0
            self.throttle = config.FORWARD_STRAIGHT
            self.message = "直進中"

        if config.TERMINAL_PRINT:
            print(self.message)

        return self.steering, self.throttle

    def right_left_3_records(
        self,
        dis_FrLH,
        dis_FrFR,
        dis_FrRH,
    ):
        self.steering, self.throttle = (
            self.right_left_3(
                dis_FrLH,
                dis_FrFR,
                dis_FrRH,
            )
        )

        self.records_steering = np.insert(
            self.records_steering,
            0,
            self.steering,
        )
        self.records_steering = np.delete(
            self.records_steering,
            -1,
        )

        self.records_throttle = np.insert(
            self.records_throttle,
            0,
            self.throttle,
        )
        self.records_throttle = np.delete(
            self.records_throttle,
            -1,
        )

        return (
            round(
                np.mean(
                    self.records_steering
                ),
                2,
            ),
            round(
                np.mean(
                    self.records_throttle
                ),
                2,
            ),
        )

    # ========================================================
    # gap_follow
    # ========================================================

    def gap_follow(
        self,
        dis_FrLH,
        dis_FrFR,
        dis_FrRH,
    ):
        L, F, R = (
            dis_FrLH,
            dis_FrFR,
            dis_FrRH,
        )

        g_steer = getattr(
            config,
            "GAP_STEER_GAIN",
            1.0,
        )

        diff = (
            L - R
        ) / (
            L + R + 1
        )

        steering = max(
            -1.0,
            min(
                1.0,
                -g_steer * diff,
            ),
        )

        if (
            F
            < getattr(
                config,
                "GAP_FOLLOW_BRAKE_DIST",
                config.DETECTION_RANGE,
            )
            or abs(steering) > 0.5
        ):
            throttle = config.FORWARD_CORNER
        else:
            throttle = config.FORWARD_STRAIGHT

        return (
            round(steering, 2),
            round(throttle, 2),
        )

    # ========================================================
    # racer
    # ========================================================

    def racer(
        self,
        dis_FrLH,
        dis_FrFR,
        dis_FrRH,
    ):
        L, F, R = (
            dis_FrLH,
            dis_FrFR,
            dis_FrRH,
        )

        g_steer = getattr(
            config,
            "RACER_STEER_GAIN",
            0.8,
        )

        g_speed = getattr(
            config,
            "RACER_SPEED_GAIN",
            1.0,
        )

        brake = getattr(
            config,
            "RACER_BRAKE_DIST",
            500,
        )

        slow = getattr(
            config,
            "RACER_STEER_SLOWDOWN",
            0.5,
        )

        ceil = getattr(
            config,
            "RACER_SPEED_CEIL",
            0.9,
        )

        vmin = config.FORWARD_CORNER
        vmax = config.FORWARD_STRAIGHT

        diff = (
            L - R
        ) / (
            L + R + 1
        )

        steering = max(
            -1.0,
            min(
                1.0,
                -g_steer * diff,
            ),
        )

        base = (
            1.0
            if F >= brake
            else max(
                0.0,
                F / brake,
            )
        )

        throttle = (
            vmin
            +
            (vmax - vmin)
            * base
            * g_speed
        )

        throttle *= (
            1.0
            - slow * abs(steering)
        )

        throttle = max(
            vmin,
            min(
                ceil,
                throttle,
            ),
        )

        return (
            round(steering, 2),
            round(throttle, 2),
        )

    # ========================================================
    # 壁角度
    # ========================================================

    def _calc_wall_angle(
        self,
        d_front_side,
        d_rear_side,
        side,
    ):
        import math

        sin45 = math.sin(
            math.radians(45)
        )
        cos45 = math.cos(
            math.radians(45)
        )

        if side == "right":
            dx = (
                d_front_side * sin45
                - d_rear_side
            )
            dy = (
                d_front_side * cos45
            )
        else:
            dx = (
                -d_front_side * sin45
                + d_rear_side
            )
            dy = (
                d_front_side * cos45
            )

        return math.atan2(
            dx,
            dy,
        )

    # ========================================================
    # wall_follow
    # ========================================================

    def wall_follow(
        self,
        dis_front,
        dis_front_side,
        dis_rear_side,
        side="right",
    ):
        if side not in [
            "right",
            "left",
        ]:
            raise ValueError(
                "Invalid side. Expected 'right' or 'left'."
            )

        target_range = config.TARGET_RANGE
        adjustment = config.TARGET_RANGE_ADJUSTMENT

        if (
            dis_front_side > target_range + adjustment
            and dis_rear_side > target_range + adjustment
        ):
            self.steering = (
                1.0
                if side == "right"
                else -1.0
            )
            self.throttle = config.FORWARD_CORNER
            self.message = (
                f"{side}手法: 壁が遠い、{side}旋回"
            )

        elif (
            dis_front_side < target_range - adjustment
            or dis_rear_side < target_range - adjustment
        ):
            self.steering = (
                -1.0
                if side == "right"
                else 1.0
            )
            self.throttle = config.FORWARD_CORNER
            self.message = (
                f"{side}手法: 壁が近い"
            )

        else:
            self.steering = 0.0
            self.throttle = config.FORWARD_STRAIGHT
            self.message = (
                f"{side}手法: 壁沿い直進中"
            )

        if getattr(
            config,
            "WALL_FOLLOW_USE_ALIGNMENT",
            getattr(
                config,
                "ALL_FOLLOW_USE_ALIGNMENT",
                False,
            ),
        ):
            wall_angle = self._calc_wall_angle(
                dis_front_side,
                dis_rear_side,
                side,
            )

            angle_correction = (
                getattr(
                    config,
                    "WALL_FOLLOW_K_ANGLE",
                    0.3,
                )
                * wall_angle
            )

            if self.steering == 0.0:
                self.steering = max(
                    -1,
                    min(
                        1,
                        angle_correction,
                    ),
                )

                if abs(wall_angle) > 0.1:
                    self.throttle = config.FORWARD_CORNER

        if config.TERMINAL_PRINT:
            print(self.message)

        return self.steering, self.throttle

    # ========================================================
    # wall_follow_pid
    # ========================================================

    def wall_follow_pid(
        self,
        ultrasonic_front,
        ultrasonic_front_side,
        ultrasonic_rear_side,
        side,
    ):
        self.time_before = self.time_current
        self.time_current = time.perf_counter()

        delta_t = (
            self.time_current
            - self.time_before
        )

        self.minimum_distance_before = (
            self.minimum_distance_current
        )

        self.minimum_distance_current = min(
            ultrasonic_front_side,
            ultrasonic_rear_side,
        )

        delta_dis = (
            self.minimum_distance_current
            - config.TARGET_RANGE
        )

        self.integral_delta_distance += delta_dis

        v = (
            self.minimum_distance_current
            - self.minimum_distance_before
        ) / delta_t if delta_t > 0 else 0

        if getattr(
            config,
            "WALL_FOLLOW_USE_ALIGNMENT",
            getattr(
                config,
                "ALL_FOLLOW_USE_ALIGNMENT",
                False,
            ),
        ):
            wall_angle = self._calc_wall_angle(
                ultrasonic_front_side,
                ultrasonic_rear_side,
                side,
            )

            angle_term = (
                getattr(
                    config,
                    "WALL_FOLLOW_K_ANGLE",
                    0.3,
                )
                * wall_angle
            )
        else:
            angle_term = 0.0

        kp = getattr(
            config,
            "K_P",
            0.0,
        )
        ki = getattr(
            config,
            "K_I",
            0.0,
        )
        kd = getattr(
            config,
            "K_D",
            0.0,
        )

        steering_gain = (
            kp * delta_dis
            - kd * v
            + ki * self.integral_delta_distance
            + angle_term
        )

        steering_gain = max(
            -1,
            min(
                1,
                steering_gain,
            ),
        )

        if config.TERMINAL_PRINT:
            self._print_pid_debug(
                side,
                steering_gain,
                delta_dis,
                self.integral_delta_distance,
                v,
            )

        if side == "right":
            self.steering = steering_gain
        elif side == "left":
            self.steering = -steering_gain
        else:
            raise ValueError(
                "Invalid side. Expected 'left' or 'right'."
            )

        if abs(self.steering) > 0.7:
            self.throttle = config.FORWARD_CORNER
        else:
            self.throttle = config.FORWARD_STRAIGHT

        return (
            round(self.steering, 2),
            round(self.throttle, 2),
        )

    # ========================================================
    # center_follow_pid
    # ========================================================

    def center_follow_pid(
        self,
        dis_front,
        dis_front_left,
        dis_rear_left,
        dis_front_right,
        dis_rear_right,
    ):
        self.time_before = self.time_current
        self.time_current = time.perf_counter()

        delta_t = max(
            self.time_current
            - self.time_before,
            0.01,
        )

        dis_left = min(
            dis_front_left,
            dis_rear_left,
        )

        dis_right = min(
            dis_front_right,
            dis_rear_right,
        )

        CENTER_FALLBACK = getattr(
            config,
            "CENTER_FALLBACK_RANGE",
            800,
        )

        left_lost = (
            dis_left
            > CENTER_FALLBACK
        )

        right_lost = (
            dis_right
            > CENTER_FALLBACK
        )

        if left_lost and right_lost:
            self.steering = 0.0
            self.throttle = config.FORWARD_STRAIGHT
            return (
                round(self.steering, 2),
                round(self.throttle, 2),
            )

        elif left_lost:
            delta_dis = (
                dis_right
                - config.TARGET_RANGE
            )
            sign = 1.0

        elif right_lost:
            delta_dis = (
                dis_left
                - config.TARGET_RANGE
            )
            sign = -1.0

        else:
            delta_dis = (
                dis_left
                - dis_right
            )
            sign = -1.0

        self.minimum_distance_before = (
            self.minimum_distance_current
        )

        self.minimum_distance_current = (
            delta_dis
        )

        v = (
            self.minimum_distance_current
            - self.minimum_distance_before
        ) / delta_t

        self.integral_delta_distance += (
            delta_dis
        )

        kp = getattr(
            config,
            "CENTER_K_P",
            getattr(config, "K_P", 0.0),
        )
        kd = getattr(
            config,
            "CENTER_K_D",
            getattr(config, "K_D", 0.0),
        )
        ki = getattr(
            config,
            "CENTER_K_I",
            getattr(config, "K_I", 0.0),
        )

        steering_gain = (
            kp * delta_dis
            - kd * v
            + ki * self.integral_delta_distance
        )

        steering_gain = max(
            -1.0,
            min(
                1.0,
                steering_gain,
            ),
        )

        self.steering = round(
            steering_gain * sign,
            2,
        )

        if abs(self.steering) > 0.7:
            self.throttle = config.FORWARD_CORNER
        else:
            self.throttle = config.FORWARD_STRAIGHT

        if config.TERMINAL_PRINT:
            print(
                f"CENTER PID: output={self.steering:.2f}, "
                f"[P={kp * delta_dis:.2f}, "
                f"I={ki * self.integral_delta_distance:.2f}, "
                f"D={kd * v:.2f}] "
                f"L={dis_left:.0f} R={dis_right:.0f}"
                + (" [L_LOST]" if left_lost else "")
                + (" [R_LOST]" if right_lost else "")
            )

        return self.steering, self.throttle

    # ========================================================
    # PID debug
    # ========================================================

    def _print_pid_debug(
        self,
        side,
        steering,
        delta_dis,
        integral_delta_distance,
        v,
    ):
        side_text = (
            "右手法"
            if side == "right"
            else "左手法"
        )

        print(
            f"{side_text} PID制御: "
            f"output={steering:.2f}, "
            f"[P={getattr(config, 'K_P', 0.0) * delta_dis:.2f}, "
            f"I={getattr(config, 'K_I', 0.0) * integral_delta_distance:.2f}, "
            f"D={getattr(config, 'K_D', 0.0) * v:.2f}]"
        )

    # ========================================================
    # NN
    # ========================================================

    def nn(self, model, *args):
        ultrasonic_values = args

        model_dtype = next(
            model.parameters()
        ).dtype

        device = next(
            model.parameters()
        ).device

        x = torch.tensor(
            ultrasonic_values,
            dtype=model_dtype,
        ).unsqueeze(0)

        norm_params = getattr(
            model,
            "_normalization_params",
            None,
        )

        if norm_params:
            norm_type = norm_params.get(
                "type",
                "zscore",
            )

            if norm_type == "clip_scale":
                clip_val = norm_params.get(
                    "clip_max",
                    2000.0,
                )
                x = torch.clamp(
                    x,
                    0,
                    clip_val,
                ) / clip_val

            elif (
                "X_mean" in norm_params
                and "X_std" in norm_params
            ):
                mean = torch.tensor(
                    norm_params["X_mean"],
                    dtype=model_dtype,
                )
                std = torch.tensor(
                    norm_params["X_std"],
                    dtype=model_dtype,
                )
                x = (
                    x - mean
                ) / (
                    std + 1e-8
                )

        else:
            x = normalize_ultrasonics(x)

        x = x.to(device)

        with torch.no_grad():
            if (
                hasattr(model, "predict")
                and norm_params is None
            ):
                output = model.predict(
                    model,
                    x,
                ).squeeze(0)
            else:
                output = model(
                    x
                ).squeeze(0)

        self.steering = float(
            output[0]
        )
        self.throttle = float(
            output[1]
        )

        return (
            self.steering,
            self.throttle,
        )

    # ========================================================
    # GPU前処理
    # ========================================================

    def _preprocess_frame_gpu(
        self,
        frame,
        img_size,
        device,
        dtype,
    ):
        t = torch.from_numpy(
            frame
        ).permute(
            2,
            0,
            1,
        ).unsqueeze(0)

        t = t.to(
            device=device,
            dtype=torch.float32,
        )

        t = torch.nn.functional.interpolate(
            t,
            size=img_size,
            mode="bilinear",
            align_corners=False,
        )

        t = t.squeeze(0) / 255.0

        return t.to(
            dtype=dtype
        )

    # ========================================================
    # 時系列モデル
    # ========================================================

    def sequence_model_inference(
        self,
        model,
        img,
        data_aggregator,
    ):
        seq_cfg = getattr(
            model,
            "_sequence_config",
            {},
        )

        seq_len = seq_cfg.get(
            "seq_len",
            8,
        )

        img_size = seq_cfg.get(
            "img_size",
            (128, 128),
        )

        try:
            device = next(
                model.parameters()
            ).device

            model_dtype = next(
                model.parameters()
            ).dtype

            frame_tensor = (
                self._preprocess_frame_gpu(
                    img,
                    img_size,
                    device,
                    model_dtype,
                )
            )

            self._seq_frame_buffer.append(
                frame_tensor
            )

            buf = list(
                self._seq_frame_buffer
            )

            while len(buf) < seq_len:
                buf.insert(
                    0,
                    buf[0],
                )

            cached_tensors = buf[-seq_len:]

            images = torch.stack(
                cached_tensors
            ).unsqueeze(
                1
            ).unsqueeze(
                0
            )

            if data_aggregator is not None:
                control_history = (
                    data_aggregator.get_control_history()
                )
            else:
                control_history = []

            while len(control_history) < seq_len:
                control_history.insert(
                    0,
                    (0.0, 0.0),
                )

            control_history = control_history[-seq_len:]

            ego_np = np.array(
                [
                    [
                        s,
                        t,
                        0.0,
                        0.0,
                        0.0,
                    ]
                    for s, t in control_history
                ],
                dtype=np.float32,
            )

            ego_states = (
                torch.from_numpy(
                    ego_np
                )
                .unsqueeze(0)
                .to(
                    device,
                    dtype=model_dtype,
                )
            )

            with torch.no_grad():
                trajectory = model(
                    images,
                    ego_states,
                )

            self.steering = float(
                trajectory[0, 0, 0].item()
            )

            self.throttle = float(
                trajectory[0, 0, 1].item()
            )

        except Exception as e:
            logger.error(
                f"Sequence model inference error: {e}"
            )
            import traceback
            traceback.print_exc()
            self.steering = 0.0
            self.throttle = 0.0

        return (
            self.steering,
            self.throttle,
        )

    # ========================================================
    # Model catalog
    # ========================================================

    def model_catalog_inference(
        self,
        model,
        img,
    ):
        if not isinstance(
            img,
            np.ndarray,
        ):
            raise TypeError(
                "Input img must be a numpy.ndarray, "
                f"but got {type(img)}"
            )

        try:
            if hasattr(
                model,
                "run",
            ):
                output = model.run(
                    img
                )
            else:
                output = (
                    self._direct_tensor_inference(
                        model,
                        img,
                    )
                )

            self._process_model_output(
                output
            )

        except Exception as e:
            print(
                f"Model inference error: {e}"
            )
            import traceback
            traceback.print_exc()
            self.steering = 0.0
            self.throttle = 0.0

        return (
            self.steering,
            self.throttle,
        )

    def _direct_tensor_inference(
        self,
        model,
        img,
    ):
        if self._seq_transform is None:
            self._seq_transform = transforms.Compose(
                [
                    transforms.Resize(
                        (224, 224)
                    ),
                    transforms.ToTensor(),
                ]
            )

        pil_img = Image.fromarray(
            img
        )

        tensor_img = (
            self._seq_transform(
                pil_img
            )
            .unsqueeze(0)
            .cuda()
        )

        with torch.no_grad():
            result = model(
                tensor_img
            )

        if result.device.type != "cpu":
            result = result.cpu()

        result = result.numpy().reshape(
            -1
        )

        return result[0], result[1]

    def _openvino_inference(
        self,
        model,
        img,
    ):
        try:
            from openvino.runtime import Core

            if hasattr(
                model,
                "run",
            ):
                return model.run(
                    img
                )

            print(
                "Direct OpenVINO inference not implemented, "
                "falling back to PyTorch"
            )

            return self._pytorch_inference(
                model,
                img,
            )

        except ImportError:
            print(
                "OpenVINO not available, falling back to PyTorch"
            )

            return self._pytorch_inference(
                model,
                img,
            )

        except Exception as e:
            print(
                f"OpenVINO inference failed, "
                f"falling back to PyTorch: {e}"
            )

            return self._pytorch_inference(
                model,
                img,
            )

    def _process_model_output(
        self,
        output,
    ):
        def _extract(
            out,
            idx,
            default=0.0,
        ):
            try:
                if torch.is_tensor(out):
                    return (
                        float(
                            out[idx].item()
                        )
                        if out.dim() > 0
                        and len(out) > idx
                        else default
                    )

                elif isinstance(
                    out,
                    (tuple, list),
                ):
                    return (
                        float(out[idx])
                        if len(out) > idx
                        else default
                    )

                elif isinstance(
                    out,
                    np.ndarray,
                ):
                    return (
                        float(out[idx])
                        if out.size > idx
                        else default
                    )

                return default

            except (
                IndexError,
                TypeError,
            ):
                return default

        if (
            isinstance(
                output,
                (
                    tuple,
                    list,
                    np.ndarray,
                ),
            )
            or torch.is_tensor(output)
        ):
            self.steering = _extract(
                output,
                0,
            )
            self.throttle = _extract(
                output,
                1,
            )
            self.speed = _extract(
                output,
                2,
            )
        else:
            print(
                f"Unexpected output format: {type(output)}"
            )
            self.steering = 0.0
            self.throttle = 0.0
            self.speed = 0.0

    # ========================================================
    # 互換用旧PID
    # ========================================================

    def right_hand_pid(
        self,
        ultrasonic_FrRH,
        ultrasonic_RrRH,
        t=0,
        integral_delta_dis=0,
        min_dis=config.TARGET_RANGE,
    ):
        t_before = t
        t = time.perf_counter()
        delta_t = t - t_before

        if delta_t <= 0:
            delta_t = 1e-6

        min_dis_before = min_dis
        min_dis = min(
            ultrasonic_FrRH,
            ultrasonic_RrRH,
        )

        delta_dis = (
            min_dis
            - config.TARGET_RANGE
        )

        integral_delta_dis += delta_dis

        v = (
            min_dis
            - min_dis_before
        ) / delta_t

        kp = getattr(
            config,
            "K_P",
            0.0,
        )
        ki = getattr(
            config,
            "K_I",
            0.0,
        )
        kd = getattr(
            config,
            "K_D",
            0.0,
        )

        steering = (
            kp * delta_dis
            - kd * v
            + ki * integral_delta_dis
        )

        steering = abs(
            max(
                -100,
                min(
                    100,
                    steering,
                ),
            ) / 100
        )

        return steering, config.FORWARD_CORNER

    def left_hand_pid(
        self,
        ultrasonic_FrLH,
        ultrasonic_RrLH,
        t=0,
        integral_delta_dis=0,
        min_dis=config.TARGET_RANGE,
    ):
        t_before = t
        t = time.perf_counter()
        delta_t = t - t_before

        if delta_t <= 0:
            delta_t = 1e-6

        min_dis_before = min_dis
        min_dis = min(
            ultrasonic_FrLH,
            ultrasonic_RrLH,
        )

        delta_dis = (
            min_dis
            - config.TARGET_RANGE
        )

        integral_delta_dis += delta_dis

        v = (
            min_dis
            - min_dis_before
        ) / delta_t

        kp = getattr(
            config,
            "K_P",
            0.0,
        )
        ki = getattr(
            config,
            "K_I",
            0.0,
        )
        kd = getattr(
            config,
            "K_D",
            0.0,
        )

        steering = (
            kp * delta_dis
            - kd * v
            + ki * integral_delta_dis
        )

        steering = abs(
            max(
                -100,
                min(
                    100,
                    steering,
                ),
            ) / 100
        )

        return steering, config.FORWARD_CORNER

    # ========================================================
    # 互換用
    # ========================================================

    def cleanup(self):
        if self.perception is not None:
            try:
                self.perception.reset()
            except Exception:
                pass

        print(
            "Planner cleanup complete."
        )


class SpeedPIDController:
    """速度フィードバックPID制御"""

    def __init__(self):
        self.kp = config.SPEED_PID_KP
        self.ki = config.SPEED_PID_KI
        self.kd = config.SPEED_PID_KD
        self.integral_limit = config.SPEED_PID_INTEGRAL_LIMIT

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.perf_counter()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.perf_counter()

    def compute(
        self,
        target_speed,
        current_speed,
    ):
        now = time.perf_counter()
        dt = now - self._prev_time
        self._prev_time = now

        if dt <= 0:
            dt = 1e-6

        error = (
            target_speed
            - current_speed
        )

        self._integral += (
            error * dt
        )

        self._integral = max(
            -self.integral_limit,
            min(
                self.integral_limit,
                self._integral,
            ),
        )

        derivative = (
            error
            - self._prev_error
        ) / dt

        self._prev_error = error

        output = (
            self.kp * error
            + self.ki * self._integral
            + self.kd * derivative
        )

        return max(
            -1.0,
            min(
                1.0,
                output,
            ),
        )


def estimate_speed(
    rpm_speed,
    of_vy,
    source="rpm",
):
    if source == "rpm":
        return abs(rpm_speed)

    if source == "optical_flow":
        return abs(of_vy)

    if source == "fused":
        if rpm_speed > 0:
            return abs(rpm_speed)
        return abs(of_vy)

    return abs(rpm_speed)


# ============================================================
# DynamicControl
# ============================================================

class DynamicControl:
    def __init__(self, mode=None):
        self.gain_steering = 1.0
        self.gain_throttle = 1.0

    def update_control(
        self,
        throttle_gain,
        steering_gain,
    ):
        self.gain_throttle = throttle_gain
        self.gain_steering = steering_gain

    def counter_steering(
        self,
        gyro_data,
        steering,
        throttle,
    ):
        if "z" not in gyro_data or not gyro_data["z"]:
            raise ValueError(
                "gyro_data に 'z' キーが存在しないか、リストが空です。"
            )

        average_rotation_speed = abs(
            sum(
                gyro_data["z"]
            )
            / len(
                gyro_data["z"]
            )
        )

        # 既存実装互換。外部側で必要な属性を設定する前提。
        rotation_speed = getattr(
            self,
            "rotation_speed",
            1.0,
        )

        counter_steering_strength = min(
            1,
            average_rotation_speed
            / max(
                rotation_speed,
                1e-6,
            ),
        )

        adjusted_steering = (
            steering
            * (
                1
                - counter_steering_strength
            )
        )

        return adjusted_steering, throttle

    def lateral_g_throttle(
        self,
        acc_data,
        jerk_data,
        steering,
        throttle,
    ):
        if "y" not in acc_data or not acc_data["y"]:
            raise ValueError(
                "acc_data に 'y' キーが存在しないか、リストが空です。"
            )

        if "y" not in jerk_data or not jerk_data["y"]:
            raise ValueError(
                "jerk_data に 'y' キーが存在しないか、リストが空です。"
            )

        last_acc_y = acc_data["y"][-1]
        last_jerk_y = jerk_data["y"][-1]

        cxy = getattr(
            self,
            "Cxy",
            1.0,
        )

        ts = getattr(
            self,
            "Ts",
            1.0,
        )

        lateral_g_control = abs(
            (
                last_acc_y
                * last_jerk_y
            )
            * cxy
            /
            (
                1
                + ts
            )
            * abs(
                last_jerk_y
            )
        )

        adjusted_throttle = min(
            1,
            lateral_g_control,
        )

        return steering, adjusted_throttle


class LapCounter:
    def __init__(self):
        self.current_lap = 0
        self.last_checkpoint_time = None

    def increment_lap(self):
        self.current_lap += 1
        print(
            f"Lap incremented: {self.current_lap}"
        )

    def reset_lap(self):
        self.current_lap = 0

    def get_lap_count(self):
        return self.current_lap


class MyCustomPlanner(Planner):
    pass


class DefaultPlanner(Planner):
    def __init__(self):
        super().__init__()

    # ========================================================
    # planning_sequence
    # ========================================================

    def planning_sequence(
        self,
        mode,
        plan,
        data_aggregator,
        model,
        inference_camera_image=None,
        position_model=None,
        position_models_dict=None,
        yolo_model=None,
        yolo_models_dict=None,
        camera_images=None,
        ranges=None,
        lidar_data=None,
    ):
        """
        判断シーケンス。

        新構成:

            超音波
              +
            カメラ
              ↓
            Perception
              ↓
            既存Planner
              ↓
            Perception補正
              ↓
            steering / throttle

        既存の走行モードはそのまま維持し、
        Perceptionは「共通補正・安全層」として利用する。
        """

        # ----------------------------------------------------
        # Recovery最優先
        # ----------------------------------------------------

        if (
            config.RECOVERY_MODE == "back"
            and mode not in (
                "user",
                "auto_str",
            )
        ):
            if self.recovery_back(
                data_aggregator
            ):
                if self.recovery_phase == "brake":
                    return 0.0, -1.0

                if self.recovery_phase == "neutral":
                    return 0.0, 0.0

                return (
                    self.recovery_steering,
                    -1.0,
                )

        # ----------------------------------------------------
        # モデル選択
        # ----------------------------------------------------

        active_model = model

        if mode != "user":
            active_model = self._select_model_by_position(
                mode,
                position_model,
                position_models_dict or {},
                camera_images or {},
                active_model,
            )

            active_model = self._select_model_by_yolo(
                mode,
                yolo_model,
                yolo_models_dict or {},
                inference_camera_image,
                active_model,
            )

        # ----------------------------------------------------
        # LiDAR
        # ----------------------------------------------------

        if lidar_data is not None:
            self._lidar_data = lidar_data

        elif "lidar" in getattr(
            config,
            "ACTIVE_SENSORS",
            [],
        ):
            try:
                self._lidar_data = (
                    data_aggregator.get_latest_sensor_value(
                        "lidar"
                    )
                )
            except Exception:
                self._lidar_data = None

        # ----------------------------------------------------
        # ranges
        # ----------------------------------------------------

        if ranges is None:
            ranges = {}

            for sensor_position in getattr(
                config,
                "ULTRASONIC_SENSOR_LIST",
                [],
            ):
                try:
                    ranges[sensor_position] = (
                        data_aggregator.get_latest_sensor_value(
                            sensor_position
                        )
                    )
                except Exception:
                    ranges[sensor_position] = 3000.0

        # 必須キーを安全化
        ranges = dict(ranges)

        for key, default in (
            ("FrLH", 3000.0),
            ("FrFR", 3000.0),
            ("FrRH", 3000.0),
        ):
            value = ranges.get(
                key,
                default,
            )

            try:
                ranges[key] = float(value)
            except (
                TypeError,
                ValueError,
            ):
                ranges[key] = default

        # ----------------------------------------------------
        # Perception更新
        # ----------------------------------------------------

        perception_result = None

        if (
            self.perception_enabled
            and mode != "user"
        ):
            perception_result = (
                self.update_perception(
                    ranges=ranges,
                    camera_image=inference_camera_image,
                    lidar_data=self._lidar_data,
                    yolo_detections=self.current_detections,
                )
            )

        # ----------------------------------------------------
        # 既存Plannerで基本制御値を計算
        # ----------------------------------------------------

        steering_value, throttle_value = (
            self.compute_motor_commands(
                mode,
                plan,
                ranges,
                active_model,
                inference_camera_image,
                data_aggregator=data_aggregator,
            )
        )

        # ----------------------------------------------------
        # YOLOによる制御補正
        # ----------------------------------------------------

        if (
            config.USE_YOLO_DETECTION
            and mode != "user"
            and self.current_detections
        ):
            obstacle_avoidance_applied = False

            if config.USE_YOLO_OBSTACLE_AVOIDANCE:
                avoidance_steering, obstacle_info = (
                    calculate_obstacle_avoidance_steering(
                        self.current_detections,
                        config.IMAGE_W,
                        config.IMAGE_H,
                    )
                )

                if obstacle_info:
                    steering_value += (
                        avoidance_steering
                    )

                    steering_value = max(
                        -1.0,
                        min(
                            1.0,
                            steering_value,
                        ),
                    )

                    obstacle_avoidance_applied = True

                    if config.YOLO_DISPLAY_DETECTIONS:
                        logger.info(
                            f"障害物回避: "
                            f"{obstacle_info['class_name']} "
                            f"(信頼度: {obstacle_info['confidence']:.2f}, "
                            f"サイズ比: {obstacle_info['area_ratio']:.2%}, "
                            f"回避方向: {obstacle_info['avoidance_direction']}, "
                            f"補正: {obstacle_info['steering_offset']:.2f})"
                        )

            if (
                config.USE_YOLO_OBJECT_TRACKING
                and not obstacle_avoidance_applied
            ):
                steering_offset, tracking_info = (
                    calculate_object_tracking_steering(
                        self.current_detections,
                        config.IMAGE_W,
                    )
                )

                if tracking_info:
                    steering_value += (
                        steering_offset
                    )

                    steering_value = max(
                        -1.0,
                        min(
                            1.0,
                            steering_value,
                        ),
                    )

                    if config.YOLO_DISPLAY_DETECTIONS:
                        logger.info(
                            f"物体追従: "
                            f"{tracking_info['class_name']} "
                            f"(信頼度: {tracking_info['confidence']:.2f}, "
                            f"オフセット: {tracking_info['offset']:.2f}, "
                            f"補正: {tracking_info['steering_offset']:.2f})"
                        )

            modified_steering, modified_throttle, applied_rule = (
                apply_detection_control_modification(
                    self.current_detections,
                    steering_value,
                    throttle_value,
                )
            )

            if applied_rule:
                if config.YOLO_DISPLAY_DETECTIONS:
                    logger.info(
                        f"制御修正適用: "
                        f"{applied_rule['description']} "
                        f"({applied_rule['class_name']}, "
                        f"{applied_rule['confidence']:.2f})"
                    )

                steering_value = modified_steering
                throttle_value = modified_throttle

        # ----------------------------------------------------
        # Perception補正
        # ----------------------------------------------------
        # YOLOより後に実行することで、
        # 最終的な安全・壁・コーナー補正を適用する。

        if (
            perception_result is not None
            and mode != "user"
        ):
            steering_value, throttle_value = (
                self.apply_perception_correction(
                    mode=plan,
                    steering_value=steering_value,
                    throttle_value=throttle_value,
                )
            )

            # デバッグログ
            if getattr(
                config,
                "PERCEPTION_DEBUG",
                False,
            ):
                try:
                    self.perception.log_debug_summary()
                except Exception:
                    pass

        # ----------------------------------------------------
        # 最終クリップ
        # ----------------------------------------------------

        steering_value = max(
            -1.0,
            min(
                1.0,
                steering_value,
            ),
        )

        throttle_value = max(
            -1.0,
            min(
                1.0,
                throttle_value,
            ),
        )

        return (
            steering_value,
            throttle_value,
        )


# ============================================================
# ROS2
# ============================================================

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String
    from geometry_msgs.msg import Twist

    class PlannerNode(Node):
        def __init__(self):
            super().__init__(
                "planner_node"
            )

            self.planner = DefaultPlanner()

            self.mode = "user"
            self.joystick_steering = 0.0
            self.joystick_throttle = 0.0
            self.ranges = {}
            self.steering = 0.0
            self.throttle = 0.0

            self.create_subscription(
                String,
                "/joy/mode",
                self.mode_callback,
                10,
            )

            self.create_subscription(
                Twist,
                "/cmd_vel_joy",
                self.joy_cmd_callback,
                10,
            )

            self.create_subscription(
                Float32MultiArray,
                "/ultrasonic_data",
                self.ultrasonic_callback,
                10,
            )

            self.cmd_pub = self.create_publisher(
                Twist,
                "/cmd_vel",
                10,
            )

            self.timer = self.create_timer(
                0.05,
                self.planning_loop,
            )

            self.get_logger().info(
                f"Planner node started "
                f"(plan={config.PLAN})"
            )

        def mode_callback(self, msg):
            self.mode = msg.data

        def joy_cmd_callback(self, msg):
            self.joystick_steering = (
                msg.angular.z
            )
            self.joystick_throttle = (
                msg.linear.x
            )

        def ultrasonic_callback(self, msg):
            for i, name in enumerate(
                getattr(
                    config,
                    "ULTRASONIC_SENSOR_LIST",
                    [],
                )
            ):
                if i < len(msg.data):
                    self.ranges[name] = msg.data[i]

        def planning_loop(self):
            cmd = Twist()

            if self.mode == "user":
                cmd.angular.z = float(
                    self.joystick_steering
                )
                cmd.linear.x = float(
                    self.joystick_throttle
                )

            else:
                steering, throttle = (
                    self.planner.compute_motor_commands(
                        self.mode,
                        config.PLAN,
                        self.ranges,
                    )
                )

                # ROS2単体モードではカメラ画像がないため、
                # Perceptionは超音波中心になる。
                steering, throttle = (
                    self.planner.apply_perception_correction(
                        config.PLAN,
                        steering,
                        throttle,
                    )
                )

                cmd.angular.z = float(
                    steering
                )
                cmd.linear.x = float(
                    throttle
                )

            self.cmd_pub.publish(
                cmd
            )

    def main_ros(args=None):
        rclpy.init(
            args=args
        )

        node = PlannerNode()

        try:
            rclpy.spin(
                node
            )

        except KeyboardInterrupt:
            pass

        finally:
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

except ImportError:
    rclpy = None


# ============================================================
# オフラインテスト
# ============================================================

def offline_test():
    """
    Raspberry Pi / 車なしで、
    planner + perceptionの接続を確認する。

    実際のカメラ画像は使わず、
    ダミーの超音波・カメラ認識情報をPerceptionに投入する。
    """

    print("=" * 70)
    print("Planner + Perception offline test")
    print("=" * 70)

    planner = DefaultPlanner()

    if planner.perception is None:
        print(
            "Perceptionが初期化されていません。"
        )
        return

    test_cases = [
        {
            "name": "通常直進",
            "ranges": {
                "FrLH": 350,
                "FrFR": 1200,
                "FrRH": 350,
            },
            "camera": {
                "left_wall": True,
                "right_wall": True,
                "front_wall": False,
                "left_confidence": 0.8,
                "right_confidence": 0.8,
                "direction": 0.0,
                "direction_confidence": 0.8,
                "corner": None,
                "corner_confidence": 0.0,
                "obstacle": False,
                "obstacle_confidence": 0.0,
            },
        },
        {
            "name": "右方向が広い",
            "ranges": {
                "FrLH": 250,
                "FrFR": 900,
                "FrRH": 800,
            },
            "camera": {
                "left_wall": True,
                "right_wall": False,
                "front_wall": False,
                "left_confidence": 0.9,
                "right_confidence": 0.7,
                "direction": "right",
                "direction_confidence": 0.9,
                "corner": "right",
                "corner_confidence": 0.75,
                "obstacle": False,
                "obstacle_confidence": 0.0,
            },
        },
        {
            "name": "前方危険",
            "ranges": {
                "FrLH": 300,
                "FrFR": 200,
                "FrRH": 500,
            },
            "camera": {
                "left_wall": True,
                "right_wall": True,
                "front_wall": True,
                "left_confidence": 0.8,
                "right_confidence": 0.8,
                "front_confidence": 0.95,
                "direction": "left",
                "direction_confidence": 0.8,
                "corner": "left",
                "corner_confidence": 0.9,
                "obstacle": True,
                "obstacle_confidence": 0.95,
                "obstacle_side": "right",
            },
        },
    ]

    for case in test_cases:
        print()
        print("-" * 70)
        print(
            f"CASE: {case['name']}"
        )

        planner.perception.update_normalized(
            ultrasonic_data=case["ranges"],
            camera_data=case["camera"],
            lidar_data=None,
            yolo_data=None,
        )

        base_steering, base_throttle = (
            planner.racer(
                case["ranges"]["FrLH"],
                case["ranges"]["FrFR"],
                case["ranges"]["FrRH"],
            )
        )

        final_steering, final_throttle = (
            planner.apply_perception_correction(
                mode="racer",
                steering_value=base_steering,
                throttle_value=base_throttle,
            )
        )

        print(
            f"base steering  : {base_steering:.3f}"
        )
        print(
            f"base throttle  : {base_throttle:.3f}"
        )
        print(
            f"final steering : {final_steering:.3f}"
        )
        print(
            f"final throttle : {final_throttle:.3f}"
        )

        try:
            print(
                "perception     :",
                planner.perception.get_debug_summary(),
            )
        except Exception:
            pass

    print()
    print("=" * 70)
    print("OFFLINE TEST COMPLETE")
    print("=" * 70)


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Planning / Perception test"
    )

    parser.add_argument(
        "--ros",
        action="store_true",
        help="Run with ROS2",
    )

    parser.add_argument(
        "--offline-test",
        action="store_true",
        help="Run Planner + Perception offline test",
    )

    args = parser.parse_args()

    if (
        args.ros
        and rclpy
    ):
        print(
            "Start with ROS2"
        )
        main_ros()

    elif args.offline_test:
        offline_test()

    else:
        offline_test()
