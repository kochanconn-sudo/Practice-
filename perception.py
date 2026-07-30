# coding:utf-8

"""
perception.py

センサーフュージョン用の認識モジュール。

役割
----
超音波:
    実距離・安全確認

カメラ:
    壁・コーナー・コース形状・障害物候補

YOLO:
    物体種別・障害物情報

LiDAR:
    将来の全周距離・空間認識

これらを統合して、planner.py が利用しやすい
共通の PerceptionResult を生成する。

設計方針
--------
camera.py
    ↓
camera_wall_detector.py
    ↓
perception.py
    ↓
planner.py
    ↓
control / motor

注意
----
このファイルはモーターPWMを直接出力しない。
-1.0～1.0の操舵
0.0～1.0の速度
など、plannerが利用しやすい抽象値を扱う。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import logging
import math
import time

import numpy as np

import config


logger = logging.getLogger(__name__)


# ============================================================
# 共通ユーティリティ
# ============================================================

def clamp(
    value: float,
    minimum: float,
    maximum: float,
) -> float:
    return max(
        minimum,
        min(maximum, value),
    )


def safe_float(
    value: Any,
    default: float = 0.0,
) -> float:
    if value is None:
        return default

    try:
        result = float(value)

        if not math.isfinite(result):
            return default

        return result

    except (TypeError, ValueError):
        return default


def safe_bool(
    value: Any,
    default: bool = False,
) -> bool:
    if value is None:
        return default

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        value = value.lower().strip()

        if value in ("true", "1", "yes", "on"):
            return True

        if value in ("false", "0", "no", "off"):
            return False

    try:
        return bool(value)
    except Exception:
        return default


def get_config(
    name: str,
    default: Any = None,
) -> Any:
    return getattr(
        config,
        name,
        default,
    )


# ============================================================
# 超音波センサー名
# ============================================================

ULTRASONIC_LEFT_KEYS = (
    "FrLH",
    "left",
    "left_front",
    "Left",
    "LEFT",
)

ULTRASONIC_FRONT_KEYS = (
    "FrFR",
    "front",
    "Front",
    "FRONT",
)

ULTRASONIC_RIGHT_KEYS = (
    "FrRH",
    "right",
    "right_front",
    "Right",
    "RIGHT",
)


# ============================================================
# 壁状態
# ============================================================

@dataclass
class WallState:
    left_detected: bool = False
    front_detected: bool = False
    right_detected: bool = False

    left_distance: Optional[float] = None
    front_distance: Optional[float] = None
    right_distance: Optional[float] = None

    left_angle: float = 0.0
    right_angle: float = 0.0

    left_confidence: float = 0.0
    front_confidence: float = 0.0
    right_confidence: float = 0.0


# ============================================================
# 障害物状態
# ============================================================

@dataclass
class ObstacleState:
    detected: bool = False

    left: bool = False
    center: bool = False
    right: bool = False

    distance: Optional[float] = None

    object_type: Optional[str] = None

    confidence: float = 0.0

    source: str = "none"

    detections: List[Dict[str, Any]] = field(
        default_factory=list
    )


# ============================================================
# コース状態
# ============================================================

@dataclass
class CourseState:
    # -1.0 = 左
    #  0.0 = 直進
    # +1.0 = 右
    direction: float = 0.0

    left_open: bool = False
    right_open: bool = False

    # 0～1
    left_free_space: float = 0.0
    right_free_space: float = 0.0

    # -1～1
    center_offset: float = 0.0

    estimated_width: Optional[float] = None

    corner_detected: bool = False
    corner_direction: Optional[str] = None

    intersection: bool = False
    dead_end: bool = False


# ============================================================
# 安全状態
# ============================================================

@dataclass
class SafetyState:
    emergency: bool = False

    danger_level: float = 0.0

    collision_risk: float = 0.0

    must_slow_down: bool = False
    must_stop: bool = False

    escape_direction: Optional[str] = None


# ============================================================
# 走行推奨
# ============================================================

@dataclass
class DrivingRecommendation:
    # -1 = 左
    #  0 = 直進
    # +1 = 右
    steering: float = 0.0

    # 0 = 停止
    # 1 = 最大
    throttle: float = 0.0

    confidence: float = 0.0

    reason: str = ""


# ============================================================
# 最終結果
# ============================================================

@dataclass
class PerceptionResult:
    timestamp: float = field(
        default_factory=time.perf_counter
    )

    wall: WallState = field(
        default_factory=WallState
    )

    obstacle: ObstacleState = field(
        default_factory=ObstacleState
    )

    course: CourseState = field(
        default_factory=CourseState
    )

    safety: SafetyState = field(
        default_factory=SafetyState
    )

    recommendation: DrivingRecommendation = field(
        default_factory=DrivingRecommendation
    )

    ultrasonic_raw: Dict[str, Any] = field(
        default_factory=dict
    )

    camera_raw: Dict[str, Any] = field(
        default_factory=dict
    )

    lidar_raw: Dict[str, Any] = field(
        default_factory=dict
    )

    vision_raw: Dict[str, Any] = field(
        default_factory=dict
    )

    debug: Dict[str, Any] = field(
        default_factory=dict
    )


# ============================================================
# Perception
# ============================================================

class Perception:

    def __init__(self):
        # ----------------------------------------------------
        # 状態
        # ----------------------------------------------------

        self.result = PerceptionResult()

        self.initialized = False

        self.frame_count = 0

        self.last_update_time = (
            time.perf_counter()
        )

        # ----------------------------------------------------
        # 距離履歴
        # ----------------------------------------------------

        self.history_size = max(
            1,
            int(
                get_config(
                    "PERCEPTION_HISTORY_SIZE",
                    3,
                )
            ),
        )

        self._left_distance_history = []
        self._front_distance_history = []
        self._right_distance_history = []

        # ----------------------------------------------------
        # 方向履歴
        # ----------------------------------------------------

        self._direction_history = []

        # ----------------------------------------------------
        # 基本距離設定
        # ----------------------------------------------------

        self.default_distance = float(
            get_config(
                "PERCEPTION_DEFAULT_DISTANCE",
                3000.0,
            )
        )

        self.wall_detection_distance = float(
            get_config(
                "PERCEPTION_WALL_DETECTION_RANGE",
                get_config(
                    "DETECTION_RANGE",
                    150.0,
                ),
            )
        )

        # 側面壁として扱う距離
        self.side_wall_detection_distance = float(
            get_config(
                "PERCEPTION_SIDE_WALL_RANGE",
                max(
                    500.0,
                    get_config(
                        "RIGHT_LEFT_RANGE",
                        400.0,
                    ),
                ),
            )
        )

        # 前方危険判定
        self.front_warning_distance = float(
            get_config(
                "PERCEPTION_FRONT_WARNING_DISTANCE",
                600.0,
            )
        )

        # 緊急停止
        self.emergency_distance = float(
            get_config(
                "PERCEPTION_EMERGENCY_DISTANCE",
                get_config(
                    "STOP_RANGE",
                    250.0,
                ),
            )
        )

        # ----------------------------------------------------
        # 空き具合
        # ----------------------------------------------------

        self.free_space_reference = float(
            get_config(
                "PERCEPTION_FREE_SPACE_REFERENCE",
                1500.0,
            )
        )

        if self.free_space_reference <= 0:
            self.free_space_reference = 1500.0

        # 重要:
        # left_free_space/right_free_space は0～1なので、
        # GAP_MARGINも0～1で扱う。
        self.gap_margin = clamp(
            float(
                get_config(
                    "PERCEPTION_GAP_MARGIN",
                    0.10,
                )
            ),
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # カメラ
        # ----------------------------------------------------

        self.camera_confidence_threshold = clamp(
            float(
                get_config(
                    "PERCEPTION_CAMERA_CONFIDENCE_THRESHOLD",
                    0.60,
                )
            ),
            0.0,
            1.0,
        )

        self.camera_direction_weight = clamp(
            float(
                get_config(
                    "PERCEPTION_CAMERA_DIRECTION_WEIGHT",
                    0.45,
                )
            ),
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        self.corner_front_distance = float(
            get_config(
                "PERCEPTION_CORNER_FRONT_DISTANCE",
                700.0,
            )
        )

        self.corner_side_difference = float(
            get_config(
                "PERCEPTION_CORNER_SIDE_DIFFERENCE",
                150.0,
            )
        )

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        # 超音波の側面センサーだけでは
        # 「壁」と「障害物」を区別しない。
        #
        # よって、
        #   左右 = 壁 / 空間
        #   前方 = 衝突危険
        #
        # と分離する。
        self.obstacle_confidence_threshold = clamp(
            float(
                get_config(
                    "PERCEPTION_OBSTACLE_CONFIDENCE_THRESHOLD",
                    0.60,
                )
            ),
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # 速度
        # ----------------------------------------------------

        self.base_throttle = clamp(
            float(
                get_config(
                    "PERCEPTION_BASE_THROTTLE",
                    get_config(
                        "FORWARD_STRAIGHT",
                        0.8,
                    ),
                )
            ),
            0.0,
            1.0,
        )

        self.minimum_throttle = clamp(
            float(
                get_config(
                    "PERCEPTION_MIN_THROTTLE",
                    0.20,
                )
            ),
            0.0,
            1.0,
        )

        self.maximum_throttle = clamp(
            float(
                get_config(
                    "PERCEPTION_MAX_THROTTLE",
                    1.0,
                )
            ),
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # 推奨ステアリング
        # ----------------------------------------------------

        self.obstacle_steering_gain = clamp(
            float(
                get_config(
                    "PERCEPTION_OBSTACLE_STEERING_GAIN",
                    0.75,
                )
            ),
            0.0,
            1.0,
        )

        self.corner_steering_gain = clamp(
            float(
                get_config(
                    "PERCEPTION_CORNER_STEERING_GAIN",
                    0.75,
                )
            ),
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # デバッグ
        # ----------------------------------------------------

        logger.info(
            "Perception 起動"
        )

        logger.info(
            "PERCEPTION settings: "
            "wall=%s side_wall=%s front_warning=%s "
            "emergency=%s gap_margin=%.2f",
            self.wall_detection_distance,
            self.side_wall_detection_distance,
            self.front_warning_distance,
            self.emergency_distance,
            self.gap_margin,
        )

    # ========================================================
    # メイン更新
    # ========================================================

    def update(
        self,
        ultrasonic_data: Optional[
            Dict[str, Any]
        ] = None,
        camera_data: Optional[
            Dict[str, Any]
        ] = None,
        lidar_data: Optional[
            Dict[str, Any]
        ] = None,
        yolo_data: Optional[
            Dict[str, Any]
        ] = None,
    ) -> PerceptionResult:

        self.frame_count += 1

        now = time.perf_counter()

        dt = (
            now
            -
            self.last_update_time
        )

        if dt <= 0:
            dt = 1e-6

        self.last_update_time = now

        # ----------------------------------------------------
        # None対策
        # ----------------------------------------------------

        ultrasonic_data = (
            ultrasonic_data
            if ultrasonic_data is not None
            else {}
        )

        camera_data = (
            camera_data
            if camera_data is not None
            else {}
        )

        lidar_data = (
            lidar_data
            if lidar_data is not None
            else {}
        )

        yolo_data = (
            yolo_data
            if yolo_data is not None
            else {}
        )

        # ----------------------------------------------------
        # カメラ/YOLO標準化
        # ----------------------------------------------------

        camera_data = self.normalize_camera_data(
            camera_data
        )

        yolo_data = self.normalize_yolo_data(
            yolo_data
        )

        # ----------------------------------------------------
        # 超音波距離
        # ----------------------------------------------------

        left_distance = self._extract_distance(
            ultrasonic_data,
            ULTRASONIC_LEFT_KEYS,
        )

        front_distance = self._extract_distance(
            ultrasonic_data,
            ULTRASONIC_FRONT_KEYS,
        )

        right_distance = self._extract_distance(
            ultrasonic_data,
            ULTRASONIC_RIGHT_KEYS,
        )

        # ----------------------------------------------------
        # 履歴
        # ----------------------------------------------------

        self._append_history(
            self._left_distance_history,
            left_distance,
        )

        self._append_history(
            self._front_distance_history,
            front_distance,
        )

        self._append_history(
            self._right_distance_history,
            right_distance,
        )

        # ----------------------------------------------------
        # 平滑化
        # ----------------------------------------------------

        left_distance = self._get_filtered_distance(
            self._left_distance_history,
            left_distance,
        )

        front_distance = self._get_filtered_distance(
            self._front_distance_history,
            front_distance,
        )

        right_distance = self._get_filtered_distance(
            self._right_distance_history,
            right_distance,
        )

        # ----------------------------------------------------
        # WallState
        # ----------------------------------------------------

        wall = self._build_wall_state(
            left_distance,
            front_distance,
            right_distance,
            camera_data,
        )

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        obstacle = self._build_obstacle_state(
            front_distance,
            left_distance,
            right_distance,
            camera_data,
            yolo_data,
        )

        # ----------------------------------------------------
        # コース
        # ----------------------------------------------------

        course = self._analyze_course(
            wall_state=wall,
            obstacle_state=obstacle,
            camera_data=camera_data,
            lidar_data=lidar_data,
        )

        # ----------------------------------------------------
        # 安全
        # ----------------------------------------------------

        safety = self._analyze_safety(
            wall_state=wall,
            obstacle_state=obstacle,
            course_state=course,
        )

        # ----------------------------------------------------
        # 推奨
        # ----------------------------------------------------

        recommendation = (
            self._calculate_recommendation(
                wall_state=wall,
                obstacle_state=obstacle,
                course_state=course,
                safety_state=safety,
            )
        )

        # ----------------------------------------------------
        # 結果
        # ----------------------------------------------------

        self.result = PerceptionResult(
            timestamp=time.perf_counter(),

            wall=wall,

            obstacle=obstacle,

            course=course,

            safety=safety,

            recommendation=recommendation,

            ultrasonic_raw=dict(
                ultrasonic_data
            ),

            camera_raw=dict(
                camera_data
            ),

            lidar_raw=dict(
                lidar_data
            ),

            vision_raw=dict(
                yolo_data
            ),

            debug={
                "frame_count":
                    self.frame_count,

                "dt":
                    dt,

                "fps":
                    1.0 / dt,

                "camera_active":
                    bool(camera_data),

                "yolo_active":
                    bool(
                        yolo_data.get(
                            "detections",
                            [],
                        )
                    ),
            },
        )

        self.initialized = True

        return self.result

    # ========================================================
    # 距離抽出
    # ========================================================

    def _extract_distance(
        self,
        data: Dict[str, Any],
        possible_keys: Tuple[str, ...],
    ) -> float:

        for key in possible_keys:

            if key not in data:
                continue

            value = safe_float(
                data[key],
                self.default_distance,
            )

            if value >= 0:
                return value

        return self.default_distance

    # ========================================================
    # 履歴
    # ========================================================

    def _append_history(
        self,
        history: List[float],
        value: float,
    ) -> None:

        history.append(
            value
        )

        if len(history) > self.history_size:
            del history[0]

    # ========================================================
    # 平滑化
    # ========================================================

    def _get_filtered_distance(
        self,
        history: List[float],
        current: float,
    ) -> float:

        if not history:
            return current

        values = [
            x
            for x in history
            if x is not None
            and math.isfinite(x)
            and x >= 0
        ]

        if not values:
            return current

        # 履歴を短くして応答性優先
        return float(
            np.mean(
                values
            )
        )

    # ========================================================
    # 壁状態
    # ========================================================

    def _build_wall_state(
        self,
        left_distance: float,
        front_distance: float,
        right_distance: float,
        camera_data: Dict[str, Any],
    ) -> WallState:

        wall = WallState()

        wall.left_distance = left_distance
        wall.front_distance = front_distance
        wall.right_distance = right_distance

        # ----------------------------------------------------
        # 左右壁
        #
        # 左右センサーは壁追従用の距離として扱う。
        # ----------------------------------------------------

        wall.left_detected = (
            left_distance
            <= self.side_wall_detection_distance
        )

        wall.right_detected = (
            right_distance
            <= self.side_wall_detection_distance
        )

        # 前方は「壁/障害物が近い」
        wall.front_detected = (
            front_distance
            <= self.front_warning_distance
        )

        # ----------------------------------------------------
        # カメラの壁判定を統合
        # ----------------------------------------------------

        left_camera = safe_bool(
            camera_data.get(
                "left_wall",
                False,
            )
        )

        right_camera = safe_bool(
            camera_data.get(
                "right_wall",
                False,
            )
        )

        front_camera = safe_bool(
            camera_data.get(
                "front_wall",
                False,
            )
        )

        left_confidence = clamp(
            safe_float(
                camera_data.get(
                    "left_confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        right_confidence = clamp(
            safe_float(
                camera_data.get(
                    "right_confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        front_confidence = clamp(
            safe_float(
                camera_data.get(
                    "front_confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        if (
            left_camera
            and left_confidence
            >= self.camera_confidence_threshold
        ):
            wall.left_detected = True

        if (
            right_camera
            and right_confidence
            >= self.camera_confidence_threshold
        ):
            wall.right_detected = True

        if (
            front_camera
            and front_confidence
            >= self.camera_confidence_threshold
        ):
            wall.front_detected = True

        wall.left_confidence = max(
            self._distance_confidence(
                left_distance,
                self.side_wall_detection_distance,
            ),
            left_confidence,
        )

        wall.right_confidence = max(
            self._distance_confidence(
                right_distance,
                self.side_wall_detection_distance,
            ),
            right_confidence,
        )

        wall.front_confidence = max(
            self._distance_confidence(
                front_distance,
                self.front_warning_distance,
            ),
            front_confidence,
        )

        # カメラ角度
        wall.left_angle = safe_float(
            camera_data.get(
                "left_angle",
                0.0,
            ),
            0.0,
        )

        wall.right_angle = safe_float(
            camera_data.get(
                "right_angle",
                0.0,
            ),
            0.0,
        )

        return wall

    # ========================================================
    # 距離信頼度
    # ========================================================

    def _distance_confidence(
        self,
        distance: float,
        reference: float,
    ) -> float:

        if reference <= 0:
            return 0.0

        # 近すぎず遠すぎない範囲で信頼度を高める
        ratio = clamp(
            1.0
            -
            abs(
                distance
                -
                reference * 0.45
            )
            /
            max(
                1.0,
                reference,
            ),
            0.0,
            1.0,
        )

        return ratio

    # ========================================================
    # 障害物状態
    # ========================================================

    def _build_obstacle_state(
        self,
        front_distance: float,
        left_distance: float,
        right_distance: float,
        camera_data: Dict[str, Any],
        yolo_data: Dict[str, Any],
    ) -> ObstacleState:

        obstacle = ObstacleState()

        # ----------------------------------------------------
        # 前方超音波
        #
        # ここでは「障害物確定」ではなく
        # 衝突危険候補として扱う。
        # ----------------------------------------------------

        front_close = (
            front_distance
            <= self.front_warning_distance
        )

        # 緊急レベルなら center=True
        if (
            front_distance
            <= self.emergency_distance
        ):
            obstacle.center = True
            obstacle.detected = True
            obstacle.distance = front_distance
            obstacle.source = "ultrasonic"
            obstacle.confidence = 1.0

        # ----------------------------------------------------
        # カメラ障害物
        # ----------------------------------------------------

        camera_obstacle = safe_bool(
            camera_data.get(
                "obstacle",
                False,
            )
        )

        camera_obstacle_confidence = clamp(
            safe_float(
                camera_data.get(
                    "obstacle_confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        if (
            camera_obstacle
            and
            camera_obstacle_confidence
            >= self.obstacle_confidence_threshold
        ):

            obstacle.detected = True

            obstacle.source = "camera"

            obstacle.confidence = max(
                obstacle.confidence,
                camera_obstacle_confidence,
            )

            obstacle.object_type = (
                camera_data.get(
                    "obstacle_type"
                )
            )

            # カメラの位置情報があれば利用
            obstacle_side = camera_data.get(
                "obstacle_side"
            )

            if obstacle_side == "left":
                obstacle.left = True

            elif obstacle_side == "right":
                obstacle.right = True

            else:
                obstacle.center = True

        # ----------------------------------------------------
        # YOLO
        # ----------------------------------------------------

        detections = yolo_data.get(
            "detections",
            [],
        )

        if isinstance(
            detections,
            list,
        ) and detections:

            max_confidence = 0.0

            best_detection = None

            for detection in detections:

                if not isinstance(
                    detection,
                    dict,
                ):
                    continue

                confidence = clamp(
                    safe_float(
                        detection.get(
                            "confidence",
                            0.0,
                        ),
                        0.0,
                    ),
                    0.0,
                    1.0,
                )

                if confidence > max_confidence:

                    max_confidence = confidence

                    best_detection = detection

            if best_detection is not None:

                obstacle.detected = True

                obstacle.source = "yolo"

                obstacle.confidence = max(
                    obstacle.confidence,
                    max_confidence,
                )

                obstacle.object_type = (
                    best_detection.get(
                        "class_name",
                        best_detection.get(
                            "class",
                            "unknown",
                        ),
                    )
                )

                x_center = safe_float(
                    best_detection.get(
                        "x_center",
                        0.5,
                    ),
                    0.5,
                )

                if x_center < 0.35:
                    obstacle.left = True

                elif x_center > 0.65:
                    obstacle.right = True

                else:
                    obstacle.center = True

        # ----------------------------------------------------
        # 前方近接だが、カメラ障害物がない場合
        #
        # 「障害物確定」にはしない。
        # safety側で前方危険として扱う。
        # ----------------------------------------------------

        if front_close and not obstacle.detected:

            obstacle.distance = front_distance

        return obstacle

    # ========================================================
    # コース解析
    # ========================================================

    def _analyze_course(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        camera_data: Dict[str, Any],
        lidar_data: Dict[str, Any],
    ) -> CourseState:

        course = CourseState()

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        front = safe_float(
            wall_state.front_distance,
            self.default_distance,
        )

        # ----------------------------------------------------
        # 空き具合
        # ----------------------------------------------------

        course.left_free_space = (
            self._calculate_free_space(
                left
            )
        )

        course.right_free_space = (
            self._calculate_free_space(
                right
            )
        )

        # ----------------------------------------------------
        # 左右どちらが広いか
        #
        # ここは0～1同士を比較する。
        # ----------------------------------------------------

        diff = (
            course.right_free_space
            -
            course.left_free_space
        )

        if diff < -self.gap_margin:
            course.left_open = True

        elif diff > self.gap_margin:
            course.right_open = True

        # ----------------------------------------------------
        # 基本方向
        # ----------------------------------------------------

        direction_gain = float(
            get_config(
                "PERCEPTION_DIRECTION_GAIN",
                1.0,
            )
        )

        course.direction = clamp(
            direction_gain
            * diff,
            -1.0,
            1.0,
        )

        # ----------------------------------------------------
        # 中央ずれ
        # ----------------------------------------------------

        course.center_offset = (
            self._calculate_center_offset(
                left,
                right,
            )
        )

        # ----------------------------------------------------
        # カメラ方向
        # ----------------------------------------------------

        camera_direction = (
            camera_data.get(
                "direction"
            )
        )

        camera_direction_confidence = clamp(
            safe_float(
                camera_data.get(
                    "direction_confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        if (
            camera_direction is not None
            and
            camera_direction_confidence
            >= self.camera_confidence_threshold
        ):

            camera_direction_value = (
                self._convert_direction_value(
                    camera_direction
                )
            )

            weight = (
                self.camera_direction_weight
                *
                camera_direction_confidence
            )

            weight = clamp(
                weight,
                0.0,
                1.0,
            )

            course.direction = clamp(
                course.direction
                * (1.0 - weight)
                +
                camera_direction_value
                * weight,
                -1.0,
                1.0,
            )

        # ----------------------------------------------------
        # 障害物回避方向
        # ----------------------------------------------------

        if obstacle_state.detected:

            if (
                obstacle_state.center
            ):

                if left > right:
                    course.direction -= (
                        self.obstacle_steering_gain
                    )

                elif right > left:
                    course.direction += (
                        self.obstacle_steering_gain
                    )

            elif obstacle_state.left:
                course.direction += (
                    self.obstacle_steering_gain
                )

            elif obstacle_state.right:
                course.direction -= (
                    self.obstacle_steering_gain
                )

        course.direction = clamp(
            course.direction,
            -1.0,
            1.0,
        )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        (
            corner_detected,
            corner_direction,
        ) = self._detect_corner(
            wall_state,
            course.direction,
            camera_data,
        )

        course.corner_detected = (
            corner_detected
        )

        course.corner_direction = (
            corner_direction
        )

        # ----------------------------------------------------
        # 行き止まり
        # ----------------------------------------------------

        course.dead_end = (
            self._detect_dead_end(
                wall_state,
                obstacle_state,
                camera_data,
            )
        )

        # ----------------------------------------------------
        # 交差点
        # ----------------------------------------------------

        course.intersection = (
            self._detect_intersection(
                wall_state,
                camera_data,
            )
        )

        # ----------------------------------------------------
        # コース幅
        # ----------------------------------------------------

        course.estimated_width = (
            left + right
        )

        # ----------------------------------------------------
        # LiDAR
        # ----------------------------------------------------

        course = (
            self._fuse_lidar_course_information(
                course,
                lidar_data,
            )
        )

        return course

    # ========================================================
    # 空き具合
    # ========================================================

    def _calculate_free_space(
        self,
        distance: float,
    ) -> float:

        return clamp(
            distance
            /
            self.free_space_reference,
            0.0,
            1.0,
        )

    # ========================================================
    # 中央ずれ
    # ========================================================

    def _calculate_center_offset(
        self,
        left_distance: float,
        right_distance: float,
    ) -> float:

        denominator = (
            left_distance
            +
            right_distance
            +
            1.0
        )

        if denominator <= 0:
            return 0.0

        # + = 右側が広い
        # - = 左側が広い
        offset = (
            right_distance
            -
            left_distance
        ) / denominator

        return clamp(
            offset,
            -1.0,
            1.0,
        )

    # ========================================================
    # 方向変換
    # ========================================================

    def _convert_direction_value(
        self,
        direction: Any,
    ) -> float:

        if isinstance(
            direction,
            str,
        ):

            value = (
                direction
                .lower()
                .strip()
            )

            if value in (
                "left",
                "l",
                "左",
            ):
                return -1.0

            if value in (
                "right",
                "r",
                "右",
            ):
                return 1.0

            if value in (
                "center",
                "straight",
                "forward",
                "front",
                "中央",
                "直進",
            ):
                return 0.0

        return clamp(
            safe_float(
                direction,
                0.0,
            ),
            -1.0,
            1.0,
        )

    # ========================================================
    # コーナー
    # ========================================================

    def _detect_corner(
        self,
        wall_state: WallState,
        course_direction: float,
        camera_data: Dict[str, Any],
    ) -> Tuple[
        bool,
        Optional[str],
    ]:

        # ----------------------------------------------------
        # カメラ優先
        # ----------------------------------------------------

        camera_corner = (
            camera_data.get(
                "corner"
            )
        )

        camera_confidence = clamp(
            safe_float(
                camera_data.get(
                    "corner_confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        if (
            camera_corner
            in (
                "left",
                "right",
            )
            and
            camera_confidence
            >= self.camera_confidence_threshold
        ):
            return (
                True,
                camera_corner,
            )

        # ----------------------------------------------------
        # 超音波ベース
        # ----------------------------------------------------

        front = safe_float(
            wall_state.front_distance,
            self.default_distance,
        )

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        if front > self.corner_front_distance:
            return False, None

        difference = (
            left
            -
            right
        )

        if (
            abs(difference)
            >=
            self.corner_side_difference
        ):

            if difference > 0:
                return True, "left"

            return True, "right"

        if course_direction < -0.30:
            return True, "left"

        if course_direction > 0.30:
            return True, "right"

        return False, None

    # ========================================================
    # 行き止まり
    # ========================================================

    def _detect_dead_end(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        camera_data: Dict[str, Any],
    ) -> bool:

        if safe_bool(
            camera_data.get(
                "dead_end",
                False,
            )
        ):
            return True

        dead_end_distance = float(
            get_config(
                "PERCEPTION_DEAD_END_DISTANCE",
                300.0,
            )
        )

        left_blocked = (
            wall_state.left_distance
            is not None
            and
            wall_state.left_distance
            <= dead_end_distance
        )

        front_blocked = (
            wall_state.front_distance
            is not None
            and
            wall_state.front_distance
            <= dead_end_distance
        )

        right_blocked = (
            wall_state.right_distance
            is not None
            and
            wall_state.right_distance
            <= dead_end_distance
        )

        return (
            left_blocked
            and
            front_blocked
            and
            right_blocked
        )

    # ========================================================
    # 交差点
    # ========================================================

    def _detect_intersection(
        self,
        wall_state: WallState,
        camera_data: Dict[str, Any],
    ) -> bool:

        if safe_bool(
            camera_data.get(
                "intersection",
                False,
            )
        ):
            return True

        intersection_open_distance = float(
            get_config(
                "PERCEPTION_INTERSECTION_OPEN_DISTANCE",
                1200.0,
            )
        )

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        return (
            left
            >=
            intersection_open_distance
            and
            right
            >=
            intersection_open_distance
        )

    # ========================================================
    # LiDAR
    # ========================================================

    def _fuse_lidar_course_information(
        self,
        course: CourseState,
        lidar_data: Dict[str, Any],
    ) -> CourseState:

        if not lidar_data:
            return course

        lidar_direction = (
            lidar_data.get(
                "direction"
            )
        )

        lidar_confidence = clamp(
            safe_float(
                lidar_data.get(
                    "confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        lidar_threshold = float(
            get_config(
                "PERCEPTION_LIDAR_CONFIDENCE_THRESHOLD",
                0.60,
            )
        )

        if (
            lidar_direction is not None
            and
            lidar_confidence
            >=
            lidar_threshold
        ):

            lidar_value = (
                self._convert_direction_value(
                    lidar_direction
                )
            )

            weight = clamp(
                lidar_confidence
                *
                float(
                    get_config(
                        "PERCEPTION_LIDAR_DIRECTION_WEIGHT",
                        0.30,
                    )
                ),
                0.0,
                1.0,
            )

            course.direction = clamp(
                course.direction
                * (1.0 - weight)
                +
                lidar_value
                * weight,
                -1.0,
                1.0,
            )

        if safe_bool(
            lidar_data.get(
                "left_open",
                False,
            )
        ):
            course.left_open = True

        if safe_bool(
            lidar_data.get(
                "right_open",
                False,
            )
        ):
            course.right_open = True

        if safe_bool(
            lidar_data.get(
                "intersection",
                False,
            )
        ):
            course.intersection = True

        if safe_bool(
            lidar_data.get(
                "dead_end",
                False,
            )
        ):
            course.dead_end = True

        return course

    # ========================================================
    # 安全解析
    # ========================================================

    def _analyze_safety(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        course_state: CourseState,
    ) -> SafetyState:

        safety = SafetyState()

        front = safe_float(
            wall_state.front_distance,
            self.default_distance,
        )

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        # ----------------------------------------------------
        # 前方危険度
        # ----------------------------------------------------

        front_risk = self._distance_risk(
            front,
            self.front_warning_distance,
            self.emergency_distance,
        )

        # ----------------------------------------------------
        # 左右は「壁があるから危険」ではなく、
        # あまりにも近い場合だけ接触リスク
        # ----------------------------------------------------

        side_warning_distance = float(
            get_config(
                "PERCEPTION_SIDE_WARNING_DISTANCE",
                100.0,
            )
        )

        left_risk = self._distance_risk(
            left,
            side_warning_distance,
            50.0,
        )

        right_risk = self._distance_risk(
            right,
            side_warning_distance,
            50.0,
        )

        safety.collision_risk = max(
            front_risk,
            left_risk,
            right_risk,
        )

        # ----------------------------------------------------
        # カメラ/YOLO障害物
        # ----------------------------------------------------

        if obstacle_state.detected:

            safety.collision_risk = max(
                safety.collision_risk,
                obstacle_state.confidence,
            )

        safety.danger_level = clamp(
            safety.collision_risk,
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # 緊急停止
        # ----------------------------------------------------

        if (
            front
            <=
            self.emergency_distance
        ):
            safety.emergency = True
            safety.must_stop = True

        # ----------------------------------------------------
        # カメラ/YOLOの高信頼度正面障害物
        # ----------------------------------------------------

        if (
            obstacle_state.center
            and
            obstacle_state.confidence
            >=
            0.85
        ):
            safety.emergency = True
            safety.must_stop = True

        # ----------------------------------------------------
        # 減速
        # ----------------------------------------------------

        slow_down_threshold = float(
            get_config(
                "PERCEPTION_SLOW_DOWN_RISK",
                0.35,
            )
        )

        safety.must_slow_down = (
            safety.danger_level
            >=
            slow_down_threshold
        )

        if course_state.corner_detected:
            safety.must_slow_down = True

        # ----------------------------------------------------
        # 回避方向
        # ----------------------------------------------------

        safety.escape_direction = (
            self._select_escape_direction(
                left,
                right,
                obstacle_state,
                course_state,
            )
        )

        return safety

    # ========================================================
    # 危険度
    # ========================================================

    def _distance_risk(
        self,
        distance: float,
        warning_distance: float,
        emergency_distance: float,
    ) -> float:

        if distance <= emergency_distance:
            return 1.0

        if distance >= warning_distance:
            return 0.0

        span = (
            warning_distance
            -
            emergency_distance
        )

        if span <= 0:
            return 1.0

        risk = (
            warning_distance
            -
            distance
        ) / span

        return clamp(
            risk,
            0.0,
            1.0,
        )

    # ========================================================
    # 回避方向
    # ========================================================

    def _select_escape_direction(
        self,
        left_distance: float,
        right_distance: float,
        obstacle_state: ObstacleState,
        course_state: CourseState,
    ) -> Optional[str]:

        if obstacle_state.left:
            return "right"

        if obstacle_state.right:
            return "left"

        if obstacle_state.center:

            if (
                left_distance
                >
                right_distance
            ):
                return "left"

            if (
                right_distance
                >
                left_distance
            ):
                return "right"

        if course_state.left_open:
            return "left"

        if course_state.right_open:
            return "right"

        return None

    # ========================================================
    # 推奨値
    # ========================================================

    def _calculate_recommendation(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        course_state: CourseState,
        safety_state: SafetyState,
    ) -> DrivingRecommendation:

        recommendation = (
            DrivingRecommendation()
        )

        # ----------------------------------------------------
        # 緊急停止
        # ----------------------------------------------------

        if safety_state.must_stop:

            recommendation.steering = 0.0
            recommendation.throttle = 0.0
            recommendation.confidence = 1.0
            recommendation.reason = (
                "emergency_stop"
            )

            return recommendation

        # ----------------------------------------------------
        # 基本ステア
        # ----------------------------------------------------

        steering = (
            course_state.direction
        )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        if course_state.corner_detected:

            if course_state.corner_direction == "left":
                steering = min(
                    steering,
                    -self.corner_steering_gain,
                )

            elif course_state.corner_direction == "right":
                steering = max(
                    steering,
                    self.corner_steering_gain,
                )

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        if obstacle_state.detected:

            escape = (
                safety_state.escape_direction
            )

            if escape == "left":

                steering = min(
                    steering,
                    -self.obstacle_steering_gain,
                )

            elif escape == "right":

                steering = max(
                    steering,
                    self.obstacle_steering_gain,
                )

        steering = clamp(
            steering,
            -1.0,
            1.0,
        )

        recommendation.steering = (
            steering
        )

        # ----------------------------------------------------
        # 基本速度
        # ----------------------------------------------------

        throttle = (
            self.base_throttle
        )

        # ----------------------------------------------------
        # 前方距離
        # ----------------------------------------------------

        front_factor = clamp(
            wall_state.front_distance
            /
            max(
                1.0,
                self.front_warning_distance,
            ),
            0.0,
            1.0,
        )

        # 前方が近いほど速度を落とす
        if front_factor < 1.0:
            throttle *= (
                0.35
                +
                0.65
                * front_factor
            )

        # ----------------------------------------------------
        # 曲がるほど減速
        # ----------------------------------------------------

        steering_slowdown = clamp(
            float(
                get_config(
                    "PERCEPTION_STEERING_SLOWDOWN",
                    0.45,
                )
            ),
            0.0,
            1.0,
        )

        throttle *= (
            1.0
            -
            steering_slowdown
            * abs(
                steering
            )
        )

        # ----------------------------------------------------
        # 危険度
        # ----------------------------------------------------

        danger_slowdown = clamp(
            float(
                get_config(
                    "PERCEPTION_DANGER_SLOWDOWN",
                    0.55,
                )
            ),
            0.0,
            1.0,
        )

        throttle *= (
            1.0
            -
            danger_slowdown
            *
            safety_state.danger_level
        )

        # ----------------------------------------------------
        # 明確な減速
        # ----------------------------------------------------

        if safety_state.must_slow_down:

            throttle *= clamp(
                float(
                    get_config(
                        "PERCEPTION_SLOW_DOWN_FACTOR",
                        0.60,
                    )
                ),
                0.0,
                1.0,
            )

        # ----------------------------------------------------
        # 最小・最大
        # ----------------------------------------------------

        if throttle > 0:
            throttle = max(
                throttle,
                self.minimum_throttle,
            )

        throttle = clamp(
            throttle,
            0.0,
            self.maximum_throttle,
        )

        recommendation.throttle = (
            throttle
        )

        # ----------------------------------------------------
        # 信頼度
        # ----------------------------------------------------

        recommendation.confidence = (
            self._calculate_confidence(
                wall_state,
                obstacle_state,
                course_state,
            )
        )

        # ----------------------------------------------------
        # 理由
        # ----------------------------------------------------

        recommendation.reason = (
            self._build_reason(
                obstacle_state,
                course_state,
                safety_state,
            )
        )

        return recommendation

    # ========================================================
    # 信頼度
    # ========================================================

    def _calculate_confidence(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        course_state: CourseState,
    ) -> float:

        values = []

        if wall_state.left_detected:
            values.append(
                wall_state.left_confidence
            )

        if wall_state.right_detected:
            values.append(
                wall_state.right_confidence
            )

        if wall_state.front_detected:
            values.append(
                wall_state.front_confidence
            )

        if obstacle_state.detected:
            values.append(
                obstacle_state.confidence
            )

        if course_state.corner_detected:
            values.append(
                0.80
            )

        if not values:
            return 0.50

        return clamp(
            float(
                np.mean(
                    values
                )
            ),
            0.0,
            1.0,
        )

    # ========================================================
    # 理由
    # ========================================================

    def _build_reason(
        self,
        obstacle_state: ObstacleState,
        course_state: CourseState,
        safety_state: SafetyState,
    ) -> str:

        if safety_state.must_stop:
            return "緊急停止"

        if course_state.dead_end:
            return "行き止まり"

        if obstacle_state.detected:
            if safety_state.escape_direction:
                return (
                    "障害物回避:"
                    +
                    safety_state.escape_direction
                )

            return "障害物"

        if course_state.corner_detected:
            return (
                "コーナー:"
                +
                str(
                    course_state.corner_direction
                )
            )

        if course_state.left_open:
            return "左側が広い"

        if course_state.right_open:
            return "右側が広い"

        return "通常走行"

    # ========================================================
    # カメラ標準化
    # ========================================================

    def normalize_camera_data(
        self,
        camera_data: Optional[
            Dict[str, Any]
        ],
    ) -> Dict[str, Any]:

        if not camera_data:
            return {}

        result = dict(
            camera_data
        )

        for key in (
            "left_confidence",
            "right_confidence",
            "front_confidence",
            "direction_confidence",
            "corner_confidence",
            "obstacle_confidence",
        ):
            result[key] = clamp(
                safe_float(
                    result.get(
                        key,
                        0.0,
                    ),
                    0.0,
                ),
                0.0,
                1.0,
            )

        result["direction"] = (
            self._convert_direction_value(
                result.get(
                    "direction",
                    0.0,
                )
            )
        )

        corner = result.get(
            "corner"
        )

        if corner not in (
            "left",
            "right",
            None,
        ):
            result["corner"] = None

        return result

    # ========================================================
    # YOLO標準化
    # ========================================================

    def normalize_yolo_data(
        self,
        yolo_data: Optional[
            Dict[str, Any]
        ],
    ) -> Dict[str, Any]:

        if not yolo_data:
            return {
                "detections": []
            }

        detections = yolo_data.get(
            "detections",
            [],
        )

        if not isinstance(
            detections,
            list,
        ):
            detections = []

        normalized = []

        for detection in detections:

            if not isinstance(
                detection,
                dict,
            ):
                continue

            item = dict(
                detection
            )

            item["confidence"] = clamp(
                safe_float(
                    item.get(
                        "confidence",
                        0.0,
                    ),
                    0.0,
                ),
                0.0,
                1.0,
            )

            item["x_center"] = clamp(
                safe_float(
                    item.get(
                        "x_center",
                        0.5,
                    ),
                    0.5,
                ),
                0.0,
                1.0,
            )

            item["y_center"] = clamp(
                safe_float(
                    item.get(
                        "y_center",
                        0.5,
                    ),
                    0.5,
                ),
                0.0,
                1.0,
            )

            normalized.append(
                item
            )

        return {
            "detections":
                normalized
        }

    # ========================================================
    # normalized update
    # ========================================================

    def update_normalized(
        self,
        ultrasonic_data: Optional[
            Dict[str, Any]
        ] = None,
        camera_data: Optional[
            Dict[str, Any]
        ] = None,
        lidar_data: Optional[
            Dict[str, Any]
        ] = None,
        yolo_data: Optional[
            Dict[str, Any]
        ] = None,
    ) -> PerceptionResult:

        return self.update(
            ultrasonic_data=ultrasonic_data,
            camera_data=self.normalize_camera_data(
                camera_data
            ),
            lidar_data=lidar_data,
            yolo_data=self.normalize_yolo_data(
                yolo_data
            ),
        )

    # ========================================================
    # 方向平滑化
    # ========================================================

    def stabilize_direction(
        self,
        direction: float,
    ) -> float:

        self._direction_history.append(
            clamp(
                direction,
                -1.0,
                1.0,
            )
        )

        if (
            len(
                self._direction_history
            )
            >
            self.history_size
        ):
            del self._direction_history[0]

        if not self._direction_history:
            return direction

        weights = np.arange(
            1,
            len(
                self._direction_history
            ) + 1,
            dtype=np.float32,
        )

        values = np.asarray(
            self._direction_history,
            dtype=np.float32,
        )

        return clamp(
            float(
                np.sum(
                    values * weights
                )
                /
                np.sum(
                    weights
                )
            ),
            -1.0,
            1.0,
        )

    # ========================================================
    # センサー信頼度
    # ========================================================

    def fuse_sensor_confidence(
        self,
        ultrasonic_confidence: float,
        camera_confidence: float = 0.0,
        lidar_confidence: float = 0.0,
        yolo_confidence: float = 0.0,
    ) -> float:

        weights = {
            "ultrasonic":
                float(
                    get_config(
                        "PERCEPTION_ULTRASONIC_CONFIDENCE_WEIGHT",
                        0.50,
                    )
                ),

            "camera":
                float(
                    get_config(
                        "PERCEPTION_CAMERA_CONFIDENCE_WEIGHT",
                        0.30,
                    )
                ),

            "lidar":
                float(
                    get_config(
                        "PERCEPTION_LIDAR_CONFIDENCE_WEIGHT",
                        0.10,
                    )
                ),

            "yolo":
                float(
                    get_config(
                        "PERCEPTION_YOLO_CONFIDENCE_WEIGHT",
                        0.10,
                    )
                ),
        }

        values = {
            "ultrasonic":
                clamp(
                    ultrasonic_confidence,
                    0.0,
                    1.0,
                ),

            "camera":
                clamp(
                    camera_confidence,
                    0.0,
                    1.0,
                ),

            "lidar":
                clamp(
                    lidar_confidence,
                    0.0,
                    1.0,
                ),

            "yolo":
                clamp(
                    yolo_confidence,
                    0.0,
                    1.0,
                ),
        }

        total = sum(
            max(
                0.0,
                weight,
            )
            for weight in weights.values()
        )

        if total <= 0:
            return 0.0

        result = sum(
            values[key]
            *
            max(
                0.0,
                weights[key],
            )
            for key in weights
        )

        return clamp(
            result / total,
            0.0,
            1.0,
        )

    # ========================================================
    # 壁角度
    # ========================================================

    def estimate_wall_angle(
        self,
        front_side_distance: float,
        rear_side_distance: float,
        side: str,
    ) -> float:

        front = safe_float(
            front_side_distance,
            self.default_distance,
        )

        rear = safe_float(
            rear_side_distance,
            self.default_distance,
        )

        if side not in (
            "left",
            "right",
        ):
            raise ValueError(
                "side must be 'left' or 'right'"
            )

        sin45 = math.sin(
            math.radians(45.0)
        )

        cos45 = math.cos(
            math.radians(45.0)
        )

        if side == "right":

            dx = (
                front
                * sin45
                -
                rear
            )

        else:

            dx = (
                -front
                * sin45
                +
                rear
            )

        dy = (
            front
            * cos45
        )

        if abs(dy) < 1e-6:
            return 0.0

        return math.atan2(
            dx,
            dy,
        )

    # ========================================================
    # モード補正
    # ========================================================

    def apply_mode_correction(
        self,
        base_steering: float,
        mode: str,
        result: Optional[
            PerceptionResult
        ] = None,
    ) -> float:

        if result is None:
            result = self.result

        steering = clamp(
            safe_float(
                base_steering,
                0.0,
            ),
            -1.0,
            1.0,
        )

        mode_name = (
            str(
                mode
                if mode is not None
                else ""
            )
            .lower()
        )

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        if result.obstacle.detected:

            gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_MODE_OBSTACLE_CORRECTION_GAIN",
                        0.35,
                    )
                ),
                0.0,
                1.0,
            )

            if (
                result.safety.escape_direction
                == "left"
            ):
                steering -= gain

            elif (
                result.safety.escape_direction
                == "right"
            ):
                steering += gain

        # ----------------------------------------------------
        # カメラ方向補正
        # ----------------------------------------------------

        camera_direction = (
            result.camera_raw.get(
                "direction"
            )
        )

        camera_confidence = clamp(
            safe_float(
                result.camera_raw.get(
                    "direction_confidence",
                    0.0,
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        camera_gain = clamp(
            float(
                get_config(
                    "PERCEPTION_MODE_CAMERA_CORRECTION_GAIN",
                    0.20,
                )
            ),
            0.0,
            1.0,
        )

        if (
            camera_direction is not None
            and
            camera_confidence
            >=
            self.camera_confidence_threshold
        ):

            steering += (
                self._convert_direction_value(
                    camera_direction
                )
                *
                camera_confidence
                *
                camera_gain
            )

        # ----------------------------------------------------
        # コーナー補正
        # ----------------------------------------------------

        if result.course.corner_detected:

            corner_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_MODE_CORNER_CORRECTION_GAIN",
                        0.20,
                    )
                ),
                0.0,
                1.0,
            )

            if (
                result.course.corner_direction
                == "left"
            ):
                steering -= corner_gain

            elif (
                result.course.corner_direction
                == "right"
            ):
                steering += corner_gain

        # ----------------------------------------------------
        # wall_follow系
        # ----------------------------------------------------

        if mode_name in (
            "wall_follow",
            "wall_follow_pid",
        ):

            gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_WALL_CORRECTION_GAIN",
                        0.12,
                    )
                ),
                0.0,
                1.0,
            )

            steering += (
                result.course.center_offset
                *
                gain
            )

        # ----------------------------------------------------
        # center_follow
        # ----------------------------------------------------

        elif mode_name == "center_follow_pid":

            gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_CENTER_CORRECTION_GAIN",
                        0.15,
                    )
                ),
                0.0,
                1.0,
            )

            steering += (
                result.course.center_offset
                *
                gain
            )

        # ----------------------------------------------------
        # racer / gap
        # ----------------------------------------------------

        elif mode_name in (
            "racer",
            "gap_follow",
            "follow_the_gap",
        ):

            gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_HIGH_SPEED_CORRECTION_GAIN",
                        0.10,
                    )
                ),
                0.0,
                1.0,
            )

            steering += (
                result.course.direction
                *
                gain
            )

        if result.safety.must_stop:
            steering = 0.0

        return clamp(
            steering,
            -1.0,
            1.0,
        )

    # ========================================================
    # 速度補正
    # ========================================================

    def apply_speed_correction(
        self,
        base_throttle: float,
        mode: str,
        result: Optional[
            PerceptionResult
        ] = None,
    ) -> float:

        if result is None:
            result = self.result

        throttle = clamp(
            safe_float(
                base_throttle,
                0.0,
            ),
            0.0,
            1.0,
        )

        if result.safety.must_stop:
            return 0.0

        # ----------------------------------------------------
        # 危険度
        # ----------------------------------------------------

        danger_gain = clamp(
            float(
                get_config(
                    "PERCEPTION_MODE_DANGER_SPEED_GAIN",
                    0.40,
                )
            ),
            0.0,
            1.0,
        )

        throttle *= (
            1.0
            -
            result.safety.danger_level
            *
            danger_gain
        )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        if result.course.corner_detected:

            corner_factor = clamp(
                float(
                    get_config(
                        "PERCEPTION_MODE_CORNER_SPEED_FACTOR",
                        0.75,
                    )
                ),
                0.0,
                1.0,
            )

            throttle *= (
                corner_factor
            )

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        if result.obstacle.detected:

            obstacle_factor = clamp(
                float(
                    get_config(
                        "PERCEPTION_MODE_OBSTACLE_SPEED_FACTOR",
                        0.60,
                    )
                ),
                0.0,
                1.0,
            )

            throttle *= (
                obstacle_factor
            )

        # ----------------------------------------------------
        # 最終
        # ----------------------------------------------------

        return clamp(
            throttle,
            0.0,
            1.0,
        )

    # ========================================================
    # Planner接続
    # ========================================================

    def get_planner_input(
        self,
        mode: Optional[str] = None,
        base_steering: Optional[float] = None,
        base_throttle: Optional[float] = None,
    ) -> Dict[str, Any]:

        result = self.result

        if base_steering is None:
            steering = (
                result.recommendation.steering
            )

        else:
            steering = safe_float(
                base_steering,
                0.0,
            )

        if base_throttle is None:
            throttle = (
                result.recommendation.throttle
            )

        else:
            throttle = safe_float(
                base_throttle,
                0.0,
            )

        if mode is not None:

            steering = (
                self.apply_mode_correction(
                    steering,
                    mode,
                    result,
                )
            )

            throttle = (
                self.apply_speed_correction(
                    throttle,
                    mode,
                    result,
                )
            )

        return {
            "steering":
                clamp(
                    steering,
                    -1.0,
                    1.0,
                ),

            "throttle":
                clamp(
                    throttle,
                    0.0,
                    1.0,
                ),

            "direction":
                result.course.direction,

            "center_offset":
                result.course.center_offset,

            "left_distance":
                result.wall.left_distance,

            "front_distance":
                result.wall.front_distance,

            "right_distance":
                result.wall.right_distance,

            "left_wall":
                result.wall.left_detected,

            "right_wall":
                result.wall.right_detected,

            "front_wall":
                result.wall.front_detected,

            "corner":
                result.course.corner_direction,

            "corner_detected":
                result.course.corner_detected,

            "obstacle":
                result.obstacle.detected,

            "obstacle_left":
                result.obstacle.left,

            "obstacle_center":
                result.obstacle.center,

            "obstacle_right":
                result.obstacle.right,

            "danger_level":
                result.safety.danger_level,

            "must_stop":
                result.safety.must_stop,

            "must_slow_down":
                result.safety.must_slow_down,

            "escape_direction":
                result.safety.escape_direction,

            "confidence":
                result.recommendation.confidence,

            "reason":
                result.recommendation.reason,
        }

    # ========================================================
    # デバッグ
    # ========================================================

    def get_debug_summary(
        self,
    ) -> Dict[str, Any]:

        result = self.result

        return {
            "frame_count":
                self.frame_count,

            "left_distance":
                result.wall.left_distance,

            "front_distance":
                result.wall.front_distance,

            "right_distance":
                result.wall.right_distance,

            "left_wall":
                result.wall.left_detected,

            "front_wall":
                result.wall.front_detected,

            "right_wall":
                result.wall.right_detected,

            "direction":
                result.course.direction,

            "center_offset":
                result.course.center_offset,

            "corner":
                result.course.corner_direction,

            "dead_end":
                result.course.dead_end,

            "intersection":
                result.course.intersection,

            "obstacle":
                result.obstacle.detected,

            "obstacle_source":
                result.obstacle.source,

            "danger_level":
                result.safety.danger_level,

            "must_stop":
                result.safety.must_stop,

            "must_slow_down":
                result.safety.must_slow_down,

            "escape_direction":
                result.safety.escape_direction,

            "recommended_steering":
                result.recommendation.steering,

            "recommended_throttle":
                result.recommendation.throttle,

            "reason":
                result.recommendation.reason,
        }

    # ========================================================
    # Debug print
    # ========================================================

    def log_debug_summary(
        self,
        force: bool = False,
    ) -> None:

        enabled = safe_bool(
            get_config(
                "PERCEPTION_DEBUG",
                False,
            )
        )

        if (
            not enabled
            and
            not force
        ):
            return

        summary = (
            self.get_debug_summary()
        )

        logger.info(
            "PERCEPTION | "
            "L=%.0f F=%.0f R=%.0f | "
            "dir=%.2f center=%.2f | "
            "steer=%.2f throttle=%.2f | "
            "obstacle=%s danger=%.2f | "
            "reason=%s",
            safe_float(
                summary["left_distance"]
            ),
            safe_float(
                summary["front_distance"]
            ),
            safe_float(
                summary["right_distance"]
            ),
            safe_float(
                summary["direction"]
            ),
            safe_float(
                summary["center_offset"]
            ),
            safe_float(
                summary["recommended_steering"]
            ),
            safe_float(
                summary["recommended_throttle"]
            ),
            summary["obstacle"],
            safe_float(
                summary["danger_level"]
            ),
            summary["reason"],
        )

    # ========================================================
    # Dictionary
    # ========================================================

    def to_dict(
        self,
        result: Optional[
            PerceptionResult
        ] = None,
    ) -> Dict[str, Any]:

        if result is None:
            result = self.result

        return {
            "timestamp":
                result.timestamp,

            "wall": {
                "left_detected":
                    result.wall.left_detected,

                "front_detected":
                    result.wall.front_detected,

                "right_detected":
                    result.wall.right_detected,

                "left_distance":
                    result.wall.left_distance,

                "front_distance":
                    result.wall.front_distance,

                "right_distance":
                    result.wall.right_distance,

                "left_angle":
                    result.wall.left_angle,

                "right_angle":
                    result.wall.right_angle,

                "left_confidence":
                    result.wall.left_confidence,

                "front_confidence":
                    result.wall.front_confidence,

                "right_confidence":
                    result.wall.right_confidence,
            },

            "obstacle": {
                "detected":
                    result.obstacle.detected,

                "left":
                    result.obstacle.left,

                "center":
                    result.obstacle.center,

                "right":
                    result.obstacle.right,

                "distance":
                    result.obstacle.distance,

                "object_type":
                    result.obstacle.object_type,

                "confidence":
                    result.obstacle.confidence,

                "source":
                    result.obstacle.source,

                "detections":
                    result.obstacle.detections,
            },

            "course": {
                "direction":
                    result.course.direction,

                "left_open":
                    result.course.left_open,

                "right_open":
                    result.course.right_open,

                "left_free_space":
                    result.course.left_free_space,

                "right_free_space":
                    result.course.right_free_space,

                "center_offset":
                    result.course.center_offset,

                "estimated_width":
                    result.course.estimated_width,

                "corner_detected":
                    result.course.corner_detected,

                "corner_direction":
                    result.course.corner_direction,

                "intersection":
                    result.course.intersection,

                "dead_end":
                    result.course.dead_end,
            },

            "safety": {
                "emergency":
                    result.safety.emergency,

                "danger_level":
                    result.safety.danger_level,

                "collision_risk":
                    result.safety.collision_risk,

                "must_slow_down":
                    result.safety.must_slow_down,

                "must_stop":
                    result.safety.must_stop,

                "escape_direction":
                    result.safety.escape_direction,
            },

            "recommendation": {
                "steering":
                    result.recommendation.steering,

                "throttle":
                    result.recommendation.throttle,

                "confidence":
                    result.recommendation.confidence,

                "reason":
                    result.recommendation.reason,
            },

            "debug":
                result.debug,
        }

    # ========================================================
    # Status
    # ========================================================

    def get_status(
        self,
    ) -> Dict[str, Any]:

        return {
            "initialized":
                self.initialized,

            "frame_count":
                self.frame_count,

            "camera_active":
                bool(
                    self.result.camera_raw
                ),

            "lidar_active":
                bool(
                    self.result.lidar_raw
                ),

            "yolo_active":
                bool(
                    self.result.vision_raw
                ),

            "emergency":
                self.result.safety.emergency,
        }

    # ========================================================
    # Reset
    # ========================================================

    def reset(
        self,
    ) -> None:

        self.result = (
            PerceptionResult()
        )

        self.frame_count = 0

        self.initialized = False

        self.last_update_time = (
            time.perf_counter()
        )

        self._left_distance_history.clear()
        self._front_distance_history.clear()
        self._right_distance_history.clear()
        self._direction_history.clear()

        logger.info(
            "Perception reset"
        )

    def full_reset(
        self,
    ) -> None:
        self.reset()


# ============================================================
# Alias
# ============================================================

PerceptionEngine = Perception


# ============================================================
# オフラインテスト
# ============================================================

def test_perception() -> None:
    """
    実車なしで perception.py のロジックを確認する。

    実行:
        python perception.py
    """

    print("=" * 70)
    print("PERCEPTION TEST")
    print("=" * 70)

    perception = Perception()

    # ----------------------------------------
    # ケース1: 左右に壁、前方は十分空いている
    # ----------------------------------------

    ultrasonic = {
        "FrLH": 180,
        "FrFR": 900,
        "FrRH": 350,
    }

    camera = {
        "left_wall": True,
        "right_wall": True,
        "front_wall": False,

        "left_confidence": 0.90,
        "right_confidence": 0.80,
        "front_confidence": 0.20,

        "direction": "right",
        "direction_confidence": 0.75,

        "corner": None,
        "corner_confidence": 0.0,

        "obstacle": False,
        "obstacle_confidence": 0.0,
    }

    result = perception.update_normalized(
        ultrasonic_data=ultrasonic,
        camera_data=camera,
        lidar_data=None,
        yolo_data=None,
    )

    print()
    print("【CASE 1】")
    print(
        perception.get_debug_summary()
    )

    # ----------------------------------------
    # ケース2: 前方が危険
    # ----------------------------------------

    ultrasonic = {
        "FrLH": 300,
        "FrFR": 200,
        "FrRH": 500,
    }

    camera = {
        "left_wall": True,
        "right_wall": True,
        "front_wall": True,

        "left_confidence": 0.80,
        "right_confidence": 0.80,
        "front_confidence": 0.95,

        "direction": "left",
        "direction_confidence": 0.85,

        "corner": "left",
        "corner_confidence": 0.90,

        "obstacle": True,
        "obstacle_confidence": 0.95,
        "obstacle_side": "left",
    }

    result = perception.update_normalized(
        ultrasonic_data=ultrasonic,
        camera_data=camera,
        lidar_data=None,
        yolo_data=None,
    )

    print()
    print("【CASE 2】")
    print(
        perception.get_debug_summary()
    )

    # ----------------------------------------
    # Planner入力
    # ----------------------------------------

    planner_input = (
        perception.get_planner_input(
            mode="racer",
            base_steering=0.0,
            base_throttle=0.8,
        )
    )

    print()
    print("【PLANNER INPUT】")

    for key, value in planner_input.items():
        print(
            f"{key}: {value}"
        )

    print()
    print("=" * 70)
    print("TEST COMPLETE")
    print("=" * 70)


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    test_perception()
