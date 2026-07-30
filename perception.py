# coding:utf-8

"""
perception.py

センサーフュージョン用の認識モジュール。

役割:
    超音波センサー
    カメラ
    YOLOなどの物体検出
    将来的なLiDAR

から得られた情報を統合し、

    ・左右の壁
    ・前方の壁
    ・障害物
    ・コーナー
    ・行き止まり
    ・コース中央からのずれ
    ・壁の角度
    ・危険度
    ・推奨速度
    ・推奨ステアリング

などの「走行判断に使いやすい情報」に変換する。

このファイルは、
「センサーから情報を作るところ」と
「その情報を使って実際に走るところ」
を分離するための中間層として使用する。

今後の想定:

    ultrasonic.py
          │
    camera.py
          │
      YOLO
          │
       LiDAR
          │
          ▼
    perception.py
          │
          ▼
      planner.py
          │
          ▼
      controller
          │
          ▼
        motor
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import math
import logging
import time

import numpy as np

import config


logger = logging.getLogger(__name__)


# ============================================================
# 共通ユーティリティ
# ============================================================

def clamp(value: float, minimum: float, maximum: float) -> float:
    """
    数値をminimum〜maximumの範囲に収める。
    """
    return max(minimum, min(maximum, value))


def safe_float(value: Any, default: float = 0.0) -> float:
    """
    数値変換を安全に行う。

    Noneや文字列などが来てもエラーにせず、
    defaultを返す。
    """
    if value is None:
        return default

    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_bool(value: Any, default: bool = False) -> bool:
    """
    真偽値を安全に取得する。
    """
    if value is None:
        return default

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        value_lower = value.lower()

        if value_lower in ("true", "1", "yes", "on"):
            return True

        if value_lower in ("false", "0", "no", "off"):
            return False

    try:
        return bool(value)
    except Exception:
        return default


def get_config(
    name: str,
    default: Any = None
) -> Any:
    """
    config.py / config_hanson.pyから安全に設定値を取得する。
    """
    return getattr(config, name, default)


# ============================================================
# センサー名の定義
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
# 壁情報
# ============================================================

@dataclass
class WallState:
    """
    左右および前方の壁の認識結果。
    """

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
# 障害物情報
# ============================================================

@dataclass
class ObstacleState:
    """
    障害物の認識結果。
    """

    detected: bool = False

    left: bool = False
    center: bool = False
    right: bool = False

    distance: Optional[float] = None

    object_type: Optional[str] = None

    confidence: float = 0.0

    source: str = "none"

    # YOLOなどから得られる詳細情報
    detections: List[Dict[str, Any]] = field(default_factory=list)


# ============================================================
# コース状態
# ============================================================

@dataclass
class CourseState:
    """
    コースの形状に関する推定結果。
    """

    # -1.0 = 左
    #  0.0 = 中央
    # +1.0 = 右
    direction: float = 0.0

    # 左右のどちらが広いか
    left_open: bool = False
    right_open: bool = False

    # 左右の空き具合
    left_free_space: float = 0.0
    right_free_space: float = 0.0

    # コーナー情報
    corner_detected: bool = False
    corner_direction: Optional[str] = None

    # 行き止まり
    dead_end: bool = False

    # 交差点
    intersection: bool = False

    # コース中央からのずれ
    center_offset: float = 0.0

    # コース幅の推定
    estimated_width: Optional[float] = None


# ============================================================
# 安全状態
# ============================================================

@dataclass
class SafetyState:
    """
    安全性に関する統合結果。
    """

    emergency: bool = False

    danger_level: float = 0.0

    collision_risk: float = 0.0

    must_slow_down: bool = False

    must_stop: bool = False

    escape_direction: Optional[str] = None


# ============================================================
# 走行推奨値
# ============================================================

@dataclass
class DrivingRecommendation:
    """
    perceptionがplannerへ渡す推奨値。

    ここでは最終的なモーターPWMを決めない。

    あくまで
        「どちらへ行きたいか」
        「どれくらい危険か」
        「どれくらい減速すべきか」
    を表現する。
    """

    # -1.0 = 左
    #  0.0 = 直進
    # +1.0 = 右
    steering: float = 0.0

    # 0.0 = 停止
    # 1.0 = 最大
    throttle: float = 0.0

    confidence: float = 0.0

    reason: str = ""


# ============================================================
# 最終認識結果
# ============================================================

@dataclass
class PerceptionResult:
    """
    perception.pyが最終的に生成するデータ。

    planner.pyは基本的にこのデータだけを見れば、
    カメラや超音波の生データを直接扱わなくてもよい構成を目指す。
    """

    timestamp: float = field(default_factory=time.perf_counter)

    # 壁
    wall: WallState = field(default_factory=WallState)

    # 障害物
    obstacle: ObstacleState = field(default_factory=ObstacleState)

    # コース
    course: CourseState = field(default_factory=CourseState)

    # 安全
    safety: SafetyState = field(default_factory=SafetyState)

    # 推奨値
    recommendation: DrivingRecommendation = field(
        default_factory=DrivingRecommendation
    )

    # 元センサーデータ
    ultrasonic_raw: Dict[str, Any] = field(default_factory=dict)

    # カメラから得られた情報
    camera_raw: Dict[str, Any] = field(default_factory=dict)

    # LiDARから得られた情報
    lidar_raw: Dict[str, Any] = field(default_factory=dict)

    # YOLO等の認識結果
    vision_raw: Dict[str, Any] = field(default_factory=dict)

    # デバッグ用
    debug: Dict[str, Any] = field(default_factory=dict)


# ============================================================
# Perception本体
# ============================================================

class Perception:
    """
    センサーフュージョンの中心クラス。

    入力:
        ultrasonic_data
        camera_data
        lidar_data
        yolo_data

    出力:
        PerceptionResult

    今後、

        ultrasonic
        camera
        YOLO
        LiDAR
        AI

    を追加しても、planner.py側はできるだけ変更しない
    ことを目的とする。
    """

    def __init__(self):
        # 前回結果
        self.result = PerceptionResult()

        # 前回の認識時間
        self.last_update_time = time.perf_counter()

        # フレーム番号
        self.frame_count = 0

        # 認識状態
        self.initialized = False

        # 平滑化用の履歴
        self._left_distance_history: List[float] = []
        self._front_distance_history: List[float] = []
        self._right_distance_history: List[float] = []

        # 最大履歴数
        self.history_size = int(
            get_config(
                "PERCEPTION_HISTORY_SIZE",
                5
            )
        )

        # センサー値が異常だった場合の安全値
        self.default_distance = float(
            get_config(
                "PERCEPTION_DEFAULT_DISTANCE",
                3000.0
            )
        )

        # 壁検出距離
        self.wall_detection_distance = float(
            get_config(
                "PERCEPTION_WALL_DETECTION_RANGE",
                get_config(
                    "DETECTION_RANGE",
                    1000.0
                )
            )
        )

        # 障害物判定距離
        self.obstacle_detection_distance = float(
            get_config(
                "PERCEPTION_OBSTACLE_DISTANCE",
                get_config(
                    "DETECTION_RANGE",
                    500.0
                )
            )
        )

        # 非常停止距離
        self.emergency_distance = float(
            get_config(
                "PERCEPTION_EMERGENCY_DISTANCE",
                get_config(
                    "STOP_RANGE",
                    150.0
                )
            )
        )

        logger.info("Perception 起動")
        logger.info(
            "wall_detection_distance=%.1f, "
            "obstacle_detection_distance=%.1f, "
            "emergency_distance=%.1f",
            self.wall_detection_distance,
            self.obstacle_detection_distance,
            self.emergency_distance,
        )

    # ========================================================
    # メイン更新
    # ========================================================

    def update(
        self,
        ultrasonic_data: Optional[Dict[str, Any]] = None,
        camera_data: Optional[Dict[str, Any]] = None,
        lidar_data: Optional[Dict[str, Any]] = None,
        yolo_data: Optional[Dict[str, Any]] = None,
    ) -> PerceptionResult:
        """
        センサー情報を統合してPerceptionResultを生成する。

        Parameters
        ----------
        ultrasonic_data:
            超音波センサーの生データ。

        camera_data:
            カメラ画像から抽出した認識情報。

        lidar_data:
            LiDARから得られた認識情報。

        yolo_data:
            YOLOなどの物体検出結果。

        Returns
        -------
        PerceptionResult
        """

        self.frame_count += 1

        now = time.perf_counter()

        dt = now - self.last_update_time

        if dt <= 0:
            dt = 1e-6

        self.last_update_time = now

        # ----------------------------------------------------
        # 入力を安全に正規化
        # ----------------------------------------------------

        if ultrasonic_data is None:
            ultrasonic_data = {}

        if camera_data is None:
            camera_data = {}

        if lidar_data is None:
            lidar_data = {}

        if yolo_data is None:
            yolo_data = {}

        # ----------------------------------------------------
        # 超音波距離抽出
        # ----------------------------------------------------

        left_distance = self._extract_distance(
            ultrasonic_data,
            ULTRASONIC_LEFT_KEYS,
            default=self.default_distance,
        )

        front_distance = self._extract_distance(
            ultrasonic_data,
            ULTRASONIC_FRONT_KEYS,
            default=self.default_distance,
        )

        right_distance = self._extract_distance(
            ultrasonic_data,
            ULTRASONIC_RIGHT_KEYS,
            default=self.default_distance,
        )

        # ----------------------------------------------------
        # 履歴更新
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
        # 平滑化値
        # ----------------------------------------------------

        left_filtered = self._get_filtered_distance(
            self._left_distance_history,
            left_distance,
        )

        front_filtered = self._get_filtered_distance(
            self._front_distance_history,
            front_distance,
        )

        right_filtered = self._get_filtered_distance(
            self._right_distance_history,
            right_distance,
        )

        # ----------------------------------------------------
        # 壁状態の初期生成
        # ----------------------------------------------------

        wall_state = WallState(
            left_distance=left_filtered,
            front_distance=front_filtered,
            right_distance=right_filtered,
        )

        # ----------------------------------------------------
        # 超音波から壁を判定
        # ----------------------------------------------------

        wall_state.left_detected = (
            left_filtered <= self.wall_detection_distance
        )

        wall_state.front_detected = (
            front_filtered <= self.wall_detection_distance
        )

        wall_state.right_detected = (
            right_filtered <= self.wall_detection_distance
        )

        # ----------------------------------------------------
        # 超音波から障害物を仮判定
        # ----------------------------------------------------

        obstacle_state = self._detect_obstacle_from_ultrasonic(
            left_filtered,
            front_filtered,
            right_filtered,
        )

        # ----------------------------------------------------
        # カメラ情報との統合
        # ----------------------------------------------------

        wall_state = self._fuse_camera_wall_information(
            wall_state,
            camera_data,
        )

        obstacle_state = self._fuse_camera_obstacle_information(
            obstacle_state,
            camera_data,
            yolo_data,
        )

        # ----------------------------------------------------
        # コース状態
        # ----------------------------------------------------

        course_state = self._analyze_course(
            wall_state=wall_state,
            obstacle_state=obstacle_state,
            camera_data=camera_data,
            lidar_data=lidar_data,
        )

        # ----------------------------------------------------
        # 安全状態
        # ----------------------------------------------------

        safety_state = self._analyze_safety(
            wall_state=wall_state,
            obstacle_state=obstacle_state,
            course_state=course_state,
        )

        # ----------------------------------------------------
        # 推奨走行
        # ----------------------------------------------------

        recommendation = self._calculate_recommendation(
            wall_state=wall_state,
            obstacle_state=obstacle_state,
            course_state=course_state,
            safety_state=safety_state,
        )

        # ----------------------------------------------------
        # 結果生成
        # ----------------------------------------------------

        self.result = PerceptionResult(
            timestamp=time.perf_counter(),

            wall=wall_state,

            obstacle=obstacle_state,

            course=course_state,

            safety=safety_state,

            recommendation=recommendation,

            ultrasonic_raw=dict(ultrasonic_data),

            camera_raw=dict(camera_data),

            lidar_raw=dict(lidar_data),

            vision_raw=dict(yolo_data),

            debug={
                "frame_count": self.frame_count,
                "dt": dt,
                "fps": 1.0 / dt,
            },
        )

        self.initialized = True

        return self.result

    # ========================================================
    # 超音波値抽出
    # ========================================================

    def _extract_distance(
        self,
        data: Dict[str, Any],
        possible_keys: Tuple[str, ...],
        default: float,
    ) -> float:
        """
        複数の候補キーから距離を取得する。
        """

        for key in possible_keys:

            if key not in data:
                continue

            value = data[key]

            if value is None:
                continue

            value = safe_float(
                value,
                default,
            )

            if value < 0:
                continue

            return value

        return default

    # ========================================================
    # 履歴管理
    # ========================================================

    def _append_history(
        self,
        history: List[float],
        value: float,
    ) -> None:
        """
        履歴に値を追加。
        """

        history.append(value)

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
        """
        移動平均による簡易平滑化。

        今後ここを
            median filter
            EMA
            Kalman filter
        に変更できる構造にしてある。
        """

        if not history:
            return current

        valid_values = [
            value
            for value in history
            if value is not None and math.isfinite(value)
        ]

        if not valid_values:
            return current

        return float(
            np.mean(valid_values)
        )

    # ========================================================
    # 超音波による障害物検出
    # ========================================================

    def _detect_obstacle_from_ultrasonic(
        self,
        left_distance: float,
        front_distance: float,
        right_distance: float,
    ) -> ObstacleState:
        """
        超音波だけを使った障害物判定。

        カメラやYOLOが無くても最低限の安全機能として動く。
        """

        obstacle = ObstacleState(
            source="ultrasonic"
        )

        left_close = (
            left_distance <= self.obstacle_detection_distance
        )

        front_close = (
            front_distance <= self.obstacle_detection_distance
        )

        right_close = (
            right_distance <= self.obstacle_detection_distance
        )

        obstacle.left = left_close
        obstacle.center = front_close
        obstacle.right = right_close

        obstacle.detected = (
            left_close
            or front_close
            or right_close
        )

        if obstacle.detected:

            close_distances = []

            if left_close:
                close_distances.append(left_distance)

            if front_close:
                close_distances.append(front_distance)

            if right_close:
                close_distances.append(right_distance)

            if close_distances:
                obstacle.distance = min(
                    close_distances
                )

        return obstacle

    # ========================================================
    # カメラ壁情報との統合
    # ========================================================

    def _fuse_camera_wall_information(
        self,
        wall_state: WallState,
        camera_data: Dict[str, Any],
    ) -> WallState:
        """
        カメラから得られた壁情報を超音波情報に追加する。

        camera_dataの想定例:

        {
            "left_wall": True,
            "right_wall": True,
            "front_wall": False,

            "left_confidence": 0.9,
            "right_confidence": 0.8,
            "front_confidence": 0.7,

            "left_angle": 0.1,
            "right_angle": -0.05
        }

        この段階では
        「カメラの判断を全面的に信用する」
        のではなく、
        confidenceに応じて統合する。
        """

        if not camera_data:
            return wall_state

        left_camera = camera_data.get(
            "left_wall"
        )

        right_camera = camera_data.get(
            "right_wall"
        )

        front_camera = camera_data.get(
            "front_wall"
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

        # カメラの信頼度が一定以上なら反映
        camera_threshold = float(
            get_config(
                "PERCEPTION_CAMERA_CONFIDENCE_THRESHOLD",
                0.60,
            )
        )

        if (
            safe_bool(left_camera)
            and left_confidence >= camera_threshold
        ):
            wall_state.left_detected = True

        if (
            safe_bool(right_camera)
            and right_confidence >= camera_threshold
        ):
            wall_state.right_detected = True

        if (
            safe_bool(front_camera)
            and front_confidence >= camera_threshold
        ):
            wall_state.front_detected = True

        # 壁角度
        if "left_angle" in camera_data:
            wall_state.left_angle = safe_float(
                camera_data["left_angle"],
                wall_state.left_angle,
            )

        if "right_angle" in camera_data:
            wall_state.right_angle = safe_float(
                camera_data["right_angle"],
                wall_state.right_angle,
            )

        wall_state.left_confidence = max(
            wall_state.left_confidence,
            left_confidence,
        )

        wall_state.right_confidence = max(
            wall_state.right_confidence,
            right_confidence,
        )

        wall_state.front_confidence = max(
            wall_state.front_confidence,
            front_confidence,
        )

        return wall_state

    # ========================================================
    # カメラ・YOLO障害物情報との統合
    # ========================================================

    def _fuse_camera_obstacle_information(
        self,
        obstacle_state: ObstacleState,
        camera_data: Dict[str, Any],
        yolo_data: Dict[str, Any],
    ) -> ObstacleState:
        """
        カメラやYOLOの障害物情報を統合する。
        """

        if camera_data:

            if safe_bool(
                camera_data.get(
                    "obstacle",
                    False,
                )
            ):
                obstacle_state.detected = True

                obstacle_state.source = (
                    "camera"
                )

                obstacle_state.confidence = max(
                    obstacle_state.confidence,
                    safe_float(
                        camera_data.get(
                            "obstacle_confidence",
                            0.0,
                        ),
                        0.0,
                    ),
                )

        if yolo_data:

            detections = yolo_data.get(
                "detections",
                []
            )

            if isinstance(
                detections,
                list,
            ):

                if detections:

                    obstacle_state.detected = True

                    obstacle_state.source = (
                        "yolo"
                    )

                    obstacle_state.detections = (
                        detections
                    )

                    max_confidence = 0.0

                    for detection in detections:

                        confidence = safe_float(
                            detection.get(
                                "confidence",
                                0.0,
                            ),
                            0.0,
                        )

                        max_confidence = max(
                            max_confidence,
                            confidence,
                        )

                    obstacle_state.confidence = max(
                        obstacle_state.confidence,
                        max_confidence,
                    )

        return obstacle_state
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
        """
        壁・障害物・カメラ・LiDAR情報から
        コースの状態を推定する。

        主な出力:

            ・左右どちらが開いているか
            ・進行方向
            ・コーナー方向
            ・行き止まり
            ・交差点
            ・コース中央からのずれ
            ・推定コース幅

        direction:
            -1.0 = 左
             0.0 = 直進
             1.0 = 右
        """

        course = CourseState()

        # ----------------------------------------------------
        # 距離を取得
        # ----------------------------------------------------

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        front = safe_float(
            wall_state.front_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        # ----------------------------------------------------
        # 左右の空き具合
        # ----------------------------------------------------

        course.left_free_space = self._calculate_free_space(
            left
        )

        course.right_free_space = self._calculate_free_space(
            right
        )

        # ----------------------------------------------------
        # 左右どちらが広いか
        # ----------------------------------------------------

        gap_margin = float(
            get_config(
                "PERCEPTION_GAP_MARGIN",
                100.0,
            )
        )

        if (
            course.left_free_space
            > course.right_free_space + gap_margin
        ):
            course.left_open = True
            course.right_open = False

        elif (
            course.right_free_space
            > course.left_free_space + gap_margin
        ):
            course.left_open = False
            course.right_open = True

        else:
            course.left_open = False
            course.right_open = False

        # ----------------------------------------------------
        # 基本進行方向
        # ----------------------------------------------------

        direction_gain = float(
            get_config(
                "PERCEPTION_DIRECTION_GAIN",
                1.0,
            )
        )

        free_space_total = (
            course.left_free_space
            + course.right_free_space
            + 1.0
        )

        direction_difference = (
            course.right_free_space
            - course.left_free_space
        )

        course.direction = clamp(
            direction_gain
            * direction_difference
            / free_space_total,
            -1.0,
            1.0,
        )

        # ----------------------------------------------------
        # カメラから方向情報が得られている場合
        # ----------------------------------------------------

        camera_direction = camera_data.get(
            "direction"
        )

        camera_confidence = clamp(
            safe_float(
                camera_data.get(
                    "direction_confidence",
                    camera_data.get(
                        "confidence",
                        0.0,
                    ),
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        camera_threshold = float(
            get_config(
                "PERCEPTION_CAMERA_CONFIDENCE_THRESHOLD",
                0.60,
            )
        )

        if (
            camera_direction is not None
            and camera_confidence >= camera_threshold
        ):

            camera_direction_value = self._convert_direction_value(
                camera_direction
            )

            camera_weight = clamp(
                float(
                    get_config(
                        "PERCEPTION_CAMERA_DIRECTION_WEIGHT",
                        0.50,
                    )
                ),
                0.0,
                1.0,
            )

            ultrasonic_weight = 1.0 - camera_weight

            course.direction = clamp(
                course.direction
                * ultrasonic_weight
                +
                camera_direction_value
                * camera_weight,
                -1.0,
                1.0,
            )

        # ----------------------------------------------------
        # 障害物による方向補正
        # ----------------------------------------------------

        if obstacle_state.detected:

            obstacle_avoidance_gain = float(
                get_config(
                    "PERCEPTION_OBSTACLE_DIRECTION_GAIN",
                    0.60,
                )
            )

            # 中央に障害物
            if obstacle_state.center:

                if (
                    right > left
                ):
                    course.direction += (
                        obstacle_avoidance_gain
                    )

                elif (
                    left > right
                ):
                    course.direction -= (
                        obstacle_avoidance_gain
                    )

            # 左側障害物
            if obstacle_state.left:

                course.direction += (
                    obstacle_avoidance_gain
                    * 0.5
                )

            # 右側障害物
            if obstacle_state.right:

                course.direction -= (
                    obstacle_avoidance_gain
                    * 0.5
                )

        course.direction = clamp(
            course.direction,
            -1.0,
            1.0,
        )

        # ----------------------------------------------------
        # コース中央からのずれ
        # ----------------------------------------------------

        course.center_offset = self._calculate_center_offset(
            left,
            right,
        )

        # ----------------------------------------------------
        # コース幅推定
        # ----------------------------------------------------

        course.estimated_width = self._estimate_course_width(
            left,
            right,
        )

        # ----------------------------------------------------
        # コーナー判定
        # ----------------------------------------------------

        corner_detected, corner_direction = (
            self._detect_corner(
                wall_state=wall_state,
                course_direction=course.direction,
                camera_data=camera_data,
            )
        )

        course.corner_detected = corner_detected
        course.corner_direction = corner_direction

        # ----------------------------------------------------
        # 行き止まり判定
        # ----------------------------------------------------

        course.dead_end = self._detect_dead_end(
            wall_state=wall_state,
            obstacle_state=obstacle_state,
            camera_data=camera_data,
        )

        # ----------------------------------------------------
        # 交差点判定
        # ----------------------------------------------------

        course.intersection = self._detect_intersection(
            wall_state=wall_state,
            camera_data=camera_data,
        )

        # ----------------------------------------------------
        # LiDAR情報を利用可能なら補正
        # ----------------------------------------------------

        course = self._fuse_lidar_course_information(
            course,
            lidar_data,
        )

        return course

    # ========================================================
    # 空き具合計算
    # ========================================================

    def _calculate_free_space(
        self,
        distance: float,
    ) -> float:
        """
        距離を「空き具合」に変換する。

        距離が大きいほど広いと判断する。

        0.0 ～ 1.0に正規化。
        """

        reference = float(
            get_config(
                "PERCEPTION_FREE_SPACE_REFERENCE",
                1500.0,
            )
        )

        if reference <= 0:
            reference = 1500.0

        return clamp(
            distance / reference,
            0.0,
            1.0,
        )

    # ========================================================
    # 方向値変換
    # ========================================================

    def _convert_direction_value(
        self,
        direction: Any,
    ) -> float:
        """
        カメラ等から来る方向情報を
        -1.0～1.0へ変換する。

        対応例:

            "left"
            "center"
            "right"

        または

            -1
             0
             1

        """

        if isinstance(
            direction,
            str,
        ):

            value = direction.lower().strip()

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
    # コース中央推定
    # ========================================================

    def _calculate_center_offset(
        self,
        left_distance: float,
        right_distance: float,
    ) -> float:
        """
        左右の壁距離から
        車体がコース中央からどれだけずれているかを推定する。

        -1.0 = 左寄り
         0.0 = 中央
        +1.0 = 右寄り
        """

        denominator = (
            left_distance
            + right_distance
            + 1.0
        )

        offset = (
            right_distance
            - left_distance
        ) / denominator

        return clamp(
            offset,
            -1.0,
            1.0,
        )

    # ========================================================
    # コース幅推定
    # ========================================================

    def _estimate_course_width(
        self,
        left_distance: float,
        right_distance: float,
    ) -> float:
        """
        左右距離からコース幅を推定する。

        実際の幾何学的なコース幅ではなく、
        「車体から見た左右の余裕量」
        として扱う。
        """

        width = (
            left_distance
            + right_distance
        )

        return max(
            0.0,
            width,
        )

    # ========================================================
    # コーナー判定
    # ========================================================

    def _detect_corner(
        self,
        wall_state: WallState,
        course_direction: float,
        camera_data: Dict[str, Any],
    ) -> Tuple[bool, Optional[str]]:
        """
        コーナーを推定する。
        """

        # ----------------------------------------------------
        # カメラが直接コーナーを認識した場合
        # ----------------------------------------------------

        camera_corner = camera_data.get(
            "corner"
        )

        camera_corner_confidence = clamp(
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

        corner_confidence_threshold = float(
            get_config(
                "PERCEPTION_CORNER_CONFIDENCE_THRESHOLD",
                0.60,
            )
        )

        if (
            camera_corner is not None
            and camera_corner_confidence
            >= corner_confidence_threshold
        ):

            if isinstance(
                camera_corner,
                str,
            ):

                value = camera_corner.lower()

                if value in (
                    "left",
                    "right",
                ):
                    return True, value

        # ----------------------------------------------------
        # 超音波から推定
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

        corner_front_threshold = float(
            get_config(
                "PERCEPTION_CORNER_FRONT_DISTANCE",
                self.wall_detection_distance,
            )
        )

        corner_side_difference = float(
            get_config(
                "PERCEPTION_CORNER_SIDE_DIFFERENCE",
                200.0,
            )
        )

        # 前方が近い
        if front <= corner_front_threshold:

            difference = (
                left - right
            )

            if abs(difference) >= corner_side_difference:

                if difference > 0:
                    return True, "left"

                return True, "right"

            # 左右差が小さい場合は
            # 現在の進行方向を参考にする
            if course_direction < -0.25:
                return True, "left"

            if course_direction > 0.25:
                return True, "right"

        return False, None

    # ========================================================
    # 行き止まり判定
    # ========================================================

    def _detect_dead_end(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        camera_data: Dict[str, Any],
    ) -> bool:
        """
        行き止まりを推定する。
        """

        # カメラが直接認識している場合
        if safe_bool(
            camera_data.get(
                "dead_end",
                False,
            )
        ):
            return True

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        front = safe_float(
            wall_state.front_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        dead_end_distance = float(
            get_config(
                "PERCEPTION_DEAD_END_DISTANCE",
                get_config(
                    "BACKWARD_RANGE",
                    300.0,
                ),
            )
        )

        left_blocked = (
            left <= dead_end_distance
        )

        front_blocked = (
            front <= dead_end_distance
        )

        right_blocked = (
            right <= dead_end_distance
        )

        # 三方向がほぼ塞がっている
        if (
            left_blocked
            and front_blocked
            and right_blocked
        ):
            return True

        # 正面＋障害物で脱出方向がない
        if (
            front_blocked
            and obstacle_state.detected
            and left_blocked
            and right_blocked
        ):
            return True

        return False

    # ========================================================
    # 交差点判定
    # ========================================================

    def _detect_intersection(
        self,
        wall_state: WallState,
        camera_data: Dict[str, Any],
    ) -> bool:
        """
        交差点を推定する。
        """

        # カメラが直接判断した場合
        if safe_bool(
            camera_data.get(
                "intersection",
                False,
            )
        ):
            return True

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        intersection_open_distance = float(
            get_config(
                "PERCEPTION_INTERSECTION_OPEN_DISTANCE",
                1200.0,
            )
        )

        # 左右両方が大きく開いている
        if (
            left >= intersection_open_distance
            and right >= intersection_open_distance
        ):
            return True

        return False

    # ========================================================
    # LiDARコース情報統合
    # ========================================================

    def _fuse_lidar_course_information(
        self,
        course: CourseState,
        lidar_data: Dict[str, Any],
    ) -> CourseState:
        """
        LiDARを使用する場合のコース情報統合。

        現時点では柔軟な入力形式に対応するため、
        共通フィールドがある場合だけ利用する。
        """

        if not lidar_data:
            return course

        lidar_direction = lidar_data.get(
            "direction"
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
            and lidar_confidence >= lidar_threshold
        ):

            lidar_value = self._convert_direction_value(
                lidar_direction
            )

            lidar_weight = clamp(
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
                * (1.0 - lidar_weight)
                +
                lidar_value
                * lidar_weight,
                -1.0,
                1.0,
            )

        # LiDARが左側の広い空間を報告
        if safe_bool(
            lidar_data.get(
                "left_open",
                False,
            )
        ):
            course.left_open = True

        # LiDARが右側の広い空間を報告
        if safe_bool(
            lidar_data.get(
                "right_open",
                False,
            )
        ):
            course.right_open = True

        # LiDARが交差点を報告
        if safe_bool(
            lidar_data.get(
                "intersection",
                False,
            )
        ):
            course.intersection = True

        # LiDARが行き止まりを報告
        if safe_bool(
            lidar_data.get(
                "dead_end",
                False,
            )
        ):
            course.dead_end = True

        return course
    # ========================================================
    # 安全状態解析
    # ========================================================

    def _analyze_safety(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        course_state: CourseState,
    ) -> SafetyState:
        """
        壁・障害物・コース状態から安全性を評価する。

        danger_level:
            0.0 = 安全
            1.0 = 非常に危険
        """

        safety = SafetyState()

        # ----------------------------------------------------
        # 距離取得
        # ----------------------------------------------------

        left = safe_float(
            wall_state.left_distance,
            self.default_distance,
        )

        front = safe_float(
            wall_state.front_distance,
            self.default_distance,
        )

        right = safe_float(
            wall_state.right_distance,
            self.default_distance,
        )

        # ----------------------------------------------------
        # 各方向の危険度
        # ----------------------------------------------------

        left_risk = self._distance_risk(
            left,
            self.obstacle_detection_distance,
            self.emergency_distance,
        )

        front_risk = self._distance_risk(
            front,
            self.obstacle_detection_distance,
            self.emergency_distance,
        )

        right_risk = self._distance_risk(
            right,
            self.obstacle_detection_distance,
            self.emergency_distance,
        )

        # ----------------------------------------------------
        # 最も高い危険度
        # ----------------------------------------------------

        safety.collision_risk = max(
            left_risk,
            front_risk,
            right_risk,
        )

        safety.danger_level = safety.collision_risk

        # ----------------------------------------------------
        # 障害物認識による危険度加算
        # ----------------------------------------------------

        if obstacle_state.detected:

            obstacle_weight = clamp(
                float(
                    get_config(
                        "PERCEPTION_OBSTACLE_RISK_WEIGHT",
                        0.30,
                    )
                ),
                0.0,
                1.0,
            )

            obstacle_risk = clamp(
                obstacle_state.confidence,
                0.0,
                1.0,
            )

            safety.danger_level = max(
                safety.danger_level,
                clamp(
                    safety.danger_level
                    + obstacle_risk
                    * obstacle_weight,
                    0.0,
                    1.0,
                ),
            )

        # ----------------------------------------------------
        # 緊急停止
        # ----------------------------------------------------

        if front <= self.emergency_distance:
            safety.emergency = True
            safety.must_stop = True

        # カメラが正面障害物を高信頼度で検出
        if (
            obstacle_state.center
            and obstacle_state.confidence
            >= float(
                get_config(
                    "PERCEPTION_EMERGENCY_CONFIDENCE",
                    0.80,
                )
            )
        ):
            safety.emergency = True
            safety.must_stop = True

        # ----------------------------------------------------
        # 減速判定
        # ----------------------------------------------------

        slow_down_risk = float(
            get_config(
                "PERCEPTION_SLOW_DOWN_RISK",
                0.35,
            )
        )

        safety.must_slow_down = (
            safety.danger_level
            >= slow_down_risk
        )

        # ----------------------------------------------------
        # 行き止まり
        # ----------------------------------------------------

        if course_state.dead_end:
            safety.must_slow_down = True

        # ----------------------------------------------------
        # 回避方向
        # ----------------------------------------------------

        safety.escape_direction = (
            self._select_escape_direction(
                left_distance=left,
                right_distance=right,
                obstacle_state=obstacle_state,
                course_state=course_state,
            )
        )

        return safety

    # ========================================================
    # 距離から危険度を計算
    # ========================================================

    def _distance_risk(
        self,
        distance: float,
        warning_distance: float,
        emergency_distance: float,
    ) -> float:
        """
        距離を0～1の危険度に変換する。

        warning_distance:
            この距離より遠ければ基本的に安全。

        emergency_distance:
            この距離以下なら危険度1.0。
        """

        if distance <= emergency_distance:
            return 1.0

        if distance >= warning_distance:
            return 0.0

        range_size = (
            warning_distance
            - emergency_distance
        )

        if range_size <= 0:
            return 1.0

        risk = (
            warning_distance
            - distance
        ) / range_size

        return clamp(
            risk,
            0.0,
            1.0,
        )

    # ========================================================
    # 回避方向選択
    # ========================================================

    def _select_escape_direction(
        self,
        left_distance: float,
        right_distance: float,
        obstacle_state: ObstacleState,
        course_state: CourseState,
    ) -> Optional[str]:
        """
        障害物を避ける方向を決める。

        Returns:
            "left"
            "right"
            None
        """

        # ----------------------------------------------------
        # 正面障害物
        # ----------------------------------------------------

        if obstacle_state.center:

            # 左右の空きを比較
            if (
                left_distance
                > right_distance
            ):
                return "left"

            if (
                right_distance
                > left_distance
            ):
                return "right"

        # ----------------------------------------------------
        # 左側障害物
        # ----------------------------------------------------

        if obstacle_state.left:
            return "right"

        # ----------------------------------------------------
        # 右側障害物
        # ----------------------------------------------------

        if obstacle_state.right:
            return "left"

        # ----------------------------------------------------
        # コースの空き具合
        # ----------------------------------------------------

        if course_state.left_open:
            return "left"

        if course_state.right_open:
            return "right"

        return None

    # ========================================================
    # 走行推奨値計算
    # ========================================================

    def _calculate_recommendation(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        course_state: CourseState,
        safety_state: SafetyState,
    ) -> DrivingRecommendation:
        """
        perceptionからplannerへ渡す推奨操舵・速度を計算する。

        重要:
            ここでは最終PWM値は決めない。

            -1.0 ～ +1.0 の方向
            0.0 ～ 1.0 の速度

        を返す。
        """

        recommendation = DrivingRecommendation()

        # ----------------------------------------------------
        # 緊急停止
        # ----------------------------------------------------

        if safety_state.must_stop:

            recommendation.steering = 0.0
            recommendation.throttle = 0.0
            recommendation.confidence = 1.0
            recommendation.reason = "emergency_stop"

            return recommendation

        # ----------------------------------------------------
        # 基本方向
        # ----------------------------------------------------

        steering = course_state.direction

        # ----------------------------------------------------
        # 障害物回避
        # ----------------------------------------------------

        if obstacle_state.detected:

            escape_direction = (
                safety_state.escape_direction
            )

            obstacle_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_OBSTACLE_STEERING_GAIN",
                        0.70,
                    )
                ),
                0.0,
                1.0,
            )

            if escape_direction == "left":

                steering = min(
                    steering,
                    -obstacle_gain,
                )

            elif escape_direction == "right":

                steering = max(
                    steering,
                    obstacle_gain,
                )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        if course_state.corner_detected:

            corner_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_CORNER_STEERING_GAIN",
                        0.80,
                    )
                ),
                0.0,
                1.0,
            )

            if course_state.corner_direction == "left":

                steering = min(
                    steering,
                    -corner_gain,
                )

            elif course_state.corner_direction == "right":

                steering = max(
                    steering,
                    corner_gain,
                )

        # ----------------------------------------------------
        # 行き止まり
        # ----------------------------------------------------

        if course_state.dead_end:

            escape_direction = (
                safety_state.escape_direction
            )

            dead_end_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_DEAD_END_STEERING_GAIN",
                        1.0,
                    )
                ),
                0.0,
                1.0,
            )

            if escape_direction == "left":

                steering = -dead_end_gain

            elif escape_direction == "right":

                steering = dead_end_gain

        # ----------------------------------------------------
        # ステアリング制限
        # ----------------------------------------------------

        steering = clamp(
            steering,
            -1.0,
            1.0,
        )

        recommendation.steering = steering

        # ----------------------------------------------------
        # 基本速度
        # ----------------------------------------------------

        base_throttle = float(
            get_config(
                "PERCEPTION_BASE_THROTTLE",
                0.70,
            )
        )

        throttle = clamp(
            base_throttle,
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # 前方距離による速度制御
        # ----------------------------------------------------

        front = safe_float(
            wall_state.front_distance,
            self.default_distance,
        )

        speed_reference = float(
            get_config(
                "PERCEPTION_SPEED_REFERENCE",
                1500.0,
            )
        )

        if speed_reference <= 0:
            speed_reference = 1500.0

        front_speed_factor = clamp(
            front / speed_reference,
            0.0,
            1.0,
        )

        front_weight = clamp(
            float(
                get_config(
                    "PERCEPTION_FRONT_SPEED_WEIGHT",
                    0.60,
                )
            ),
            0.0,
            1.0,
        )

        throttle *= (
            1.0
            -
            front_weight
            * (
                1.0
                -
                front_speed_factor
            )
        )

        # ----------------------------------------------------
        # ステアリングによる減速
        # ----------------------------------------------------

        steering_slowdown = float(
            get_config(
                "PERCEPTION_STEERING_SLOWDOWN",
                0.35,
            )
        )

        throttle *= (
            1.0
            -
            steering_slowdown
            * abs(steering)
        )

        # ----------------------------------------------------
        # 危険度による減速
        # ----------------------------------------------------

        danger_slowdown = float(
            get_config(
                "PERCEPTION_DANGER_SLOWDOWN",
                0.60,
            )
        )

        throttle *= (
            1.0
            -
            danger_slowdown
            * safety_state.danger_level
        )

        # ----------------------------------------------------
        # 明確な減速要求
        # ----------------------------------------------------

        if safety_state.must_slow_down:

            slow_down_factor = clamp(
                float(
                    get_config(
                        "PERCEPTION_SLOW_DOWN_FACTOR",
                        0.60,
                    )
                ),
                0.0,
                1.0,
            )

            throttle *= slow_down_factor

        # ----------------------------------------------------
        # 最小速度
        # ----------------------------------------------------

        minimum_throttle = clamp(
            float(
                get_config(
                    "PERCEPTION_MIN_THROTTLE",
                    0.20,
                )
            ),
            0.0,
            1.0,
        )

        # 緊急停止ではない場合のみ最低速度を保証
        if (
            throttle > 0.0
            and not safety_state.must_stop
        ):
            throttle = max(
                throttle,
                minimum_throttle,
            )

        # ----------------------------------------------------
        # 行き止まりなら速度を抑える
        # ----------------------------------------------------

        if course_state.dead_end:

            dead_end_throttle = clamp(
                float(
                    get_config(
                        "PERCEPTION_DEAD_END_THROTTLE",
                        0.20,
                    )
                ),
                0.0,
                1.0,
            )

            throttle = min(
                throttle,
                dead_end_throttle,
            )

        # ----------------------------------------------------
        # 最終速度制限
        # ----------------------------------------------------

        maximum_throttle = clamp(
            float(
                get_config(
                    "PERCEPTION_MAX_THROTTLE",
                    1.0,
                )
            ),
            0.0,
            1.0,
        )

        throttle = clamp(
            throttle,
            0.0,
            maximum_throttle,
        )

        recommendation.throttle = throttle

        # ----------------------------------------------------
        # 信頼度
        # ----------------------------------------------------

        confidence = self._calculate_recommendation_confidence(
            wall_state=wall_state,
            obstacle_state=obstacle_state,
            course_state=course_state,
            safety_state=safety_state,
        )

        recommendation.confidence = confidence

        # ----------------------------------------------------
        # 理由
        # ----------------------------------------------------

        recommendation.reason = (
            self._build_recommendation_reason(
                wall_state=wall_state,
                obstacle_state=obstacle_state,
                course_state=course_state,
                safety_state=safety_state,
            )
        )

        return recommendation

    # ========================================================
    # 推奨値の信頼度
    # ========================================================

    def _calculate_recommendation_confidence(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        course_state: CourseState,
        safety_state: SafetyState,
    ) -> float:
        """
        走行推奨値の信頼度を計算する。

        センサーが一致しているほど高くする。
        """

        confidence_values = []

        # 壁認識
        if wall_state.left_detected:
            confidence_values.append(
                wall_state.left_confidence
            )

        if wall_state.front_detected:
            confidence_values.append(
                wall_state.front_confidence
            )

        if wall_state.right_detected:
            confidence_values.append(
                wall_state.right_confidence
            )

        # 障害物
        if obstacle_state.detected:
            confidence_values.append(
                obstacle_state.confidence
            )

        # コーナーなど
        if course_state.corner_detected:
            confidence_values.append(
                0.80
            )

        # 安全性
        if safety_state.emergency:
            confidence_values.append(
                1.0
            )

        if not confidence_values:
            return 0.50

        confidence = float(
            np.mean(
                confidence_values
            )
        )

        # 危険状態では安全判断の信頼度を高くする
        if safety_state.must_stop:
            confidence = max(
                confidence,
                0.90,
            )

        return clamp(
            confidence,
            0.0,
            1.0,
        )

    # ========================================================
    # 推奨理由生成
    # ========================================================

    def _build_recommendation_reason(
        self,
        wall_state: WallState,
        obstacle_state: ObstacleState,
        course_state: CourseState,
        safety_state: SafetyState,
    ) -> str:
        """
        デバッグ・ログ用の理由を生成する。
        """

        if safety_state.must_stop:
            return "緊急停止"

        if course_state.dead_end:
            if safety_state.escape_direction:
                return (
                    f"行き止まり回避:"
                    f"{safety_state.escape_direction}"
                )

            return "行き止まり"

        if obstacle_state.detected:

            if safety_state.escape_direction:
                return (
                    f"障害物回避:"
                    f"{safety_state.escape_direction}"
                )

            return "障害物検出"

        if course_state.corner_detected:

            if course_state.corner_direction:
                return (
                    f"コーナー:"
                    f"{course_state.corner_direction}"
                )

            return "コーナー"

        if course_state.intersection:
            return "交差点"

        if course_state.left_open:
            return "左側が広い"

        if course_state.right_open:
            return "右側が広い"

        if (
            abs(
                course_state.center_offset
            )
            > 0.20
        ):
            if course_state.center_offset < 0:
                return "左寄り補正"

            return "右寄り補正"

        return "通常走行"
            # ========================================================
    # センサー信頼度統合
    # ========================================================

    def fuse_sensor_confidence(
        self,
        ultrasonic_confidence: float,
        camera_confidence: float = 0.0,
        lidar_confidence: float = 0.0,
        yolo_confidence: float = 0.0,
    ) -> float:
        """
        複数センサーの信頼度を統合する。

        それぞれのセンサーの信頼度を重み付きで統合する。

        今後センサーを増やしても、
        この関数に追加すれば対応できる。
        """

        ultrasonic_weight = clamp(
            float(
                get_config(
                    "PERCEPTION_ULTRASONIC_CONFIDENCE_WEIGHT",
                    0.45,
                )
            ),
            0.0,
            1.0,
        )

        camera_weight = clamp(
            float(
                get_config(
                    "PERCEPTION_CAMERA_CONFIDENCE_WEIGHT",
                    0.30,
                )
            ),
            0.0,
            1.0,
        )

        lidar_weight = clamp(
            float(
                get_config(
                    "PERCEPTION_LIDAR_CONFIDENCE_WEIGHT",
                    0.15,
                )
            ),
            0.0,
            1.0,
        )

        yolo_weight = clamp(
            float(
                get_config(
                    "PERCEPTION_YOLO_CONFIDENCE_WEIGHT",
                    0.10,
                )
            ),
            0.0,
            1.0,
        )

        total_weight = (
            ultrasonic_weight
            + camera_weight
            + lidar_weight
            + yolo_weight
        )

        if total_weight <= 0:
            return 0.0

        confidence = (
            safe_float(
                ultrasonic_confidence,
                0.0,
            )
            * ultrasonic_weight
            +
            safe_float(
                camera_confidence,
                0.0,
            )
            * camera_weight
            +
            safe_float(
                lidar_confidence,
                0.0,
            )
            * lidar_weight
            +
            safe_float(
                yolo_confidence,
                0.0,
            )
            * yolo_weight
        )

        return clamp(
            confidence / total_weight,
            0.0,
            1.0,
        )

    # ========================================================
    # センサー矛盾検出
    # ========================================================

    def detect_sensor_conflict(
        self,
        ultrasonic_distance: Optional[float],
        camera_distance: Optional[float],
        tolerance: float = 300.0,
    ) -> bool:
        """
        超音波とカメラの距離推定が大きく食い違っているか確認する。

        例:

            超音波: 300mm
            カメラ: 900mm

        のような場合は、
        カメラまたは超音波のどちらかに
        問題がある可能性がある。
        """

        if (
            ultrasonic_distance is None
            or camera_distance is None
        ):
            return False

        ultrasonic_value = safe_float(
            ultrasonic_distance,
            -1.0,
        )

        camera_value = safe_float(
            camera_distance,
            -1.0,
        )

        if (
            ultrasonic_value < 0
            or camera_value < 0
        ):
            return False

        return (
            abs(
                ultrasonic_value
                - camera_value
            )
            > tolerance
        )

    # ========================================================
    # 壁角度推定
    # ========================================================

    def estimate_wall_angle(
        self,
        front_side_distance: float,
        rear_side_distance: float,
        side: str,
    ) -> float:
        """
        前後2点の距離から壁の角度を推定する。

        Returns:
            rad（ラジアン）

        0:
            車体と壁がほぼ平行

        正:
            一方向へ開いている

        負:
            反対方向へ開いている
        """

        front_distance = safe_float(
            front_side_distance,
            self.default_distance,
        )

        rear_distance = safe_float(
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
                front_distance * sin45
                - rear_distance
            )

            dy = (
                front_distance * cos45
            )

        else:

            dx = (
                -front_distance * sin45
                + rear_distance
            )

            dy = (
                front_distance * cos45
            )

        if abs(dy) < 1e-6:
            return 0.0

        return math.atan2(
            dx,
            dy,
        )

    # ========================================================
    # 壁角度による操舵補正
    # ========================================================

    def wall_angle_steering_correction(
        self,
        wall_angle: float,
        side: str,
    ) -> float:
        """
        壁の角度から操舵補正値を作る。

        これは今後のwall_followやPIDと
        組み合わせることを想定している。
        """

        gain = float(
            get_config(
                "PERCEPTION_WALL_ANGLE_GAIN",
                1.0,
            )
        )

        correction = (
            wall_angle
            * gain
        )

        if side == "left":
            correction *= -1.0

        return clamp(
            correction,
            -1.0,
            1.0,
        )

    # ========================================================
    # 左右回避安全度
    # ========================================================

    def calculate_escape_score(
        self,
        left_distance: float,
        right_distance: float,
        left_wall: bool = False,
        right_wall: bool = False,
    ) -> Tuple[float, float]:
        """
        左右それぞれの「逃げやすさ」を計算する。

        Returns:
            (left_score, right_score)

        0.0:
            危険

        1.0:
            安全
        """

        left = safe_float(
            left_distance,
            0.0,
        )

        right = safe_float(
            right_distance,
            0.0,
        )

        reference = float(
            get_config(
                "PERCEPTION_ESCAPE_REFERENCE",
                1000.0,
            )
        )

        if reference <= 0:
            reference = 1000.0

        left_score = clamp(
            left / reference,
            0.0,
            1.0,
        )

        right_score = clamp(
            right / reference,
            0.0,
            1.0,
        )

        if left_wall:
            left_score *= 0.25

        if right_wall:
            right_score *= 0.25

        return (
            left_score,
            right_score,
        )

    # ========================================================
    # 最も安全な方向
    # ========================================================

    def get_safest_direction(
        self,
        left_distance: float,
        front_distance: float,
        right_distance: float,
    ) -> Optional[str]:
        """
        左右＋前の距離から最も安全な方向を返す。

        Returns:
            "left"
            "right"
            "center"
            None
        """

        left = safe_float(
            left_distance,
            0.0,
        )

        front = safe_float(
            front_distance,
            0.0,
        )

        right = safe_float(
            right_distance,
            0.0,
        )

        if (
            left <= 0
            and front <= 0
            and right <= 0
        ):
            return None

        if (
            front > left
            and front > right
        ):
            return "center"

        if left >= right:
            return "left"

        return "right"

    # ========================================================
    # センサー異常検出
    # ========================================================

    def validate_distance(
        self,
        value: Any,
        minimum: float = 0.0,
        maximum: float = 5000.0,
    ) -> bool:
        """
        距離データが正常範囲か確認する。
        """

        try:
            distance = float(value)
        except (
            TypeError,
            ValueError,
        ):
            return False

        if not math.isfinite(
            distance
        ):
            return False

        return (
            minimum
            <= distance
            <= maximum
        )

    # ========================================================
    # センサー値の異常補正
    # ========================================================

    def sanitize_distance(
        self,
        value: Any,
        default: Optional[float] = None,
    ) -> float:
        """
        異常な距離値を安全な値に変換する。
        """

        if default is None:
            default = self.default_distance

        if not self.validate_distance(
            value
        ):
            return default

        return float(
            value
        )

    # ========================================================
    # カメラ情報の標準化
    # ========================================================

    def normalize_camera_data(
        self,
        camera_data: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """
        カメラ側の実装が変わっても、
        perception.py内部では同じ形式で扱えるようにする。

        将来、

            OpenCV
            DonkeyCar
            ResNet
            MobileViT
            EdgeNeXt
            GRU
            TCN
            Causal CNN
            YOLO

        のどれを使っても、
        ここで共通形式に変換する。
        """

        if not camera_data:
            return {}

        normalized: Dict[str, Any] = {}

        # ----------------------------------------------------
        # 壁
        # ----------------------------------------------------

        normalized["left_wall"] = safe_bool(
            camera_data.get(
                "left_wall",
                False,
            )
        )

        normalized["right_wall"] = safe_bool(
            camera_data.get(
                "right_wall",
                False,
            )
        )

        normalized["front_wall"] = safe_bool(
            camera_data.get(
                "front_wall",
                False,
            )
        )

        # ----------------------------------------------------
        # 信頼度
        # ----------------------------------------------------

        normalized["left_confidence"] = clamp(
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

        normalized["right_confidence"] = clamp(
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

        normalized["front_confidence"] = clamp(
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

        # ----------------------------------------------------
        # 方向
        # ----------------------------------------------------

        normalized["direction"] = (
            self._convert_direction_value(
                camera_data.get(
                    "direction",
                    0.0,
                )
            )
        )

        normalized["direction_confidence"] = clamp(
            safe_float(
                camera_data.get(
                    "direction_confidence",
                    camera_data.get(
                        "confidence",
                        0.0,
                    ),
                ),
                0.0,
            ),
            0.0,
            1.0,
        )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        corner = camera_data.get(
            "corner"
        )

        if (
            isinstance(
                corner,
                str,
            )
            and corner.lower()
            in (
                "left",
                "right",
            )
        ):
            normalized["corner"] = (
                corner.lower()
            )
        else:
            normalized["corner"] = None

        normalized["corner_confidence"] = clamp(
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

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        normalized["obstacle"] = safe_bool(
            camera_data.get(
                "obstacle",
                False,
            )
        )

        normalized["obstacle_confidence"] = clamp(
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

        normalized["obstacle_type"] = (
            camera_data.get(
                "obstacle_type"
            )
        )

        # ----------------------------------------------------
        # 行き止まり・交差点
        # ----------------------------------------------------

        normalized["dead_end"] = safe_bool(
            camera_data.get(
                "dead_end",
                False,
            )
        )

        normalized["intersection"] = safe_bool(
            camera_data.get(
                "intersection",
                False,
            )
        )

        # ----------------------------------------------------
        # 壁角度
        # ----------------------------------------------------

        normalized["left_angle"] = safe_float(
            camera_data.get(
                "left_angle",
                0.0,
            ),
            0.0,
        )

        normalized["right_angle"] = safe_float(
            camera_data.get(
                "right_angle",
                0.0,
            ),
            0.0,
        )

        # ----------------------------------------------------
        # 推定距離
        # ----------------------------------------------------

        normalized["left_distance"] = (
            self.sanitize_distance(
                camera_data.get(
                    "left_distance"
                ),
                self.default_distance,
            )
            if camera_data.get(
                "left_distance"
            ) is not None
            else None
        )

        normalized["front_distance"] = (
            self.sanitize_distance(
                camera_data.get(
                    "front_distance"
                ),
                self.default_distance,
            )
            if camera_data.get(
                "front_distance"
            ) is not None
            else None
        )

        normalized["right_distance"] = (
            self.sanitize_distance(
                camera_data.get(
                    "right_distance"
                ),
                self.default_distance,
            )
            if camera_data.get(
                "right_distance"
            ) is not None
            else None
        )

        return normalized

    # ========================================================
    # YOLO結果の標準化
    # ========================================================

    def normalize_yolo_data(
        self,
        yolo_data: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """
        YOLO等の物体検出結果を
        perception.py共通形式へ変換する。
        """

        if not yolo_data:
            return {
                "detections": []
            }

        detections = yolo_data.get(
            "detections",
            []
        )

        if not isinstance(
            detections,
            list,
        ):
            detections = []

        normalized_detections = []

        for detection in detections:

            if not isinstance(
                detection,
                dict,
            ):
                continue

            normalized_detections.append(
                {
                    "class_name":
                        detection.get(
                            "class_name",
                            detection.get(
                                "class",
                                "unknown",
                            ),
                        ),

                    "confidence":
                        clamp(
                            safe_float(
                                detection.get(
                                    "confidence",
                                    0.0,
                                ),
                                0.0,
                            ),
                            0.0,
                            1.0,
                        ),

                    "x_center":
                        safe_float(
                            detection.get(
                                "x_center",
                                0.5,
                            ),
                            0.5,
                        ),

                    "y_center":
                        safe_float(
                            detection.get(
                                "y_center",
                                0.5,
                            ),
                            0.5,
                        ),

                    "width":
                        safe_float(
                            detection.get(
                                "width",
                                0.0,
                            ),
                            0.0,
                        ),

                    "height":
                        safe_float(
                            detection.get(
                                "height",
                                0.0,
                            ),
                            0.0,
                        ),
                }
            )

        return {
            "detections":
                normalized_detections
        }

    # ========================================================
    # 認識情報の更新（標準化版）
    # ========================================================

    def update_normalized(
        self,
        ultrasonic_data: Optional[Dict[str, Any]] = None,
        camera_data: Optional[Dict[str, Any]] = None,
        lidar_data: Optional[Dict[str, Any]] = None,
        yolo_data: Optional[Dict[str, Any]] = None,
    ) -> PerceptionResult:
        """
        update()の前段階で
        カメラ・YOLOの入力を標準化する。

        今後、認識エンジンを変更しても
        Perception本体を変更しなくて済むようにする。
        """

        normalized_camera = (
            self.normalize_camera_data(
                camera_data
            )
        )

        normalized_yolo = (
            self.normalize_yolo_data(
                yolo_data
            )
        )

        return self.update(
            ultrasonic_data=ultrasonic_data,
            camera_data=normalized_camera,
            lidar_data=lidar_data,
            yolo_data=normalized_yolo,
        )

    # ========================================================
    # デバッグ情報取得
    # ========================================================

    def get_debug_summary(self) -> Dict[str, Any]:
        """
        現在の認識状態を辞書で返す。

        ログ・CSV・ブラウザモニターなどに利用できる。
        """

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

            "obstacle":
                result.obstacle.detected,

            "obstacle_type":
                result.obstacle.object_type,

            "corner":
                result.course.corner_direction,

            "dead_end":
                result.course.dead_end,

            "intersection":
                result.course.intersection,

            "direction":
                result.course.direction,

            "center_offset":
                result.course.center_offset,

            "course_width":
                result.course.estimated_width,

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

            "recommended_steering":
                result.recommendation.steering,

            "recommended_throttle":
                result.recommendation.throttle,

            "recommendation_confidence":
                result.recommendation.confidence,

            "recommendation_reason":
                result.recommendation.reason,
        }

    # ========================================================
    # デバッグ表示
    # ========================================================

    def log_debug_summary(
        self,
        force: bool = False,
    ) -> None:
        """
        認識状態をログへ出力する。

        PERCEPTION_DEBUGがFalseなら通常は出力しない。
        """

        enabled = safe_bool(
            get_config(
                "PERCEPTION_DEBUG",
                False,
            )
        )

        if (
            not enabled
            and not force
        ):
            return

        summary = (
            self.get_debug_summary()
        )

        logger.info(
            "PERCEPTION | "
            "L=%.0f "
            "F=%.0f "
            "R=%.0f | "
            "dir=%.2f "
            "steer=%.2f "
            "throttle=%.2f | "
            "danger=%.2f | "
            "reason=%s",
            safe_float(
                summary["left_distance"],
                0.0,
            ),
            safe_float(
                summary["front_distance"],
                0.0,
            ),
            safe_float(
                summary["right_distance"],
                0.0,
            ),
            safe_float(
                summary["direction"],
                0.0,
            ),
            safe_float(
                summary["recommended_steering"],
                0.0,
            ),
            safe_float(
                summary["recommended_throttle"],
                0.0,
            ),
            safe_float(
                summary["danger_level"],
                0.0,
            ),
            summary["recommendation_reason"],
        )

    # ========================================================
    # 結果を辞書へ変換
    # ========================================================

    def to_dict(
        self,
        result: Optional[PerceptionResult] = None,
    ) -> Dict[str, Any]:
        """
        PerceptionResultをJSON/CSV等で扱いやすい
        辞書形式へ変換する。
        """

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

                "corner_detected":
                    result.course.corner_detected,

                "corner_direction":
                    result.course.corner_direction,

                "dead_end":
                    result.course.dead_end,

                "intersection":
                    result.course.intersection,

                "center_offset":
                    result.course.center_offset,

                "estimated_width":
                    result.course.estimated_width,
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
    # 状態リセット
    # ========================================================

    def reset(self) -> None:
        """
        perceptionの状態を初期化する。
        """

        self.result = PerceptionResult()

        self.last_update_time = (
            time.perf_counter()
        )

        self.frame_count = 0

        self.initialized = False

        self._left_distance_history.clear()
        self._front_distance_history.clear()
        self._right_distance_history.clear()

        logger.info(
            "Perception state reset"
        )
            # ========================================================
    # 高度センサーフュージョン
    # ========================================================

    def fuse_direction(
        self,
        ultrasonic_direction: float,
        camera_direction: Optional[float] = None,
        lidar_direction: Optional[float] = None,
        ai_direction: Optional[float] = None,
        camera_confidence: float = 0.0,
        lidar_confidence: float = 0.0,
        ai_confidence: float = 0.0,
    ) -> float:
        """
        複数の認識系から得られた進行方向を統合する。

        direction:
            -1.0 = 左
             0.0 = 直進
             1.0 = 右

        AIやカメラが無い場合でも、
        超音波だけで動作できる。

        これにより、

            超音波だけ
            カメラだけ
            超音波 + カメラ
            超音波 + カメラ + AI
            超音波 + カメラ + LiDAR

        を同じインターフェースで扱える。
        """

        # ----------------------------------------------------
        # ベース
        # ----------------------------------------------------

        values = [
            (
                clamp(
                    safe_float(
                        ultrasonic_direction,
                        0.0,
                    ),
                    -1.0,
                    1.0,
                ),
                float(
                    get_config(
                        "PERCEPTION_ULTRASONIC_DIRECTION_WEIGHT",
                        0.50,
                    )
                ),
            )
        ]

        # ----------------------------------------------------
        # カメラ
        # ----------------------------------------------------

        if camera_direction is not None:
            values.append(
                (
                    clamp(
                        safe_float(
                            camera_direction,
                            0.0,
                        ),
                        -1.0,
                        1.0,
                    ),
                    max(
                        0.0,
                        camera_confidence,
                    )
                    *
                    float(
                        get_config(
                            "PERCEPTION_CAMERA_DIRECTION_WEIGHT",
                            0.30,
                        )
                    ),
                )
            )

        # ----------------------------------------------------
        # LiDAR
        # ----------------------------------------------------

        if lidar_direction is not None:
            values.append(
                (
                    clamp(
                        safe_float(
                            lidar_direction,
                            0.0,
                        ),
                        -1.0,
                        1.0,
                    ),
                    max(
                        0.0,
                        lidar_confidence,
                    )
                    *
                    float(
                        get_config(
                            "PERCEPTION_LIDAR_DIRECTION_WEIGHT",
                            0.15,
                        )
                    ),
                )
            )

        # ----------------------------------------------------
        # AI
        # ----------------------------------------------------

        if ai_direction is not None:
            values.append(
                (
                    clamp(
                        safe_float(
                            ai_direction,
                            0.0,
                        ),
                        -1.0,
                        1.0,
                    ),
                    max(
                        0.0,
                        ai_confidence,
                    )
                    *
                    float(
                        get_config(
                            "PERCEPTION_AI_DIRECTION_WEIGHT",
                            0.25,
                        )
                    ),
                )
            )

        # ----------------------------------------------------
        # 重み付き平均
        # ----------------------------------------------------

        total_weight = sum(
            weight
            for _, weight in values
            if weight > 0
        )

        if total_weight <= 0:
            return 0.0

        direction = sum(
            value * weight
            for value, weight in values
            if weight > 0
        ) / total_weight

        return clamp(
            direction,
            -1.0,
            1.0,
        )

    # ========================================================
    # 時系列安定化
    # ========================================================

    def stabilize_direction(
        self,
        direction: float,
    ) -> float:
        """
        進行方向を時間方向に平滑化する。

        カメラAIなどのフレームごとの
        小さな揺れを抑える。

        例:

            -0.20
            -0.15
            -0.30
            -0.18

        のような値を安定させる。
        """

        if not hasattr(
            self,
            "_direction_history",
        ):
            self._direction_history = []

        history_size = int(
            get_config(
                "PERCEPTION_DIRECTION_HISTORY_SIZE",
                5,
            )
        )

        history_size = max(
            1,
            history_size,
        )

        value = clamp(
            safe_float(
                direction,
                0.0,
            ),
            -1.0,
            1.0,
        )

        self._direction_history.append(
            value
        )

        if len(
            self._direction_history
        ) > history_size:
            del self._direction_history[0]

        # 最新値を強くする指数型に近い簡易重み
        weighted_sum = 0.0
        weight_sum = 0.0

        for index, item in enumerate(
            self._direction_history
        ):
            weight = float(
                index + 1
            )

            weighted_sum += (
                item * weight
            )

            weight_sum += weight

        if weight_sum <= 0:
            return value

        return clamp(
            weighted_sum / weight_sum,
            -1.0,
            1.0,
        )

    # ========================================================
    # 速度の上限計算
    # ========================================================

    def calculate_speed_limit(
        self,
        front_distance: float,
        steering: float,
        danger_level: float,
        corner_detected: bool = False,
        obstacle_detected: bool = False,
    ) -> float:
        """
        周囲の状況から速度上限を計算する。

        戻り値:
            0.0 ～ 1.0
        """

        max_speed = clamp(
            float(
                get_config(
                    "PERCEPTION_MAX_THROTTLE",
                    1.0,
                )
            ),
            0.0,
            1.0,
        )

        min_speed = clamp(
            float(
                get_config(
                    "PERCEPTION_MIN_THROTTLE",
                    0.20,
                )
            ),
            0.0,
            1.0,
        )

        speed = max_speed

        # ----------------------------------------------------
        # 前方距離
        # ----------------------------------------------------

        safe_front_reference = float(
            get_config(
                "PERCEPTION_SPEED_REFERENCE",
                1500.0,
            )
        )

        if safe_front_reference <= 0:
            safe_front_reference = 1500.0

        front_factor = clamp(
            safe_float(
                front_distance,
                self.default_distance,
            )
            / safe_front_reference,
            0.0,
            1.0,
        )

        distance_weight = clamp(
            float(
                get_config(
                    "PERCEPTION_FRONT_SPEED_WEIGHT",
                    0.60,
                )
            ),
            0.0,
            1.0,
        )

        speed *= (
            (1.0 - distance_weight)
            +
            distance_weight
            * front_factor
        )

        # ----------------------------------------------------
        # 操舵量
        # ----------------------------------------------------

        steering_slowdown = clamp(
            float(
                get_config(
                    "PERCEPTION_STEERING_SLOWDOWN",
                    0.35,
                )
            ),
            0.0,
            1.0,
        )

        speed *= (
            1.0
            -
            steering_slowdown
            * abs(steering)
        )

        # ----------------------------------------------------
        # 危険度
        # ----------------------------------------------------

        danger_slowdown = clamp(
            float(
                get_config(
                    "PERCEPTION_DANGER_SLOWDOWN",
                    0.60,
                )
            ),
            0.0,
            1.0,
        )

        speed *= (
            1.0
            -
            danger_slowdown
            * clamp(
                danger_level,
                0.0,
                1.0,
            )
        )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        if corner_detected:

            corner_factor = clamp(
                float(
                    get_config(
                        "PERCEPTION_CORNER_SPEED_FACTOR",
                        0.55,
                    )
                ),
                0.0,
                1.0,
            )

            speed *= corner_factor

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        if obstacle_detected:

            obstacle_factor = clamp(
                float(
                    get_config(
                        "PERCEPTION_OBSTACLE_SPEED_FACTOR",
                        0.50,
                    )
                ),
                0.0,
                1.0,
            )

            speed *= obstacle_factor

        return clamp(
            speed,
            min_speed,
            max_speed,
        )

    # ========================================================
    # モード用データ取得
    # ========================================================

    def get_mode_inputs(
        self,
        result: Optional[PerceptionResult] = None,
    ) -> Dict[str, Any]:
        """
        各走行モードが利用しやすい形で
        perception情報を返す。

        今後、

            wall_follow
            wall_follow_pid
            center_follow_pid
            gap_follow
            racer
            right_left_3
            AI
            camera

        など、どのモードからでも共通して利用できる。
        """

        if result is None:
            result = self.result

        return {
            # ------------------------------------------------
            # 距離
            # ------------------------------------------------

            "left_distance":
                result.wall.left_distance,

            "front_distance":
                result.wall.front_distance,

            "right_distance":
                result.wall.right_distance,

            # ------------------------------------------------
            # 壁
            # ------------------------------------------------

            "left_wall":
                result.wall.left_detected,

            "front_wall":
                result.wall.front_detected,

            "right_wall":
                result.wall.right_detected,

            # ------------------------------------------------
            # 壁角度
            # ------------------------------------------------

            "left_wall_angle":
                result.wall.left_angle,

            "right_wall_angle":
                result.wall.right_angle,

            # ------------------------------------------------
            # コース
            # ------------------------------------------------

            "direction":
                result.course.direction,

            "center_offset":
                result.course.center_offset,

            "course_width":
                result.course.estimated_width,

            "left_free_space":
                result.course.left_free_space,

            "right_free_space":
                result.course.right_free_space,

            "left_open":
                result.course.left_open,

            "right_open":
                result.course.right_open,

            "corner":
                result.course.corner_direction,

            "corner_detected":
                result.course.corner_detected,

            "intersection":
                result.course.intersection,

            "dead_end":
                result.course.dead_end,

            # ------------------------------------------------
            # 障害物
            # ------------------------------------------------

            "obstacle":
                result.obstacle.detected,

            "obstacle_left":
                result.obstacle.left,

            "obstacle_center":
                result.obstacle.center,

            "obstacle_right":
                result.obstacle.right,

            "obstacle_distance":
                result.obstacle.distance,

            "obstacle_type":
                result.obstacle.object_type,

            "obstacle_confidence":
                result.obstacle.confidence,

            # ------------------------------------------------
            # 安全
            # ------------------------------------------------

            "danger_level":
                result.safety.danger_level,

            "collision_risk":
                result.safety.collision_risk,

            "emergency":
                result.safety.emergency,

            "must_stop":
                result.safety.must_stop,

            "must_slow_down":
                result.safety.must_slow_down,

            "escape_direction":
                result.safety.escape_direction,

            # ------------------------------------------------
            # 推奨値
            # ------------------------------------------------

            "recommended_steering":
                result.recommendation.steering,

            "recommended_throttle":
                result.recommendation.throttle,

            "confidence":
                result.recommendation.confidence,

            "reason":
                result.recommendation.reason,
        }

    # ========================================================
    # 走行モード向けステアリング補正
    # ========================================================

    def apply_mode_correction(
        self,
        base_steering: float,
        mode: str,
        result: Optional[PerceptionResult] = None,
    ) -> float:
        """
        既存の走行方式に
        perceptionの情報を追加するための共通関数。

        例えば:

            wall_follow
                +
            camera correction

            racer
                +
            obstacle avoidance

            center_follow_pid
                +
            camera corner detection

        のような構成に使う。
        """

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

        mode_name = str(
            mode
            if mode is not None
            else ""
        ).lower()

        # ----------------------------------------------------
        # 障害物
        # ----------------------------------------------------

        if result.obstacle.detected:

            obstacle_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_MODE_OBSTACLE_CORRECTION_GAIN",
                        0.35,
                    )
                ),
                0.0,
                1.0,
            )

            escape = (
                result.safety.escape_direction
            )

            if escape == "left":
                steering -= obstacle_gain

            elif escape == "right":
                steering += obstacle_gain

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        if result.course.corner_detected:

            corner_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_MODE_CORNER_CORRECTION_GAIN",
                        0.25,
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
        # カメラ方向
        # ----------------------------------------------------

        camera_direction = (
            safe_float(
                result.camera_raw.get(
                    "direction",
                    0.0,
                ),
                0.0,
            )
        )

        camera_confidence = clamp(
            safe_float(
                result.camera_raw.get(
                    "direction_confidence",
                    result.camera_raw.get(
                        "confidence",
                        0.0,
                    ),
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

        if camera_confidence > 0.0:

            steering += (
                camera_direction
                * camera_confidence
                * camera_gain
            )

        # ----------------------------------------------------
        # モード固有の補正
        # ----------------------------------------------------

        if (
            mode_name
            in (
                "racer",
                "gap_follow",
                "follow_the_gap",
            )
        ):

            high_speed_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_HIGH_SPEED_CORRECTION_GAIN",
                        0.15,
                    )
                ),
                0.0,
                1.0,
            )

            steering += (
                result.course.direction
                * high_speed_gain
            )

        elif (
            mode_name
            in (
                "wall_follow",
                "wall_follow_pid",
            )
        ):

            wall_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_WALL_CORRECTION_GAIN",
                        0.15,
                    )
                ),
                0.0,
                1.0,
            )

            steering += (
                result.course.center_offset
                * wall_gain
            )

        elif mode_name == "center_follow_pid":

            center_gain = clamp(
                float(
                    get_config(
                        "PERCEPTION_CENTER_CORRECTION_GAIN",
                        0.20,
                    )
                ),
                0.0,
                1.0,
            )

            steering += (
                result.course.center_offset
                * center_gain
            )

        # ----------------------------------------------------
        # 緊急停止時には操舵を中央へ
        # ----------------------------------------------------

        if result.safety.must_stop:
            steering = 0.0

        return clamp(
            steering,
            -1.0,
            1.0,
        )

    # ========================================================
    # 走行モード向け速度補正
    # ========================================================

    def apply_speed_correction(
        self,
        base_throttle: float,
        mode: str,
        result: Optional[PerceptionResult] = None,
    ) -> float:
        """
        既存の走行方式の速度に
        perceptionの安全情報を追加する。
        """

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

        mode_name = str(
            mode
            if mode is not None
            else ""
        ).lower()

        # ----------------------------------------------------
        # 緊急停止
        # ----------------------------------------------------

        if result.safety.must_stop:
            return 0.0

        # ----------------------------------------------------
        # 危険度
        # ----------------------------------------------------

        danger_gain = clamp(
            float(
                get_config(
                    "PERCEPTION_MODE_DANGER_SPEED_GAIN",
                    0.50,
                )
            ),
            0.0,
            1.0,
        )

        throttle *= (
            1.0
            -
            result.safety.danger_level
            * danger_gain
        )

        # ----------------------------------------------------
        # コーナー
        # ----------------------------------------------------

        if result.course.corner_detected:

            corner_factor = clamp(
                float(
                    get_config(
                        "PERCEPTION_MODE_CORNER_SPEED_FACTOR",
                        0.70,
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
                        0.55,
                    )
                ),
                0.0,
                1.0,
            )

            throttle *= (
                obstacle_factor
            )

        # ----------------------------------------------------
        # 高速モードは前方が開いているときだけ許可
        # ----------------------------------------------------

        if (
            mode_name
            in (
                "racer",
                "gap_follow",
                "follow_the_gap",
            )
        ):

            front = safe_float(
                result.wall.front_distance,
                self.default_distance,
            )

            high_speed_distance = float(
                get_config(
                    "PERCEPTION_HIGH_SPEED_DISTANCE",
                    1000.0,
                )
            )

            if (
                front
                < high_speed_distance
            ):
                throttle *= 0.70

        # ----------------------------------------------------
        # 行き止まり
        # ----------------------------------------------------

        if result.course.dead_end:

            throttle = min(
                throttle,
                float(
                    get_config(
                        "PERCEPTION_DEAD_END_THROTTLE",
                        0.20,
                    )
                ),
            )

        # ----------------------------------------------------
        # 最終値
        # ----------------------------------------------------

        return clamp(
            throttle,
            0.0,
            1.0,
        )

    # ========================================================
    # 既存Plannerとの接続用
    # ========================================================

    def get_planner_input(
        self,
        mode: Optional[str] = None,
        base_steering: Optional[float] = None,
        base_throttle: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        planner.pyから呼びやすい
        共通インターフェース。

        例:

            planner_input = perception.get_planner_input(
                mode="wall_follow",
                base_steering=steering,
                base_throttle=throttle,
            )

        """

        result = self.result

        # ----------------------------------------------------
        # ベース値
        # ----------------------------------------------------

        if base_steering is None:
            steering = (
                result.recommendation.steering
            )
        else:
            steering = (
                safe_float(
                    base_steering,
                    result.recommendation.steering,
                )
            )

        if base_throttle is None:
            throttle = (
                result.recommendation.throttle
            )
        else:
            throttle = (
                safe_float(
                    base_throttle,
                    result.recommendation.throttle,
                )
            )

        # ----------------------------------------------------
        # モード補正
        # ----------------------------------------------------

        if mode is not None:

            steering = (
                self.apply_mode_correction(
                    base_steering=steering,
                    mode=mode,
                    result=result,
                )
            )

            throttle = (
                self.apply_speed_correction(
                    base_throttle=throttle,
                    mode=mode,
                    result=result,
                )
            )

        # ----------------------------------------------------
        # 最終結果
        # ----------------------------------------------------

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

            "danger_level":
                result.safety.danger_level,

            "obstacle":
                result.obstacle.detected,

            "corner":
                result.course.corner_direction,

            "dead_end":
                result.course.dead_end,

            "escape_direction":
                result.safety.escape_direction,

            "confidence":
                result.recommendation.confidence,

            "reason":
                result.recommendation.reason,
        }

    # ========================================================
    # 完全リセット
    # ========================================================

    def full_reset(self) -> None:
        """
        perception内部のすべての履歴をリセットする。
        """

        self.reset()

        if hasattr(
            self,
            "_direction_history",
        ):
            self._direction_history.clear()

        logger.info(
            "Perception full reset"
        )

    # ========================================================
    # 簡易ステータス
    # ========================================================

    def get_status(self) -> Dict[str, Any]:
        """
        システムの状態を返す。
        """

        return {
            "initialized":
                self.initialized,

            "frame_count":
                self.frame_count,

            "history_size":
                self.history_size,

            "last_update":
                self.last_update_time,

            "camera_enabled":
                bool(
                    self.result.camera_raw
                ),

            "lidar_enabled":
                bool(
                    self.result.lidar_raw
                ),

            "yolo_enabled":
                bool(
                    self.result.vision_raw
                ),

            "emergency":
                self.result.safety.emergency,

            "danger_level":
                self.result.safety.danger_level,
        }


# ============================================================
# 外部から扱いやすくするエイリアス
# ============================================================

PerceptionEngine = Perception


# ============================================================
# 簡易テスト
# ============================================================

def create_test_perception() -> Perception:
    """
    実車センサーなしでPerceptionだけをテストするための関数。
    """

    return Perception()


def test_perception() -> None:
    """
    perception.py単体テスト。

    実際のカメラや超音波を使用せず、
    ダミーデータで認識処理が動作するか確認する。

    実行:
        python perception.py
    """

    print("=" * 60)
    print("Perception test")
    print("=" * 60)

    perception = Perception()

    ultrasonic_data = {
        "FrLH": 450,
        "FrFR": 900,
        "FrRH": 300,
    }

    camera_data = {
        "left_wall": True,
        "right_wall": True,
        "front_wall": False,

        "left_confidence": 0.90,
        "right_confidence": 0.85,
        "front_confidence": 0.20,

        "direction": "right",
        "direction_confidence": 0.80,

        "corner": None,
        "corner_confidence": 0.0,

        "obstacle": False,
        "obstacle_confidence": 0.0,
    }

    yolo_data = {
        "detections": []
    }

    result = perception.update_normalized(
        ultrasonic_data=ultrasonic_data,
        camera_data=camera_data,
        lidar_data={},
        yolo_data=yolo_data,
    )

    print()
    print("=== WALL ===")
    print(
        f"LEFT  : {result.wall.left_distance}"
    )
    print(
        f"FRONT : {result.wall.front_distance}"
    )
    print(
        f"RIGHT : {result.wall.right_distance}"
    )

    print()
    print("=== COURSE ===")
    print(
        f"DIRECTION : {result.course.direction:.3f}"
    )
    print(
        f"CENTER    : {result.course.center_offset:.3f}"
    )
    print(
        f"WIDTH     : {result.course.estimated_width:.1f}"
    )
    print(
        f"CORNER    : {result.course.corner_direction}"
    )
    print(
        f"DEAD END  : {result.course.dead_end}"
    )
    print(
        f"INTERSECT : {result.course.intersection}"
    )

    print()
    print("=== SAFETY ===")
    print(
        f"DANGER    : {result.safety.danger_level:.3f}"
    )
    print(
        f"COLLISION : {result.safety.collision_risk:.3f}"
    )
    print(
        f"STOP      : {result.safety.must_stop}"
    )
    print(
        f"SLOW      : {result.safety.must_slow_down}"
    )
    print(
        f"ESCAPE    : {result.safety.escape_direction}"
    )

    print()
    print("=== RECOMMENDATION ===")
    print(
        f"STEERING  : {result.recommendation.steering:.3f}"
    )
    print(
        f"THROTTLE  : {result.recommendation.throttle:.3f}"
    )
    print(
        f"CONFIDENCE: {result.recommendation.confidence:.3f}"
    )
    print(
        f"REASON    : {result.recommendation.reason}"
    )

    print()
    print("=== PLANNER INPUT ===")

    planner_input = (
        perception.get_planner_input(
            mode="wall_follow",
            base_steering=0.0,
            base_throttle=0.70,
        )
    )

    for key, value in planner_input.items():
        print(
            f"{key}: {value}"
        )

    print()
    print("=== STATUS ===")

    status = perception.get_status()

    for key, value in status.items():
        print(
            f"{key}: {value}"
        )

    print()
    print("=" * 60)
    print("Perception test completed")
    print("=" * 60)


# ============================================================
# メイン
# ============================================================

if __name__ == "__main__":
    test_perception()
