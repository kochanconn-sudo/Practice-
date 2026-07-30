# coding:utf-8

"""
camera_wall_detector.py

カメラ画像からコース・壁・コーナー・障害物候補を推定するモジュール。

目的
----
camera.py:
    カメラ画像を取得する

camera_wall_detector.py:
    カメラ画像を解析する

perception.py:
    超音波 + カメラ + 将来のLiDAR/YOLOなどを統合する

planner.py:
    実際の走行方法を決める

という役割分担にする。

Raspberry Pi 5では現在のcamera.pyがOpenCVを無効化しているため、
本モジュールは基本的にNumPyだけで画像解析を行う。

想定入力
--------
RGB / BGR の numpy.ndarray

想定出力
--------
{
    "left_wall": bool,
    "right_wall": bool,
    "front_wall": bool,

    "left_confidence": float,
    "right_confidence": float,
    "front_confidence": float,

    "direction": float,
    "direction_confidence": float,

    "center_offset": float,

    "corner": None / "left" / "right",
    "corner_confidence": float,

    "obstacle": bool,
    "obstacle_confidence": float,
    "obstacle_type": str | None,

    "left_distance": float | None,
    "right_distance": float | None,
    "front_distance": float | None,

    "debug": {...}
}

注意
----
この段階では「画像から正確なmm距離」を直接求めるものではない。

カメラから得るのは主に、

    ・左右どちらに壁があるか
    ・壁の位置
    ・コース中央
    ・コーナー方向
    ・障害物候補

であり、

    ・正確な距離

は超音波を優先する。

つまり、

    カメラ = 形状・方向・見通し
    超音波 = 実距離・安全確認

という役割分担を前提にしている。
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Dict, Optional, Tuple
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
        return float(value)
    except (TypeError, ValueError):
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
# 結果データ
# ============================================================

@dataclass
class CameraWallResult:
    """
    カメラ画像1枚の解析結果。
    """

    # 壁
    left_wall: bool = False
    right_wall: bool = False
    front_wall: bool = False

    # 壁信頼度
    left_confidence: float = 0.0
    right_confidence: float = 0.0
    front_confidence: float = 0.0

    # 方向
    direction: float = 0.0
    direction_confidence: float = 0.0

    # 中央ずれ
    center_offset: float = 0.0

    # コーナー
    corner: Optional[str] = None
    corner_confidence: float = 0.0

    # 障害物
    obstacle: bool = False
    obstacle_confidence: float = 0.0
    obstacle_type: Optional[str] = None

    # カメラからの相対的距離推定
    # ※mmではなく0.0～1.0の近さ指標
    left_distance: Optional[float] = None
    right_distance: Optional[float] = None
    front_distance: Optional[float] = None

    # デバッグ
    debug: Dict[str, Any] = None

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)

        if data["debug"] is None:
            data["debug"] = {}

        return data


# ============================================================
# メインクラス
# ============================================================

class CameraWallDetector:

    def __init__(self):
        # ----------------------------------------------------
        # 画像サイズ
        # ----------------------------------------------------

        self.image_w = int(
            get_config(
                "IMAGE_W",
                224,
            )
        )

        self.image_h = int(
            get_config(
                "IMAGE_H",
                126,
            )
        )

        # ----------------------------------------------------
        # ROI
        #
        # 画像全体を見るのではなく、
        # 走行に必要な下側中心部分を重点的に見る。
        # ----------------------------------------------------

        self.roi_top_ratio = clamp(
            safe_float(
                get_config(
                    "CAMERA_WALL_ROI_TOP_RATIO",
                    0.30,
                ),
                0.30,
            ),
            0.0,
            0.95,
        )

        self.roi_bottom_ratio = clamp(
            safe_float(
                get_config(
                    "CAMERA_WALL_ROI_BOTTOM_RATIO",
                    0.98,
                ),
                0.98,
            ),
            0.05,
            1.0,
        )

        self.roi_left_ratio = clamp(
            safe_float(
                get_config(
                    "CAMERA_WALL_ROI_LEFT_RATIO",
                    0.05,
                ),
                0.05,
            ),
            0.0,
            0.5,
        )

        self.roi_right_ratio = clamp(
            safe_float(
                get_config(
                    "CAMERA_WALL_ROI_RIGHT_RATIO",
                    0.95,
                ),
                0.95,
            ),
            0.5,
            1.0,
        )

        # ----------------------------------------------------
        # 壁判定
        # ----------------------------------------------------

        self.edge_threshold = float(
            get_config(
                "CAMERA_WALL_EDGE_THRESHOLD",
                22.0,
            )
        )

        self.left_wall_score_threshold = float(
            get_config(
                "CAMERA_LEFT_WALL_SCORE_THRESHOLD",
                0.10,
            )
        )

        self.right_wall_score_threshold = float(
            get_config(
                "CAMERA_RIGHT_WALL_SCORE_THRESHOLD",
                0.10,
            )
        )

        self.front_wall_darkness_threshold = float(
            get_config(
                "CAMERA_FRONT_WALL_DARKNESS_THRESHOLD",
                0.30,
            )
        )

        # ----------------------------------------------------
        # コーナー判定
        # ----------------------------------------------------

        self.corner_direction_threshold = float(
            get_config(
                "CAMERA_CORNER_DIRECTION_THRESHOLD",
                0.18,
            )
        )

        self.corner_confidence_threshold = float(
            get_config(
                "CAMERA_CORNER_CONFIDENCE_THRESHOLD",
                0.45,
            )
        )

        # ----------------------------------------------------
        # 障害物判定
        # ----------------------------------------------------

        self.obstacle_edge_density_threshold = float(
            get_config(
                "CAMERA_OBSTACLE_EDGE_DENSITY_THRESHOLD",
                0.16,
            )
        )

        self.obstacle_bottom_mass_threshold = float(
            get_config(
                "CAMERA_OBSTACLE_BOTTOM_MASS_THRESHOLD",
                0.20,
            )
        )

        # ----------------------------------------------------
        # 平滑化
        # ----------------------------------------------------

        self.history_size = max(
            1,
            int(
                get_config(
                    "CAMERA_DETECTOR_HISTORY_SIZE",
                    5,
                )
            ),
        )

        self.direction_history = []
        self.center_history = []

        # ----------------------------------------------------
        # 状態
        # ----------------------------------------------------

        self.frame_count = 0
        self.last_result = CameraWallResult()

        logger.info(
            "CameraWallDetector 起動 "
            "(%dx%d)",
            self.image_w,
            self.image_h,
        )

    # ========================================================
    # メイン
    # ========================================================

    def analyze(
        self,
        frame: Optional[np.ndarray],
    ) -> Dict[str, Any]:
        """
        カメラ画像を解析する。

        Parameters
        ----------
        frame:
            RGB / BGR の numpy.ndarray

        Returns
        -------
        dict
        """

        self.frame_count += 1

        if frame is None:
            return self._empty_result(
                reason="frame_none"
            )

        if not isinstance(
            frame,
            np.ndarray,
        ):
            return self._empty_result(
                reason="invalid_frame_type"
            )

        if frame.ndim != 3:
            return self._empty_result(
                reason="invalid_frame_dimension"
            )

        if frame.shape[2] < 3:
            return self._empty_result(
                reason="invalid_channels"
            )

        try:
            # ------------------------------------------------
            # 入力画像を正規化
            # ------------------------------------------------

            image = self._prepare_image(
                frame
            )

            # ------------------------------------------------
            # ROI
            # ------------------------------------------------

            roi = self._extract_roi(
                image
            )

            # ------------------------------------------------
            # グレースケール
            # ------------------------------------------------

            gray = self._to_gray(
                roi
            )

            # ------------------------------------------------
            # 明るさ
            # ------------------------------------------------

            brightness = self._calculate_brightness(
                gray
            )

            # ------------------------------------------------
            # エッジ
            # ------------------------------------------------

            edge_map = self._calculate_edges(
                gray
            )

            # ------------------------------------------------
            # 左右壁
            # ------------------------------------------------

            left_score = self._calculate_left_wall_score(
                gray,
                edge_map,
            )

            right_score = self._calculate_right_wall_score(
                gray,
                edge_map,
            )

            # ------------------------------------------------
            # 前方
            # ------------------------------------------------

            front_score = self._calculate_front_wall_score(
                gray,
                edge_map,
            )

            # ------------------------------------------------
            # 左右壁判定
            # ------------------------------------------------

            left_wall = (
                left_score
                >= self.left_wall_score_threshold
            )

            right_wall = (
                right_score
                >= self.right_wall_score_threshold
            )

            front_wall = (
                front_score
                >= self.front_wall_darkness_threshold
            )

            # ------------------------------------------------
            # コース中央
            # ------------------------------------------------

            center_offset = (
                self._estimate_center_offset(
                    edge_map
                )
            )

            center_offset = (
                self._stabilize_center(
                    center_offset
                )
            )

            # ------------------------------------------------
            # 進行方向
            # ------------------------------------------------

            direction = (
                self._estimate_direction(
                    center_offset=center_offset,
                    left_score=left_score,
                    right_score=right_score,
                    edge_map=edge_map,
                )
            )

            direction = (
                self._stabilize_direction(
                    direction
                )
            )

            # ------------------------------------------------
            # コーナー
            # ------------------------------------------------

            corner, corner_confidence = (
                self._detect_corner(
                    edge_map=edge_map,
                    gray=gray,
                    direction=direction,
                    left_score=left_score,
                    right_score=right_score,
                )
            )

            # ------------------------------------------------
            # 障害物
            # ------------------------------------------------

            obstacle, obstacle_confidence, obstacle_type = (
                self._detect_obstacle(
                    gray=gray,
                    edge_map=edge_map,
                )
            )

            # ------------------------------------------------
            # カメラ相対距離
            # ------------------------------------------------

            left_relative_distance = (
                self._calculate_relative_distance(
                    left_score
                )
            )

            right_relative_distance = (
                self._calculate_relative_distance(
                    right_score
                )
            )

            front_relative_distance = (
                self._calculate_front_relative_distance(
                    front_score
                )
            )

            # ------------------------------------------------
            # 信頼度
            # ------------------------------------------------

            left_confidence = self._wall_confidence(
                left_score
            )

            right_confidence = self._wall_confidence(
                right_score
            )

            front_confidence = self._wall_confidence(
                front_score
            )

            direction_confidence = (
                self._calculate_direction_confidence(
                    center_offset=center_offset,
                    left_score=left_score,
                    right_score=right_score,
                )
            )

            # ------------------------------------------------
            # 結果
            # ------------------------------------------------

            debug = {
                "frame_count": self.frame_count,

                "image_shape":
                    tuple(image.shape),

                "roi_shape":
                    tuple(roi.shape),

                "brightness":
                    brightness,

                "left_score":
                    left_score,

                "right_score":
                    right_score,

                "front_score":
                    front_score,

                "edge_density":
                    float(
                        np.mean(
                            edge_map
                        )
                    ),

                "left_confidence":
                    left_confidence,

                "right_confidence":
                    right_confidence,

                "front_confidence":
                    front_confidence,

                "direction_confidence":
                    direction_confidence,
            }

            result = CameraWallResult(
                left_wall=left_wall,
                right_wall=right_wall,
                front_wall=front_wall,

                left_confidence=left_confidence,
                right_confidence=right_confidence,
                front_confidence=front_confidence,

                direction=direction,
                direction_confidence=direction_confidence,

                center_offset=center_offset,

                corner=corner,
                corner_confidence=corner_confidence,

                obstacle=obstacle,
                obstacle_confidence=obstacle_confidence,
                obstacle_type=obstacle_type,

                left_distance=left_relative_distance,
                right_distance=right_relative_distance,
                front_distance=front_relative_distance,

                debug=debug,
            )

            self.last_result = result

            return result.to_dict()

        except Exception as exc:

            logger.exception(
                "CameraWallDetector analyze error: %s",
                exc,
            )

            return self._empty_result(
                reason="analysis_exception"
            )

    # ========================================================
    # 画像準備
    # ========================================================

    def _prepare_image(
        self,
        frame: np.ndarray,
    ) -> np.ndarray:
        """
        入力画像をuint8 RGB相当にする。

        色そのものよりも明るさ・エッジを重視するため、
        BGR/RGBの違いによる影響を小さくする。
        """

        image = np.asarray(
            frame
        )

        if image.dtype != np.uint8:

            image = np.clip(
                image,
                0,
                255,
            ).astype(
                np.uint8
            )

        return image

    # ========================================================
    # ROI
    # ========================================================

    def _extract_roi(
        self,
        image: np.ndarray,
    ) -> np.ndarray:
        height, width = image.shape[:2]

        top = int(
            height
            * self.roi_top_ratio
        )

        bottom = int(
            height
            * self.roi_bottom_ratio
        )

        left = int(
            width
            * self.roi_left_ratio
        )

        right = int(
            width
            * self.roi_right_ratio
        )

        top = max(
            0,
            min(
                height - 1,
                top,
            )
        )

        bottom = max(
            top + 1,
            min(
                height,
                bottom,
            )
        )

        left = max(
            0,
            min(
                width - 1,
                left,
            )
        )

        right = max(
            left + 1,
            min(
                width,
                right,
            )
        )

        return image[
            top:bottom,
            left:right,
        ]

    # ========================================================
    # グレースケール
    # ========================================================

    def _to_gray(
        self,
        image: np.ndarray,
    ) -> np.ndarray:
        """
        RGB/BGRのどちらでも大きく破綻しにくい簡易グレースケール。
        """

        if image.ndim == 2:
            gray = image.astype(
                np.float32
            )

            return gray

        # 単純平均
        gray = np.mean(
            image[:, :, :3],
            axis=2,
        )

        return gray.astype(
            np.float32
        )

    # ========================================================
    # 明るさ
    # ========================================================

    def _calculate_brightness(
        self,
        gray: np.ndarray,
    ) -> float:
        return float(
            np.mean(gray) / 255.0
        )

    # ========================================================
    # エッジ検出
    # ========================================================

    def _calculate_edges(
        self,
        gray: np.ndarray,
    ) -> np.ndarray:
        """
        NumPyだけで簡易的な勾配ベースエッジを作る。

        Sobel等を使わず、
        左右差・上下差からエッジの強さを求める。
        """

        gray_norm = (
            gray / 255.0
        )

        dx = np.zeros_like(
            gray_norm,
            dtype=np.float32,
        )

        dy = np.zeros_like(
            gray_norm,
            dtype=np.float32,
        )

        # X方向
        dx[:, 1:-1] = (
            gray_norm[:, 2:]
            -
            gray_norm[:, :-2]
        ) * 0.5

        # Y方向
        dy[1:-1, :] = (
            gray_norm[2:, :]
            -
            gray_norm[:-2, :]
        ) * 0.5

        magnitude = np.sqrt(
            dx * dx
            +
            dy * dy
        )

        threshold = (
            self.edge_threshold
            / 255.0
        )

        edges = (
            magnitude
            >= threshold
        )

        return edges.astype(
            np.float32
        )

    # ========================================================
    # 左壁スコア
    # ========================================================

    def _calculate_left_wall_score(
        self,
        gray: np.ndarray,
        edge_map: np.ndarray,
    ) -> float:
        """
        ROI左側に壁らしい構造があるか推定する。
        """

        height, width = gray.shape

        # 左25%
        x1 = 0
        x2 = max(
            1,
            int(
                width * 0.35
            )
        )

        region_gray = gray[
            :,
            x1:x2,
        ]

        region_edges = edge_map[
            :,
            x1:x2,
        ]

        if region_gray.size == 0:
            return 0.0

        edge_density = float(
            np.mean(
                region_edges
            )
        )

        # 下側ほど重要
        lower = region_edges[
            int(
                region_edges.shape[0]
                * 0.45
            ):
        ]

        lower_density = float(
            np.mean(lower)
        ) if lower.size else 0.0

        vertical_structure = (
            self._vertical_structure_score(
                region_gray
            )
        )

        score = (
            edge_density * 0.35
            +
            lower_density * 0.40
            +
            vertical_structure * 0.25
        )

        return clamp(
            score,
            0.0,
            1.0,
        )

    # ========================================================
    # 右壁スコア
    # ========================================================

    def _calculate_right_wall_score(
        self,
        gray: np.ndarray,
        edge_map: np.ndarray,
    ) -> float:

        height, width = gray.shape

        x1 = min(
            width - 1,
            int(
                width * 0.65
            )
        )

        x2 = width

        region_gray = gray[
            :,
            x1:x2,
        ]

        region_edges = edge_map[
            :,
            x1:x2,
        ]

        if region_gray.size == 0:
            return 0.0

        edge_density = float(
            np.mean(
                region_edges
            )
        )

        lower = region_edges[
            int(
                region_edges.shape[0]
                * 0.45
            ):
        ]

        lower_density = float(
            np.mean(lower)
        ) if lower.size else 0.0

        vertical_structure = (
            self._vertical_structure_score(
                region_gray
            )
        )

        score = (
            edge_density * 0.35
            +
            lower_density * 0.40
            +
            vertical_structure * 0.25
        )

        return clamp(
            score,
            0.0,
            1.0,
        )

    # ========================================================
    # 前方壁スコア
    # ========================================================

    def _calculate_front_wall_score(
        self,
        gray: np.ndarray,
        edge_map: np.ndarray,
    ) -> float:
        """
        前方中央に大きな障害・壁がある可能性を推定。
        """

        height, width = gray.shape

        x1 = int(
            width * 0.28
        )

        x2 = int(
            width * 0.72
        )

        y1 = int(
            height * 0.05
        )

        y2 = int(
            height * 0.70
        )

        region_gray = gray[
            y1:y2,
            x1:x2,
        ]

        region_edges = edge_map[
            y1:y2,
            x1:x2,
        ]

        if region_gray.size == 0:
            return 0.0

        darkness = 1.0 - (
            float(
                np.mean(
                    region_gray
                )
            )
            / 255.0
        )

        edge_density = float(
            np.mean(
                region_edges
            )
        )

        # 前方は「暗い」だけで壁と決めない。
        # エッジと組み合わせる。
        score = (
            darkness * 0.45
            +
            edge_density * 0.55
        )

        return clamp(
            score,
            0.0,
            1.0,
        )

    # ========================================================
    # 縦方向構造
    # ========================================================

    def _vertical_structure_score(
        self,
        gray: np.ndarray,
    ) -> float:
        """
        縦方向に連続した構造があるかを推定。
        """

        if gray.size == 0:
            return 0.0

        gradient_x = np.abs(
            gray[:, 2:]
            -
            gray[:, :-2]
        )

        if gradient_x.size == 0:
            return 0.0

        threshold = float(
            self.edge_threshold
        )

        vertical_edges = (
            gradient_x
            >= threshold
        )

        return clamp(
            float(
                np.mean(
                    vertical_edges
                )
            ) * 2.0,
            0.0,
            1.0,
        )

    # ========================================================
    # コース中央
    # ========================================================

    def _estimate_center_offset(
        self,
        edge_map: np.ndarray,
    ) -> float:
        """
        下側ROIのエッジ分布からコース中央を推定。

        -1 = 左寄り
         0 = 中央
        +1 = 右寄り
        """

        height, width = edge_map.shape

        y1 = int(
            height * 0.45
        )

        lower = edge_map[
            y1:,
            :
        ]

        if lower.size == 0:
            return 0.0

        column_score = np.mean(
            lower,
            axis=0,
        )

        # 全体
        columns = np.arange(
            width,
            dtype=np.float32,
        )

        total = float(
            np.sum(
                column_score
            )
        )

        if total <= 1e-6:
            return 0.0

        weighted_x = float(
            np.sum(
                columns
                * column_score
            )
            / total
        )

        normalized_x = (
            weighted_x
            /
            max(
                1.0,
                width - 1,
            )
        )

        # 0.0 ～ 1.0 → -1.0 ～ +1.0
        offset = (
            normalized_x
            - 0.5
        ) * 2.0

        return clamp(
            offset,
            -1.0,
            1.0,
        )

    # ========================================================
    # 進行方向
    # ========================================================

    def _estimate_direction(
        self,
        center_offset: float,
        left_score: float,
        right_score: float,
        edge_map: np.ndarray,
    ) -> float:
        """
        複数の情報から進行方向を推定。

        -1 = 左
         0 = 直進
        +1 = 右
        """

        steering_gain = float(
            get_config(
                "CAMERA_DIRECTION_GAIN",
                1.0,
            )
        )

        gap_gain = float(
            get_config(
                "CAMERA_DIRECTION_GAP_GAIN",
                0.5,
            )
        )

        # 左右壁の差
        gap_difference = (
            right_score
            -
            left_score
        )

        direction = (
            -center_offset
            * steering_gain
            +
            gap_difference
            * gap_gain
        )

        # 画像全体の傾向
        lane_direction = (
            self._estimate_lane_direction(
                edge_map
            )
        )

        lane_weight = float(
            get_config(
                "CAMERA_LANE_DIRECTION_WEIGHT",
                0.35,
            )
        )

        direction = (
            direction
            * (1.0 - lane_weight)
            +
            lane_direction
            * lane_weight
        )

        return clamp(
            direction,
            -1.0,
            1.0,
        )

    # ========================================================
    # レーン方向
    # ========================================================

    def _estimate_lane_direction(
        self,
        edge_map: np.ndarray,
    ) -> float:
        """
        画像上下でのエッジ位置変化から
        道の向きを粗く推定する。
        """

        height, width = edge_map.shape

        if height < 4:
            return 0.0

        upper = edge_map[
            :int(
                height * 0.35
            ),
            :
        ]

        lower = edge_map[
            int(
                height * 0.65
            ):,
            :
        ]

        if (
            upper.size == 0
            or lower.size == 0
        ):
            return 0.0

        upper_profile = np.mean(
            upper,
            axis=0,
        )

        lower_profile = np.mean(
            lower,
            axis=0,
        )

        upper_total = float(
            np.sum(
                upper_profile
            )
        )

        lower_total = float(
            np.sum(
                lower_profile
            )
        )

        if (
            upper_total <= 1e-6
            or lower_total <= 1e-6
        ):
            return 0.0

        x = np.arange(
            width,
            dtype=np.float32,
        )

        upper_center = (
            np.sum(
                x
                * upper_profile
            )
            / upper_total
        )

        lower_center = (
            np.sum(
                x
                * lower_profile
            )
            / lower_total
        )

        delta = (
            upper_center
            -
            lower_center
        )

        normalized = (
            delta
            /
            max(
                1.0,
                width,
            )
        )

        return clamp(
            normalized * 2.0,
            -1.0,
            1.0,
        )

    # ========================================================
    # コーナー検出
    # ========================================================

    def _detect_corner(
        self,
        edge_map: np.ndarray,
        gray: np.ndarray,
        direction: float,
        left_score: float,
        right_score: float,
    ) -> Tuple[
        Optional[str],
        float,
    ]:

        height, width = edge_map.shape

        if height < 10:
            return None, 0.0

        # 上側と下側でエッジ分布を見る
        upper = edge_map[
            :int(
                height * 0.40
            ),
            :
        ]

        lower = edge_map[
            int(
                height * 0.55
            ):,
            :
        ]

        upper_profile = np.mean(
            upper,
            axis=0,
        )

        lower_profile = np.mean(
            lower,
            axis=0,
        )

        def center_of_profile(
            profile: np.ndarray,
        ) -> float:

            total = float(
                np.sum(
                    profile
                )
            )

            if total <= 1e-6:
                return 0.5

            x = np.arange(
                len(profile),
                dtype=np.float32,
            )

            center = (
                np.sum(
                    x
                    * profile
                )
                / total
            )

            return (
                center
                /
                max(
                    1.0,
                    len(profile) - 1,
                )
            )

        upper_center = (
            center_of_profile(
                upper_profile
            )
        )

        lower_center = (
            center_of_profile(
                lower_profile
            )
        )

        shift = (
            upper_center
            -
            lower_center
        )

        # 左右の壁差
        side_difference = (
            right_score
            -
            left_score
        )

        # 総合
        corner_signal = (
            shift * 0.65
            +
            side_difference * 0.35
        )

        corner_signal = clamp(
            corner_signal,
            -1.0,
            1.0,
        )

        magnitude = abs(
            corner_signal
        )

        if (
            magnitude
            <
            self.corner_direction_threshold
        ):
            return None, 0.0

        confidence = clamp(
            magnitude
            /
            max(
                1e-6,
                self.corner_direction_threshold * 3.0,
            ),
            0.0,
            1.0,
        )

        confidence = (
            confidence * 0.7
            +
            abs(direction) * 0.3
        )

        if (
            confidence
            <
            self.corner_confidence_threshold
        ):
            return None, confidence

        if corner_signal < 0:
            return "left", confidence

        return "right", confidence

    # ========================================================
    # 障害物検出
    # ========================================================

    def _detect_obstacle(
        self,
        gray: np.ndarray,
        edge_map: np.ndarray,
    ) -> Tuple[
        bool,
        float,
        Optional[str],
    ]:
        """
        前方下側の局所的な構造から
        障害物候補を探す。

        これはYOLOではない。

        「何という物体か」は分からないが、
        「前方に何かある可能性」を検出する。
        """

        height, width = gray.shape

        x1 = int(
            width * 0.25
        )

        x2 = int(
            width * 0.75
        )

        y1 = int(
            height * 0.50
        )

        y2 = height

        region_gray = gray[
            y1:y2,
            x1:x2,
        ]

        region_edges = edge_map[
            y1:y2,
            x1:x2,
        ]

        if (
            region_gray.size == 0
            or region_edges.size == 0
        ):
            return False, 0.0, None

        edge_density = float(
            np.mean(
                region_edges
            )
        )

        # 下側に物体があると、
        # 明暗差が局所的に大きくなることが多い
        gray_std = float(
            np.std(
                region_gray
            )
        )

        std_score = clamp(
            gray_std / 80.0,
            0.0,
            1.0,
        )

        density_score = clamp(
            edge_density / max(
                1e-6,
                self.obstacle_edge_density_threshold,
            ),
            0.0,
            1.0,
        )

        confidence = (
            density_score * 0.65
            +
            std_score * 0.35
        )

        detected = (
            edge_density
            >= self.obstacle_edge_density_threshold
            and
            std_score
            >= self.obstacle_bottom_mass_threshold
        )

        obstacle_type = None

        if detected:
            obstacle_type = "unknown"

        return (
            detected,
            clamp(
                confidence,
                0.0,
                1.0,
            ),
            obstacle_type,
        )

    # ========================================================
    # カメラ相対距離
    # ========================================================

    def _calculate_relative_distance(
        self,
        wall_score: float,
    ) -> float:

        # 壁らしさが高いほど
        # 「近い」とみなす
        return clamp(
            1.0 - wall_score,
            0.0,
            1.0,
        )

    # ========================================================
    # 前方相対距離
    # ========================================================

    def _calculate_front_relative_distance(
        self,
        front_score: float,
    ) -> float:

        return clamp(
            1.0 - front_score,
            0.0,
            1.0,
        )

    # ========================================================
    # 壁信頼度
    # ========================================================

    def _wall_confidence(
        self,
        score: float,
    ) -> float:

        return clamp(
            score * 2.0,
            0.0,
            1.0,
        )

    # ========================================================
    # 方向信頼度
    # ========================================================

    def _calculate_direction_confidence(
        self,
        center_offset: float,
        left_score: float,
        right_score: float,
    ) -> float:

        # 中央付近なら直進判断の信頼度が高い
        center_confidence = (
            1.0
            -
            min(
                1.0,
                abs(
                    center_offset
                ),
            )
        )

        # 左右差が明確なら方向判断しやすい
        gap_difference = abs(
            right_score
            -
            left_score
        )

        gap_confidence = clamp(
            gap_difference * 2.0,
            0.0,
            1.0,
        )

        confidence = (
            center_confidence * 0.4
            +
            gap_confidence * 0.6
        )

        return clamp(
            confidence,
            0.0,
            1.0,
        )

    # ========================================================
    # 方向平滑化
    # ========================================================

    def _stabilize_direction(
        self,
        direction: float,
    ) -> float:

        self.direction_history.append(
            direction
        )

        if (
            len(
                self.direction_history
            )
            >
            self.history_size
        ):
            del self.direction_history[0]

        if not self.direction_history:
            return direction

        weights = np.arange(
            1,
            len(
                self.direction_history
            ) + 1,
            dtype=np.float32,
        )

        values = np.asarray(
            self.direction_history,
            dtype=np.float32,
        )

        result = float(
            np.sum(
                values * weights
            )
            /
            np.sum(
                weights
            )
        )

        return clamp(
            result,
            -1.0,
            1.0,
        )

    # ========================================================
    # 中央平滑化
    # ========================================================

    def _stabilize_center(
        self,
        center: float,
    ) -> float:

        self.center_history.append(
            center
        )

        if (
            len(
                self.center_history
            )
            >
            self.history_size
        ):
            del self.center_history[0]

        if not self.center_history:
            return center

        return clamp(
            float(
                np.mean(
                    self.center_history
                )
            ),
            -1.0,
            1.0,
        )

    # ========================================================
    # 空結果
    # ========================================================

    def _empty_result(
        self,
        reason: str,
    ) -> Dict[str, Any]:

        result = CameraWallResult(
            debug={
                "frame_count":
                    self.frame_count,

                "error":
                    reason,
            }
        )

        self.last_result = result

        return result.to_dict()

    # ========================================================
    # 最新結果
    # ========================================================

    def get_last_result(
        self,
    ) -> Dict[str, Any]:

        return self.last_result.to_dict()

    # ========================================================
    # 状態リセット
    # ========================================================

    def reset(
        self,
    ) -> None:

        self.direction_history.clear()
        self.center_history.clear()

        self.frame_count = 0

        self.last_result = (
            CameraWallResult()
        )

        logger.info(
            "CameraWallDetector reset"
        )


# ============================================================
# エイリアス
# ============================================================

WallDetector = CameraWallDetector


# ============================================================
# 単体テスト
# ============================================================

def create_test_image(
    width: int = 224,
    height: int = 126,
) -> np.ndarray:
    """
    カメラなしでテストするための
    簡易画像を生成する。

    左右に壁らしい縦構造を作り、
    中央を明るくする。
    """

    image = np.zeros(
        (
            height,
            width,
            3,
        ),
        dtype=np.uint8,
    )

    # 背景
    image[:, :, :] = 120

    # 左壁
    left_x1 = int(
        width * 0.05
    )

    left_x2 = int(
        width * 0.22
    )

    image[
        :,
        left_x1:left_x2,
        :
    ] = 55

    # 右壁
    right_x1 = int(
        width * 0.78
    )

    right_x2 = int(
        width * 0.95
    )

    image[
        :,
        right_x1:right_x2,
        :
    ] = 55

    # 中央
    center_x1 = int(
        width * 0.30
    )

    center_x2 = int(
        width * 0.70
    )

    image[
        :,
        center_x1:center_x2,
        :
    ] = 180

    # 前方に横壁らしい構造
    wall_y = int(
        height * 0.25
    )

    image[
        wall_y:wall_y + 8,
        int(width * 0.35):
        int(width * 0.65),
        :
    ] = 70

    return image


def test_camera_wall_detector() -> None:
    """
    単体テスト。

    Raspberry Piにカメラがなくても実行可能。

    実行:
        python camera_wall_detector.py
    """

    print("=" * 60)
    print("CameraWallDetector test")
    print("=" * 60)

    detector = (
        CameraWallDetector()
    )

    frame = create_test_image(
        width=224,
        height=126,
    )

    result = detector.analyze(
        frame
    )

    print()

    print("=== WALL ===")

    print(
        "left_wall:",
        result["left_wall"]
    )

    print(
        "right_wall:",
        result["right_wall"]
    )

    print(
        "front_wall:",
        result["front_wall"]
    )

    print()

    print("=== CONFIDENCE ===")

    print(
        "left:",
        result["left_confidence"]
    )

    print(
        "right:",
        result["right_confidence"]
    )

    print(
        "front:",
        result["front_confidence"]
    )

    print()

    print("=== DIRECTION ===")

    print(
        "direction:",
        result["direction"]
    )

    print(
        "direction_confidence:",
        result["direction_confidence"]
    )

    print(
        "center_offset:",
        result["center_offset"]
    )

    print()

    print("=== CORNER ===")

    print(
        "corner:",
        result["corner"]
    )

    print(
        "corner_confidence:",
        result["corner_confidence"]
    )

    print()

    print("=== OBSTACLE ===")

    print(
        "obstacle:",
        result["obstacle"]
    )

    print(
        "obstacle_confidence:",
        result["obstacle_confidence"]
    )

    print(
        "obstacle_type:",
        result["obstacle_type"]
    )

    print()

    print("=== DEBUG ===")

    for key, value in result[
        "debug"
    ].items():
        print(
            f"{key}: {value}"
        )

    print()

    print("=" * 60)
    print(
        "CameraWallDetector test completed"
    )
    print("=" * 60)


# ============================================================
# メイン
# ============================================================

if __name__ == "__main__":
    test_camera_wall_detector()
