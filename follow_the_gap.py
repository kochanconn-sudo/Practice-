# follow_the_gap.py
# coding:utf-8
"""
Follow the Gap アルゴリズムの実装

LiDARスキャンデータから最も広いギャップ（障害物のない空間）を見つけ、
その方向に車両を向けるアルゴリズム。
"""

import time

import numpy as np
import config
import logging

logger = logging.getLogger(__name__)


class FollowTheGap:
    """Follow the Gapアルゴリズムによる自律走行プランナー"""

    def __init__(self):
        """初期化"""
        # 設定パラメータを読み込み
        self.safety_distance = getattr(config, 'FTG_SAFETY_DISTANCE', 300)  # 安全距離 (mm)
        self.max_distance = getattr(config, 'FTG_MAX_DISTANCE', 3000)  # 最大検出距離 (mm)
        self.bubble_radius = getattr(config, 'FTG_BUBBLE_RADIUS', 150)  # 安全バブル半径 (mm)
        self.disparity_threshold = getattr(config, 'FTG_DISPARITY_THRESHOLD', 200)  # 距離差閾値 (mm)

        # 角度範囲（前方のみ使用）
        self.angle_start = getattr(config, 'FTG_ANGLE_START', -90)  # 開始角度 (度)
        self.angle_end = getattr(config, 'FTG_ANGLE_END', 90)  # 終了角度 (度)

        # ステアリングゲイン
        self.steering_gain = getattr(config, 'FTG_STEERING_GAIN', 1.0)

        # スムージング用
        self.prev_steering = 0.0
        self.smoothing_factor = getattr(config, 'FTG_SMOOTHING_FACTOR', 0.3)

        # ステアリング制御方式
        self.steering_method = getattr(config, 'FTG_STEERING_METHOD', 'linear')

        # PID用の状態変数
        self.pid_kp = getattr(config, 'FTG_PID_KP', 0.8)
        self.pid_ki = getattr(config, 'FTG_PID_KI', 0.0)
        self.pid_kd = getattr(config, 'FTG_PID_KD', 0.1)
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_prev_time = time.perf_counter()

        # Pure Pursuit用パラメータ
        self.wheelbase = getattr(config, 'FTG_WHEELBASE', 300)         # mm
        self.lookahead_distance = getattr(config, 'FTG_LOOKAHEAD_DISTANCE', 500)  # mm

        # モニター表示用の診断情報
        self._ftg_info = None

        logger.info(f"FollowTheGap initialized: safety_distance={self.safety_distance}mm, "
                   f"angle_range=[{self.angle_start}, {self.angle_end}], "
                   f"steering_method={self.steering_method}")

    def preprocess_lidar(self, measurements):
        """
        LiDARデータの前処理

        Args:
            measurements: 生のLiDARスキャンデータ（距離のリスト）

        Returns:
            processed: 前処理済みのデータ
            angles: 対応する角度のリスト（度）
        """
        if measurements is None or len(measurements) == 0:
            return np.array([]), np.array([])

        # numpy配列に変換
        distances = np.array(measurements, dtype=np.float32)

        # 角度配列を生成（LiDARの設定に基づく）
        num_points = len(distances)
        lidar_angle_start = getattr(config, 'LIDAR_ANGLE_START', -135)
        lidar_angle_end = getattr(config, 'LIDAR_ANGLE_END', 135)
        angles = np.linspace(lidar_angle_start, lidar_angle_end, num_points)

        # 角度を-180〜180に正規化（TMINI等の0〜360範囲に対応）
        angles = (angles + 180.0) % 360.0 - 180.0

        # 前方の角度範囲のみを抽出
        mask = (angles >= self.angle_start) & (angles <= self.angle_end)
        distances = distances[mask]
        angles = angles[mask]

        # 角度順にソート（0-360→-180-180変換で順序が崩れる場合の対策）
        sort_idx = np.argsort(angles)
        distances = distances[sort_idx]
        angles = angles[sort_idx]

        # 無効な値（0やnegative）を最大距離で置換
        invalid_mask = (distances <= 0) | (distances > self.max_distance)
        distances[invalid_mask] = self.max_distance

        return distances, angles

    def find_closest_point(self, distances):
        """
        最も近い障害物を見つける

        Args:
            distances: 距離データ

        Returns:
            closest_idx: 最も近い点のインデックス
            closest_dist: 最も近い点の距離
        """
        if len(distances) == 0:
            return -1, self.max_distance

        closest_idx = np.argmin(distances)
        closest_dist = distances[closest_idx]

        return closest_idx, closest_dist

    def apply_safety_bubble(self, distances, closest_idx):
        """
        最も近い障害物の周りに安全バブルを適用
        障害物付近の点を0（通行不可）にする

        Args:
            distances: 距離データ
            closest_idx: 最も近い点のインデックス

        Returns:
            modified_distances: 安全バブル適用後の距離データ
        """
        if closest_idx < 0 or len(distances) == 0:
            return distances.copy()

        modified = distances.copy()
        closest_dist = distances[closest_idx]

        # 安全バブルが必要かチェック
        if closest_dist > self.safety_distance:
            return modified

        # バブル半径に基づいて周囲の点も0にする
        # 角度あたりの距離を考慮してバブルサイズを計算
        num_points = len(distances)
        angle_increment = (self.angle_end - self.angle_start) / max(num_points - 1, 1)

        # バブル半径を角度に変換（近似）
        if closest_dist > 0:
            bubble_angle = np.degrees(np.arctan2(self.bubble_radius, closest_dist))
            bubble_points = int(bubble_angle / angle_increment) if angle_increment > 0 else 0
        else:
            bubble_points = 5  # デフォルト値

        # バブル内の点を0にする
        start_idx = max(0, closest_idx - bubble_points)
        end_idx = min(num_points, closest_idx + bubble_points + 1)
        modified[start_idx:end_idx] = 0

        return modified

    def extend_disparities(self, distances, angles):
        """
        距離の不連続点（disparity）を拡張
        急激な距離変化がある箇所の近い側を拡張して安全マージンを確保

        Args:
            distances: 距離データ
            angles: 角度データ

        Returns:
            extended_distances: 拡張後の距離データ
        """
        if len(distances) < 2:
            return distances.copy()

        extended = distances.copy()
        num_points = len(distances)

        # 隣接点との距離差をチェック
        for i in range(num_points - 1):
            diff = distances[i + 1] - distances[i]

            if abs(diff) > self.disparity_threshold:
                # 距離差が大きい場合、近い方の値を周囲に拡張
                if diff > 0:
                    # 右側が遠い -> 左側（近い方）を右に拡張
                    extend_dist = distances[i]
                    extend_points = int(self.bubble_radius / max(extend_dist, 100) * 10)
                    for j in range(i + 1, min(i + 1 + extend_points, num_points)):
                        if extended[j] > extend_dist:
                            extended[j] = extend_dist
                else:
                    # 左側が遠い -> 右側（近い方）を左に拡張
                    extend_dist = distances[i + 1]
                    extend_points = int(self.bubble_radius / max(extend_dist, 100) * 10)
                    for j in range(max(0, i - extend_points + 1), i + 1):
                        if extended[j] > extend_dist:
                            extended[j] = extend_dist

        return extended

    def find_best_gap(self, distances, angles, original_distances=None):
        """
        最も広いギャップを見つける

        Args:
            distances: 処理済みの距離データ
            angles: 角度データ
            original_distances: 元の距離データ（Pure Pursuit用、Noneの場合はdistancesを使用）

        Returns:
            gap_start: ギャップ開始インデックス
            gap_end: ギャップ終了インデックス
            gap_center_angle: ギャップ中心の角度（度）
            target_distance: 目標点の距離（mm）
        """
        if len(distances) == 0:
            return -1, -1, 0.0, 0.0

        # 安全距離以上の点を「通過可能」とする
        passable = distances >= self.safety_distance

        # ギャップ（連続する通過可能領域）を見つける
        gaps = []
        in_gap = False
        gap_start_idx = 0

        for i, is_passable in enumerate(passable):
            if is_passable and not in_gap:
                # ギャップ開始
                gap_start_idx = i
                in_gap = True
            elif not is_passable and in_gap:
                # ギャップ終了
                gaps.append((gap_start_idx, i - 1))
                in_gap = False

        # 最後までギャップが続いていた場合
        if in_gap:
            gaps.append((gap_start_idx, len(distances) - 1))

        # ギャップがない場合
        if not gaps:
            # 最も遠い点の方向に向かう
            best_idx = np.argmax(distances)
            dist_src = original_distances if original_distances is not None else distances
            return best_idx, best_idx, angles[best_idx], dist_src[best_idx]

        # 最も広いギャップを選択（幅と距離、中央への近さを考慮）
        best_gap = None
        best_score = -1
        center_idx = len(distances) // 2

        for gs, ge in gaps:
            gap_width = ge - gs + 1
            gap_distances = distances[gs:ge + 1]
            avg_distance = np.mean(gap_distances)

            # スコア = 幅 × 平均距離（両方を考慮）
            score = gap_width * avg_distance

            # 中央に近いギャップを優遇（中央ボーナスを強化）
            gap_center_idx = (gs + ge) // 2
            center_distance = abs(gap_center_idx - center_idx) / len(distances)
            center_bonus = 1.0 - 0.3 * center_distance  # 中央ボーナスを強化
            score *= center_bonus

            if score > best_score:
                best_score = score
                best_gap = (gs, ge)

        if best_gap is None:
            return -1, -1, 0.0, 0.0

        gap_start, gap_end = best_gap

        # ギャップの中心を目標にする（ただし中央寄りを優先）
        gap_center_idx = (gap_start + gap_end) // 2

        # ギャップ内の距離が均一でない場合、最も遠い点を優先しつつ中央に近い点を選ぶ
        gap_distances = distances[gap_start:gap_end + 1]
        gap_angles = angles[gap_start:gap_end + 1]

        # 最も遠い距離の90%以上の点から、中央（0度）に最も近い点を選ぶ
        max_dist = np.max(gap_distances)
        threshold = max_dist * 0.9
        good_indices = np.where(gap_distances >= threshold)[0]

        if len(good_indices) > 0:
            # 良い点の中から0度に最も近い点を選択
            good_angles = gap_angles[good_indices]
            best_in_gap = good_indices[np.argmin(np.abs(good_angles))]
        else:
            # フォールバック：ギャップの中心
            best_in_gap = len(gap_distances) // 2

        target_idx = gap_start + best_in_gap
        target_angle = angles[target_idx]
        dist_src = original_distances if original_distances is not None else distances
        target_distance = dist_src[target_idx]

        return gap_start, gap_end, target_angle, target_distance

    def calculate_steering(self, target_angle, target_distance=None):
        """
        目標角度からステアリング値を計算

        Args:
            target_angle: 目標角度（度）、0が正面、負が左、正が右
            target_distance: 目標点の距離（mm、Pure Pursuit用）

        Returns:
            steering: ステアリング値（-1.0〜1.0）
        """
        if self.steering_method == 'pid':
            steering = self._steering_pid(target_angle)
        elif self.steering_method == 'pure_pursuit':
            steering = self._steering_pure_pursuit(target_angle, target_distance)
        else:  # linear（現状と同じ）
            steering = self._steering_linear(target_angle)

        # 共通: 範囲制限
        steering = np.clip(steering, -1.0, 1.0)

        # 共通: EMAスムージング
        steering = (self.smoothing_factor * steering +
                   (1 - self.smoothing_factor) * self.prev_steering)
        self.prev_steering = steering

        return steering

    def _steering_linear(self, target_angle):
        """線形マッピング（現状ロジック）"""
        max_angle = max(abs(self.angle_start), abs(self.angle_end))
        if max_angle > 0:
            return (target_angle / max_angle) * self.steering_gain
        return 0.0

    def _steering_pid(self, target_angle):
        """PID制御（target_angle=0が目標 → target_angle自体が誤差）"""
        now = time.perf_counter()
        dt = now - self.pid_prev_time
        self.pid_prev_time = now

        error = target_angle  # 目標は0°（正面）

        # I項（ワインドアップ防止付き）
        self.pid_integral += error * dt
        self.pid_integral = np.clip(self.pid_integral, -100.0, 100.0)

        # D項
        derivative = (error - self.pid_prev_error) / dt if dt > 0 else 0.0
        self.pid_prev_error = error

        # PID出力（角度→ステアリングへの正規化にsteering_gainを乗算）
        max_angle = max(abs(self.angle_start), abs(self.angle_end))
        output = self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative
        if max_angle > 0:
            output /= max_angle
        return output * self.steering_gain

    def _steering_pure_pursuit(self, target_angle, target_distance=None):
        """Pure Pursuit幾何学的追従"""
        alpha = np.radians(target_angle)
        ld = target_distance if target_distance and target_distance > 0 else self.lookahead_distance

        # 曲率: κ = 2 * sin(α) / Ld
        # ステアリング: δ = atan(L * κ) = atan(2 * L * sin(α) / Ld)
        curvature = 2.0 * np.sin(alpha) / ld
        steering_rad = np.arctan(self.wheelbase * curvature)

        # ラジアン→正規化（最大ステアリング角を±45°と仮定）
        max_steering_rad = np.radians(45)
        return (steering_rad / max_steering_rad) * self.steering_gain

    def calculate_throttle(self, distances, steering):
        """
        前方の状況とステアリングに基づいてスロットルを計算

        Args:
            distances: 距離データ
            steering: ステアリング値

        Returns:
            throttle: スロットル値
        """
        if len(distances) == 0:
            return config.FORWARD_CORNER

        # 前方（中央付近）の最小距離を取得
        center_start = len(distances) // 3
        center_end = 2 * len(distances) // 3
        front_distances = distances[center_start:center_end]

        if len(front_distances) > 0:
            min_front_distance = np.min(front_distances)
        else:
            min_front_distance = self.max_distance

        # 距離とステアリングに基づいてスロットルを調整
        if min_front_distance < self.safety_distance:
            # 非常に近い -> 低速
            throttle = config.FORWARD_CORNER * 0.5
        elif min_front_distance < self.safety_distance * 2:
            # 近い -> やや低速
            throttle = config.FORWARD_CORNER
        elif abs(steering) > 0.5:
            # 大きく曲がる -> コーナー速度
            throttle = config.FORWARD_CORNER
        else:
            # 通常 -> ストレート速度
            throttle = config.FORWARD_STRAIGHT

        return throttle

    def compute(self, lidar_data):
        """
        Follow the Gapアルゴリズムのメイン計算

        Args:
            lidar_data: LiDARデータ辞書 {'measurements': [...], ...} または距離リスト

        Returns:
            steering: ステアリング値（-1.0〜1.0）
            throttle: スロットル値
        """
        # LiDARデータを取得
        if isinstance(lidar_data, dict):
            measurements = lidar_data.get('measurements', [])
        else:
            measurements = lidar_data

        if measurements is None or len(measurements) == 0:
            logger.warning("No LiDAR data available for Follow the Gap")
            return 0.0, config.FORWARD_CORNER

        # 1. LiDARデータの前処理
        distances, angles = self.preprocess_lidar(measurements)

        if len(distances) == 0:
            return 0.0, config.FORWARD_CORNER

        # 2. 最も近い点を見つける
        closest_idx, closest_dist = self.find_closest_point(distances)

        # 3. 安全バブルを適用
        processed_distances = self.apply_safety_bubble(distances, closest_idx)

        # 4. 距離の不連続点を拡張
        processed_distances = self.extend_disparities(processed_distances, angles)

        # 5. 最も広いギャップを見つける
        gap_start, gap_end, target_angle, target_distance = self.find_best_gap(
            processed_distances, angles, distances)

        # 6. ステアリングを計算
        steering = self.calculate_steering(target_angle, target_distance)

        # 7. スロットルを計算
        throttle = self.calculate_throttle(distances, steering)

        # モニター表示用の診断情報を保存
        self._ftg_info = {
            'target_angle': float(target_angle),
            'target_distance': float(target_distance),
            'gap_start_angle': float(angles[gap_start]) if 0 <= gap_start < len(angles) else 0.0,
            'gap_end_angle': float(angles[gap_end]) if 0 <= gap_end < len(angles) else 0.0,
            'closest_dist': float(closest_dist),
            'steering_method': self.steering_method,
        }

        # デバッグ出力
        if config.TERMINAL_PRINT:
            gap_width = gap_end - gap_start + 1 if gap_start >= 0 else 0
            logger.debug(f"FTG: target_angle={target_angle:.1f}, gap_width={gap_width}, "
                        f"closest_dist={closest_dist:.0f}mm, steering={steering:.2f}")

        return steering, throttle


# グローバルインスタンス（必要に応じて使用）
_ftg_instance = None

def get_follow_the_gap_instance():
    """Follow the Gapインスタンスを取得（シングルトン）"""
    global _ftg_instance
    if _ftg_instance is None:
        _ftg_instance = FollowTheGap()
    return _ftg_instance


def follow_the_gap(lidar_data):
    """
    Follow the Gapアルゴリズムを実行する便利関数

    Args:
        lidar_data: LiDARデータ

    Returns:
        steering, throttle: 制御値
    """
    ftg = get_follow_the_gap_instance()
    return ftg.compute(lidar_data)
