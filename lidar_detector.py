# coding:utf-8
"""
最適化された壁検出システム
各検出手法を独立したクラスとして実装
"""

import numpy as np
import math
from enum import Enum
from typing import List, Dict, Optional
import logging
from abc import ABC, abstractmethod

logger = logging.getLogger("donkeycar.parts.wall_detector")


class DetectionMethod(Enum):
    """壁検出手法の列挙型"""
    DISTANCE_BASED = "distance_based"  # 隣接点距離ベース（max_linearity=0で全領域検出）
    SPLIT_MERGE = "split_merge"        # Split-and-Merge法
    RANSAC = "ransac"                  # RANSAC法
    HYBRID = "hybrid"                   # ハイブリッド法
    SLIDING_WINDOW = "sliding_window"   # スライディングウィンドウ + RANSAC



class BaseDetector(ABC):
    """検出器の基底クラス"""
    
    def __init__(self):
        # 共通パラメータ
        self.min_wall_points = 10      # 壁セグメントの最小点数
        self.max_linearity = 0.08      # 最大許容直線偏差
        
    @abstractmethod
    def detect(self, points: np.ndarray) -> List[Dict]:
        """壁セグメントを検出"""
        pass
    
    def _create_segment(self, points: np.ndarray) -> Optional[Dict]:
        """セグメントデータを作成"""
        if len(points) < 2:
            return None
        
        start = points[0]
        end = points[-1]
        
        # 角度と長さ
        angle = math.atan2(end[1] - start[1], end[0] - start[0])
        length = np.linalg.norm(end - start)
        
        # 直線性を計算
        linearity = self._calculate_linearity(points)
        
        return {
            'start': {'x': float(start[0]), 'y': float(start[1])},
            'end': {'x': float(end[0]), 'y': float(end[1])},
            'angle': float(angle),
            'length': float(length),
            'linearity': float(linearity),
            'points': [{'x': float(p[0]), 'y': float(p[1])} for p in points],
            'num_points': len(points)
        }
    
    def _calculate_linearity(self, points: np.ndarray) -> float:
        """直線性を計算（低いほど直線的）"""
        if len(points) < 3:
            return 0.0
        
        # 最小二乗法で直線フィット
        x = points[:, 0]
        y = points[:, 1]
        
        x_range = np.max(x) - np.min(x)
        y_range = np.max(y) - np.min(y)
        
        try:
            if x_range < y_range * 0.1:  # 垂直に近い
                A = np.vstack([y, np.ones(len(y))]).T
                a, b = np.linalg.lstsq(A, x, rcond=None)[0]
                deviations = np.abs(x - (a * y + b)) / np.sqrt(1 + a*a)
            else:
                A = np.vstack([x, np.ones(len(x))]).T
                a, b = np.linalg.lstsq(A, y, rcond=None)[0]
                deviations = np.abs(y - (a * x + b)) / np.sqrt(1 + a*a)
            
            avg_deviation = np.mean(deviations)
            length = np.linalg.norm(points[-1] - points[0])
            
            return avg_deviation / length if length > 0 else float('inf')
        except:
            return float('inf')


class DistanceBasedDetector(BaseDetector):
    """
    距離ベース検出器（高速・シンプル）

    max_linearity=0の場合、直線性チェックを無効化して全ての連続領域を検出
    （CONNECTED_POINTSと同等の動作）
    draw_polyline=Trueの場合、全点を接続して描画（ポリライン）
    draw_polyline=Falseの場合、始点と終点のみを接続して描画（直線）
    """

    def __init__(self):
        super().__init__()
        self.max_gap = 500          # 点間の最大距離 (mm)
        self.min_wall_points = 3    # 最小点数（デフォルト3点に変更）
        self.max_linearity = 0.0    # デフォルト0で直線性チェック無効
        self.draw_polyline = True   # デフォルトでポリライン描画

    def detect(self, points: np.ndarray) -> List[Dict]:
        """隣接点間の距離のみで判定（max_linearity=0なら直線性チェックなし）"""
        if len(points) < self.min_wall_points:
            return []

        segments = []
        current_segment = [points[0]]

        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i-1])

            if dist < self.max_gap:
                current_segment.append(points[i])
            else:
                if len(current_segment) >= self.min_wall_points:
                    segment = self._create_segment(np.array(current_segment))
                    if segment:
                        # max_linearity=0の場合は直線性チェックをスキップ
                        if self.max_linearity == 0 or segment['linearity'] < self.max_linearity:
                            segment['draw_polyline'] = self.draw_polyline  # 描画モードを追加
                            segments.append(segment)
                current_segment = [points[i]]

        # 最後のセグメント
        if len(current_segment) >= self.min_wall_points:
            segment = self._create_segment(np.array(current_segment))
            if segment:
                # max_linearity=0の場合は直線性チェックをスキップ
                if self.max_linearity == 0 or segment['linearity'] < self.max_linearity:
                    segment['draw_polyline'] = self.draw_polyline  # 描画モードを追加
                    segments.append(segment)

        return segments


class SplitMergeDetector(BaseDetector):
    """最適化されたSplit-and-Merge検出器"""

    def __init__(self):
        super().__init__()
        self.split_epsilon = 60        # 分割閾値 (mm) - テスト結果により調整
        self.min_segment_length = 400  # 最小セグメント長 (mm) - 短いセグメントも検出できるよう調整
        self.use_adaptive = True       # 適応的閾値を使用
        self.use_2d_optimization = True  # 2D最適化を使用
        self.min_wall_points = 3       # 最小点数を3に変更（BaseDetectorの10を上書き）
        self.draw_polyline = True      # デフォルトでポリライン描画
        self.max_gap = 500             # 点間の最大距離 (mm) - 前処理で使用
    
    def detect(self, points: np.ndarray) -> List[Dict]:
        """Split-and-Merge法で検出"""
        if len(points) < self.min_wall_points:
            print(f"SplitMerge: Not enough points ({len(points)} < {self.min_wall_points})")
            return []

        # print(f"SplitMerge: Processing {len(points)} points, epsilon={self.split_epsilon}, max_linearity={self.max_linearity}")

        # 前処理: 隣接点間の距離で連続領域に分割
        continuous_groups = self._group_by_distance(points)

        # 各連続領域に対してSplit-and-Merge処理
        all_segments = []
        for group in continuous_groups:
            if len(group) < self.min_wall_points:
                continue

            # 適応的閾値を使用するか選択
            if self.use_adaptive:
                split_segments = self._split_merge_adaptive(group, self.split_epsilon)
            else:
                split_segments = self._split_merge_optimized(group, self.split_epsilon)

            all_segments.extend(split_segments)

        # print(f"SplitMerge: Found {len(all_segments)} raw segments")

        segments = []
        for i, seg_points in enumerate(all_segments):
            if len(seg_points) >= self.min_wall_points:
                segment = self._create_segment(seg_points)
                if segment:
                    # print(f"SplitMerge: Segment {i}: {len(seg_points)} points, linearity={segment['linearity']:.4f}")
                    if segment['linearity'] < self.max_linearity:
                        segment['draw_polyline'] = self.draw_polyline  # 描画モードを追加
                        segments.append(segment)
        #                 print(f"SplitMerge: Segment {i} accepted")
        #             else:
        #                 print(f"SplitMerge: Segment {i} rejected (linearity too high)")
        #         else:
        #             print(f"SplitMerge: Segment {i} failed to create")
        #     else:
        #         print(f"SplitMerge: Segment {i} too small ({len(seg_points)} < {self.min_wall_points})")
        
        # print(f"SplitMerge: Final segments: {len(segments)}")
        return segments

    def _group_by_distance(self, points: np.ndarray) -> List[np.ndarray]:
        """隣接点間の距離で連続領域にグループ化"""
        if len(points) == 0:
            return []

        groups = []
        current_group = [points[0]]

        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i-1])

            if dist < self.max_gap:
                current_group.append(points[i])
            else:
                # 現在のグループを保存して新しいグループを開始
                if len(current_group) > 0:
                    groups.append(np.array(current_group))
                current_group = [points[i]]

        # 最後のグループを追加
        if len(current_group) > 0:
            groups.append(np.array(current_group))

        return groups

    def _split_merge_optimized(self, points: np.ndarray, epsilon: float) -> List[np.ndarray]:
        """最適化版Split-and-Merge"""
        if len(points) < 3:
            return [points]
        
        segments = []
        self._split_merge_recursive(points, 0, len(points) - 1, epsilon, segments)
        return segments
    
    def _split_merge_recursive(self, points: np.ndarray, start_idx: int, end_idx: int, 
                               epsilon: float, segments: List):
        """再帰的分割（インデックスベース）"""
        num_points = end_idx - start_idx + 1
        
        if num_points < 3:
            segments.append(points[start_idx:end_idx+1])
            return
        
        # 最大偏差点を高速に探索
        split_idx = self._find_split_point_optimized(
            points[start_idx:end_idx+1], epsilon
        )
        
        if split_idx == -1:
            # 直線として採用
            segments.append(points[start_idx:end_idx+1])
        else:
            # 実際のインデックスに変換
            actual_split_idx = start_idx + split_idx
            
            # 左右を再帰的に処理
            self._split_merge_recursive(points, start_idx, actual_split_idx, epsilon, segments)
            self._split_merge_recursive(points, actual_split_idx, end_idx, epsilon, segments)
    
    def _find_split_point_optimized(self, points: np.ndarray, epsilon: float) -> int:
        """最適化された分割点探索"""
        if len(points) < 3:
            return -1
        
        start, end = points[0], points[-1]
        
        # 2D最適化を使用するか判定
        if self.use_2d_optimization and points.shape[1] == 2:
            distances = self._calculate_distances_2d_optimized(points[1:-1], start, end)
        else:
            distances = self._calculate_distances_vectorized(points[1:-1], start, end)
        
        if len(distances) == 0:
            return -1
        
        max_dist_idx = np.argmax(distances)
        max_dist = distances[max_dist_idx]
        
        if max_dist > epsilon:
            return max_dist_idx + 1
        
        return -1
    
    def _calculate_distances_vectorized(self, points: np.ndarray, start: np.ndarray, 
                                       end: np.ndarray) -> np.ndarray:
        """ベクトル化された垂直距離計算"""
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        
        if line_len < 1e-10:
            return np.linalg.norm(points - start[np.newaxis, :], axis=1)
        
        line_unit = line_vec / line_len
        
        # 全点を一度に処理
        vecs_to_points = points - start
        proj_lengths = np.dot(vecs_to_points, line_unit)
        proj_vecs = proj_lengths[:, np.newaxis] * line_unit[np.newaxis, :]
        perpendicular_vecs = vecs_to_points - proj_vecs
        distances = np.linalg.norm(perpendicular_vecs, axis=1)
        
        return distances
    
    def _calculate_distances_2d_optimized(self, points: np.ndarray, start: np.ndarray, 
                                         end: np.ndarray) -> np.ndarray:
        """2D外積を使った高速垂直距離計算"""
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        
        if line_len < 1e-10:
            return np.linalg.norm(points - start[np.newaxis, :], axis=1)
        
        # 2D外積による計算（より高速）
        cross_products = np.abs(
            line_vec[0] * (start[1] - points[:, 1]) - 
            line_vec[1] * (start[0] - points[:, 0])
        )
        
        distances = cross_products / line_len
        return distances
    
    def _split_merge_adaptive(self, points: np.ndarray, base_epsilon: float) -> List[np.ndarray]:
        """適応的閾値を使用するSplit-and-Merge"""
        if len(points) < 3:
            return [points]
        
        segments = []
        self._split_merge_adaptive_recursive(
            points, 0, len(points) - 1, base_epsilon, segments
        )
        return segments
    
    def _split_merge_adaptive_recursive(self, points: np.ndarray, start_idx: int, 
                                       end_idx: int, base_epsilon: float, segments: List):
        """適応的な閾値を使った再帰処理"""
        num_points = end_idx - start_idx + 1
        
        if num_points < 3:
            segments.append(points[start_idx:end_idx+1])
            return
        
        # セグメントの長さを計算
        segment_length = np.linalg.norm(points[end_idx] - points[start_idx])
        
        # 長いセグメントには緩い閾値を適用 (mm単位)
        if segment_length > 5000:
            epsilon = base_epsilon * 1.5
        elif segment_length < 1000:
            epsilon = base_epsilon * 0.7
        else:
            epsilon = base_epsilon
        
        # 短すぎるセグメントは分割しない
        if segment_length < self.min_segment_length:
            segments.append(points[start_idx:end_idx+1])
            return
        
        split_idx = self._find_split_point_optimized(
            points[start_idx:end_idx+1], epsilon
        )
        
        if split_idx == -1:
            segments.append(points[start_idx:end_idx+1])
        else:
            actual_split_idx = start_idx + split_idx
            self._split_merge_adaptive_recursive(
                points, start_idx, actual_split_idx, base_epsilon, segments
            )
            self._split_merge_adaptive_recursive(
                points, actual_split_idx, end_idx, base_epsilon, segments
            )


class RANSACDetector(BaseDetector):
    """RANSAC法検出器（ノイズに強い）"""

    def __init__(self):
        super().__init__()
        self.ransac_threshold = 60     # RANSAC残差閾値 (mm) - テスト結果により緩和
        self.min_inlier_ratio = 0.6    # 最小インライア率 - ノイズ環境に対応
        self.max_trials = 150          # 最大試行回数 - 成功率向上のため増加
        self.early_stop_ratio = 0.9    # 早期終了閾値 - 少し緩和
        self.min_wall_points = 3       # 最小点数を3に変更
    
    def detect(self, points: np.ndarray) -> List[Dict]:
        """RANSAC法で検出"""
        if len(points) < self.min_wall_points:
            return []
        
        segments = []
        remaining_points = points.copy()
        remaining_mask = np.ones(len(points), dtype=bool)
        
        while np.sum(remaining_mask) >= self.min_wall_points:
            # RANSACで直線を検出
            best_segment = self._fit_line_ransac(remaining_points[remaining_mask])
            
            if best_segment is None:
                break
            
            # インライアのインデックスを元の点群に対応させる
            inlier_indices = np.where(remaining_mask)[0][best_segment['inlier_mask']]
            
            # セグメントを追加
            segment_points = points[inlier_indices]
            segment = self._create_segment(segment_points)
            if segment:
                segment['confidence'] = best_segment['confidence']
                segments.append(segment)
            
            # インライアを除去
            remaining_mask[inlier_indices] = False
        
        return segments
    
    def _fit_line_ransac(self, points: np.ndarray) -> Optional[Dict]:
        """RANSAC法で直線フィッティング（最適化版）"""
        if len(points) < self.min_wall_points:
            return None
        
        n_points = len(points)
        best_inliers = None
        best_count = 0
        
        # 早期終了のための閾値
        early_stop_count = int(n_points * self.early_stop_ratio)
        
        for _ in range(self.max_trials):
            # ランダムに2点選択
            idx = np.random.choice(n_points, 2, replace=False)
            p1, p2 = points[idx[0]], points[idx[1]]
            
            # 直線のパラメータ
            line_vec = p2 - p1
            line_len = np.linalg.norm(line_vec)
            if line_len < 1:  # 1mm未満の線は無効
                continue
            
            # ベクトル化された距離計算
            distances = self._calculate_point_line_distances(points, p1, line_vec, line_len)
            
            # インライアを判定
            inlier_mask = distances < self.ransac_threshold
            inlier_count = np.sum(inlier_mask)
            
            # 最良の直線を更新
            if inlier_count > best_count:
                best_count = inlier_count
                best_inliers = inlier_mask
                
                # 早期終了条件
                if best_count >= early_stop_count:
                    break
        
        # インライア率をチェック
        if best_inliers is None or best_count / n_points < self.min_inlier_ratio:
            return None
        
        return {
            'inlier_mask': best_inliers,
            'confidence': best_count / n_points,
            'n_inliers': best_count
        }
    
    def _calculate_point_line_distances(self, points: np.ndarray, line_point: np.ndarray,
                                       line_vec: np.ndarray, line_len: float) -> np.ndarray:
        """点群から直線への距離を高速計算"""
        line_unit = line_vec / line_len
        
        # ベクトル化計算
        vecs_to_points = points - line_point
        proj_lengths = np.dot(vecs_to_points, line_unit)
        proj_vecs = proj_lengths[:, np.newaxis] * line_unit[np.newaxis, :]
        perpendicular_vecs = vecs_to_points - proj_vecs
        
        return np.linalg.norm(perpendicular_vecs, axis=1)


class SlidingWindowDetector(BaseDetector):
    """スライディングウィンドウ + RANSAC検出器"""

    def __init__(self):
        super().__init__()
        self.window_size = 20          # ウィンドウサイズ
        self.window_stride = 5         # ウィンドウの移動幅
        self.min_wall_points = 3       # 最小点数を3に変更
        self.overlap_threshold = 700   # 重複閾値 (mm) - デフォルト値
        
        # RANSAC検出器を内部で使用
        self.ransac_detector = RANSACDetector()
    
    def detect(self, points: np.ndarray) -> List[Dict]:
        """スライディングウィンドウで検出"""
        if len(points) < self.window_size:
            return []
        
        segments = []
        
        for i in range(0, len(points) - self.window_size + 1, self.window_stride):
            window_points = points[i:i+self.window_size]
            
            # RANSACで直線検出
            window_segments = self.ransac_detector.detect(window_points)
            
            if window_segments:
                # ウィンドウ内の最良セグメントを選択
                best_segment = max(window_segments, 
                                 key=lambda s: s.get('confidence', 0))
                
                if best_segment.get('confidence', 0) > self.ransac_detector.min_inlier_ratio:
                    # 重複チェック
                    if not self._is_duplicate(best_segment, segments):
                        segments.append(best_segment)
        
        return segments
    
    def _is_duplicate(self, new_segment: Dict, existing_segments: List[Dict]) -> bool:
        """セグメントの重複をチェック"""
        new_start = np.array([new_segment['start']['x'], new_segment['start']['y']])
        new_end = np.array([new_segment['end']['x'], new_segment['end']['y']])
        
        for seg in existing_segments:
            seg_start = np.array([seg['start']['x'], seg['start']['y']])
            seg_end = np.array([seg['end']['x'], seg['end']['y']])
            
            # 端点間の距離で重複判定
            dist1 = np.linalg.norm(new_start - seg_start)
            dist2 = np.linalg.norm(new_end - seg_end)
            dist3 = np.linalg.norm(new_start - seg_end)
            dist4 = np.linalg.norm(new_end - seg_start)
            
            min_dist = min(dist1 + dist2, dist3 + dist4)
            
            if min_dist < self.overlap_threshold:
                return True
        
        return False


class HybridDetector(BaseDetector):
    """ハイブリッド検出器（Split-Merge + RANSAC）"""

    def __init__(self):
        super().__init__()
        # 内部で使用する検出器
        self.split_merge_detector = SplitMergeDetector()
        self.ransac_detector = RANSACDetector()

        # ハイブリッド固有のパラメータ
        self.confidence_threshold = 0.8  # RANSAC検証の信頼度閾値
        self.min_wall_points = 3       # 最小点数を3に変更
    
    def detect(self, points: np.ndarray) -> List[Dict]:
        """ハイブリッド手法で検出"""
        if len(points) < self.min_wall_points:
            return []
        
        # Step 1: Split-and-Mergeで高速に候補抽出
        split_segments = self.split_merge_detector._split_merge_optimized(
            points, self.split_merge_detector.split_epsilon
        )
        
        segments = []
        for seg_points in split_segments:
            if len(seg_points) < self.min_wall_points:
                continue
            
            # Step 2: RANSACで検証・精製
            refined = self.ransac_detector._fit_line_ransac(seg_points)
            
            if refined and refined.get('confidence', 0) > self.confidence_threshold:
                # インライアのみでセグメントを再構成
                inlier_points = seg_points[refined['inlier_mask']]
                segment = self._create_segment(inlier_points)
                
                if segment:
                    segment['confidence'] = refined['confidence']
                    segment['detection_method'] = 'hybrid'
                    segments.append(segment)
        
        return segments


class WallDetector:
    """
    統合壁検出クラス
    複数の検出アルゴリズムを管理し、動的に切り替え可能
    """
    
    def __init__(self, method: DetectionMethod = DetectionMethod.HYBRID):
        """
        初期化
        
        Args:
            method: 使用する検出手法
        """
        self.method = method
        
        # 各検出器のインスタンスを作成
        self.detectors = {
            DetectionMethod.DISTANCE_BASED: DistanceBasedDetector(),
            DetectionMethod.SPLIT_MERGE: SplitMergeDetector(),
            DetectionMethod.RANSAC: RANSACDetector(),
            DetectionMethod.SLIDING_WINDOW: SlidingWindowDetector(),
            DetectionMethod.HYBRID: HybridDetector()
        }
        
        # 現在の検出器
        self.current_detector = self.detectors[method]
        
        # 共通パラメータ（前処理用）- デフォルト値をmm単位に設定
        self.min_distance = 100        # 最小検出距離 (mm) - ignore_distanceと統合
        self.max_distance = 20000      # 最大検出距離 (mm)
        
        # 物体検出範囲（この範囲内の点は壁検出から除外）
        self.wall_distance = 300       # 物体検出距離 (mm) - デフォルト値
        
        # 角度設定（LiDARから）
        self.angle_start = -135        # 開始角度 (度)
        self.angle_end = 135           # 終了角度 (度)
        self.angle_offset = 0          # 角度オフセット (度)
        self.clockwise = False         # スキャン方向（True:時計回り、False:反時計回り）

        # セグメント統合用パラメータ
        self.merge_angle_threshold = 10     # 統合時の角度閾値 (度)
        self.merge_distance_threshold = 100 # 統合時の距離閾値 (mm) - デフォルト値
        
        # 検出結果
        self.wall_segments = []
        
    def set_method(self, method: DetectionMethod):
        """検出手法を変更"""
        if method in self.detectors:
            self.method = method
            self.current_detector = self.detectors[method]
            logger.info(f"Detection method changed to: {method.value}")
        else:
            logger.error(f"Invalid detection method: {method}")
    
    def set_parameters(self, **kwargs):
        """パラメータを動的に設定"""
        # 共通パラメータの設定
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
        
        # 各検出器のパラメータを設定
        for detector in self.detectors.values():
            for key, value in kwargs.items():
                if hasattr(detector, key):
                    setattr(detector, key, value)
        
        logger.debug(f"Parameters updated: {kwargs}")
    
    def detect(self, points: List[float]) -> List[Dict]:
        """
        壁セグメントを検出
        
        Args:
            points: LiDAR距離データ (mm単位)
            
        Returns:
            検出された壁セグメントのリスト
        """
        # 前処理
        points_xy = self._preprocess_points(points)
        if points_xy is None or len(points_xy) < self.current_detector.min_wall_points:
            print(f"WallDetector: Preprocessing failed or not enough points. points_xy={points_xy}, min_wall_points={self.current_detector.min_wall_points}")
            self.wall_segments = []
            return []
        
        # print(f"WallDetector: Using {self.method.value} detector with {len(points_xy)} preprocessed points")
        
        # 選択された検出器で検出
        segments = self.current_detector.detect(points_xy)
        # print(f"WallDetector: {self.method.value} detector found {len(segments)} segments")
        
        # セグメントの統合
        if len(segments) > 1:
            segments = self._merge_similar_segments(segments)
        
        # メタデータを追加
        for segment in segments:
            segment['method'] = self.method.value
        
        self.wall_segments = segments
        return segments
    
    def _preprocess_points(self, points: List[float]) -> Optional[np.ndarray]:
        """点群の前処理"""
        if len(points) == 0:
            print(f"WallDetector: No points to process")
            return None
        
        # すでにmm単位なので変換不要
        ranges = np.array(points)
        # print(f"WallDetector: Processing {len(ranges)} points, min_distance={self.min_distance}, max_distance={self.max_distance}")
        
        # 角度計算
        angle_start = self.angle_start * np.pi / 180
        angle_end = self.angle_end * np.pi / 180

        # clockwise設定に基づいて角度配列を生成
        if self.clockwise:
            # 時計回り：角度を降順で生成
            angles = np.linspace(angle_end, angle_start, len(ranges))
        else:
            # 反時計回り：角度を昇順で生成
            angles = np.linspace(angle_start, angle_end, len(ranges))

        # 角度オフセットを適用
        angle_offset_rad = self.angle_offset * np.pi / 180
        angles = angles + angle_offset_rad
        
        # 有効な点を抽出
        valid_indices = np.where((ranges > self.min_distance) & 
                                (ranges < self.max_distance))[0]
        
        # print(f"WallDetector: Found {len(valid_indices)} valid points out of {len(ranges)}")
        
        if len(valid_indices) < 3:
            print(f"WallDetector: Not enough valid points ({len(valid_indices)} < 3)")
            return None
        
        valid_angles = angles[valid_indices]
        valid_ranges = ranges[valid_indices]
        
        # デカルト座標に変換
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        return np.column_stack((x, y))
    
    def _merge_similar_segments(self, segments: List[Dict]) -> List[Dict]:
        """類似セグメントを統合"""
        if len(segments) <= 1:
            return segments
        
        merged = []
        used = [False] * len(segments)
        
        for i in range(len(segments)):
            if used[i]:
                continue
            
            current_group = [segments[i]]
            used[i] = True
            
            for j in range(i + 1, len(segments)):
                if used[j]:
                    continue
                
                if self._are_segments_similar(segments[i], segments[j]):
                    current_group.append(segments[j])
                    used[j] = True
            
            # グループを統合
            if len(current_group) > 1:
                merged_segment = self._merge_segment_group(current_group)
                merged.append(merged_segment)
            else:
                merged.append(segments[i])
        
        return merged
    
    def _are_segments_similar(self, seg1: Dict, seg2: Dict) -> bool:
        """2つのセグメントの類似性判定"""
        # 角度の差
        angle_diff = abs(seg1['angle'] - seg2['angle']) * 180 / np.pi
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        if angle_diff > self.merge_angle_threshold:
            return False
        
        # 最短距離
        dist = self._segment_distance(seg1, seg2)
        
        return dist < self.merge_distance_threshold
    
    def _segment_distance(self, seg1: Dict, seg2: Dict) -> float:
        """2つのセグメント間の最短距離"""
        points = [
            np.array([seg1['start']['x'], seg1['start']['y']]),
            np.array([seg1['end']['x'], seg1['end']['y']]),
            np.array([seg2['start']['x'], seg2['start']['y']]),
            np.array([seg2['end']['x'], seg2['end']['y']])
        ]
        
        min_dist = float('inf')
        for i in range(2):
            for j in range(2, 4):
                dist = np.linalg.norm(points[i] - points[j])
                min_dist = min(min_dist, dist)
        
        return min_dist
    
    def _merge_segment_group(self, segments: List[Dict]) -> Dict:
        """セグメントグループを1つに統合"""
        all_points = []
        for seg in segments:
            all_points.extend([(p['x'], p['y']) for p in seg['points']])
        
        all_points = np.array(all_points)
        
        # 全点から最も離れた2点を端点とする
        distances = []
        n_points = len(all_points)
        for i in range(n_points):
            for j in range(i+1, n_points):
                dist = np.linalg.norm(all_points[i] - all_points[j])
                distances.append((dist, i, j))
        
        if distances:
            distances.sort(reverse=True)
            _, idx1, idx2 = distances[0]
            
            # 始点と終点を決定（角度で判定）
            if all_points[idx1][0] < all_points[idx2][0]:
                start_idx, end_idx = idx1, idx2
            else:
                start_idx, end_idx = idx2, idx1
            
            start = all_points[start_idx]
            end = all_points[end_idx]
        else:
            start = all_points[0]
            end = all_points[-1]
        
        # 統合されたセグメントの情報を計算
        angle = math.atan2(end[1] - start[1], end[0] - start[0])
        length = np.linalg.norm(end - start)
        
        # 平均信頼度を計算（存在する場合）
        confidences = [seg.get('confidence', 1.0) for seg in segments]
        avg_confidence = np.mean(confidences)
        
        return {
            'start': {'x': float(start[0]), 'y': float(start[1])},
            'end': {'x': float(end[0]), 'y': float(end[1])},
            'angle': float(angle),
            'length': float(length),
            'linearity': 0.0,  # 統合後は直線性を再計算しない
            'points': [{'x': float(p[0]), 'y': float(p[1])} for p in all_points],
            'num_points': len(all_points),
            'merged_from': len(segments),
            'confidence': float(avg_confidence)
        }
    
    def get_detector_info(self) -> Dict:
        """現在の検出器の情報を取得"""
        info = {
            'method': self.method.value,
            'common_parameters': {
                'min_distance': self.min_distance,
                'max_distance': self.max_distance,
                'wall_distance': self.wall_distance,
                'angle_start': self.angle_start,
                'angle_end': self.angle_end,
                'angle_offset': self.angle_offset,
                'clockwise': self.clockwise,
                'merge_angle_threshold': self.merge_angle_threshold,
                'merge_distance_threshold': self.merge_distance_threshold
            },
            'detector_parameters': {}
        }
        
        # 現在の検出器のパラメータを取得
        detector = self.current_detector

        if isinstance(detector, DistanceBasedDetector):
            info['detector_parameters'] = {
                'max_gap': detector.max_gap,
                'min_wall_points': detector.min_wall_points,
                'max_linearity': detector.max_linearity,
                'draw_polyline': detector.draw_polyline
            }
        elif isinstance(detector, SplitMergeDetector):
            info['detector_parameters'] = {
                'split_epsilon': detector.split_epsilon,
                'min_segment_length': detector.min_segment_length,
                'use_adaptive': detector.use_adaptive,
                'use_2d_optimization': detector.use_2d_optimization,
                'min_wall_points': detector.min_wall_points,
                'max_linearity': detector.max_linearity,
                'draw_polyline': detector.draw_polyline,
                'max_gap': detector.max_gap
            }
        elif isinstance(detector, RANSACDetector):
            info['detector_parameters'] = {
                'ransac_threshold': detector.ransac_threshold,
                'min_inlier_ratio': detector.min_inlier_ratio,
                'max_trials': detector.max_trials,
                'early_stop_ratio': detector.early_stop_ratio,
                'min_wall_points': detector.min_wall_points,
                'max_linearity': detector.max_linearity
            }
        elif isinstance(detector, SlidingWindowDetector):
            info['detector_parameters'] = {
                'window_size': detector.window_size,
                'window_stride': detector.window_stride,
                'overlap_threshold': detector.overlap_threshold,
                'min_wall_points': detector.min_wall_points,
                'max_linearity': detector.max_linearity
            }
        elif isinstance(detector, HybridDetector):
            info['detector_parameters'] = {
                'confidence_threshold': detector.confidence_threshold,
                'split_epsilon': detector.split_merge_detector.split_epsilon,
                'ransac_threshold': detector.ransac_detector.ransac_threshold,
                'min_inlier_ratio': detector.ransac_detector.min_inlier_ratio,
                'min_wall_points': detector.min_wall_points,
                'max_linearity': detector.max_linearity
            }
        
        return info


# ベンチマーク関数
def benchmark_detectors():
    """各検出器の性能を比較"""
    import time
    
    # テストデータ生成
    np.random.seed(42)
    n_points = 1081  # LiDARの実際の点数
    
    # 廊下のようなデータを生成
    angles = np.linspace(-135 * np.pi / 180, 135 * np.pi / 180, n_points)
    
    # 壁までの距離（ノイズ付き）
    distances = []
    for angle in angles:
        if -2.0 < angle < -1.5:  # 左壁
            dist = 2.0 / np.cos(angle) + np.random.normal(0, 0.02)
        elif 1.5 < angle < 2.0:  # 右壁
            dist = 2.0 / np.cos(angle) + np.random.normal(0, 0.02)
        elif -0.5 < angle < 0.5:  # 前壁
            dist = 5.0 + np.random.normal(0, 0.03)
        else:
            dist = 20.0  # 遠方
        
        distances.append(max(20, min(20000, dist * 1000)))  # mからmmに変換
    
    # 各検出器をテスト
    methods = [
        DetectionMethod.DISTANCE_BASED,
        DetectionMethod.SPLIT_MERGE,
        DetectionMethod.RANSAC,
        DetectionMethod.SLIDING_WINDOW,
        DetectionMethod.HYBRID
    ]
    
    results = {}
    
    for method in methods:
        detector = WallDetector(method)
        
        # 速度測定
        start_time = time.time()
        for _ in range(10):
            segments = detector.detect(distances)
        elapsed_time = (time.time() - start_time) / 10
        
        # 結果を保存
        results[method.value] = {
            'time': elapsed_time,
            'segments': len(segments),
            'total_points': sum(seg['num_points'] for seg in segments)
        }
        
        print(f"{method.value:20s}: {elapsed_time*1000:.2f}ms, "
              f"{len(segments)} segments, {results[method.value]['total_points']} points")
    
    return results


if __name__ == "__main__":
    # ベンチマーク実行
    print("Wall Detector Benchmark")
    print("-" * 50)
    results = benchmark_detectors()
    
    print("\n" + "=" * 50)
    print("Performance Summary:")
    print("-" * 50)
    
    # 最速の手法を見つける
    fastest = min(results.items(), key=lambda x: x[1]['time'])
    print(f"Fastest: {fastest[0]} ({fastest[1]['time']*1000:.2f}ms)")
    
    # 最も多くのセグメントを検出した手法
    most_segments = max(results.items(), key=lambda x: x[1]['segments'])
    print(f"Most segments: {most_segments[0]} ({most_segments[1]['segments']} segments)")