# Follow the Gap（FTG）アルゴリズム

LiDARの点群データを使って障害物を回避しながら走行する「Follow the Gap」アルゴリズムを学びます。

---

## 概要

Follow the Gap（FTG）は、LiDARで検出した点群から**最も広い空間（Gap）**を見つけ、そこに向かって走行するアルゴリズムです。

```mermaid
graph LR
    A[LiDAR点群] --> B[Find the Gap]
    B --> C[目標点決定]
    C --> D[Pure Pursuit]
    D --> E[ステアリング出力]
```

| ステップ | 処理内容 |
|---------|---------|
| **Find the Gap** | 点群からGap（空間）を検出 |
| **目標点決定** | Gapの中心または最深点を選択 |
| **Pure Pursuit** | 目標点に向かうステアリングを計算 |

---

## Step 1: Find the Gap

### 基本的な考え方

LiDARの点群データから、障害物のない**空間（Gap）**を見つけます。

```
        LiDAR点群の例（上から見た図）

              障害物A    Gap!    障害物B
                ●●●              ●●●
               ●   ●            ●   ●
              ●     ●          ●     ●

                     ↑
                   車両
```

### アルゴリズムの流れ

```
1. LiDARデータを角度順にソート
2. 各点間の距離差を計算
3. 距離差が大きい箇所 = Gap
4. 最も広いGapを選択
```

### 実装例

```python
import numpy as np

def find_the_gap(ranges, angles, threshold=0.5):
    """
    LiDAR点群からGapを検出する

    Args:
        ranges: 各角度の距離データ (mm)
        angles: 各点の角度 (rad)
        threshold: Gap判定の距離閾値 (m)

    Returns:
        gap_start: Gapの開始角度
        gap_end: Gapの終了角度
        gap_center: Gapの中心角度
    """
    # 距離をメートルに変換
    ranges_m = np.array(ranges) / 1000.0

    # 無効な値を最大距離に置換
    max_range = 10.0
    ranges_m = np.where(ranges_m <= 0, max_range, ranges_m)

    # 連続する点間の距離差を計算
    gaps = []
    for i in range(len(ranges_m) - 1):
        diff = abs(ranges_m[i+1] - ranges_m[i])
        if diff > threshold:
            gaps.append({
                'start_idx': i,
                'end_idx': i + 1,
                'start_angle': angles[i],
                'end_angle': angles[i + 1],
                'width': abs(angles[i + 1] - angles[i])
            })

    # 最も広いGapを選択
    if gaps:
        best_gap = max(gaps, key=lambda g: g['width'])
        gap_center = (best_gap['start_angle'] + best_gap['end_angle']) / 2
        return best_gap['start_angle'], best_gap['end_angle'], gap_center

    # Gapが見つからない場合は正面を返す
    return -0.1, 0.1, 0.0
```

### Disparity Extender（発展）

単純なGap検出では、障害物の端ぎりぎりを通ろうとして衝突する可能性があります。
**Disparity Extender**は、障害物の「影」を拡張して安全マージンを確保します。

```
Before (単純なGap検出):
    ●●●       ●●●
       ↑Gap↑        ← 狭いGapを通ろうとする

After (Disparity Extender適用):
    ●●●●●   ●●●●●
         ↑Gap↑      ← 安全なGapのみ選択
```

```python
def disparity_extender(ranges, angles, car_width=0.3):
    """
    障害物の影を車幅分拡張する

    Args:
        ranges: 距離データ (m)
        angles: 角度データ (rad)
        car_width: 車幅 (m)
    """
    extended_ranges = ranges.copy()

    for i in range(len(ranges) - 1):
        # 距離の急激な変化を検出
        if abs(ranges[i+1] - ranges[i]) > 0.5:
            # 近い方の点を基準に、車幅分の角度を計算
            closer_dist = min(ranges[i], ranges[i+1])
            extend_angle = np.arctan2(car_width / 2, closer_dist)

            # 該当する角度範囲の距離を「壁」として設定
            extend_points = int(extend_angle / (angles[1] - angles[0]))
            for j in range(max(0, i - extend_points),
                          min(len(ranges), i + extend_points + 1)):
                extended_ranges[j] = min(extended_ranges[j], closer_dist)

    return extended_ranges
```

---

## Step 2: Pure Pursuit

### 基本的な考え方

Pure Pursuitは、車両前方の**目標点（Look-ahead point）**に向かって滑らかに曲がるアルゴリズムです。

```
                    ○ 目標点（Look-ahead point）
                   /
                  /  Look-ahead距離 (Ld)
                 /
                /
    ┌─────────┐/
    │  車両   │─────→ 進行方向
    └─────────┘
```

### 幾何学的関係

目標点への角度と旋回半径の関係は、アッカーマン・ジオメトリから導出されます。

```
              目標点
                ○
               /|
              / |
             /  | y
            /   |
           / α  |
    車両 ●─────────
              x

    α: 目標点への角度
    Ld: Look-ahead距離 = √(x² + y²)
```

**ステアリング角の計算式**:

```
δ = arctan(2 × L × sin(α) / Ld)

δ: ステアリング角
L: ホイールベース
α: 目標点への角度
Ld: Look-ahead距離
```

### 実装例

```python
import math

def pure_pursuit(target_x, target_y, wheelbase=0.15, max_steering=0.5):
    """
    Pure Pursuitでステアリング角を計算

    Args:
        target_x: 目標点のX座標（前方向、m）
        target_y: 目標点のY座標（左方向が正、m）
        wheelbase: ホイールベース (m)
        max_steering: 最大ステアリング値

    Returns:
        steering: ステアリング値 (-1.0 ~ 1.0)
    """
    # Look-ahead距離
    ld = math.sqrt(target_x**2 + target_y**2)

    if ld < 0.01:  # 目標点が近すぎる場合
        return 0.0

    # 目標点への角度
    alpha = math.atan2(target_y, target_x)

    # ステアリング角を計算
    steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), ld)

    # -1.0 ~ 1.0 に正規化
    steering = steering_angle / (math.pi / 4)  # ±45度を±1.0に
    steering = max(-max_steering, min(max_steering, steering))

    return steering
```

### Look-ahead距離の調整

| Look-ahead距離 | 特徴 |
|---------------|------|
| **短い** | 反応が速いが、ふらつきやすい |
| **長い** | 滑らかだが、障害物への反応が遅い |

!!! tip "速度に応じた調整"
    一般的に、速度が高いほどLook-ahead距離を長くします。
    ```python
    Ld = Ld_min + k * velocity
    ```

---

## Step 3: Follow the Gap 統合

Find the GapとPure Pursuitを組み合わせた完全な実装です。

```python
import numpy as np
import math

class FollowTheGap:
    def __init__(self, wheelbase=0.15, car_width=0.2):
        self.wheelbase = wheelbase
        self.car_width = car_width
        self.look_ahead = 0.5  # Look-ahead距離 (m)

    def process(self, ranges, angles):
        """
        LiDARデータからステアリングを計算

        Args:
            ranges: 距離データ (mm)
            angles: 角度データ (rad)

        Returns:
            steering: ステアリング値 (-1.0 ~ 1.0)
            throttle: スロットル値 (0.0 ~ 1.0)
        """
        # Step 1: 距離データをメートルに変換
        ranges_m = np.array(ranges) / 1000.0

        # Step 2: Disparity Extenderで安全マージン確保
        safe_ranges = self._disparity_extender(ranges_m, angles)

        # Step 3: Find the Gap
        gap_start, gap_end, gap_center = self._find_gap(safe_ranges, angles)

        # Step 4: 目標点を計算
        target_x = self.look_ahead * math.cos(gap_center)
        target_y = self.look_ahead * math.sin(gap_center)

        # Step 5: Pure Pursuitでステアリング計算
        steering = self._pure_pursuit(target_x, target_y)

        # Step 6: 前方障害物に応じてスロットル調整
        front_dist = self._get_front_distance(ranges_m, angles)
        throttle = self._calculate_throttle(front_dist)

        return steering, throttle

    def _disparity_extender(self, ranges, angles):
        """障害物の影を拡張"""
        extended = ranges.copy()
        angle_step = angles[1] - angles[0] if len(angles) > 1 else 0.01

        for i in range(len(ranges) - 1):
            if abs(ranges[i+1] - ranges[i]) > 0.3:
                closer = min(ranges[i], ranges[i+1])
                extend_angle = math.atan2(self.car_width, closer)
                extend_n = int(extend_angle / angle_step)

                for j in range(max(0, i - extend_n),
                              min(len(ranges), i + extend_n + 1)):
                    extended[j] = min(extended[j], closer)

        return extended

    def _find_gap(self, ranges, angles):
        """最大のGapを検出"""
        # 前方180度のみを対象
        front_mask = (angles > -math.pi/2) & (angles < math.pi/2)
        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]

        if len(front_ranges) == 0:
            return -0.1, 0.1, 0.0

        # 最も遠い点の周辺をGapとする（シンプルな方法）
        max_idx = np.argmax(front_ranges)

        # Gapの範囲を探索
        threshold = front_ranges[max_idx] * 0.7
        gap_indices = np.where(front_ranges > threshold)[0]

        if len(gap_indices) > 0:
            gap_start = front_angles[gap_indices[0]]
            gap_end = front_angles[gap_indices[-1]]
            gap_center = front_angles[max_idx]
            return gap_start, gap_end, gap_center

        return -0.1, 0.1, 0.0

    def _pure_pursuit(self, target_x, target_y):
        """Pure Pursuitでステアリング計算"""
        ld = math.sqrt(target_x**2 + target_y**2)
        if ld < 0.01:
            return 0.0

        alpha = math.atan2(target_y, target_x)
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), ld)

        # 正規化 (-1.0 ~ 1.0)
        steering = steering_angle / (math.pi / 4)
        return max(-1.0, min(1.0, steering))

    def _get_front_distance(self, ranges, angles):
        """前方の最小距離を取得"""
        front_mask = (angles > -0.2) & (angles < 0.2)
        front_ranges = ranges[front_mask]
        return np.min(front_ranges) if len(front_ranges) > 0 else 10.0

    def _calculate_throttle(self, front_dist):
        """前方距離に応じてスロットル調整"""
        if front_dist < 0.3:
            return 0.0
        elif front_dist < 0.5:
            return 0.2
        elif front_dist < 1.0:
            return 0.3
        else:
            return 0.4
```

---

## 使用方法

### config.pyの設定

```python
# 判断モードをFTGに設定
PLAN = "ftg"

# LiDARを有効化
HAVE_LIDAR = True
ACTIVE_SENSORS = ["lidar", "camera_0"]

# FTG基本パラメータ
FTG_SAFETY_DISTANCE = 300       # 安全距離 (mm)
FTG_MAX_DISTANCE = 3000         # 最大検出距離 (mm)

# ステアリング制御方式: "linear", "pid", "pure_pursuit"
FTG_STEERING_METHOD = "linear"
FTG_STEERING_GAIN = 1.0         # ステアリングゲイン（全方式共通）
FTG_SMOOTHING_FACTOR = 0.3      # EMAスムージング係数（全方式共通）

# PID制御パラメータ（FTG_STEERING_METHOD = "pid" 時に使用）
FTG_PID_KP = 0.8
FTG_PID_KI = 0.0
FTG_PID_KD = 0.1

# Pure Pursuit パラメータ（FTG_STEERING_METHOD = "pure_pursuit" 時に使用）
FTG_WHEELBASE = 300              # ホイールベース (mm)
FTG_LOOKAHEAD_DISTANCE = 500     # ルックアヘッド距離 (mm)
```

### planner.pyへの組み込み

```python
from follow_the_gap import FollowTheGap

class Planner:
    def __init__(self):
        self.ftg = FollowTheGap(
            wheelbase=config.WHEELBASE,
            car_width=config.FTG_CAR_WIDTH
        )

    def plan(self, sensor_data):
        if config.PLAN == "ftg":
            ranges = sensor_data.get('lidar_ranges', [])
            angles = sensor_data.get('lidar_angles', [])
            return self.ftg.process(ranges, angles)
        # ... 他のプランナー
```

---

## パラメータチューニング

| パラメータ | 推奨値 | 効果 |
|-----------|-------|------|
| `FTG_STEERING_METHOD` | "linear" / "pid" / "pure_pursuit" | ステアリング制御方式の選択 |
| `FTG_STEERING_GAIN` | 0.5〜2.0 | ステアリングの感度（全方式共通） |
| `FTG_SMOOTHING_FACTOR` | 0.1〜0.5 | EMAスムージング（0=滑らか、1=即応答） |
| `FTG_PID_KP` | 0.5〜1.5 | PID比例ゲイン（応答速度） |
| `FTG_PID_KI` | 0.0〜0.1 | PID積分ゲイン（定常偏差除去） |
| `FTG_PID_KD` | 0.05〜0.3 | PID微分ゲイン（振動抑制） |
| `FTG_LOOKAHEAD_DISTANCE` | 300〜800mm | Pure Pursuit ルックアヘッド距離（大きいほど滑らか） |
| `FTG_BUBBLE_RADIUS` | 100〜200mm | 安全バブル半径（車幅の半分程度） |
| `FTG_DISPARITY_THRESHOLD` | 150〜300mm | 距離差閾値（大きいほど保守的） |

---

## 可視化・デバッグ

```python
import cv2
import numpy as np

def visualize_ftg(ranges, angles, gap_center, steering):
    """FTGの状態を可視化"""
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    center = (200, 350)
    scale = 100  # 1m = 100px

    # 点群を描画
    for r, a in zip(ranges, angles):
        if r > 0 and r < 3:
            x = int(center[0] + r * scale * np.sin(a))
            y = int(center[1] - r * scale * np.cos(a))
            cv2.circle(img, (x, y), 2, (0, 255, 0), -1)

    # Gap方向を描画
    gap_x = int(center[0] + 150 * np.sin(gap_center))
    gap_y = int(center[1] - 150 * np.cos(gap_center))
    cv2.arrowedLine(img, center, (gap_x, gap_y), (255, 0, 0), 2)

    # ステアリング表示
    cv2.putText(img, f"Steering: {steering:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    return img
```

---

## 考えてみよう

### 1. Look-ahead距離を速度に応じて変えるとどうなる？

??? hint "ヒント"
    - 高速時に短いLook-ahead距離だと、どんな問題が起きる？
    - 低速時に長いLook-ahead距離だと、どんな問題が起きる？

??? success "解答例"
    **速度適応型Look-ahead距離**

    ```python
    Ld = Ld_min + k * velocity
    ```

    - **高速時にLdを長く**: 急な方向転換を避け、安定した走行
    - **低速時にLdを短く**: 狭い場所でも正確に曲がれる

    | 速度 | Look-ahead | 理由 |
    |-----|-----------|------|
    | 0.2 m/s | 0.3m | 狭い通路での反応性 |
    | 0.5 m/s | 0.5m | バランス |
    | 1.0 m/s | 0.8m | 安定性重視 |

### 2. Disparity Extenderがないとどうなる？

??? hint "ヒント"
    - 障害物の「端」ぎりぎりを通ろうとすると...
    - 車両には「幅」がある

??? success "解答例"
    **障害物との衝突リスクが増加します。**

    ```
    Disparity Extenderなし:

        ●●●    ↑車両経路    ●●●
           ╲    　  　    ╱
            ╲　　　　　╱  ← 端ぎりぎりを通過
              車両幅で衝突！
    ```

    Disparity Extenderは、LiDARの「点」で見えている障害物を、
    車幅を考慮した「面」として扱うことで安全マージンを確保します。

### 3. Pure PursuitとPID制御の違いは？

??? hint "ヒント"
    - PID: 「誤差」を見て修正
    - Pure Pursuit: 「目標点」を見て追従
    - どちらが「先読み」している？

??? success "解答例"
    | 項目 | PID制御 | Pure Pursuit |
    |------|--------|--------------|
    | **基準** | 現在の誤差 | 前方の目標点 |
    | **先読み** | なし（反応的） | あり（予測的） |
    | **軌道** | ジグザグになりやすい | 滑らかな曲線 |
    | **調整** | Kp, Ki, Kd | Look-ahead距離 |
    | **適用** | 壁沿い走行 | 経路追従 |

    Pure Pursuitは「目標点に向かう円弧」を描くため、
    PIDより滑らかな軌道になります。

### 4. FTGが苦手な状況は？

??? hint "ヒント"
    - 「最も広い空間」に向かうアルゴリズム
    - 目的地が狭い通路の先にある場合は？

??? success "解答例"
    **FTGが苦手な状況:**

    1. **袋小路**: 広いが行き止まりの空間に向かってしまう
    2. **狭い通路**: より広い空間があると、そちらに逸れる
    3. **動的障害物**: LiDARのスキャン間隔より速い物体

    ```
    例：袋小路の問題

    ┌───────────────┐
    │               │ ← 広い（Gapとして検出）
    │    袋小路     │
    │               │
    └───┐       ┌───┘
        │ 通路 │ ← 狭いが正しい経路
        └──↑───┘
          車両
    ```

    **対策**: グローバルプランナーと組み合わせる
