# Follow the Gap アルゴリズム

## 概要

Follow the Gap（FTG）は、LiDARセンサーデータを使用して障害物を回避しながら自律走行するアルゴリズムです。スキャンデータから最も広い「ギャップ」（障害物のない空間）を見つけ、その方向に車両を向けます。

## アルゴリズムの動作原理

### 1. LiDARデータの前処理
- 生のLiDARスキャンデータを取得
- 指定された角度範囲（デフォルト: -90度〜90度）のデータを抽出
- 無効な値（0や負の値）を最大距離で置換

### 2. 安全バブルの適用
- 最も近い障害物を検出
- その周囲に「安全バブル」を適用し、通行不可領域としてマーク
- 衝突リスクを低減

### 3. 距離不連続点（Disparity）の拡張
- 急激な距離変化がある箇所を検出
- 近い側の値を周囲に拡張して安全マージンを確保
- 狭い隙間への進入を防止

### 4. ギャップの検出
- 安全距離以上の連続領域（ギャップ）を識別
- 複数のギャップがある場合、以下の基準でスコアリング:
  - ギャップの幅
  - 平均距離
  - 中央（前方）への近さ

### 5. 目標角度の決定
- 最も高スコアのギャップを選択
- ギャップ内で最も遠い点かつ中央に近い点を目標に設定

### 6. ステアリング計算
- 目標角度をステアリング値（-1.0〜1.0）に変換
- 3つの制御方式から選択可能（linear / pid / pure_pursuit）
- スムージングを適用して急激な操舵を抑制

## 設定パラメータ

`config.py`で以下のパラメータを調整できます：

```python
# ============================================================================
# Follow the Gap 設定
# ============================================================================
FTG_SAFETY_DISTANCE = 300       # 安全距離 (mm) - この距離以下は障害物とみなす
FTG_MAX_DISTANCE = 3000         # 最大検出距離 (mm) - これ以上は無限遠とみなす
FTG_BUBBLE_RADIUS = 150         # 安全バブル半径 (mm) - 最近接障害物周りの安全マージン
FTG_DISPARITY_THRESHOLD = 200   # 距離差閾値 (mm) - この差以上で不連続点とみなす
FTG_ANGLE_START = -90           # スキャン開始角度 (度) - 負が左側
FTG_ANGLE_END = 90              # スキャン終了角度 (度) - 正が右側

# ステアリング制御方式: "linear", "pid", "pure_pursuit"
FTG_STEERING_METHOD = "linear"
FTG_STEERING_GAIN = 1.0         # ステアリングゲイン (0.5-2.0推奨、全方式共通)
FTG_SMOOTHING_FACTOR = 0.3      # スムージング係数 (0.0-1.0、全方式共通) - 大きいほど応答が速い

# PID制御パラメータ
FTG_PID_KP = 0.8
FTG_PID_KI = 0.0
FTG_PID_KD = 0.1

# Pure Pursuit パラメータ
FTG_WHEELBASE = 300              # ホイールベース (mm)
FTG_LOOKAHEAD_DISTANCE = 500     # ルックアヘッド距離 (mm)
```

### パラメータの詳細

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `FTG_SAFETY_DISTANCE` | 300mm | この距離以下の点は障害物として扱われます。値を大きくすると早めに回避動作を開始します。 |
| `FTG_MAX_DISTANCE` | 3000mm | これより遠い距離は無限遠として扱われます。LiDARの最大検出距離に合わせて調整してください。 |
| `FTG_BUBBLE_RADIUS` | 150mm | 最も近い障害物の周りに設定する安全マージンです。車両幅の半分程度を推奨。 |
| `FTG_DISPARITY_THRESHOLD` | 200mm | 隣接点間の距離差がこの値を超えると不連続点として処理されます。 |
| `FTG_ANGLE_START` | -90度 | スキャンデータの使用開始角度。負の値は左側を意味します。 |
| `FTG_ANGLE_END` | 90度 | スキャンデータの使用終了角度。正の値は右側を意味します。 |
| `FTG_STEERING_METHOD` | "linear" | ステアリング制御方式。"linear"、"pid"、"pure_pursuit"から選択。 |
| `FTG_STEERING_GAIN` | 1.0 | ステアリングの感度（全方式共通）。値を大きくするとより大きく曲がります。 |
| `FTG_SMOOTHING_FACTOR` | 0.3 | ステアリングのスムージング（全方式共通）。0に近いほど滑らか、1に近いほど即応性が高い。 |
| `FTG_PID_KP` | 0.8 | PID制御の比例ゲイン。 |
| `FTG_PID_KI` | 0.0 | PID制御の積分ゲイン。 |
| `FTG_PID_KD` | 0.1 | PID制御の微分ゲイン。 |
| `FTG_WHEELBASE` | 300mm | Pure Pursuit用のホイールベース。車両の前後輪間距離。 |
| `FTG_LOOKAHEAD_DISTANCE` | 500mm | Pure Pursuit用のルックアヘッド距離。大きいほど滑らか。 |

## ステアリング制御方式

`FTG_STEERING_METHOD` で3つの制御方式を切り替えられます。

### linear（デフォルト）

目標角度をステアリング値に線形マッピングします。シンプルで安定していますが、定常偏差の補正機能はありません。

```python
FTG_STEERING_METHOD = "linear"
```

### pid

PID制御により、偏差の蓄積（I項）と変化率（D項）を加味します。振動抑制と定常偏差の除去が可能です。

```python
FTG_STEERING_METHOD = "pid"
FTG_PID_KP = 0.8   # 比例ゲイン（応答速度）
FTG_PID_KI = 0.0   # 積分ゲイン（定常偏差除去）
FTG_PID_KD = 0.1   # 微分ゲイン（振動抑制）
```

### pure_pursuit

幾何学的追従アルゴリズムです。大角度でも応答が穏やか（sin特性）で、ルックアヘッド距離で平滑度を調整できます。

```python
FTG_STEERING_METHOD = "pure_pursuit"
FTG_WHEELBASE = 300            # ホイールベース (mm)
FTG_LOOKAHEAD_DISTANCE = 500   # ルックアヘッド距離 (mm) - 大きいほど滑らか
```

### 3方式の比較

| 方式 | 入力 | 特徴 |
|------|------|------|
| **linear** | target_angle | 角度→ステアリングの線形変換。シンプルだが定常偏差の補正なし |
| **pid** | target_angle（=誤差） | 偏差の蓄積(I)・変化率(D)を加味。振動抑制と定常偏差除去 |
| **pure_pursuit** | target_angle + target_distance | 幾何学的追従。大角度で応答が穏やか、lookahead距離で平滑度を調整 |

## 使用方法

### 1. 基本設定

`config.py`で以下を設定：

```python
# LiDARを有効化
ACTIVE_SENSORS = ["lidar", ...]

# プランをfollow_the_gapに設定
PLAN = "follow_the_gap"
```

### 2. LiDARの設定

LiDARが正しく設定されていることを確認：

```python
# LiDARタイプの選択
LIDAR_TYPE = "TMINI"  # または "UST20"

# LiDAR角度設定（LiDARの仕様に合わせる）
LIDAR_ANGLE_START = -135  # LiDARのスキャン開始角度
LIDAR_ANGLE_END = 135     # LiDARのスキャン終了角度
```

### 3. 実行

```bash
python run.py
```

## ファイル構成

```
togikaidrive-dev/
├── follow_the_gap.py      # Follow the Gapアルゴリズム本体
├── config.py              # 設定ファイル（FTG_*パラメータ）
├── planner.py             # プランナー（follow_the_gapプランの呼び出し）
├── run.py                 # メイン実行ファイル
└── docs/
    └── FOLLOW_THE_GAP.md  # このドキュメント
```

## クラス・関数リファレンス

### FollowTheGap クラス

```python
from follow_the_gap import FollowTheGap

ftg = FollowTheGap()
steering, throttle = ftg.compute(lidar_data)
```

#### メソッド

| メソッド | 説明 |
|---------|------|
| `__init__()` | config.pyからパラメータを読み込んで初期化 |
| `preprocess_lidar(measurements)` | LiDARデータの前処理 |
| `find_closest_point(distances)` | 最も近い障害物を検出 |
| `apply_safety_bubble(distances, closest_idx)` | 安全バブルを適用 |
| `extend_disparities(distances, angles)` | 距離不連続点を拡張 |
| `find_best_gap(distances, angles, original_distances)` | 最適なギャップを検出 |
| `calculate_steering(target_angle, target_distance)` | ステアリング値を計算（方式に応じて分岐） |
| `calculate_throttle(distances, steering)` | スロットル値を計算 |
| `compute(lidar_data)` | メイン計算（上記を順に実行） |

### 便利関数

```python
from follow_the_gap import follow_the_gap

# シンプルな呼び出し
steering, throttle = follow_the_gap(lidar_data)
```

## チューニングガイド

### 回避動作が遅い場合
- `FTG_SAFETY_DISTANCE` を増加（例: 300 → 400）
- `FTG_STEERING_GAIN` を増加（例: 1.0 → 1.5）

### 回避動作が過敏な場合
- `FTG_SAFETY_DISTANCE` を減少（例: 300 → 200）
- `FTG_SMOOTHING_FACTOR` を減少（例: 0.3 → 0.1）

### 狭い通路でうまく走れない場合
- `FTG_BUBBLE_RADIUS` を減少（例: 150 → 100）
- `FTG_DISPARITY_THRESHOLD` を増加（例: 200 → 300）

### ステアリングが振動する場合
- `FTG_SMOOTHING_FACTOR` を減少（例: 0.3 → 0.1）
- `FTG_STEERING_METHOD = "pid"` に変更し、`FTG_PID_KD` で微分制御を追加

### 滑らかなカーブ追従が必要な場合
- `FTG_STEERING_METHOD = "pure_pursuit"` を使用
- `FTG_LOOKAHEAD_DISTANCE` を調整（大きいほど滑らか、小さいほど応答性が高い）

## 動作例

### シナリオ1: 前方に障害物なし
- 全方向が安全距離以上
- 最大のギャップ = 全範囲
- 目標角度 = 0度（直進）
- 結果: `steering ≈ 0.0`

### シナリオ2: T字路（前方が壁）
- 前方が安全距離以下
- 左右にギャップが存在
- より広いまたは中央に近いギャップを選択
- 結果: `steering > 0` または `steering < 0`

### シナリオ3: 右カーブ
- 前方と左が障害物
- 右側のみギャップ
- 目標角度 = 正の角度
- 結果: `steering > 0`（右旋回）

## トラブルシューティング

### 問題: LiDARデータが取得できない
- `ACTIVE_SENSORS`に`"lidar"`が含まれているか確認
- LiDARの接続とシリアルポート設定を確認
- `LIDAR_TYPE`が正しいか確認

### 問題: 全く回避しない
- `FTG_SAFETY_DISTANCE`が適切か確認（障害物の距離より小さい可能性）
- LiDARデータが正しく取得されているかログで確認

### 問題: 常に同じ方向に曲がる
- `FTG_ANGLE_START`と`FTG_ANGLE_END`が対称か確認
- LiDARの取り付け角度を確認

## 参考文献

- [The Disparity Extender Algorithm](https://f1tenth-coursekit.readthedocs.io/en/stable/lectures/ModuleC/lecture12.html) - F1TENTH
- [Follow the Gap Method](https://arxiv.org/abs/1710.11177) - Gap Following for Autonomous Racing

## 更新履歴

| 日付 | 変更内容 |
|------|---------|
| 2026-02-23 | ステアリング制御方式の選択機能追加（linear/pid/pure_pursuit） |
| 2026-01-23 | 初版作成 |
