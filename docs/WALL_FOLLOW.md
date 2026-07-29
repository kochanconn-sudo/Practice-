# Wall Follow アルゴリズム

## 概要

Wall Follow（壁沿い走行）は、壁との距離を一定に保ちながら走行するアルゴリズムです。右手法（右壁沿い）と左手法（左壁沿い）の切り替えに対応しています。

2つのモードがあります：
- **`wall_follow`**: ルールベースの3状態判定（遠い/近い/適切）
- **`wall_follow_pid`**: PID制御による連続的な距離制御（推奨）

どちらのモードも、オプションで **2点壁角度アライメント補正** を有効にでき、壁との平行度を維持しながら走行できます。

## センサー配置

```
        FrFR (0°)
       /         \
  FrLH (315°)   FrRH (45°)
      |           |
  RrLH (270°)   RrRH (90°)
```

| センサー名 | 角度 | 用途 |
|-----------|------|------|
| FrFR | 0°（前方） | 前方障害物検知 |
| FrRH | 45°（右前方） | 右壁距離（前方側） |
| RrRH | 90°（真右） | 右壁距離（後方側） |
| FrLH | 315°（左前方） | 左壁距離（前方側） |
| RrLH | 270°（真左） | 左壁距離（後方側） |

右手法では FrRH + RrRH、左手法では FrLH + RrLH の2点を使用します。

## アルゴリズムの動作原理

### wall_follow（ルールベース）

壁側2センサーの距離を `TARGET_RANGE ± TARGET_RANGE_ADJUSTMENT` と比較し、3状態で判定します。

```
壁が遠い（両方 > TARGET_RANGE + ADJ）→ 壁側へステアリング、低速
壁が近い（いずれか < TARGET_RANGE - ADJ）→ 壁と反対へステアリング、低速
壁が適切 → 直進、通常速度
  └─ アライメント補正ON時: 壁角度に応じた微調整ステアリング
```

### wall_follow_pid（PID制御）

壁側2センサーの最小距離と `TARGET_RANGE` の偏差を PID 制御でステアリングに変換します。

```
偏差 = min(front_side, rear_side) - TARGET_RANGE

steering = K_P × 偏差 - K_D × 距離変化速度 + K_I × 偏差積分 + 壁角度項

  P項: 壁から離れていれば壁側へ、近ければ反対側へ
  D項: 急速に壁に近づいている場合に抑制
  I項: 定常偏差の除去（通常は0で運用）
  壁角度項: 壁との平行度を補正（アライメント補正ON時）
```

## 壁角度アライメント補正

### なぜ必要か

距離のみの制御では、車両が壁に対して**斜め**に走行していても気づけません。壁に平行でない場合、徐々に壁に近づく/離れる動作が発生し、ジグザグ走行の原因になります。

壁側の2つのセンサー（例: FrRH と RrRH）の距離差から壁に対する車両の角度を算出し、平行走行を維持します。

### 壁角度の幾何学

車両座標系（x=右, y=前方）で、2センサーの測定点から壁方向ベクトルを求めます。

**右手法（FrRH + RrRH）：**
```
P1 = (d1 × sin45°, d1 × cos45°)    # FrRH の測定点
P2 = (d2, 0)                         # RrRH の測定点

壁ベクトル = P1 - P2
wall_angle = atan2(d1×sin45° - d2, d1×cos45°)
```

**左手法（FrLH + RrLH）：**
```
P1 = (-d1×sin45°, d1×cos45°)        # FrLH の測定点
P2 = (-d2, 0)                        # RrLH の測定点

壁ベクトル = P1 - P2
wall_angle = atan2(-d1×sin45° + d2, d1×cos45°)
```

### 角度の意味

| wall_angle | 状態 | 補正方向（右手法） |
|-----------|------|-----------------|
| 0 | 壁と平行 | 補正なし |
| 負（< 0） | ノーズが壁に近づいている | 壁から離れる方向へ |
| 正（> 0） | ノーズが壁から離れている | 壁に近づく方向へ |

```
  壁 =====================

  angle < 0        angle = 0       angle > 0
   ╲                 │                ╱
    ╲  車両          │  車両         ╱  車両
     ╲               │              ╱
  ノーズが壁へ     平行          ノーズが離れる
```

### wall_follow での補正

距離判定が「適切」（NEUTRAL）の場合のみ角度補正を適用します。壁が近い/遠い場合は距離ベースの制御が優先されます。

```python
if self.steering == NEUTRAL:
    self.steering = clamp(K_ANGLE × wall_angle, -1, 1)
    if |wall_angle| > 0.1:
        throttle = FORWARD_CORNER  # 角度が大きい場合は減速
```

### wall_follow_pid での補正

PID出力に壁角度項を加算します。距離制御と角度制御が常に同時に動作します。

```python
steering = K_P × delta_dis - K_D × v + K_I × integral + K_ANGLE × wall_angle
```

## 設定パラメータ

`config.py`で以下のパラメータを調整できます：

```python
# ============================================================================
# 各種走行モード固有のパラメータ
# ============================================================================
# wall_follow モード
HAND_SIDE = "right"  # "right" or "left"

# wall_follow 壁角度アライメント
WALL_FOLLOW_USE_ALIGNMENT = True    # 2点壁角度補正を有効にする
WALL_FOLLOW_K_ANGLE = 0.5           # 壁角度補正ゲイン（rad→steering変換）

# 壁沿い走行の目標距離
TARGET_RANGE = 200             # 目標距離 (mm)
TARGET_RANGE_ADJUSTMENT = 25   # 目標距離付近での操作変更基準値（±mm）

# PIDパラメータ（wall_follow_pid用）
K_P = 0.005    # 比例ゲイン
K_I = 0.0      # 積分ゲイン（通常0）
K_D = 0.0005   # 微分ゲイン

# スロットル出力
FORWARD_STRAIGHT = 0.6  # 直進時
FORWARD_CORNER = 0.3    # カーブ時
```

### パラメータの詳細

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `HAND_SIDE` | "right" | 壁沿いの方向。"right"で右壁沿い、"left"で左壁沿い。 |
| `WALL_FOLLOW_USE_ALIGNMENT` | True | 2点壁角度アライメント補正の有効/無効。 |
| `WALL_FOLLOW_K_ANGLE` | 0.5 | 壁角度補正ゲイン。大きくすると角度補正が強くなる。 |
| `TARGET_RANGE` | 200mm | 壁との目標距離。コースの幅に応じて調整。 |
| `TARGET_RANGE_ADJUSTMENT` | 25mm | 「適切」と判定する許容範囲（±）。小さくすると厳密になる。 |
| `K_P` | 0.005 | PID比例ゲイン。壁との距離偏差に対する応答速度。 |
| `K_I` | 0.0 | PID積分ゲイン。定常偏差の除去。通常は0で運用。 |
| `K_D` | 0.0005 | PID微分ゲイン。急激な距離変化の抑制。 |
| `FORWARD_STRAIGHT` | 0.6 | 直進時のスロットル値。 |
| `FORWARD_CORNER` | 0.3 | カーブ時のスロットル値。 |

## 2つのモードの比較

| 特性 | wall_follow | wall_follow_pid |
|------|------------|-----------------|
| 制御方式 | 3状態ルール判定 | PID連続制御 |
| ステアリング出力 | -1, 0, +1 の離散値 | -1.0〜+1.0 の連続値 |
| 距離精度 | ±TARGET_RANGE_ADJUSTMENT | 連続的に目標距離を追従 |
| 角度補正の適用 | NEUTRAL時のみ | PID出力に常時加算 |
| 滑らかさ | ステアリングが急に切り替わる | 滑らかな操舵 |
| 推奨用途 | テスト・デバッグ | 実車走行 |

## 使用方法

### 1. 基本設定

`config.py`で以下を設定：

```python
# プランを選択
PLAN = "wall_follow_pid"  # または "wall_follow"

# 壁沿い方向
HAND_SIDE = "right"  # 右壁沿い

# 壁角度アライメント補正を有効化
WALL_FOLLOW_USE_ALIGNMENT = True
WALL_FOLLOW_K_ANGLE = 0.5
```

### 2. センサーの確認

壁沿い走行には壁側の2センサーが必要です：

```python
# 超音波センサーの場合: 5つ全て有効にする
ULTRASONIC_SENSOR_LIST = ["RrLH", "FrLH", "FrFR", "FrRH", "RrRH"]

# LiDARの場合: 5ゾーン全てが利用可能であること
ZONE_NAMES = ["RrLH", "FrLH", "FrFR", "FrRH", "RrRH"]
```

> **注意**: 後方側面センサー（RrRH/RrLH）が利用できない場合、前方側面センサーの値がフォールバックとして使用されます。この場合、角度補正は常に0になります。

### 3. 実行

```bash
python run.py
```

### 4. モニターUI での調整

ブラウザで `http://<device-ip>:8000` を開き、Config セクションから以下を調整できます：
- **HAND_SIDE**: 壁沿い方向の切り替え
- **ALIGNMENT**: 角度補正のON/OFF
- **K_ANGLE**: 角度補正ゲインの調整
- **K_P / K_I / K_D**: PIDゲインの調整
- **TARGET_RANGE**: 目標距離の変更

## ファイル構成

```
togikaidrive-dev/
├── planner.py             # Wall Followアルゴリズム本体
│   ├── _calc_wall_angle()       # 壁角度計算
│   ├── wall_follow()            # ルールベース壁沿い走行
│   └── wall_follow_pid()        # PID壁沿い走行
├── config.py              # 設定ファイル（HAND_SIDE, K_P/I/D, K_ANGLE等）
├── run.py                 # メイン実行ファイル
├── monitor.py             # WebモニターUI（パラメータ変更対応）
├── templates/
│   └── monitor.html       # モニターUIテンプレート
└── docs/
    └── WALL_FOLLOW.md     # このドキュメント
```

## チューニングガイド

### 壁から離れすぎる場合
- `TARGET_RANGE` を減少（例: 200 → 150）
- `K_P` を増加（例: 0.005 → 0.008）

### 壁に近づきすぎる場合
- `TARGET_RANGE` を増加（例: 200 → 300）
- `K_D` を増加して急接近を抑制（例: 0.0005 → 0.001）

### ジグザグ走行する場合
- `WALL_FOLLOW_USE_ALIGNMENT` を `True` にして角度補正を有効化
- `K_ANGLE` を調整（0.3〜0.8 の範囲で試す）
- PIDモードの場合、`K_D` を増加して振動を抑制

### 斜めに走行し続ける場合
- `WALL_FOLLOW_USE_ALIGNMENT` が `True` であることを確認
- `K_ANGLE` を増加（例: 0.5 → 0.8）
- 後方側面センサー（RrRH/RrLH）が正しく動作しているか確認

### カーブで壁にぶつかる場合
- `FORWARD_CORNER` を減少（例: 0.3 → 0.2）して低速化
- `TARGET_RANGE` を増加して壁との余裕を確保

### ステアリングが過敏な場合（PIDモード）
- `K_P` を減少（例: 0.005 → 0.003）
- `K_ANGLE` を減少（例: 0.5 → 0.3）
- `K_D` を増加して変動を抑制

## 動作例

### シナリオ1: 壁と平行に走行中
- FrRH = 200mm, RrRH = 283mm（≈200/sin45°）
- wall_angle ≈ 0°
- 偏差 ≈ 0
- 結果: `steering ≈ 0.0`（直進維持）

### シナリオ2: ノーズが壁に向いている
- FrRH = 200mm, RrRH = 300mm（後方が壁から遠い）
- wall_angle < 0（負の値）
- 結果: `steering < 0`（壁から離れる方向に補正）

### シナリオ3: 壁から離れつつある
- FrRH = 400mm, RrRH = 200mm（前方が壁から遠い）
- wall_angle > 0（正の値）
- 結果: `steering > 0`（壁に近づく方向に補正）

### シナリオ4: 壁が遠い
- FrRH = 500mm, RrRH = 500mm（両方とも TARGET_RANGE + ADJ 以上）
- 距離判定: 「壁が遠い」
- 結果: `steering = RIGHT`（壁側へ最大旋回）

## トラブルシューティング

### 問題: 角度補正が効かない
- `WALL_FOLLOW_USE_ALIGNMENT` が `True` か確認
- 後方側面センサー（RrRH/RrLH）のデータが取得できているか確認
- モニターUIの Realtime Data でセンサー値を確認

### 問題: wall_follow で角度補正が効かない（wall_follow_pid では効く）
- wall_follow ではNEUTRAL判定時のみ補正が適用されます
- 壁距離が TARGET_RANGE ± ADJ の範囲内にあることを確認

### 問題: 常に同じ方向に曲がる
- `HAND_SIDE` の設定が走行する壁の方向と一致しているか確認
- センサーの取り付け角度が正しいか確認

### 問題: PIDが発散する
- `K_I` を 0 にリセット（積分項のワインドアップ）
- `K_P` を下げてみる
- `K_D` を上げて振動を抑制

## 更新履歴

| 日付 | 変更内容 |
|------|---------|
| 2026-02-26 | 2点壁角度アライメント補正機能を追加、初版作成 |
