# config.py 設定リファレンス

config.pyの主要な設定項目を解説します。

---

## 設定ファイルの仕組み

| ファイル | 役割 | git管理 |
|---------|------|---------|
| `config_default.py` | デフォルト設定（原本） | される（編集しない） |
| `config.py` | 個人設定（自由に編集可） | されない |

- `python run.py` 起動時に `config.py` が存在しない場合、`config_default.py` から自動コピーされます
- 設定を変更したい場合は `config.py` を直接編集してください
- デフォルトに戻したい場合は `config.py` を削除して再起動すれば再生成されます

```bash
# デフォルトに戻す
rm config.py
python run.py
# → config_default.py から config.py が再生成される
```

---

## ハンズオン向けクイック設定

### 走行モード一覧

| モード | 説明 | 用途 |
|-------|------|------|
| `manual` | 手動操作 | データ収集 |
| `go_straight` | 直進 | 動作確認 |
| `right_left_3` | 3センサーで障害物回避 | ルールベース走行 |
| `wall_follow` | 壁沿い走行 | 右手法/左手法 |
| `wall_follow_pid` | PID制御壁沿い走行 | PID制御の学習 |
| `nn` | ニューラルネットワーク | センサー値ベースの学習 |
| `donkeycar` | 軽量CNN | 画像ベースの学習（おすすめ） |
| `resnet18` | ResNet18 | 高精度画像認識 |

### 最小限の設定変更

```python
# 1. 走行モードを選択
PLAN = "donkeycar"  # または "manual", "nn" など

# 2. 使用するセンサーを選択
ACTIVE_SENSORS = ["lidar", "camera_0"]  # または ["ultrasonic", "camera_0"]

# 3. 速度を調整
FORWARD_STRAIGHT = 0.4  # 直線用（0.3〜0.5推奨）
FORWARD_CORNER = 0.3    # カーブ用（0.2〜0.4推奨）

# 4. 学習済みモデルを指定（自動走行時）
MODEL_NAME = "donkeycar_20251205_150000.pth"
```

---

## デバイス設定

```python
# デバイス自動検出（通常は変更不要）
DEVICE_TYPE = "auto"  # "rpi4", "rpi5", "jetson", "auto"
```

## 走行モード

```python
# 判断モード選択
PLAN_LIST = [
    "manual",
    "go_straight",
    "right_left_3",
    "right_left_3_records",
    "wall_follow",
    "wall_follow_pid",
    "nn",
    "donkeycar",
    "resnet18",
    "mobilevit_xxs",
    "edgenext_xx_small"
]
PLAN = "right_left_3"
```

## センサー設定

### 超音波センサー

```python
# センサーリスト
ULTRASONIC_SENSOR_LIST = ["FrLH", "FrFR", "FrRH"]

# 測定パラメータ
SAMPLING_TIMES = 3          # サンプリング回数
CUTOFF_RANGE = 2000         # カットオフ距離(mm)
DETECTION_RANGE = 500       # 検知範囲(mm)
STOP_RANGE = 100            # 停止距離(mm)
RIGHT_LEFT_RANGE = 400      # 左右判断距離(mm)
```

### カメラ

```python
# カメラ設定
HAVE_CAMERA = True
CAMERA_WIDTH = 224
CAMERA_HEIGHT = 224
CAMERA_FPS = 30
```

### カメラスロットにLiDAR画像を割り当てる

LiDARが生成するリアルタイム画像（224x224 RGB）をカメラスロットとして利用できます。
これにより、LiDAR画像が記録保存・モニター表示・CNNモデル推論のパイプラインに乗ります。

```python
# LiDAR画像をcamera_0として使用
ACTIVE_SENSORS = ["lidar", "camera_0"]
CAMERA_0_TYPE = "lidar"
```

2カメラ構成でLiDAR画像を2枚目として使う場合:

```python
ACTIVE_SENSORS = ["lidar", "camera_0", "camera_1"]
CAMERA_0_TYPE = None       # 実カメラ（自動検出）
CAMERA_1_TYPE = "lidar"    # LiDAR画像
```

!!! note
    `CAMERA_X_TYPE = "lidar"` を使用する場合、`ACTIVE_SENSORS` に `"lidar"` が含まれている必要があります（LiDARインスタンスがカメラより先に初期化されます）。

### LiDAR

```python
# LiDARタイプ（自動検出 / 手動指定）
LIDAR_TYPE = "AUTO"  # "AUTO", "TMINI", "UST20", "NONE"
```

| 値 | 動作 |
|----|------|
| `"AUTO"` | 起動時にTMINI→UST20の順で自動検出（デフォルト） |
| `"TMINI"` | YDLIDAR TMINIを使用（シリアル接続） |
| `"UST20"` | 北陽 UST-20を使用（Ethernet接続） |
| `"NONE"` | LiDARを使用しない |

`"AUTO"` に設定すると、`run.py` の起動時に以下の順序で検出を行います:

1. シリアルポート（`/dev/ttyAMA0`）が存在すれば **TMINI** と判定
2. TCP接続（デフォルト `192.168.0.139:10940`）に成功すれば **UST20** と判定
3. どちらも見つからなければ **NONE** にフォールバック（警告ログを出力）

検出後、ZONE_INDEX等の機種別パラメータも自動で設定されます。
手動で `"TMINI"` や `"UST20"` を指定した場合は自動検出をスキップします。

## モーター設定

### ステアリング

```python
# ステアリングのPWM値
STEERING_CENTER_PWM = 370
STEERING_WIDTH_PWM = 80
STEERING_RIGHT_PWM = STEERING_CENTER_PWM + STEERING_WIDTH_PWM
STEERING_LEFT_PWM = STEERING_CENTER_PWM - STEERING_WIDTH_PWM
```

### スロットル

```python
# スロットルのPWM値
THROTTLE_STOPPED_PWM = 370
THROTTLE_FORWARD_PWM = 500
THROTTLE_REVERSE_PWM = 300
```

### 出力値

```python
# ステアリング出力（-1〜1）
LEFT = -1
NEUTRAL = 0
RIGHT = 1

# スロットル出力（-1〜1）
FORWARD_S = 0.6       # ストレート速度
FORWARD_C = 0.4       # カーブ速度
STOP = 0
REVERSE = -1
```

## コントローラー設定

```python
# コントローラータイプ
CONTROLLER_TYPE = "joystick"  # "joystick", "pwm", "keyboard"

# ジョイスティック設定
HAVE_JOYSTICK = True
JOYSTICK_STEERING_SCALE = 1.0
JOYSTICK_THROTTLE_SCALE = -1.0
JOYSTICK_DEVICE_FILE = "/dev/input/js0"

# ボタン割り当て
JOYSTICK_A = 0
JOYSTICK_B = 1
JOYSTICK_X = 2
JOYSTICK_Y = 3
JOYSTICK_S = 7

# 軸割り当て
JOYSTICK_AXIS_LEFT = 0
JOYSTICK_AXIS_RIGHT = 4
```

## Follow the Gap 設定

```python
# FTG基本パラメータ
FTG_SAFETY_DISTANCE = 300       # 安全距離 (mm)
FTG_MAX_DISTANCE = 3000         # 最大検出距離 (mm)
FTG_BUBBLE_RADIUS = 150         # 安全バブル半径 (mm)
FTG_DISPARITY_THRESHOLD = 200   # 距離差閾値 (mm)
FTG_ANGLE_START = -90           # 使用角度範囲 開始 (度)
FTG_ANGLE_END = 90              # 使用角度範囲 終了 (度)

# ステアリング制御方式: "linear", "pid", "pure_pursuit"
FTG_STEERING_METHOD = "linear"
FTG_STEERING_GAIN = 1.0         # ステアリングゲイン（全方式共通）
FTG_SMOOTHING_FACTOR = 0.3      # EMAスムージング係数（全方式共通）

# PID制御パラメータ（FTG_STEERING_METHOD = "pid" 時）
FTG_PID_KP = 0.8
FTG_PID_KI = 0.0
FTG_PID_KD = 0.1

# Pure Pursuit パラメータ（FTG_STEERING_METHOD = "pure_pursuit" 時）
FTG_WHEELBASE = 300              # ホイールベース (mm)
FTG_LOOKAHEAD_DISTANCE = 500     # ルックアヘッド距離 (mm)
```

| 方式 | 特徴 |
|------|------|
| `linear` | 角度→ステアリングの線形変換（デフォルト） |
| `pid` | 偏差の蓄積(I)・変化率(D)を加味。振動抑制と定常偏差除去 |
| `pure_pursuit` | 幾何学的追従。大角度で応答が穏やか、lookahead距離で平滑度を調整 |

## PID制御設定

```python
# 壁沿い走行
HAND_SIDE = "right"   # "left", "right"
TARGET_RANGE = 200    # 目標距離(mm)

# PIDパラメータ
K_P = 0.005
K_I = 0.0
K_D = 0.0005
```

## 機械学習設定

```python
# NN有効化
HAVE_NN = True

# モデルパス
MODEL_DIR = "models"
MODEL_NAME = "model.pth"
MODEL_PATH = os.path.join(MODEL_DIR, MODEL_NAME)

# ハイパーパラメータ
HIDDEN_DIM = 64
NUM_HIDDEN_LAYERS = 3
BATCH_SIZE = 8
EPOCHS = 5

# 推論エンジン
INFERENCE_ENGINE = "pytorch"  # "tensorrt", "openvino"

# 正規化
NORMALIZE_RANGE = 2000
```

## データ保存設定

```python
# 保存形式
SAVE_FORMAT = "donkeycar"  # "csv", "donkeycar"

# 保存先
DATA_DIR = "data"

# 終了時に記録フォルダを自動でzip圧縮する
AUTO_ZIP_ON_EXIT = True  # False で無効化
```

## 復帰モード設定

```python
# 復帰モード
RECOVERY_MODE = "back"        # "none", "back"
RECOVERY_STREERING = LEFT
RECOVERY_TIME_DURATION = 1    # 秒
RECOVERY_BRAKING = 1
```

## I2C設定（プロポ用）

```python
# PWMコントローラー
PWM_I2C_ADDRESS = 0x08
PWM_I2C_BUS = 7               # Jetson: 7, RPi: 1

# キャリブレーション値
PWM_CH1_LEFT_RAW = 1098
PWM_CH1_CENTER_RAW = 1519
PWM_CH1_RIGHT_RAW = 1916
PWM_CH2_FORWARD_RAW = 1896
PWM_CH2_NEUTRAL_RAW = 1468
PWM_CH2_REVERSE_RAW = 1098
```

---

## 開発者向け: コード解説

### デバイス自動検出

実行環境に応じて適切なライブラリやパラメータを自動選択します。

#### `detect_device_type` 関数

```python
import os
import platform

def detect_device_type() -> str:
    """
    実行デバイスを自動検出

    Returns:
        str: "rpi4", "rpi5", "jetson", "unknown" のいずれか
    """
    # /proc/device-tree/model からデバイス情報を取得
    model_path = '/proc/device-tree/model'

    if not os.path.exists(model_path):
        return "unknown"

    try:
        with open(model_path, 'r') as f:
            model = f.read().lower()

        if 'raspberry pi 5' in model:
            return "rpi5"
        elif 'raspberry pi 4' in model:
            return "rpi4"
        elif 'raspberry pi' in model:
            return "rpi4"  # 3以前も4と同じ扱い
        elif 'jetson' in model or 'orin' in model:
            return "jetson"
        else:
            return "unknown"

    except Exception:
        return "unknown"


def get_gpio_library(device_type: str):
    """
    デバイスに応じたGPIOライブラリを取得

    Args:
        device_type: デバイスタイプ

    Returns:
        GPIOライブラリモジュール
    """
    if device_type == "rpi5":
        from gpiozero import DigitalOutputDevice, DigitalInputDevice
        return "gpiozero"
    elif device_type in ("rpi4", "rpi3"):
        import RPi.GPIO as GPIO
        return GPIO
    elif device_type == "jetson":
        import Jetson.GPIO as GPIO
        return GPIO
    else:
        raise RuntimeError(f"Unsupported device: {device_type}")
```

#### デバイス別の差異

| 項目 | Raspberry Pi 4 | Raspberry Pi 5 | Jetson Orin Nano |
|------|---------------|----------------|------------------|
| GPIOライブラリ | `RPi.GPIO` | `gpiozero` | `Jetson.GPIO` |
| I2Cバス | 1 | 1 | 7 |
| PWM出力 | ソフトウェア | ハードウェア | ハードウェア |

---

### 設定値の検証

設定値が適切な範囲内かを検証します。

#### `ConfigValidator` クラス

```python
from dataclasses import dataclass
from typing import Any, Optional

@dataclass
class ValidationResult:
    """検証結果"""
    is_valid: bool
    message: str
    corrected_value: Optional[Any] = None


class ConfigValidator:
    """設定値検証クラス"""

    @staticmethod
    def validate_pwm(value: int, name: str, min_val: int = 100, max_val: int = 600) -> ValidationResult:
        """
        PWM値を検証

        Args:
            value: 検証対象の値
            name: 設定項目名
            min_val: 最小許容値
            max_val: 最大許容値

        Returns:
            ValidationResult: 検証結果
        """
        if not isinstance(value, int):
            return ValidationResult(False, f"{name}は整数である必要があります", int(value))

        if value < min_val or value > max_val:
            corrected = max(min_val, min(max_val, value))
            return ValidationResult(
                False,
                f"{name}={value}は範囲外です（{min_val}-{max_val}）",
                corrected
            )

        return ValidationResult(True, "OK")

    @staticmethod
    def validate_normalized(value: float, name: str) -> ValidationResult:
        """
        正規化値（-1〜1）を検証

        Args:
            value: 検証対象の値
            name: 設定項目名

        Returns:
            ValidationResult: 検証結果
        """
        if not isinstance(value, (int, float)):
            return ValidationResult(False, f"{name}は数値である必要があります", 0.0)

        if value < -1.0 or value > 1.0:
            corrected = max(-1.0, min(1.0, float(value)))
            return ValidationResult(
                False,
                f"{name}={value}は範囲外です（-1.0〜1.0）",
                corrected
            )

        return ValidationResult(True, "OK")

    @staticmethod
    def validate_positive(value: float, name: str) -> ValidationResult:
        """
        正の値を検証

        Args:
            value: 検証対象の値
            name: 設定項目名

        Returns:
            ValidationResult: 検証結果
        """
        if value <= 0:
            return ValidationResult(False, f"{name}は正の値である必要があります", abs(value))

        return ValidationResult(True, "OK")


def validate_config(config_module) -> list[ValidationResult]:
    """
    config.pyの全設定を検証

    Args:
        config_module: configモジュール

    Returns:
        list[ValidationResult]: 検証結果のリスト
    """
    validator = ConfigValidator()
    results = []

    # PWM値の検証
    results.append(validator.validate_pwm(
        config_module.STEERING_CENTER_PWM, "STEERING_CENTER_PWM"))
    results.append(validator.validate_pwm(
        config_module.THROTTLE_STOPPED_PWM, "THROTTLE_STOPPED_PWM"))

    # 速度設定の検証
    results.append(validator.validate_normalized(
        config_module.FORWARD_STRAIGHT, "FORWARD_STRAIGHT"))
    results.append(validator.validate_normalized(
        config_module.FORWARD_CORNER, "FORWARD_CORNER"))

    # PIDパラメータの検証
    results.append(validator.validate_positive(
        config_module.K_P, "K_P"))

    return [r for r in results if not r.is_valid]
```

#### 使用例

```python
import config

errors = validate_config(config)
if errors:
    print("設定エラー:")
    for error in errors:
        print(f"  - {error.message}")
        if error.corrected_value is not None:
            print(f"    推奨値: {error.corrected_value}")
```

---

### 環境変数との連携

本番環境と開発環境で設定を切り替えます。

#### 環境変数による上書き

```python
import os

def get_config_value(key: str, default: Any, value_type: type = str) -> Any:
    """
    環境変数で設定値を上書き可能にする

    Args:
        key: 設定キー名
        default: デフォルト値
        value_type: 値の型（str, int, float, bool）

    Returns:
        設定値（環境変数があればその値、なければデフォルト）
    """
    env_key = f"TOGIKAI_{key}"
    env_value = os.environ.get(env_key)

    if env_value is None:
        return default

    if value_type == bool:
        return env_value.lower() in ('true', '1', 'yes')
    elif value_type == int:
        return int(env_value)
    elif value_type == float:
        return float(env_value)
    else:
        return env_value


# config.py での使用例
PLAN = get_config_value("PLAN", "donkeycar")
FORWARD_STRAIGHT = get_config_value("FORWARD_STRAIGHT", 0.4, float)
HAVE_CAMERA = get_config_value("HAVE_CAMERA", True, bool)
```

#### 環境変数の設定方法

```bash
# 一時的な設定（現在のセッションのみ）
export TOGIKAI_PLAN="manual"
export TOGIKAI_FORWARD_STRAIGHT="0.3"

# 永続的な設定（.bashrcに追加）
echo 'export TOGIKAI_PLAN="manual"' >> ~/.bashrc

# 実行時に指定
TOGIKAI_PLAN="nn" python run.py
```

| 環境変数 | 対応するconfig | 用途 |
|---------|---------------|------|
| `TOGIKAI_PLAN` | `PLAN` | 走行モード切り替え |
| `TOGIKAI_FORWARD_STRAIGHT` | `FORWARD_STRAIGHT` | 速度調整 |
| `TOGIKAI_HAVE_CAMERA` | `HAVE_CAMERA` | カメラ有効/無効 |
| `TOGIKAI_MODEL_NAME` | `MODEL_NAME` | モデルファイル指定 |
