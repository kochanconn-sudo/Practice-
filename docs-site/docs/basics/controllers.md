# コントローラー設定

車両の操作には、以下の3種類のコントローラーを使用できます。

## コントローラータイプの選択

config.pyで使用するコントローラーを指定します：

```python
# コントローラータイプの選択
CONTROLLER_TYPE = "joystick"  # "joystick", "pwm", "keyboard"
```

---

## 1. ジョイスティック/ゲームパッド（デフォルト）

USBゲームパッド（例：Logicool F710）を使用して車両を操作します。

### ボタン配置（Logicool F710の例）

![コントローラー](../assets/images/controller.png)

<div style="display: grid; grid-template-columns: auto 1fr; gap: 1.5em; font-size: 0.82em; margin: 1em 0;">
<div>
<b>スティック</b><br>
左（左右）：ステアリング<br>
右（上下）：スロットル
<br><br>
<b>設定</b><br>
MODEボタン：光っていない状態<br>
背面スイッチ：X<br>
Sボタン：モード切替
</div>
<div style="text-align: center;">
<b>ボタン</b>
<div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 2px; max-width: 280px; margin: 0.3em auto;">
<div></div><div style="border: 1px solid #ccc; padding: 0.3em 0.5em; border-radius: 4px;"><b>Y</b> 記録</div><div></div>
<div style="border: 1px solid #ccc; padding: 0.3em 0.5em; border-radius: 4px;"><b>X</b> 速度1</div><div></div><div style="border: 1px solid #ccc; padding: 0.3em 0.5em; border-radius: 4px;"><b>B</b> 速度2</div>
<div></div><div style="border: 1px solid #ccc; padding: 0.3em 0.5em; border-radius: 4px;"><b>A</b> ブレーキ</div><div></div>
</div>
</div>
</div>

### 走行モードの切り替え

**Sボタン**を押すと走行モードが切り替わります：

```
user → auto_str → auto → user → ...
```

| モード | ステアリング | スロットル | 用途 |
|-------|------------|----------|------|
| **user** | 手動 | 手動 | データ収集、練習 |
| **auto_str** | 自動 | 手動 | モデルのステアリングをテスト |
| **auto** | 自動 | 自動 | 完全自動走行 |

!!! tip "ハンズオンでの推奨手順"
    1. **user**モードでデータ収集（Yボタンで記録開始/停止）
    2. data_viewerでデータ確認・学習
    3. **auto_str**モードでステアリングのみテスト
    4. 問題なければ**auto**モードで完全自動走行

### config.py設定

```python
# ジョイスティックの設定
HAVE_JOYSTICK = True
JOYSTICK_STEERING_SCALE = 1.0    # ステアリング感度（-1.0で反転）
JOYSTICK_THROTTLE_SCALE = -1.0   # スロットル感度（-1.0で反転）
JOYSTICK_DEVICE_FILE = "/dev/input/js0"

# ボタン割り当て（F710の場合）
JOYSTICK_A = 0  # ブレーキ
JOYSTICK_B = 1  # アクセル2
JOYSTICK_X = 2  # アクセル1
JOYSTICK_Y = 3  # 記録停止開始
JOYSTICK_S = 7  # 自動/手動走行切り替え

# スティック軸割り当て
JOYSTICK_AXIS_LEFT = 0   # ステアリング（左右）
JOYSTICK_AXIS_RIGHT = 4  # スロットル（上下）

# 一定速度の設定
FORWARD_STRAIGHT = 0.4  # アクセル1（Xボタン）の速度
FORWARD_CORNER = 0.3    # アクセル2（Bボタン）の速度
```

!!! tip "他のゲームパッドを使用する場合"
    ボタン番号を確認するには：
    ```bash
    jstest /dev/input/js0
    ```
    ボタンを押してどの番号が反応するか確認し、config.pyの値を変更してください。

---

## 2. プロポ（PWM信号入力）

プロポ（送信機）からの信号を直接読み取って車両を操作します。

### 機能

- I2C経由でプロポのPWM信号を読み取り（CH1〜CH3 + RPM）
- ジョイスティックと同じインターフェースで操作値を提供
- CH3スイッチによる走行モード切り替え（user → auto_str → auto）
- RPMセンサー値の読み取り
- キャリブレーション機能付き

### 必要なハードウェア

- プロポ送信機・受信機
- Seeed Studio XIAO ESP32S3（I2Cスレーブとして動作）

### ファームウェア書き込み

XIAO ESP32S3にファームウェアを書き込む必要があります。

```
firmware/esp32s3/esp32s3.ino
```

**書き込み手順:**

1. Arduino IDEを開く
2. ボードマネージャから **esp32 by Espressif Systems 2.0.17** をインストール
3. ボード選択: **XIAO_ESP32S3**
4. `firmware/esp32s3/esp32s3.ino` を開いて書き込み

!!! warning "Arduino Board Manager のバージョン"
    **2.0.17** を使用してください。他のバージョンでは `pulseInLong` の動作が異なる場合があります。

### I2Cデータフォーマット

ファームウェアはI2Cスレーブ（アドレス `0x08`）として動作し、レジスタ `0x01` で以下の16バイトを送信します：

| オフセット | サイズ | 内容 |
|-----------|--------|------|
| 0-3 | 4バイト | CH1: ステアリングPWM値（μs） |
| 4-7 | 4バイト | CH2: スロットルPWM値（μs） |
| 8-11 | 4バイト | CH3: モード切替信号（μs） |
| 12-15 | 4バイト | RPM: モーター回転数 |

### CH3によるモード切り替え

プロポの3chスイッチでAI/RCモードを切り替えます。RC→AI側に切り替えるたびに走行モードがローテーションします：

```
user → auto → auto_str → user → ...
```

| CH3信号値 | 状態 |
|----------|------|
| > 1500μs | AIモード（LED: レインボー） |
| 100〜1500μs | RCモード（LED: 緑） |
| < 100μs | 無信号（LED: 青） |

### キャリブレーション

初回セットアップ時に、プロポの操作範囲を測定します：

```bash
# 仮想環境を有効化
source ~/venv/bin/activate

# キャリブレーションモードで実行
python3 pwm_controller.py calibrate
# または短縮形
python3 pwm_controller.py c
```

**操作手順:**

1. プロポのステアリングを左右に最大まで動かす
2. スロットルを前進・後退に最大まで動かす
3. 各位置で1〜2秒間保持
4. Ctrl+Cで終了

終了時に、以下のようなキャリブレーション結果が表示されます：

```
config.pyに設定する値:
PWM_CH1_LEFT_RAW = 1098
PWM_CH1_CENTER_RAW = 1519
PWM_CH1_RIGHT_RAW = 1916
PWM_CH2_FORWARD_RAW = 1896
PWM_CH2_NEUTRAL_RAW = 1468
PWM_CH2_REVERSE_RAW = 1098
```

### config.py設定

```python
# コントローラータイプの選択
CONTROLLER_TYPE = "pwm"  # joystick, pwm, keyboard

# プロポPWM信号設定
PWM_I2C_ADDRESS = 0x08         # PWMコントローラーのI2Cアドレス
PWM_I2C_BUS = 7                # I2Cバス番号（Jetson: 7, RPi: 1）

# キャリブレーション値（上記で測定した値を設定）
PWM_CH1_LEFT_RAW = 1098        # CH1: ステアリング左最大
PWM_CH1_CENTER_RAW = 1519      # CH1: ステアリング中央
PWM_CH1_RIGHT_RAW = 1916       # CH1: ステアリング右最大
PWM_CH2_FORWARD_RAW = 1896     # CH2: スロットル前進最大
PWM_CH2_NEUTRAL_RAW = 1468     # CH2: スロットル中立
PWM_CH2_REVERSE_RAW = 1098     # CH2: スロットル後退最大
```

### 動作確認

```bash
# 監視モード（正規化された値 + RAW値 + CH3 + RPMを表示）
python3 pwm_controller.py monitor

# 簡易テストモード（正規化された値のみ表示）
python3 pwm_controller.py test
```

監視モードの出力例：

```
時刻         Steering   Throttle     RAW1     RAW2     RAW3      RPM Mode
-------
12:34:56        0.000      0.000     1478     1478     1820        0 ai
12:34:56        0.315      0.000     1577     1478     1820      120 ai
```

!!! note "I2C権限について"
    I2Cデバイスにアクセスするには、適切な権限が必要です：
    ```bash
    sudo usermod -aG i2c $USER
    # ログアウト/ログインして反映
    ```

!!! tip "トラブルシューティング"
    PWMコントローラーが検出されない場合：
    ```bash
    # I2Cデバイスを確認
    sudo i2cdetect -y -r 7  # Jetson Orin Nano
    sudo i2cdetect -y -r 1  # Raspberry Pi

    # 0x08にデバイスが表示されることを確認
    ```

---

## 3. キーボード

ジョイスティックやプロポが使用できない場合、キーボードで操作できます。

### 使用方法

config.pyで以下のように設定：

```python
# コントローラータイプの選択
CONTROLLER_TYPE = "keyboard"  # joystick, pwm, keyboard
```

または、マニュアルプラン（`PLAN = "manual"`）の場合は自動的にキーボードモードになります。

### キー配置

**移動操作:**

| キー | 操作 |
|-----|------|
| ↑（上矢印） | 前進 |
| ↓（下矢印） | 後退 |
| ←（左矢印） | 左旋回 |
| →（右矢印） | 右旋回 |
| スペースキー | 停止 |

**記録操作:**

| キー | 操作 |
|-----|------|
| R | 記録開始/停止 |

**モード切り替え:**

| キー | 操作 |
|-----|------|
| M | 走行モード切り替え（user → auto_str → auto） |

**終了:**

| キー | 操作 |
|-----|------|
| Q または ESC | プログラム終了 |

!!! note "キーボードモードの制限"
    - キーボードはオンオフ入力のため、微細な操作ができません
    - データ収集や学習用の走行には、ジョイスティックまたはプロポの使用を推奨します
    - テストや簡易的な動作確認には十分使用できます

---

## コントローラーの比較

| 項目 | ジョイスティック | プロポ | キーボード |
|------|-----------------|--------|-----------|
| **主な利点** | コマンド数が多い | 細かい操作が可能 | 手軽 |
| **詳細** | 記録開始/停止、モード切替、一定速度走行など多機能 | アナログ入力で滑らかな操作 | USBデバイス不要、すぐに使える |
| **推奨用途** | 学習・開発全般 | 精密走行・データ収集 | テスト・デモ |

---

## 開発者向け: コード解説

### ジョイスティック入力処理

Linuxの`evdev`を使用してジョイスティックのイベントを読み取ります。

#### `JoystickController` クラス

```python
import struct
import threading
from typing import Callable

class JoystickController:
    """ジョイスティック入力制御クラス"""

    # イベントタイプ
    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02

    def __init__(self, device_path: str = "/dev/input/js0"):
        """
        Args:
            device_path: ジョイスティックデバイスのパス
        """
        self.device_path = device_path
        self.axes = {}      # {軸番号: 値}
        self.buttons = {}   # {ボタン番号: 状態}
        self.callbacks = {} # {ボタン番号: コールバック関数}
        self._running = False

    def start(self):
        """イベント読み取りスレッドを開始"""
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """イベント読み取りを停止"""
        self._running = False

    def _read_loop(self):
        """イベント読み取りループ"""
        with open(self.device_path, 'rb') as js:
            while self._running:
                # イベント構造体: timestamp(4) + value(2) + type(1) + number(1)
                event = js.read(8)
                if len(event) < 8:
                    continue

                _, value, event_type, number = struct.unpack('IhBB', event)

                if event_type == self.JS_EVENT_AXIS:
                    # 軸の値を-1.0〜1.0に正規化
                    self.axes[number] = value / 32767.0

                elif event_type == self.JS_EVENT_BUTTON:
                    self.buttons[number] = bool(value)
                    # ボタン押下時にコールバック実行
                    if value and number in self.callbacks:
                        self.callbacks[number]()

    def get_axis(self, axis_number: int) -> float:
        """
        軸の現在値を取得

        Args:
            axis_number: 軸番号

        Returns:
            float: -1.0〜1.0の値
        """
        return self.axes.get(axis_number, 0.0)

    def get_button(self, button_number: int) -> bool:
        """
        ボタンの現在状態を取得

        Args:
            button_number: ボタン番号

        Returns:
            bool: 押されていればTrue
        """
        return self.buttons.get(button_number, False)

    def register_callback(self, button_number: int, callback: Callable):
        """
        ボタン押下時のコールバックを登録

        Args:
            button_number: ボタン番号
            callback: 引数なしの関数
        """
        self.callbacks[button_number] = callback
```

#### 使用例

```python
js = JoystickController("/dev/input/js0")
js.register_callback(3, lambda: print("Yボタン押下！"))  # 記録開始/停止
js.start()

while True:
    steering = js.get_axis(0)   # 左スティック左右
    throttle = js.get_axis(4)   # 右スティック上下
    print(f"Steering: {steering:.2f}, Throttle: {throttle:.2f}")
    time.sleep(0.05)
```

---

### デッドゾーン処理

スティックの中央付近のノイズを除去し、滑らかな入力を実現します。

#### `apply_deadzone` 関数

```python
def apply_deadzone(value: float, deadzone: float = 0.05) -> float:
    """
    デッドゾーンを適用

    Args:
        value: 入力値（-1.0〜1.0）
        deadzone: デッドゾーン幅（0.0〜1.0）。デフォルト0.05

    Returns:
        float: デッドゾーン適用後の値（-1.0〜1.0）
    """
    if abs(value) < deadzone:
        return 0.0

    # デッドゾーン外の値を0〜1にリマップ
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def apply_smoothing(current: float, target: float, factor: float = 0.3) -> float:
    """
    指数移動平均によるスムージング

    Args:
        current: 現在の値
        target: 目標値
        factor: スムージング係数（0.0〜1.0）。大きいほど追従が速い

    Returns:
        float: スムージング後の値
    """
    return current + factor * (target - current)
```

#### デッドゾーンの効果

```
入力値:     -1.0  -0.5  -0.05  0.0  0.05  0.5  1.0
                          ↓デッドゾーン↓
適用後:     -1.0  -0.47  0.0   0.0  0.0   0.47 1.0
```

---

### PWMコントローラーのI2C通信

プロポ受信機からのPWM信号をI2C経由で読み取ります。raw I2C（write/read分離トランザクション）でESP32S3と通信します。

#### 通信フロー

```
Jetson (Master)                    ESP32S3 (Slave 0x08)
    |                                    |
    |--- write [0x01] --> STOP           |  onReceive: registerIndex = 0x01
    |                                    |
    |--- read 16 bytes -->               |  onRequest: CH1(4) + CH2(4) + CH3(4) + RPM(4)
    |<-- [data 16 bytes] ---             |
```

#### RAW値と正規化の対応

| 操作 | RAW値例 | 正規化後 |
|------|--------|---------|
| ステアリング左最大 | 1098 | -1.0 |
| ステアリング中央 | 1519 | 0.0 |
| ステアリング右最大 | 1916 | 1.0 |
| スロットル後退 | 1098 | -1.0 |
| スロットル中立 | 1468 | 0.0 |
| スロットル前進 | 1896 | 1.0 |
