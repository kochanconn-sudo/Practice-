# 環境構築

togikidriveの環境構築手順を解説します。

## 対応デバイス

| デバイス | GPIO | 推奨用途 |
|---------|------|---------|
| Raspberry Pi 4 | RPi.GPIO | 入門・学習 |
| Raspberry Pi 5 | gpiozero | 入門・学習 |
| Jetson Orin Nano | Jetson.GPIO | 高度な画像認識 |

## 基本セットアップ

### 1. リポジトリのクローン

```bash
git clone https://github.com/autonomous-minicar-battle/togikaidrive-dev.git
cd togikaidrive-dev
```

### 2. 設定ファイルについて

初回の `python run.py` 実行時に、`config_default.py` から `config.py` が自動生成されます。
設定を変更する場合は `config.py` を直接編集してください。`config_default.py` は原本なので編集しないでください。

デフォルト設定に戻したい場合は `config.py` を削除して再起動するだけでOKです。

```bash
rm config.py
python run.py  # config_default.py から再生成される
```

### 3. 仮想環境の作成

```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
```

次回ログイン時に自動で仮想環境を有効化するには、`.bashrc` に以下を追記します:

```bash
echo 'source ~/togikaidrive-dev/venv/bin/activate' >> ~/.bashrc
```

### 4. 依存パッケージのインストール

プラットフォームを自動検出し、依存パッケージのインストールと `config.py` の生成を行います:

```bash
bash setup/install.sh
```

手動で実行する場合:

```bash
sudo apt-get update
sudo apt-get install -y libcap-dev
pip install -r setup/requirements-rpi.txt
cp config_default.py config.py
```

---

## Raspberry Pi セットアップ

### GPIO権限の設定

```bash
sudo usermod -aG gpio $USER
# ログアウト/ログインして反映
```

### I2C有効化（プロポ使用時）

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

### UART有効化（LiDAR使用時）

LiDARセンサー（YDLidar等）をGPIOのシリアルポート（`/dev/ttyAMA0`）で接続する場合、UARTの設定が必要です。
Raspberry PiではデフォルトでシリアルポートがBluetoothに割り当てられているため、GPIOで使用するには切り替えが必要です。

```bash
sudo nano /boot/firmware/config.txt
```

末尾に以下を追加:

```ini
# UART enable
enable_uart=1

# Use PL011 for GPIO (disable Bluetooth)
dtoverlay=disable-bt
```

設定後、再起動して反映します:

```bash
sudo reboot
```

再起動後、デバイスが認識されていることを確認します:

```bash
ls -l /dev/ttyAMA0
# crw-rw---- 1 root dialout 204, 64 ... /dev/ttyAMA0 と表示されればOK
```

!!! warning "Bluetooth無効化"
    `dtoverlay=disable-bt` によりBluetoothが無効になります。Bluetoothコントローラーを使用する場合はUSB接続にしてください。

---

## Jetson Orin Nano セットアップ

### 前提条件

- JetPack 6.1 または 6.2 がインストール済み
- Python 3.10.12 がプリインストール済み

### JetPack確認

```bash
cat /etc/nv_tegra_release
```

### 仮想環境のセットアップ

```bash
# 必要なパッケージをインストール
sudo apt update
sudo apt install -y python3.10-venv python3.10-dev

# user home下に作成
cd /home/jetson

# 仮想環境を作成（Python 3.10を使用、システムパッケージを参照）
python3.10 -m venv --system-site-packages venv

# 仮想環境を有効化
source venv/bin/activate

# pipをアップグレード
pip install --upgrade pip
```

次回ログイン時に自動で仮想環境を有効化するには、`.bashrc` に以下を追記します:

```bash
echo 'source /home/jetson/venv/bin/activate' >> ~/.bashrc
```

### UART有効化（LiDAR使用時）

Jetson Orin NanoではUARTはデフォルトで有効ですが、シリアルデバイスへのアクセス権限が必要です。

```bash
# dialoutグループに追加（シリアルポートへのアクセス権限）
sudo usermod -aG dialout $USER
# ログアウト/ログインして反映
```

デバイスが認識されていることを確認します:

```bash
ls -l /dev/ttyTHS1
# crw-rw---- 1 root dialout 238, 1 ... /dev/ttyTHS1 と表示されればOK
```

!!! note "Jetsonのシリアルポート"
    Jetson Orin Nanoでは `/dev/ttyTHS1` がGPIOのUARTポートです（Raspberry Piの `/dev/ttyAMA0` に相当）。config.pyの `LIDAR_SERIAL_PORT` で指定します。

### 依存パッケージのインストール

プラットフォームを自動検出し、依存パッケージのインストールと `config.py` の生成を行います:

```bash
bash setup/install.sh
```

手動で実行する場合:

```bash
pip install -r setup/requirements-jetson.txt
cp config_default.py config.py
```

!!! note "requirements-jetson.txtの内容"
    以下のパッケージが含まれています：

    - Web Framework (Flask, tornado等)
    - Jetson GPIO
    - Hardware Control (Adafruit-PCA9685, smbus, pmw3901等)
    - Data Processing (numpy, pandas, scipy, matplotlib等)
    - Utilities (onnx, pygame, pillow等)

    PyTorch、torchvision、OpenCVなど特別な対応が必要なパッケージは、以下の個別手順でインストールします。

### GPIO権限の設定

```bash
sudo groupadd -f -r gpio
sudo usermod -aG gpio $USER
sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# ログアウト/ログインして反映
```

JetPack 6.2 で利用する場合は、追加の設定が必要な場合があります：

- 参考: [FaBo JetsonGPIO 設定ガイド](https://github.com/FaBoPlatform/FaBo/tree/master/0608_donkeycar/JetsonGPIO/ai_car_board)

### I2C権限の設定

```bash
sudo usermod -aG i2c $USER
# ログアウト/ログインして反映
```

### PyTorchのインストール（NVIDIA公式Wheel）

```bash
# PyTorch 2.5.0 for Jetson（JetPack 6.1/6.2共通、Python 3.10用）
pip3 install --no-cache https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```

!!! warning "Python バージョンに注意"
    NVIDIA の PyTorch wheel は Python 3.10 用です。必ず Python 3.10 の仮想環境を使用してください。

### libcusparseLtのインストール（必須）

PyTorchの実行に必要なCUDAライブラリをインストールします：

```bash
# cuSPARSELt パッケージをダウンロード・インストール
wget https://developer.download.nvidia.com/compute/cusparselt/0.6.3/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.6.3_1.0-1_arm64.deb
sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.6.3_1.0-1_arm64.deb
sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.6.3/cusparselt-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev
```

参考: [NVIDIA cuSPARSELt Downloads](https://developer.nvidia.com/cusparselt-downloads?target_os=Linux&target_arch=aarch64-jetson&Compilation=Native&Distribution=Ubuntu&target_version=22.04&target_type=deb_local)

### torchvisionのインストール（ソースからビルド）

```bash
# 依存パッケージをインストール
sudo apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev \
    libavcodec-dev libavformat-dev libswscale-dev

# torchvision をクローン
git clone --branch release/0.20 https://github.com/pytorch/vision torchvision
cd torchvision

# ビルドしてインストール
export BUILD_VERSION=0.20.0
python3 setup.py install

cd ..
```

### OpenCVの確認

Jetson には OpenCV がプリインストールされています：

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

### インストール確認

```bash
# Python とパッケージのバージョン確認
python --version
pip list

# PyTorch の動作確認
python -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"

# GPIO ライブラリの確認
python -c "import Jetson.GPIO; print('Jetson.GPIO OK')"
```

---

## ハードウェア接続

### 超音波センサー（HC-SR04）

| センサーピン | Raspberry Pi | Jetson |
|------------|--------------|--------|
| VCC | 5V | 5V |
| GND | GND | GND |
| TRIG | GPIO指定 | GPIO指定 |
| ECHO | GPIO指定 | GPIO指定 |

!!! warning "電圧に注意"
    HC-SR04のECHOピンは5V出力です。Raspberry Pi/JetsonのGPIOは3.3V入力のため、分圧回路が必要な場合があります。

### サーボ/ESC（PCA9685経由）

| PCA9685 | Raspberry Pi | Jetson |
|---------|--------------|--------|
| VCC | 3.3V | 3.3V |
| GND | GND | GND |
| SDA | GPIO2 (SDA) | Pin 3 (SDA) |
| SCL | GPIO3 (SCL) | Pin 5 (SCL) |
| V+ | 外部電源 | 外部電源 |

### カメラ

**Raspberry Pi:**
- CSIカメラまたはUSBカメラ

```bash
# カメラの接続確認
rpicam-hello --list-cameras
```

**Jetson:**
- CSIカメラ（推奨）またはUSBカメラ

---

## YDLIDAR TMINI セットアップ

YDLIDAR TMINIを使用する場合の設定手順です。

### 1. YDLidar SDKのインストール

```bash
# 必要なパッケージをインストール
sudo apt-get update
sudo apt-get install cmake
sudo apt-get install swig python3-dev

# SDKをクローン
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK

# ビルド
mkdir build
cd build
cmake ..
make
sudo make install

# Pythonバインディングをインストール
cd ..
pip install .
```

### 2. シリアルポートの設定

YDLIDAR TMINIはシリアル（UART）接続を使用します。

**Raspberry Pi:**

```bash
# Bluetoothを無効化してハードウェアUARTを開放
sudo raspi-config
# Interface Options → Serial Port
# - login shell over serial: No
# - serial port hardware: Yes

# /boot/firmware/config.txtに追記
echo "dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt

# 再起動
sudo reboot
```

**シリアルポートの権限設定:**

```bash
# dialoutグループに追加
sudo usermod -aG dialout $USER
# ログアウト/ログインして反映
```

### 3. udevルールの設定（推奨）

デバイス名を固定するためのudevルールを設定します。

```bash
# udevルールファイルを作成
sudo nano /etc/udev/rules.d/99-ydlidar.rules
```

以下の内容を追加:
```
KERNEL=="ttyAMA0", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="ydlidar"
```

```bash
# ルールを適用
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4. config.pyの設定

```python
# LiDARを有効化
HAVE_LIDAR = True
LIDAR_TYPE = "TMINI"

# シリアルポート設定
LIDAR_SERIAL_PORT = "/dev/ttyAMA0"  # RPiのハードウェアUART
LIDAR_SERIAL_BAUDRATE = 230400

# アクティブセンサーにLiDARを追加
ACTIVE_SENSORS = ["lidar"]  # または ["ultrasonic", "lidar"]
```

### 5. 接続図

| YDLIDAR TMINI | Raspberry Pi |
|---------------|--------------|
| TX | GPIO15 (RXD) |
| RX | GPIO14 (TXD) |
| VCC | 5V |
| GND | GND |

!!! warning "電源に注意"
    YDLIDAR TMINIは5V電源が必要です。GPIO5Vピン、USB電源からの供給、または別電源を使用してください。

### 6. 動作確認

```bash
# シリアルポートの確認
ls -la /dev/ttyAMA0

# LiDAR単体テスト
python lidar.py
```

ブラウザで `http://localhost:8080` にアクセスして点群データを確認できます。

---

## 動作確認

### 1. 超音波センサー

```bash
python ultrasonic.py
```

### 2. モーター

```bash
python motor.py
```

### 3. カメラ

```bash
python camera.py
```

### 4. LiDAR（YDLIDAR TMINIの場合）

```bash
python lidar.py
```

ブラウザで `http://localhost:8080` にアクセスして確認。

### 5. 統合テスト

```bash
python run.py
```

---

## トラブルシューティング

### GPIOアクセスエラー

```
RuntimeError: No access to /dev/mem
```

**対策:**
```bash
sudo usermod -aG gpio $USER
# ログアウト/ログイン
```

### I2Cデバイスが見つからない

```bash
# デバイス確認
sudo i2cdetect -y 1  # Raspberry Pi
sudo i2cdetect -y 7  # Jetson
```

### カメラが認識されない

```bash
# Raspberry Pi
vcgencmd get_camera

# Jetson
ls /dev/video*
```

### モーターが動かない

1. 電源が入っているか確認
2. PWM値が適切か確認（motor.pyで調整）
3. 配線を確認

### LiDARが動作しない（YDLIDAR TMINI）

**シリアルポートが見つからない:**
```bash
# デバイス確認
ls -la /dev/ttyAMA0
ls -la /dev/ttyUSB*

# 権限確認
groups  # dialoutグループに所属しているか確認
```

**ydlidarモジュールが見つからない:**
```bash
# SDKの再インストール
cd ~/YDLidar-SDK
pip install .
```

**「Fail to get baseplate device information」エラー:**
```
[error] Fail to get baseplate device information!
[error] Timeout count: 1
```
UARTのシリアルポート設定が正しくない可能性があります。`raspi-config`で手動設定してください:
```bash
sudo raspi-config
# Interface Options → Serial Port
# - login shell over serial: No
# - serial port hardware: Yes
sudo reboot
```

**データが取得できない:**
```bash
# Bluetoothが無効化されているか確認（RPiの場合）
hciconfig  # 何も表示されなければ無効化済み

# シリアルポートのボーレート確認
# config.pyのLIDAR_SERIAL_BAUDRATEが230400になっているか確認
```

### OLEDが表示されない

**I2C接続を確認:**
```bash
sudo i2cdetect -y -r 7  # Jetson
sudo i2cdetect -y 1     # Raspberry Pi
```

**I2Cアドレスが異なる場合:**

`oled_ip_display.py` の `addr=0x3C` を実際のアドレスに変更してください。

**サービスログを確認:**
```bash
sudo journalctl -u oled-ip-display.service -f
```

### PyTorchでCUDAが認識されない（Jetson）

```bash
# libcusparseLtがインストールされているか確認
dpkg -l | grep cusparselt

# 未インストールの場合は再インストール
sudo apt-get install libcusparselt0 libcusparselt-dev
```

---

## OLEDディスプレイのセットアップ（オプション）

起動時にIPアドレスを表示するOLEDディスプレイ（SSD1306）を使用する場合の設定です。

### 1. I2Cの有効化

```bash
# I2Cツールをインストール
sudo apt install -y i2c-tools

# I2Cデバイスを確認（0x3Cにディスプレイが表示されるはず）
sudo i2cdetect -y -r 7  # Jetson
sudo i2cdetect -y 1     # Raspberry Pi
```

### 2. サービスのインストール

!!! note "OLEDライブラリ"
    OLEDスクリプトはsmbus + Pillowのみで動作します（追加ライブラリ不要）。

インストールスクリプトが現在のユーザー名とパスを自動検出してサービスを設定します:

```bash
bash setup/ip_address_display/install_oled_service.sh
```

### 3. 手動でのテスト

```bash
# プロジェクトディレクトリから実行
./setup/ip_address_display/oled_ip_display.sh

# またはPythonスクリプトを直接実行
source venv/bin/activate
python3 setup/ip_address_display/oled_ip_display.py
```

### OLED接続図

| OLED (SSD1306) | Jetson / Raspberry Pi |
|----------------|----------------------|
| VCC | 3.3V |
| GND | GND |
| SDA | Pin 3 (GPIO2/SDA) |
| SCL | Pin 5 (GPIO3/SCL) |

---

## ROS2のインストール（オプション）

ROS2のインストール・セットアップ手順は [ROS2対応](advanced/ros2.md#ros2_1) ページを参照してください。

---

## 参考リンク

- [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson)
- [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio)
- [YDLidar SDK](https://github.com/YDLIDAR/YDLidar-SDK)

---
