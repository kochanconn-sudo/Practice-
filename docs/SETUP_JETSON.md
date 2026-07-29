# Jetson Orin Nano 環境構築ガイド

## 前提条件
- JetPack 6.1 または 6.2 がインストール済み
- Python 3.10.12 がプリインストール済み

## 仮想環境のセットアップ

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

## 依存パッケージのインストール

### 1. 基本パッケージ（requirements-jetson.txtから一括インストール）

```bash
# 基本パッケージを一括インストール
pip install -r setup/requirements-jetson.txt
```

> [!NOTE]
> requirements-jetson.txtには以下のパッケージが含まれています：
> - Web Framework (Flask, tornado等)
> - Jetson GPIO
> - Hardware Control (Adafruit-PCA9685, smbus, pmw3901等)
> - Data Processing (numpy, pandas, scipy, matplotlib等)
> - Utilities (onnx, pygame, pillow等)
>
> PyTorch、torchvision、OpenCVなど特別な対応が必要なパッケージは、requirements-jetson.txt内にコメントで記載されており、以下の個別手順でインストールします。

### 2. Jetson.GPIO の設定

JetPack 6.2 で利用する場合は、GPIO の設定が必要です：
- 参考: [FaBo JetsonGPIO 設定ガイド](https://github.com/FaBoPlatform/FaBo/tree/master/0608_donkeycar/JetsonGPIO/ai_car_board)

### 3. PyTorch のインストール（NVIDIA 公式 Wheel）

```bash
# PyTorch 2.5.0 for Jetson（JetPack 6.1/6.2共通、Python 3.10用）
pip3 install --no-cache https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```

**注意**: NVIDIA の PyTorch wheel は Python 3.10 用です。このガイドでは Python 3.10 の仮想環境を使用します。

### 4. libcusparseLt のインストール（必須）

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

### 5. torchvision のインストール（ソースからビルド）

```bash
# 依存パッケージをインストール
sudo apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev \
    libavcodec-dev libavformat-dev libswscale-dev

# torchvision をクローン
git clone --branch release/0.20 https://github.com/pytorch/vision torchvision
cd torchvision

# ビルドしてインストール
export BUILD_VERSION=0.20.0
python3 setup.py install  # 仮想環境内でインストール

cd ..
```

## OpenCV の確認

Jetson には OpenCV がプリインストールされています。確認方法：

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

プリインストール版が使えない場合は、requirements-jetson.txt のコメントを参照してください。

## インストール確認

```bash
# Python とパッケージのバージョン確認
python --version
pip list

# PyTorch の動作確認
python -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"

# GPIO ライブラリの確認
python -c "import Jetson.GPIO; print('Jetson.GPIO OK')"
```

## 仮想環境の使用方法

```bash
# 有効化
source venv/bin/activate

# 無効化
deactivate
```

## トラブルシューティング

### GPIO のパーミッションエラー

```bash
sudo usermod -aG gpio $USER
# ログアウト後、再ログインが必要
```

## OLED ディスプレイのセットアップ（オプション）

起動時にIPアドレスを表示するOLEDディスプレイ（SSD1306）を使用する場合：

### 1. I2Cの有効化

```bash
# I2Cツールをインストール
sudo apt install -y i2c-tools

# I2Cデバイスを確認（0x3Cにディスプレイが表示されるはず）
sudo i2cdetect -y -r 7
```

### 2. スクリプトに実行権限を付与

> [!NOTE]
> OLEDスクリプトはsmbus + Pillowのみで動作します（追加ライブラリ不要）。

```bash
# 実行権限を付与
chmod +x setup/ip_address_display/oled_ip_display.sh
chmod +x setup/ip_address_display/oled_ip_display.py
```

### 3. systemdサービスの設定

```bash
# サービスファイルをコピー
sudo cp setup/ip_address_display/oled-ip-display.service /etc/systemd/system/

# systemdをリロード
sudo systemctl daemon-reload

# サービスを有効化して起動
sudo systemctl enable oled-ip-display.service
sudo systemctl start oled-ip-display.service

# ステータス確認
sudo systemctl status oled-ip-display.service
```

### 4. 手動でのテスト

```bash
# プロジェクトディレクトリから実行
./setup/ip_address_display/oled_ip_display.sh

# またはPythonスクリプトを直接実行
source venv/bin/activate
python3 setup/ip_address_display/oled_ip_display.py
```

### 5. ログの確認

```bash
# サービスログを確認
sudo journalctl -u oled-ip-display.service -f
```

### トラブルシューティング

**OLED が表示されない場合:**

1. I2C接続を確認:
```bash
sudo i2cdetect -y -r 7
```

2. I2Cアドレスが異なる場合は、`oled_ip_display.py` の `addr=0x3C` を変更

3. 配線を確認:
   - VCC → 3.3V
   - GND → GND
   - SDA → Pin 3 (GPIO2)
   - SCL → Pin 5 (GPIO3)

## ROS2 のインストール（オプション）

ROS2を使用する場合は、以下の手順でインストールします：

### 1. セットアップ

```bash
# 必要なパッケージをインストール
sudo apt install software-properties-common
sudo add-apt-repository universe

# ROSリポジトリを追加
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade
```

### 2. インストール

```bash
# ROS2 Humbleをインストール
sudo apt install ros-humble-desktop ros-dev-tools
```

### 3. 環境変数設定

```bash
# bashrcに追加して自動で読み込まれるようにする
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. 動作テスト

別々のターミナルで以下を実行して、通信を確認します：

**ターミナル1: Talkerを実行**
```bash
ros2 run demo_nodes_cpp talker
```

**ターミナル2: Listenerを実行**
```bash
ros2 run demo_nodes_py listener
```

Listenerが「I heard: [Hello World: X]」のようなメッセージを受信できれば成功です。

## 参考リンク

- [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson)
- [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio)
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)