#!/bin/bash
# build_opencv_basic_python310.sh
# 基本的なOpenCV機能のみの高速ビルドスクリプト
# Python 3.10 仮想環境対応

set -e

# === 設定 ===
OPENCV_VERSION="4.10.0"
PYTHON_VERSION="3.10"

# 仮想環境の自動検出
if [ -n "$VIRTUAL_ENV" ]; then
    VENV_PATH="$VIRTUAL_ENV"
else
    VENV_PATH="/home/jetson/togikaidrive"
fi

PYTHON_EXECUTABLE="$VENV_PATH/bin/python3"
PYTHON_SITE_PACKAGES="$VENV_PATH/lib/python$PYTHON_VERSION/site-packages"
BUILD_DIR="$HOME/opencv_basic_build"

echo "=== OpenCV基本版ビルド ==="
echo "仮想環境: $VENV_PATH"
echo "Python: $($PYTHON_EXECUTABLE --version)"

# === 既存OpenCVの削除 ===
echo "既存OpenCVをアンインストール..."
$VENV_PATH/bin/pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python || true

# === 必要最小限の依存関係 ===
echo "依存ライブラリをインストール..."
sudo apt update
sudo apt install -y \
    build-essential cmake git \
    libjpeg-dev libpng-dev \
    python3-dev python3-numpy \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# === OpenCVのダウンロード ===
echo "OpenCV $OPENCV_VERSION をダウンロード..."
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

wget -O opencv.zip "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip"
unzip opencv.zip
mv "opencv-${OPENCV_VERSION}" opencv
cd opencv
mkdir build && cd build

# === 基本的なCMake設定 ===
echo "CMake設定（基本版）..."
cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE="$PYTHON_EXECUTABLE" \
    -D PYTHON3_PACKAGES_PATH="$PYTHON_SITE_PACKAGES" \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_DOCS=OFF \
    -D WITH_GSTREAMER=ON \
    -D WITH_V4L=ON \
    -D WITH_JPEG=ON \
    -D WITH_PNG=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    ..

# === 高速ビルド ===
echo "ビルド実行中..."
make -j$(nproc)
sudo make install
sudo ldconfig

# === Pythonバインディングのリンク ===
echo "Pythonバインディングをリンク..."
SO_FILE=$(find . -name "cv2.cpython-*-*.so" | head -1)
if [ -n "$SO_FILE" ]; then
    mkdir -p "$PYTHON_SITE_PACKAGES"
    ln -sf "$PWD/$SO_FILE" "$PYTHON_SITE_PACKAGES/cv2.so"
    echo "リンク完了: $PYTHON_SITE_PACKAGES/cv2.so"
else
    echo "エラー: cv2.so が見つかりません"
    exit 1
fi

# === 動作確認 ===
echo "動作確認中..."
cd "$HOME"
if $PYTHON_EXECUTABLE -c "import cv2; print(f'OpenCV {cv2.__version__} インストール成功')"; then
    echo "✅ OpenCV基本版インストール完了！"
else
    echo "❌ インストールに失敗しました"
    exit 1
fi

# カメラデーモン再起動
if systemctl is-active --quiet nvargus-daemon 2>/dev/null; then
    sudo systemctl restart nvargus-daemon
fi

echo "🎉 OpenCV基本版ビルド完了！"