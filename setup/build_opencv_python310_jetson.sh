#!/bin/bash
# build_opencv_python310_jetson.sh
# OpenCV build script for Jetson Orin Nano with Python 3.10 support
# Optimized for current virtual environment

set -e  # Exit on any error

# === 設定 ===
OPENCV_VERSION="4.10.0"  # 安定版を指定
OPENCV_CONTRIB_VERSION="4.10.0"
PYTHON_VERSION="3.10"

# 仮想環境の自動検出
if [ -n "$VIRTUAL_ENV" ]; then
    VENV_PATH="$VIRTUAL_ENV"
    echo "検出された仮想環境: $VENV_PATH"
else
    VENV_PATH="/home/jetson/togikaidrive"  # デフォルト
    echo "デフォルト仮想環境を使用: $VENV_PATH"
fi

PYTHON_EXECUTABLE="$VENV_PATH/bin/python3"
PYTHON_SITE_PACKAGES="$VENV_PATH/lib/python$PYTHON_VERSION/site-packages"
BUILD_DIR="$HOME/opencv_build"

# Python実行可能ファイルの確認
if [ ! -f "$PYTHON_EXECUTABLE" ]; then
    echo "エラー: Python実行ファイルが見つかりません: $PYTHON_EXECUTABLE"
    exit 1
fi

echo "使用するPython: $PYTHON_EXECUTABLE"
echo "Pythonバージョン: $($PYTHON_EXECUTABLE --version)"
echo "インストール先: $PYTHON_SITE_PACKAGES"

# === 既存のOpenCVをアンインストール ===
echo "既存のOpenCVをアンインストール中..."
$VENV_PATH/bin/pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python || true

# === 依存ライブラリのインストール ===
echo "依存ライブラリをインストール中..."
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config unzip \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev libatlas-base-dev gfortran \
    python3-dev python3-numpy \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev \
    libeigen3-dev liblapack-dev libopenblas-dev \
    qtbase5-dev

# CUDA開発ツールの確認
if command -v nvcc &> /dev/null; then
    echo "CUDA開発環境が検出されました: $(nvcc --version | grep release)"
    CUDA_SUPPORT=ON
    CUDA_ARCH_BIN="8.7"  # Jetson Orin Nano
else
    echo "CUDA開発環境が見つかりません。CPU版をビルドします。"
    CUDA_SUPPORT=OFF
    CUDA_ARCH_BIN=""
fi

# === 作業ディレクトリの準備 ===
echo "作業ディレクトリを準備中..."
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# === OpenCV と OpenCV Contrib の取得 ===
echo "OpenCV $OPENCV_VERSION をダウンロード中..."
wget -O opencv.zip "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip"
wget -O opencv_contrib.zip "https://github.com/opencv/opencv_contrib/archive/${OPENCV_CONTRIB_VERSION}.zip"

unzip opencv.zip
unzip opencv_contrib.zip

mv "opencv-${OPENCV_VERSION}" opencv
mv "opencv_contrib-${OPENCV_CONTRIB_VERSION}" opencv_contrib

cd opencv
mkdir -p build && cd build

# === Numpy情報の取得 ===
echo "NumPy情報を取得中..."
NUMPY_INCLUDE_PATH=$($PYTHON_EXECUTABLE -c "import numpy; print(numpy.get_include())")
echo "NumPy include path: $NUMPY_INCLUDE_PATH"

# === CMake 設定 ===
echo "CMake コンフィグを生成中..."

CMAKE_ARGS=(
    -D CMAKE_BUILD_TYPE=Release
    -D CMAKE_INSTALL_PREFIX=/usr/local
    
    # Python設定
    -D BUILD_opencv_python3=ON
    -D PYTHON3_EXECUTABLE="$PYTHON_EXECUTABLE"
    -D PYTHON3_INCLUDE_DIR=$($PYTHON_EXECUTABLE -c "from sysconfig import get_paths; print(get_paths()['include'])")
    -D PYTHON3_PACKAGES_PATH="$PYTHON_SITE_PACKAGES"
    -D PYTHON3_NUMPY_INCLUDE_DIRS="$NUMPY_INCLUDE_PATH"
    
    # OpenCV Contrib
    -D OPENCV_EXTRA_MODULES_PATH="$BUILD_DIR/opencv_contrib/modules"
    
    # ビルド設定
    -D BUILD_EXAMPLES=OFF
    -D BUILD_TESTS=OFF
    -D BUILD_PERF_TESTS=OFF
    -D BUILD_DOCS=OFF
    -D BUILD_opencv_apps=ON
    
    # 画像・動画形式サポート
    -D WITH_JPEG=ON
    -D WITH_PNG=ON
    -D WITH_TIFF=ON
    -D WITH_WEBP=ON
    -D WITH_FFMPEG=ON
    -D WITH_V4L=ON
    
    # GStreamer サポート (Jetson重要)
    -D WITH_GSTREAMER=ON
    -D WITH_GSTREAMER_0_10=OFF
    
    # Qt サポート
    -D WITH_QT=ON
    -D WITH_OPENGL=ON
    
    # 最適化
    -D BUILD_WITH_DEBUG_INFO=OFF
    -D BUILD_WITH_STATIC_CRT=OFF
    -D CMAKE_BUILD_WITH_INSTALL_RPATH=ON
    
    # パッケージ設定
    -D OPENCV_GENERATE_PKGCONFIG=ON
)

# CUDA設定 (利用可能な場合)
if [ "$CUDA_SUPPORT" = "ON" ]; then
    CMAKE_ARGS+=(
        -D WITH_CUDA=ON
        -D WITH_CUDNN=ON
        -D CUDA_ARCH_BIN="$CUDA_ARCH_BIN"
        -D CUDA_ARCH_PTX=""
        -D WITH_CUBLAS=ON
        -D ENABLE_FAST_MATH=ON
        -D CUDA_FAST_MATH=ON
        -D OPENCV_DNN_CUDA=ON
        -D WITH_NVCUVID=ON
    )
    echo "CUDA サポートを有効化"
else
    CMAKE_ARGS+=(
        -D WITH_CUDA=OFF
    )
    echo "CUDA サポートを無効化"
fi

# CMake実行
cmake "${CMAKE_ARGS[@]}" ..

# === ビルド設定の確認 ===
echo ""
echo "=== ビルド設定確認 ==="
echo "Python 3 interpreter: $(grep "Python 3:" CMakeCache.txt | cut -d= -f2)"
echo "Python 3 include path: $(grep "PYTHON3_INCLUDE_DIR:" CMakeCache.txt | cut -d= -f2)"
echo "Python 3 packages path: $(grep "PYTHON3_PACKAGES_PATH:" CMakeCache.txt | cut -d= -f2)"
echo "NumPy include path: $(grep "PYTHON3_NUMPY_INCLUDE_DIRS:" CMakeCache.txt | cut -d= -f2)"
echo ""

# ユーザー確認
read -p "設定を確認してください。ビルドを続行しますか？ (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "ビルドをキャンセルしました。"
    exit 1
fi

# === ビルド実行 ===
echo "OpenCV をビルド中... (これは時間がかかります)"
echo "進行状況: make -j$(nproc)"
make -j$(nproc)

echo "システムにインストール中..."
sudo make install
sudo ldconfig

# === Python バインディングの確認とリンク ===
echo "Python バインディングを確認中..."

# cv2.so ファイルを検索
SO_FILES=($(find . -name "cv2.cpython-*-*.so" 2>/dev/null))

if [ ${#SO_FILES[@]} -eq 0 ]; then
    echo "エラー: cv2.so ファイルが見つかりませんでした"
    echo "ビルドディレクトリを確認してください: $PWD"
    find . -name "*.so" | head -10
    exit 1
fi

# 最初に見つかったファイルを使用
SO_FILE="${SO_FILES[0]}"
echo "見つかったPythonバインディング: $SO_FILE"

# site-packages ディレクトリの作成
mkdir -p "$PYTHON_SITE_PACKAGES"

# シンボリックリンクの作成
TARGET_FILE="$PYTHON_SITE_PACKAGES/cv2.so"
ln -sf "$PWD/$SO_FILE" "$TARGET_FILE"
echo "リンク作成完了: $TARGET_FILE -> $PWD/$SO_FILE"

# === インストール確認 ===
echo "インストールを確認中..."
cd "$HOME"  # ビルドディレクトリから離れる

if $PYTHON_EXECUTABLE -c "import cv2; print(f'OpenCV version: {cv2.__version__}'); print(f'Build info: {cv2.getBuildInformation()[:200]}...')" 2>/dev/null; then
    echo ""
    echo "✅ OpenCV インストール成功！"
    $PYTHON_EXECUTABLE -c "
import cv2
print(f'OpenCV version: {cv2.__version__}')
print(f'CUDA support: {cv2.cuda.getCudaEnabledDeviceCount() > 0 if hasattr(cv2, \"cuda\") else \"N/A\"}')
print(f'GStreamer support: {\"GStreamer\" in cv2.getBuildInformation()}')
print(f'Installation path: {cv2.__file__}')
"
else
    echo "❌ OpenCV インストールに失敗しました"
    echo "エラーの詳細:"
    $PYTHON_EXECUTABLE -c "import cv2" 2>&1 || true
    exit 1
fi

# === カメラデーモンの再起動 (Jetson用) ===
if systemctl is-active --quiet nvargus-daemon 2>/dev/null; then
    echo "nvargus-daemon を再起動中..."
    sudo systemctl restart nvargus-daemon
    echo "カメラデーモン再起動完了"
fi

# === クリーンアップオプション ===
echo ""
read -p "ビルドファイルを削除しますか？ (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "ビルドファイルを削除中..."
    rm -rf "$BUILD_DIR"
    echo "クリーンアップ完了"
fi

echo ""
echo "🎉 OpenCV $OPENCV_VERSION ビルド＆インストール完了！"
echo "仮想環境: $VENV_PATH"
echo "Python: $PYTHON_VERSION"
echo "インストール確認: python3 -c 'import cv2; print(cv2.__version__)'"