#!/bin/bash
# dkms_xpad_install.sh
# DKMSを使用してxpadドライバーをインストール
# 使い方: bash setup/dkms_xpad_install.sh (sudoなしで実行)

set -e

# sudoで実行された場合は警告して終了
if [ "$(id -u)" -eq 0 ]; then
    echo "sudoなしで実行してください: bash $0"
    exit 1
fi

echo "DKMS版xpadドライバーのインストール..."

# 依存関係のインストール
echo "依存関係をインストール..."
sudo apt update

# Jetson用カーネルヘッダーのインストール
echo "利用可能なカーネルヘッダーを検索中..."
KERNEL_VERSION=$(uname -r)
echo "現在のカーネル: $KERNEL_VERSION"

# Jetson用のヘッダーパッケージを探す
AVAILABLE_HEADERS=$(apt-cache search linux-headers | grep tegra | head -1 | awk '{print $1}')
if [ -z "$AVAILABLE_HEADERS" ]; then
    echo "Jetson用linux-headersが見つかりません。nvidia-jetpack-devをインストールします..."
    sudo apt install -y dkms build-essential
    sudo apt install -y nvidia-jetpack-dev || {
        echo "nvidia-jetpack-devのインストールに失敗。基本パッケージのみでビルドを試行します..."
        sudo apt install -y dkms build-essential
    }
else
    echo "見つかったヘッダー: $AVAILABLE_HEADERS"
    sudo apt install -y dkms build-essential "$AVAILABLE_HEADERS"
fi

# 既存のxpadを無効化
echo "既存のxpadモジュールを無効化..."
sudo modprobe -r xpad 2>/dev/null || true

# 一時ディレクトリでクローン・ビルド
WORK_DIR=$(mktemp -d)
echo "作業ディレクトリ: $WORK_DIR"

echo "xpadリポジトリをクローン..."
git clone https://github.com/paroj/xpad.git "$WORK_DIR/xpad"

cd "$WORK_DIR/xpad"

echo "DKMSの設定ファイルを作成..."
cat > dkms.conf << 'EOF'
PACKAGE_NAME="xpad"
PACKAGE_VERSION="0.4"
BUILT_MODULE_NAME[0]="xpad"
BUILT_MODULE_LOCATION[0]="."
DEST_MODULE_LOCATION[0]="/kernel/drivers/input/joystick"
AUTOINSTALL="yes"
EOF

# DKMSにモジュールを追加
echo "DKMSにxpadモジュールを追加..."
sudo dkms add .

# モジュールをビルド
echo "xpadモジュールをビルド..."
sudo dkms build xpad/0.4

# モジュールをインストール
echo "xpadモジュールをインストール..."
sudo dkms install xpad/0.4

# モジュールをロード
echo "xpadモジュールをロード..."
sudo modprobe xpad

# 自動ロードの設定
echo "起動時の自動ロードを設定..."
echo "xpad" | sudo tee /etc/modules-load.d/xpad.conf > /dev/null

# 作業ディレクトリを削除（DKMSインストール後はソース不要）
echo "作業ディレクトリを削除..."
rm -rf "$WORK_DIR"

echo "DKMSを使用したxpadドライバーのインストールが完了しました！"
echo ""
echo "インストール確認:"
lsmod | grep xpad || echo "xpadモジュールがロードされていません"
echo ""
echo "接続されているジョイスティック:"
ls /dev/input/js* 2>/dev/null || echo "ジョイスティックデバイスが見つかりません"
echo ""
echo "テスト方法: jstest /dev/input/js0"