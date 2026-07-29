# OpenCV Build Scripts for Jetson Orin Nano + Python 3.10

このディレクトリには、Jetson Orin Nano環境でPython 3.10仮想環境向けのOpenCVをビルドするスクリプトが含まれています。

## スクリプト一覧

### 1. `build_opencv_python310_jetson.sh` - フル機能版
**推奨**: 完全なOpenCV機能が必要な場合

**特徴:**
- OpenCV 4.10.0 + OpenCV Contrib
- CUDA/cuDNN サポート (利用可能な場合)
- GStreamer サポート (Jetsonカメラ用)
- Qt GUI サポート
- 豊富な画像/動画形式対応
- 機械学習・画像処理の全機能

**ビルド時間:** 約2-3時間

```bash
./build_opencv_python310_jetson.sh
```

### 2. `build_opencv_basic_python310.sh` - 基本版
**推奨**: 基本的なカメラ/画像処理のみ必要な場合

**特徴:**
- OpenCV 4.10.0 基本機能のみ
- GStreamer サポート (カメラ用)
- 基本的な画像形式対応
- 最小限の依存関係

**ビルド時間:** 約30-60分

```bash
./build_opencv_basic_python310.sh
```

## 事前準備

### 1. 仮想環境の確認
```bash
# 現在の仮想環境を確認
echo $VIRTUAL_ENV
python3 --version

# 仮想環境をアクティベート（必要な場合）
source /home/jetson/togikaidrive/bin/activate
```

### 2. 十分な空き容量の確認
```bash
# 空き容量確認（フル版は約10GB、基本版は約5GB必要）
df -h
```

### 3. swap領域の確保（推奨）
```bash
# 一時的にswapファイルを作成（ビルド中のメモリ不足を防ぐ）
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# ビルド完了後に削除
# sudo swapoff /swapfile
# sudo rm /swapfile
```

## 使用方法

### ステップ1: スクリプトの選択と実行
```bash
# 基本版（推奨・高速）
./build_opencv_basic_python310.sh

# または フル機能版
./build_opencv_python310_jetson.sh
```

### ステップ2: インストール確認
```bash
# 仮想環境で確認
python3 -c "
import cv2
print(f'OpenCV version: {cv2.__version__}')
print(f'Build info available: {len(cv2.getBuildInformation()) > 100}')
print(f'GStreamer support: {\"GStreamer\" in cv2.getBuildInformation()}')
"
```

### ステップ3: カメラテスト
```bash
# カメラテスト用簡単なスクリプト
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    print('✅ カメラ接続成功')
    ret, frame = cap.read()
    if ret:
        print(f'✅ フレーム取得成功: {frame.shape}')
    else:
        print('❌ フレーム取得失敗')
else:
    print('❌ カメラ開けませんでした')
cap.release()
"
```

## トラブルシューティング

### メモリ不足エラー
```bash
# swapファイルの作成
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### ビルドエラー時のクリーンアップ
```bash
# ビルドディレクトリの削除
rm -rf ~/opencv_build ~/opencv_basic_build

# 既存のOpenCVライブラリの削除
pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python
sudo apt remove --purge libopencv* python3-opencv
sudo apt autoremove
```

### カメラが動作しない場合
```bash
# カメラデバイスの確認
ls /dev/video*

# nvargus-daemonの再起動
sudo systemctl restart nvargus-daemon

# 権限の確認
sudo usermod -a -G video $USER
# ログアウト・ログインが必要
```

## 注意事項

1. **ビルド時間**: フル版は2-3時間、基本版は30-60分かかります
2. **メモリ使用量**: 4GB以上の空きメモリ推奨
3. **ストレージ**: フル版10GB、基本版5GBの空き容量必要
4. **既存OpenCV**: スクリプトが自動的にpip版を削除します
5. **仮想環境**: 必ず正しい仮想環境で実行してください

## スクリプトの特徴

### 自動検出機能
- 仮想環境パスの自動検出
- CUDA環境の自動検出
- Python設定の自動取得

### Jetson最適化
- Jetson Orin Nano用CUDA_ARCH_BIN設定
- GStreamerサポート
- nvargus-daemon自動再起動

### エラーハンドリング
- 設定確認ステップ
- インストール検証
- 詳細なエラーメッセージ

## パフォーマンス比較

| 機能 | 基本版 | フル版 |
|------|--------|--------|
| ビルド時間 | 30-60分 | 2-3時間 |
| 容量 | ~500MB | ~2GB |
| 画像処理 | ✅ | ✅ |
| カメラ | ✅ | ✅ |
| CUDA | ❌ | ✅ |
| 機械学習 | 基本のみ | 全機能 |
| GUI | 限定的 | 全機能 |

**推奨**: togikaidriveプロジェクトでは基本版で十分です。