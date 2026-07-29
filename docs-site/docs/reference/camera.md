# カメラ調整

## Jetson CSIカメラ

### ピンク色の色合いを修正

Jetsonの一部のCSIカメラでピンク色の色合いが発生する場合があります。

```bash
# プリインストール済みファイルを使用
cp /var/nvidia/nvcam/settings/camera_overrides.isp /home/jetson/

# または、Dropboxから最新版をダウンロード
# https://www.dropbox.com/s/...
```

ISPファイルを適用後、再起動が必要です。

---

## config.py カメラ設定

```python
# ============================================================================
# カメラ設定
# ============================================================================

# カメラの有効/無効
HAVE_CAMERA = True

# カメラ解像度
CAMERA_WIDTH = 224
CAMERA_HEIGHT = 224

# フレームレート
CAMERA_FPS = 30

# カメラ0の反転設定
CAMERA_0_FLIP_HORIZONTAL = False
CAMERA_0_FLIP_VERTICAL = False

# カメラ1の反転設定（デュアルカメラ使用時）
CAMERA_1_FLIP_HORIZONTAL = False
CAMERA_1_FLIP_VERTICAL = False
```

---

## デュアルカメラ設定

2台のカメラを使用する場合の設定です。

```python
# デュアルカメラ有効化
HAVE_DUAL_CAMERA = True

# 結合方向の設定
CAMERA_CONCAT_DIRECTION = "horizontal"  # "horizontal" or "vertical"

# 結合画像の保存設定
SAVE_CONCAT_IMAGE = True
```

---

## カメラテスト

```bash
# カメラの動作確認
python camera_test.py

# または
python -c "import camera; camera.test()"
```

---

## トラブルシューティング

### picamera2のtransformエラー（Raspberry Pi）

```
AttributeError: 'libcamera._libcamera.CameraConfiguration' object has no attribute 'transform'
```

libcamera 0.5.x で `transform` が `orientation` に変更されたため、古いpicamera2との間でバージョン不整合が発生しています。picamera2をアップグレードしてください：

```bash
pip install --upgrade picamera2
```

### カメラが認識されない

**Raspberry Pi:**
```bash
# カメラの状態確認
vcgencmd get_camera

# raspi-configで有効化
sudo raspi-config
# Interface Options → Camera → Enable
```

**Jetson:**
```bash
# デバイス確認
ls /dev/video*

# GStreamerパイプラインテスト
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink
```

### 画像が暗い/明るすぎる

```python
# config.py
CAMERA_EXPOSURE = "auto"  # "auto", "manual"
CAMERA_EXPOSURE_VALUE = 0  # マニュアル時の露出値
CAMERA_GAIN = "auto"
```

### フレームレートが低い

- 解像度を下げる
- GStreamerパイプラインを最適化（[GStreamer最適化](../dev/gstreamer.md)参照）
- USBカメラの場合はUSB3.0ポートを使用

---

## カメラキャリブレーション

レンズ歪みの補正が必要な場合：

```python
import cv2
import numpy as np

# キャリブレーションデータ（事前に計算）
camera_matrix = np.array([...])
dist_coeffs = np.array([...])

# 歪み補正
undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
```

キャリブレーションデータの取得には、チェッカーボードパターンを使用します。
