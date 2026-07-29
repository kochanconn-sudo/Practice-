# Jetson向けGStreamerパフォーマンス最適化

Jetsonでのカメラ取得パフォーマンスを最大化するためのGStreamer設定について解説します。

## 基本パイプライン

### シンプルなパイプライン

```python
import cv2

# 基本的なGStreamerパイプライン
pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=224, height=224, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
```

### 高性能パイプライン（camera.py実装済み）

```python
# 高性能パイプライン例
pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1, format=NV12 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, width=224, height=224, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true sync=false"
)
```

## パイプライン要素の解説

| 要素 | 説明 |
|------|------|
| `nvarguscamerasrc` | NVIDIA CSIカメラソース |
| `sensor-id=0` | カメラID（デュアルカメラ時は0/1） |
| `memory:NVMM` | GPU統合メモリを使用（高速） |
| `nvvidconv` | GPU上でのビデオ変換 |
| `flip-method=0` | 画像反転（0=なし, 2=180度） |
| `drop=true` | フレームドロップ許可（遅延防止） |
| `sync=false` | 同期なし（リアルタイム性優先） |

## パフォーマンス比較

| 設定 | FPS | 遅延 | 備考 |
|------|-----|------|------|
| 基本パイプライン | ~30 | ~100ms | 安定 |
| 高性能パイプライン | ~60 | ~30ms | 推奨 |
| CPU処理のみ | ~15 | ~200ms | 非推奨 |

## 最適化のポイント

### 1. NVMMメモリを使用

```python
# ✅ GPU統合メモリ（高速）
"video/x-raw(memory:NVMM), ..."

# ❌ システムメモリ（低速）
"video/x-raw, ..."
```

### 2. 適切な解像度で取得

```python
# カメラネイティブ解像度で取得し、GPU上でリサイズ
"width=1280, height=720 ! nvvidconv ! width=224, height=224"
```

### 3. フレームドロップを許可

```python
# 処理が追いつかない場合はフレームをスキップ
"appsink drop=true"
```

### 4. 同期を無効化

```python
# リアルタイム性を優先
"appsink sync=false"
```

## デュアルカメラ設定

```python
# カメラ0
pipeline_0 = "nvarguscamerasrc sensor-id=0 ! ..."

# カメラ1
pipeline_1 = "nvarguscamerasrc sensor-id=1 ! ..."
```

## トラブルシューティング

### パイプラインが起動しない

```bash
# GStreamerの動作確認
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink

# プラグインの確認
gst-inspect-1.0 nvarguscamerasrc
```

### フレームレートが出ない

1. 解像度を下げる
2. `drop=true`を追加
3. 処理側のボトルネックを確認

### 画像が乱れる

1. パイプラインの形式を確認
2. `nvvidconv`の設定を見直す
3. カメラケーブルの接続を確認

## USBカメラの場合

USBカメラはGStreamerではなく、通常のOpenCVで取得：

```python
# USBカメラ
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 224)
cap.set(cv2.CAP_PROP_FPS, 30)
```

!!! tip "USB3.0推奨"
    高解像度・高フレームレートの場合はUSB3.0ポートを使用してください。
