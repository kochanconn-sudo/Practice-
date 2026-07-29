# 自動運転ミニカープロジェクト

世界で活用されている代表的な自動運転ミニカープロジェクトを紹介します。

---

## プロジェクト比較

| プロジェクト | 開発元 | 主な特徴 | 対応ハードウェア |
|-------------|--------|---------|-----------------|
| **Donkey Car** | コミュニティ | 最も普及、豊富なドキュメント | Raspberry Pi, Jetson Nano |
| **JetRacer** | NVIDIA | Jetson最適化、高速推論 | Jetson Nano/Orin |
| **RumiCar** | 日本コミュニティ | 日本語ドキュメント、教育向け | Arduino, Raspberry Pi  |
| **Roboracer** | F1TENTH派生 | 本格的なレース、ROS2対応 | Jetson, 高性能PC |

---

## Donkey Car

最も広く使われているオープンソースの自動運転ミニカープラットフォームです。

### 特徴

- **豊富なコミュニティ**: 世界中のユーザーが情報を共有
- **シンプルな構成**: Raspberry Pi + カメラ + RCカーで構築可能
- **Keras/TensorFlow**: 機械学習フレームワークを使用
- **Webインターフェース**: ブラウザからの操作・監視

### 技術仕様

| 項目 | 仕様 |
|------|------|
| フレームワーク | Python, Keras/TensorFlow |
| 入力 | カメラ画像（160×120または224×224） |
| 出力 | ステアリング、スロットル |
| モデル | Linear, Categorical, RNN など |
| データ形式 | Tub形式（JSON + 画像） |

### リンク

- [公式サイト](https://www.donkeycar.com/)
- [ドキュメント](https://docs.donkeycar.com/)
- [GitHub](https://github.com/autorope/donkeycar)

---

## JetRacer

NVIDIA Jetsonに最適化された自動運転ミニカープロジェクトです。
JetRacerは株式会社FaBoによる[FaBo JetRacer Docs](https://faboplatform.github.io/JetracerDocs/)を参照し、最新の実装やドキュメントを参照するのが望ましい。

### 特徴

- **NVIDIA最適化**: TensorRT、CUDA による高速推論
- **JetBot派生**: NVIDIA JetBotの技術をベースに開発
- **Jupyter Notebook**: 対話的な開発環境
- **リアルタイム処理**: GPUによる高速画像処理

### 技術仕様

| 項目 | 仕様 |
|------|------|
| フレームワーク | Python, PyTorch, TensorRT |
| 対応デバイス | Jetson Nano, Jetson Orin Nano |
| 入力 | CSIカメラ（224×224） |
| 推論速度 | 30FPS以上（TensorRT使用時） |
| 特徴 | Road Following, Object Avoidance |

### リンク

- [GitHub](https://github.com/NVIDIA-AI-IOT/jetracer)
- [JetBot（関連プロジェクト）](https://github.com/NVIDIA-AI-IOT/jetbot)

---

## RumiCar

日本発の教育向け自動運転ミニカープロジェクトです。

### 特徴

- **日本語ドキュメント**: 日本語での情報が充実
- **教育向け設計**: 学校や研修での利用を想定
- **Arduino対応**: 小型マイコンで動作可能
- **ToFセンサー**: 距離センサーを活用した制御

### 技術仕様

| 項目 | 仕様 |
|------|------|
| フレームワーク | Arduino, MicroPython |
| 対応デバイス | Arduino, Raspberry Pi |
| センサー | ToFセンサー（VL53L0X等） |
| 制御方式 | ルールベース、機械学習 |
| 特徴 | 低コスト、教育向け |

### リンク

- [公式サイト](https://rumicar.com/)
- [GitHub](https://github.com/RumiCar-group/RumiCar)

---

## Roboracer / F1TENTH

本格的な自動運転レースを目指すプロジェクトです。

### 特徴

- **ROS2対応**: ロボット開発の標準フレームワーク
- **高速走行**: 実際のレースで時速30km以上
- **LiDAR必須**: 2D LiDARによる環境認識
- **国際大会**: 世界各地でレースイベント開催

### 技術仕様

| 項目 | 仕様 |
|------|------|
| フレームワーク | ROS2, Python/C++ |
| 対応デバイス | Jetson, 高性能PC |
| センサー | 2D LiDAR（必須）、カメラ（オプション） |
| 走行アルゴリズム | Pure Pursuit, Follow the Gap, MPC |
| 車両サイズ | 1/10スケール |

### リンク

- [F1TENTH公式](https://f1tenth.org/)
- [F1TENTH GitHub](https://github.com/f1tenth)
- [Roboracer](https://roboracer.io/)

---

## togikidriveの位置づけ

togikidriveは、これらのプロジェクトの良い点を取り入れつつ、日本の教育現場に最適化したプロジェクトです。

### togikidriveの特徴

- **Donkey Car互換**: データ形式がDonkey Carと互換性あり
- **PyTorch採用**: 最新の機械学習フレームワーク
- **LiDAR統合**: 超音波センサーとLiDARの両方に対応
- **ROS2対応（開発中）**: 各センサーをROS2ノードとして起動可能。SLAM・Nav2連携を目指す
- **日本語完全対応**: ドキュメント、コメントすべて日本語

プロジェクト比較の詳細は [ROS2対応 — F1TENTH リファレンス](advanced/ros2.md#f1tenth-リファレンス) を参照してください。
