# togikaidrive

## ***Mobility for All to Study!***

超音波センサ、LiDAR、カメラなどのセンサーと1/10~サイズのミニカーで自動運転するプログラム。
自動運転ミニカーバトルや出前授業等で活用できます。


## ドキュメント

**詳細なドキュメント・講座資料はこちら:**

https://autonomous-minicar-battle.github.io/togikaidrive-site/

## 対応デバイス

| デバイス | GPIO | 備考 |
|---------|------|------|
| Raspberry Pi 4 | RPi.GPIO | 自動検出 |
| Raspberry Pi 5 | gpiozero | 自動検出 |
| Jetson Orin Nano | Jetson.GPIO | 自動検出 |

## セットアップ

```bash
# リポジトリのクローン
#git clone https://github.com/autonomous-minicar-battle/togikaidrive-dev.git
## latest用
git clone --recurse-submodules -b latest https://github.com/autonomous-minicar-battle/togikaidrive-dev.git
cd togikaidrive-dev

# 仮想環境の作成
python3 -m venv venv --system-site-packages
source venv/bin/activate

# 依存パッケージのインストール
## Raspberry Pi 4/5
pip install -r setup/requirements-rpi.txt 
## Jetson Orin Nano
pip install -r setup/requirements-jetson.txt 

# 仮想環境を起動時に自動適用、フォルダ/pjt名は変更になる可能性があります。
echo 'source ~/togikaidrive-dev/venv/bin/activate' >> ~/.bashrc
source ~/.bashrc
```

詳細なセットアップ手順は[ドキュメント](https://autonomous-minicar-battle.github.io/togikaidrive-site/setup/)を参照してください。

## 走行モード

| モード | 説明 |
|--------|------|
| manual | 手動操作 |
| right_left_3 | 3センサー障害物回避 |
| wall_follow_pid | PID壁沿い走行 |
| nn | ニューラルネット（超音波） |
| donkeycar | CNN画像認識 |
| resnet18 / mobilevit_xxs / edgenext_xx_small | 高度な画像認識 |

## 主なプログラム概要

`python run.py` で走行！

| プログラム名 | 説明 |
| ------------ | ---- |
| run.py | 走行時のループ処理をするメインプログラム |
| config.py | パラメータ用プログラム（デバイス自動検出機能付き） |
| ultrasonic.py | 超音波測定用プログラム（RPi4/5/Jetson自動対応） |
| planner.py | 走行ロジック用プログラム |
| motor.py | 操舵・モーター出力/調整用プログラム |
| train_pytorch.py | 機械学習用プログラム |
| [data_viewer/app.py](https://github.com/Romihi/data_viewer) | 走行データ可視化/学習用Webアプリ |

> [!NOTE]
> 画像入力を使ったCNNモデルによる推論には[annotation_training_d2j](https://github.com/Romihi/annotation_training_d2j)の統合が必要です。

![プログラム構成図](docs-site\docs\assets\images\program_diagram.png)

## クイックスタート

### 1. センサー確認
```bash
python ultrasonic.py
```

### 2. モーター調整
```bash
python motor.py
```

### 3. 走行開始
```bash
python run.py
```

## リンク

- [講座ドキュメント](https://autonomous-minicar-battle.github.io/togikaidrive-dev/)
- [annotation_training_d2j](https://github.com/Romihi/annotation_training_d2j) - 画像アノテーション・学習ツール

## ライセンス

MIT License
