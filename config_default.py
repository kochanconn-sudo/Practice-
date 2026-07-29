# config_default.py
# coding:utf-8
#
# デフォルト設定ファイル（原本）
# このファイルは直接編集しないでください。
# run.py 起動時に config.py が自動生成されるので、そちらを編集してください。

# ============================================================================
# デバイス設定
# ============================================================================
# デフォルト値（run.py で device_detection.py により自動上書きされる）
DEVICE_TYPE = "UNKNOWN"
PLATFORM_NAME = "Unknown"
GPIO_BACKEND = "gpiozero"
I2C_BUS = 7

# 使用するセンサー
ACTIVE_SENSORS = ["lidar","camera_0"]
# ACTIVE_SENSORS = ["ultrasonic","camera_0", "camera_1"]

# ============================================================================
# モーター制御基本設定
# ============================================================================
# モーターへの入力値（-1〜1 で設定）

# スロットル出力
FORWARD_STRAIGHT = 0.6  # ストレートでの値
FORWARD_CORNER = 0.3    # カーブでの値

# ============================================================================
# 測距センサー検知範囲設定（超音波/LiDAR共通、単位: mm）
# ============================================================================
# 停止・後退判定
STOP_RANGE = 250       # 停止判断に使用する距離
BACKWARD_RANGE = 130   # 後退判断に使用する距離

# 障害物回避の基準距離
DETECTION_RANGE = 300    # 検知開始距離
RIGHT_LEFT_RANGE = 550   # 右左折判定基準距離

# 壁沿い走行の目標距離
TARGET_RANGE = 200             # 目標距離
TARGET_RANGE_ADJUSTMENT = 25   # 目標距離付近での操作変更基準値（±）

# PIDパラメータ（PDまでを推奨）
K_P = 0.005
K_I = 0.0
K_D = 0.0005

# --- gap_follow ---
GAP_STEER_GAIN      = 1.0
GAP_FOLLOW_BRAKE_DIST = 400

# --- racer（やり込み枠・4パラメータ）---
RACER_STEER_GAIN     = 0.8
RACER_SPEED_GAIN     = 1.0
RACER_BRAKE_DIST     = 500
RACER_STEER_SLOWDOWN = 0.5
RACER_SPEED_CEIL     = 0.9

# ============================================================================
# 走行プラン（判断モード）選択
# ============================================================================
PLAN_LIST = [
    "manual",
    "go_straight",
    "right_left_3",
    "right_left_3_records",
    "wall_follow",
    "wall_follow_pid",
    "center_follow_pid",
    "gap_follow",
    "racer",
    "follow_the_gap",
    "nn",
    "donkeycar",
    "resnet18",
    "mobilevit_xxs",
    "edgenext_xx_small",
    "gru",
    "tcn",
    "causal_cnn"
]

PLAN = "edgenext_xx_small"

# ============================================================================
# 各種走行モード固有のパラメータ
# ============================================================================
# wall_follow モード
HAND_SIDE = "right"  # "right" or "left"
WALL_FOLLOW_USE_ALIGNMENT = False
WALL_FOLLOW_K_ANGLE       = 0.3

# right_left_3_records モード: 過去の操作値記録回数
RIGHT_LEFT_RECORD_NUMBER = 3

# center_follow_pid モード
CENTER_FALLBACK_RANGE = 800  # mm: この値を超えたら壁なしと判断
CENTER_K_P = 0.003           # 左右差[mm]→ステア変換（差100mm → steering=0.3 が目安）
CENTER_K_D = 0.001
CENTER_K_I = 0.0

# ============================================================================
# 復帰モード設定
# ============================================================================
RECOVERY_MODE = "back"           # "none" or "back"
RECOVERY_STEERING = "auto"    # 復帰時のステアリング値（"auto": センサーで自動判定, 数値: 固定値）
RECOVERY_TIME_DURATION = 0.5       # 復帰処理を行う時間（秒）
RECOVERY_BRAKING = 2             # ブレーキ→ニュートラルのセット回数（推奨値: 2）
RECOVERY_BRAKE_DURATION = 0.1    # ブレーキフェーズの時間（秒）
RECOVERY_NEUTRAL_DURATION = 0.1  # ニュートラルフェーズの時間（秒）

# ============================================================================
# 車両調整用パラメータ（ハードウェア設定）
# ============================================================================
# motor.py で調整した後、値を入れる

# PWM信号のチャネル
CHANNEL_STEERING = 0
CHANNEL_THROTTLE = 1

# ステアリングのPWM値
STEERING_CENTER_PWM = 390
STEERING_WIDTH_PWM = 80
STEERING_RIGHT_PWM = STEERING_CENTER_PWM - STEERING_WIDTH_PWM
STEERING_LEFT_PWM = STEERING_CENTER_PWM + STEERING_WIDTH_PWM
# ステアリング保護用の上限下限値
STEERING_HI_LIMIT = 500
STEERING_LO_LIMIT = 300

# スロットルのPWM値
# モーターの回転音を聞き、音が変わらないところが最大/最小値とする
THROTTLE_STOPPED_PWM = 380   # めやす: 370〜400
THROTTLE_FORWARD_PWM = 450
THROTTLE_REVERSE_PWM = 330
THROTTLE_WIDTH_PWM = 100     # motor.py の確認用

# ============================================================================
# 機械学習モデル設定（NN/CNN）
# ============================================================================
# モデルのパス
# 推論エンジンはファイル拡張子から自動判定される:
#   .pth    → PyTorch（学習時・開発時・RPi）
#   .engine → TensorRT（Jetson推奨）
#   .xml    → OpenVINO（x86/RPi推奨）
MODEL_DIR = "models"
MODEL_NAME = "edgenext_xx_small_20260207_165735.pth"
MODEL_PATH = f"{MODEL_DIR}/{MODEL_NAME}"

# 推論エンジンはMODEL_NAMEの拡張子から自動判定（.pth→PyTorch, .engine→TensorRT, .xml→OpenVINO）
USE_FP16 = False  # FP16推論（GPUメモリ削減＋高速化、Falseで FP32）

# モデル出力次元数
# 2: [steering, throttle]（従来互換）
# 3: [steering, throttle, speed]（speed付き、Speed PID制御との組み合わせ用）
NUM_OUTPUTS = 2

# NNモデルのパラメータ
HIDDEN_DIM = 64           # 隠れ層のノード数
NUM_HIDDEN_LAYERS = 3     # 隠れ層の数

# 学習時のパラメータ
BATCH_SIZE = 16
EPOCHS = 30

# CNNモデル（donkeycar, resnet18等）の入力画像設定
# "cam0/image_array": camera_0 の画像
# "cam1/image_array": camera_1 の画像
# "cam2/image_array": camera_2 の画像
# "cam3/image_array": camera_3 の画像
MODEL_INPUT_IMAGE = "cam0/image_array"

# Early Stopping設定
USE_EARLY_STOPPING = False
EARLY_STOPPING_PATIENCE = 5     # 検証損失が改善しなくなってから待機するエポック数
EARLY_STOPPING_MIN_DELTA = 1e-6 # 改善と判断する最小変化量

# データオーグメンテーション設定
USE_DATA_AUGMENTATION = True
# 水平反転
AUG_USE_FLIP = True
AUG_FLIP_PROB = 0.5
# 色調整
AUG_USE_COLOR = True
AUG_BRIGHTNESS = 0.2
AUG_CONTRAST = 0.2
AUG_SATURATION = 0.2
# 幾何変換
AUG_USE_GEOMETRY = True
AUG_ROTATION_DEGREES = 5
AUG_TRANSLATE_RATIO = 0.1
# ランダムイレース
AUG_USE_ERASE = True
AUG_ERASE_PROB = 0.5
AUG_ERASE_MIN_RATIO = 0.02
AUG_ERASE_MAX_RATIO = 0.2

# モデルの種類: "regression" or "categorical"
MODEL_TYPE = "regression"
# categorical のカテゴリ設定（カテゴリ数は揃えること）
# 超音波センサーの距離値を正規化するスケール
NORMALIZE_RANGE = 2000

# ============================================================================
# 超音波センサ設定
# ============================================================================
# 最大測定距離 (mm)
CUTOFF_RANGE = 2000
# 測定回数（ultrasonic.py チェック用）
SAMPLING_TIMES = 100

# 超音波センサの配置設定
# ["RrLH", "FrLH", "FrFR", "FrRH", "RrRH"] = [真左, 前方左, 前方, 前方右, 真右]
# 計測ループが遅い場合は数を減らす
# ULTRASONIC_SENSOR_LIST = ["FrLH","FrFR","FrRH"]          # 3つ
ULTRASONIC_SENSOR_LIST = ["RrLH", "FrLH", "FrFR", "FrRH","RrRH"]  # 5つ
# ULTRASONIC_SENSOR_LIST.extend(["BackRH", "Back", "BackLH"])       # 8つに拡張

# 超音波センサの配置角度（度単位、車両前方を0度、右回りに増加）
SENSOR_LIST_ALLOCATION_ANGLE = [270, 315, 0, 45, 90]

# GPIOピン番号の対応
ULTRASONIC_ECHO_PIN_NUMBER=[11,13,15,29,31,33,35,37]
ULTRASONIC_TRIGER_PIN_NUMBER=[12,16,18,22,32,36,38,40]
ULTRASONIC_ECHO_PINS = {name: ULTRASONIC_ECHO_PIN_NUMBER[i] for i, name in enumerate(ULTRASONIC_SENSOR_LIST)}
ULTRASONIC_TRIG_PINS = {name: ULTRASONIC_TRIGER_PIN_NUMBER[i] for i, name in enumerate(ULTRASONIC_SENSOR_LIST)}

# ============================================================================
# カメラ設定
# ============================================================================
IMAGE_W = 224
IMAGE_H = 224
IMAGE_DEPTH = 3          # RGB=3, モノクロ=1
CAMERA_FRAMERATE = 60

# カメラTuningファイル（ピンクかぶり補正など、不要ならNone）
CAMERA_TUNING_FILE = "None" 
# CAMERA_TUNING_FILE = "/home/pi/togikaidrive-dev/setup/imx219_200d.json"

# カメラ0のフリップ設定
CAMERA_0_VFLIP = False
CAMERA_0_HFLIP = False
# カメラ1のフリップ設定
CAMERA_1_VFLIP = False
CAMERA_1_HFLIP = False

# カメラスロット設定
# TYPE: None=プラットフォーム自動検出, "jetson", "pi", "sv125", "usb_generic", "lidar"
# DEVICE_ID: None=自動検出, 整数=手動指定（CSI: sensor-id, USB: /dev/videoN のN）
CAMERA_0_TYPE = None
CAMERA_0_DEVICE_ID = 0
CAMERA_1_TYPE = None
CAMERA_1_DEVICE_ID = 1
CAMERA_2_TYPE = None
CAMERA_2_DEVICE_ID = 2
CAMERA_3_TYPE = None
CAMERA_3_DEVICE_ID = 3
# USBカメラ自動検出キーワード（v4l2-ctl --list-devices 出力の部分一致）
CAMERA_DETECT_KEYWORDS = {
    "sv125":       "USB2.0 Camera RGB",
    "usb_generic": "USB 2.0 Camera",
}

# USBカメラデバイスID固定設定（None = 自動検出）
CAMERA_ID_SV125 = None
CAMERA_ID_USB_GENERIC = None

# 複数カメラ画像の結合方向: "horizontal" or "vertical"
IMAGE_CONCAT_DIRECTION = "horizontal"
SAVE_CONCATENATED_IMAGE = False   # 結合画像を保存するか
RESIZE_CONCATENATED_IMAGE = True  # 結合画像を IMAGE_W x IMAGE_H にリサイズするか

# ============================================================================
# LiDAR設定
# ============================================================================
HAVE_LIDAR = True
LIDAR_TYPE = "AUTO"  # "AUTO", "TMINI", "UST20", "NONE"
# LiDAR画像設定
LIDAR_IMAGE_W = 224
LIDAR_IMAGE_H = 224
SAVE_LIDAR_IMAGES = True     # LiDAR画像保存
SAVE_LIDAR_DATA = False      # LiDAR点群データを保存
LIDAR_BINARY_IMAGE = False   # 白黒2値で表示するか
WEB_SERVER_PORT = 8080       # LiDAR単体確認用ウェブサーバーポート

# LiDAR画像縮尺設定
LIDAR_IMAGE_SCALE_FACTOR = 0.8     # 画像サイズに対するスケール係数 (0.0-1.0)
LIDAR_IMAGE_METERS_PER_PIXEL = 0.018  # 1ピクセルあたりの実距離（メートル）

# LiDAR搭載位置オフセット（mm単位）
# 車両中心を原点として、前方が正のY、右が正のX
LIDAR_OFFSET_X = 0           # 左右方向のオフセット（右が正）
LIDAR_OFFSET_Y = 330-450/2   # 前後方向のオフセット（前が正）

# ゾーン名の定義（5ゾーン、超音波センサーと共通）
ZONE_NAMES = ["RrLH", "FrLH", "FrFR", "FrRH", "RrRH"]

# 検出点数閾値（LiDAR機種ごとにデータ点数が異なるため注意）
LIDAR_DETECT_POINTS_THRESHOLD = 10
# ゾーン別検出点数閾値
LIDAR_DETECT_POINTS_THRESHOLD_ZONE = [
    LIDAR_DETECT_POINTS_THRESHOLD*2,  # Zone 0: 左後方 (RrLH)
    LIDAR_DETECT_POINTS_THRESHOLD,    # Zone 1: 左前方 (FrLH)
    LIDAR_DETECT_POINTS_THRESHOLD,    # Zone 2: 前方   (FrFR)
    LIDAR_DETECT_POINTS_THRESHOLD,    # Zone 3: 右前方 (FrRH)
    LIDAR_DETECT_POINTS_THRESHOLD*2   # Zone 4: 右後方 (RrRH)
]

# 検出距離閾値 (mm)
LIDAR_DETECT_DISTANCE_THRESHOLD = 300
# ゾーン別検出距離閾値 (mm)
LIDAR_DETECT_DISTANCE_THRESHOLD_ZONE = [
    LIDAR_DETECT_DISTANCE_THRESHOLD,        # Zone 0: 左後方 (RrLH)
    LIDAR_DETECT_DISTANCE_THRESHOLD,        # Zone 1: 左前方 (FrLH)
    LIDAR_DETECT_DISTANCE_THRESHOLD + 100,  # Zone 2: 前方   (FrFR)
    LIDAR_DETECT_DISTANCE_THRESHOLD,        # Zone 3: 右前方 (FrRH)
    LIDAR_DETECT_DISTANCE_THRESHOLD         # Zone 4: 右後方 (RrRH)
]
LIDAR_WALL_DISTANCE = LIDAR_DETECT_DISTANCE_THRESHOLD  # 壁検出（描画）用

# 壁検出設定
LIDAR_DETECT_WALLS = False

# 壁検出手法
# 選択肢: "distance_based", "split_merge", "sliding_window", "ransac", "hybrid"
LIDAR_DETECTION_METHOD = 'distance_based'

# 壁として認識する点間の最大距離 (mm)
LIDAR_WALL_MAX_GAP = 300
# 壁セグメントとして必要な最小点数
LIDAR_WALL_MIN_POINTS = 25
# 最大許容直線偏差（低いほど厳密な直線を要求）
LIDAR_WALL_MAX_LINEARITY = 0.08

# Split-Merge法用パラメータ
LIDAR_SPLIT_EPSILON = 90          # 分割閾値 (mm)
LIDAR_MIN_SEGMENT_LENGTH = 900    # 最小セグメント長 (mm)
LIDAR_USE_ADAPTIVE = True         # 適応的閾値を使用するか
LIDAR_USE_2D_OPTIMIZATION = True  # 2D最適化を使用するか

# RANSAC法用パラメータ
LIDAR_RANSAC_THRESHOLD = 60    # 残差閾値 (mm)
LIDAR_MIN_INLIER_RATIO = 0.6  # 最小インライア率
LIDAR_RANSAC_MAX_TRIALS = 150  # 最大試行回数
LIDAR_EARLY_STOP_RATIO = 0.9  # 早期終了閾値

# スライディングウィンドウ法用パラメータ
LIDAR_WINDOW_SIZE = 20       # ウィンドウサイズ（点数）
LIDAR_WINDOW_STRIDE = 5     # ウィンドウの移動幅（点数）
LIDAR_OVERLAP_THRESHOLD = 700  # 重複閾値 (mm)

# Hybrid法用パラメータ
LIDAR_CONFIDENCE_THRESHOLD = 0.8  # RANSAC検証の信頼度閾値

# セグメント統合用パラメータ
LIDAR_MERGE_ANGLE_THRESHOLD = 10     # 統合時の角度閾値（度）
LIDAR_MERGE_DISTANCE_THRESHOLD = 100 # 統合時の距離閾値 (mm)

# ============================================================================
# LiDAR機種別設定
# ============================================================================

if LIDAR_TYPE == "TMINI":
    # YDLIDAR TMINI 設定
    LIDAR_SCAN_RATE = 10         # スキャンレート (Hz) - 6〜12
    LIDAR_DATA_POINTS = int(4000/LIDAR_SCAN_RATE)  # 最大測定周波数4000kHz、スキャンレートに応じて変化
    LIDAR_ANGLE_RANGE = 360     # 度
    LIDAR_ANGLE_START = 0       # 度
    LIDAR_ANGLE_END = 360       # 度
    LIDAR_ANGLE_OFFSET = 90     # 度 - LiDARの向きを調整するオフセット値（正の値で右回転）
    LIDAR_CLOCKWISE = True      # スキャン方向（True: 時計回り）

    # 通信設定
    LIDAR_COMM_TYPE = "serial"
    LIDAR_SERIAL_PORT = "/dev/ttyAMA0"  # Bluetooth無効化後のハードウェアUART (GPIO 14/15)
    LIDAR_SERIAL_BAUDRATE = 230400

    # 単位系設定
    LIDAR_UNIT_TYPE = "m"       # TMINIのネイティブ単位系
    LIDAR_TARGET_UNIT = "mm"    # システム内部で使用する単位系

    # 測定範囲 (mm)
    LIDAR_MIN_DISTANCE = 20
    LIDAR_MAX_DISTANCE = 4000
    LIDAR_IGNORE_DISTANCE = 150    # LiDAR近傍の部品を無視する距離

    # ゾーンインデックスの定義（ZONE_NAMESに対応）
    # TMINIの400点を5ゾーンに分割（360度/400点 = 1点あたり0.9度）
    # 角度オフセット90度により、インデックス0は車両の右方向（90度）
    # 各ゾーン50点（45度幅）、45度間隔で隙間なく配置
    ZONE_INDEX = [
        [x for x in range(274, 324)],                                  # RrLH: 真左（180°）中心299
        [x for x in range(324, 374)],                                  # FrLH: 左斜め45°（135°）中心349
        [x for x in range(374, 400)]+[x for x in range(0, 24)],       # FrFR: 真正面（90°）中心399/0
        [x for x in range(24, 74)],                                    # FrRH: 右斜め45°（45°）中心49
        [x for x in range(74, 124)]                                    # RrRH: 真右（0°）中心99
    ]

elif LIDAR_TYPE == "UST20":
    # 北陽 UST-20 設定
    LIDAR_SCAN_RATE = 40        # スキャンレート (Hz)
    LIDAR_DATA_POINTS = 1081
    LIDAR_CLOCKWISE = False     # スキャン方向（UST-20は反時計回り）
    LIDAR_ANGLE_RANGE = 270     # 度
    LIDAR_ANGLE_START = -135    # 度（インデックス0の角度）
    LIDAR_ANGLE_END = 135       # 度（最後のインデックスの角度）
    LIDAR_ANGLE_STEP = 4
    LIDAR_ANGLE_OFFSET = 90     # 度（車両の向き調整、0が右向き）

    # 通信設定
    LIDAR_COMM_TYPE = "ethernet"
    LIDAR_IP_ADDRESS = "192.168.0.139"
    LIDAR_PORT = 10940

    # 単位系設定
    LIDAR_UNIT_TYPE = "mm"      # UST-20のネイティブ単位系
    LIDAR_TARGET_UNIT = "mm"    # システム内部で使用する単位系

    # 測定範囲 (mm)
    LIDAR_MIN_DISTANCE = 100
    LIDAR_MAX_DISTANCE = 20000
    LIDAR_IGNORE_DISTANCE = 100    # LiDAR近傍の部品を無視する距離

    # ゾーンインデックスの定義（ZONE_NAMESに対応）
    ZONE_INDEX = [
        [x for x in range(180 *LIDAR_ANGLE_STEP, 240 *LIDAR_ANGLE_STEP)],  # RrLH: 左後方
        [x for x in range(150 *LIDAR_ANGLE_STEP, 180 *LIDAR_ANGLE_STEP)],  # FrLH: 左前方
        [x for x in range(120 *LIDAR_ANGLE_STEP, 150 *LIDAR_ANGLE_STEP)],  # FrFR: 前方
        [x for x in range(90 *LIDAR_ANGLE_STEP,  120 *LIDAR_ANGLE_STEP)],  # FrRH: 右前方
        [x for x in range(30 *LIDAR_ANGLE_STEP,   90 *LIDAR_ANGLE_STEP)]   # RrRH: 右後方
    ]

else:
    # LiDARが設定されていない場合、または AUTO（run.py で自動検出）の場合のデフォルト値
    # 注意: LIDAR_TYPE = "AUTO" を "NONE" に上書きしない（detect_lidar() で自動検出させるため）
    if LIDAR_TYPE != "AUTO":
        LIDAR_TYPE = "NONE"
    LIDAR_DATA_POINTS = 0
    LIDAR_ANGLE_RANGE = 0
    LIDAR_ANGLE_START = 0
    LIDAR_ANGLE_END = 0
    LIDAR_ANGLE_OFFSET = 0
    LIDAR_CLOCKWISE = True
    LIDAR_SCAN_RATE = 1
    LIDAR_MIN_DISTANCE = 20
    LIDAR_MAX_DISTANCE = 4000
    LIDAR_IGNORE_DISTANCE = 150
    ZONE_INDEX = [[], [], [], [], []]

# ============================================================================
# Follow the Gap 設定
# ============================================================================
FTG_SAFETY_DISTANCE = 300       # 安全距離 (mm)
FTG_MAX_DISTANCE = 3000         # 最大検出距離 (mm)
FTG_BUBBLE_RADIUS = 150         # 安全バブル半径 (mm)
FTG_DISPARITY_THRESHOLD = 200   # 距離差閾値 (mm)
FTG_ANGLE_START = -90           # 使用角度範囲 開始 (度)
FTG_ANGLE_END = 90              # 使用角度範囲 終了 (度)

# ステアリング制御方式: "linear", "pid", "pure_pursuit"
FTG_STEERING_METHOD = "linear"
FTG_STEERING_GAIN = 1.0         # ステアリングゲイン（全方式共通）
FTG_SMOOTHING_FACTOR = 0.3      # EMAスムージング係数（全方式共通、0=前回値維持、1=即応答）

# PID制御パラメータ
FTG_PID_KP = 0.8
FTG_PID_KI = 0.0
FTG_PID_KD = 0.1

# Pure Pursuit パラメータ
FTG_WHEELBASE = 300              # ホイールベース (mm)
FTG_LOOKAHEAD_DISTANCE = 500     # ルックアヘッド距離 (mm)

# ============================================================================
# LiDAR自動スロットル調整機能
# ============================================================================
# 特定のゾーンに障害物がない場合、スロットルを自動的に設定値に変更する
LIDAR_THROTTLE_ENABLED = False

# 監視するゾーンのインデックス
# 0: 左後方 (RrLH), 1: 左前方 (FrLH), 2: 前方 (FrFR), 3: 右前方 (FrRH), 4: 右後方 (RrRH)
LIDAR_THROTTLE_ZONE = 2

# 障害物検出の距離閾値 (mm)
# この距離より近くに障害物がない場合、スロットルを設定値に変更
LIDAR_THROTTLE_DISTANCE = 4000

# 障害物がない時のスロットル値
LIDAR_THROTTLE_VALUE = 1

# ============================================================================
# コントローラー設定
# ============================================================================
# コントローラータイプ: "joystick", "pwm", "keyboard"
CONTROLLER_TYPE = "joystick"
HAVE_JOYSTICK = True
JOYSTICK_STEERING_SCALE = 1.0   # left=-1, right=1 に調整
JOYSTICK_THROTTLE_SCALE = -1.0  # reverse=-1, forward=1 に調整
JOYSTICK_DEVICE_FILE = "/dev/input/js0"

# ジョイスティックのボタン割り当て（F710）
JOYSTICK_A = 0       # ブレーキ
JOYSTICK_B = 1       # アクセル2
JOYSTICK_X = 2       # アクセル1
JOYSTICK_Y = 3       # 記録開始/停止
JOYSTICK_LB = 4
JOYSTICK_RB = 5
JOYSTICK_BACK = 6
JOYSTICK_S = 7       # 自動/手動走行切り替え
JOYSTICK_LOGICOOL = 8
JOYSTICK_LSTICKB = 9
JOYSTICK_RSTICKB = 10

# ジョイスティックの軸割り当て
JOYSTICK_AXIS_LEFT = 0   # ステアリング（左右）
JOYSTICK_AXIS_RIGHT = 4  # スロットル（上下）
JOYSTICK_HAT_LR = 0
JOYSTICK_HAT_DU = 1

# プロポPWM信号設定（read_pwm_signals.py でキャリブレーション）
PWM_I2C_ADDRESS = 0x08         # PWMコントローラのI2Cアドレス
PWM_I2C_BUS = 7                # I2Cバス番号（Jetson: 7, RPi: 1）
PWM_RAW_TO_US_SCALE = 1000.0   # RAW値をマイクロ秒に変換するスケール

# プロポPWM値の範囲（キャリブレーション後に設定）
# CH1: ステアリングチャンネル
PWM_CH1_LEFT_RAW = 1163       # 左最大時のRAW値
PWM_CH1_CENTER_RAW = 1478     # 中央時のRAW値
PWM_CH1_RIGHT_RAW = 1970      # 右最大時のRAW値

# CH2: スロットルチャンネル
PWM_CH2_FORWARD_RAW = 986     # 前進最大時のRAW値
PWM_CH2_NEUTRAL_RAW = 1478    # 中立時のRAW値
PWM_CH2_REVERSE_RAW = 1971    # 後退最大時のRAW値

# デッドゾーン設定（正規化値 0.0〜1.0、この値未満の入力は0にクリップ）
PWM_DEADZONE_STEERING = 0.03
PWM_DEADZONE_THROTTLE = 0.03
# 記録開始/停止の連続判定回数
PWM_RECORDING_CONSECUTIVE_COUNT = 3

# ============================================================================
# IMU/ジャイロ設定
# ============================================================================
IMU_TYPE = "AUTO"  # "AUTO", "BNO055", "BNO085"
# ジャイロを使った動的制御モード
MODE_DYNAMIC_CONTROL = "counter_steering"  # "counter_steering" or "lateral_g_throttle"

# ============================================================================
# RPMセンサー設定
# ============================================================================
RPM_MODE = 'i2c'              # 'i2c' (XIAO ESP32S3経由) or 'gpio' (GPIO直接計測)
RPM_I2C_BUS = 7               # I2Cバス番号（Jetson: 7, RPi: 1）
RPM_I2C_ADDRESS = 0x08        # XIAO ESP32S3 I2Cアドレス
RPM_GPIO_PIN = 4              # GPIOモード用ピン番号
RPM_MOTOR_POLE_PAIRS = 1      # モーターポールペア数（2極=1, 4極=2）
TIRE_DIAMETER_MM = 64.0       # タイヤ径 (mm)
GEAR_RATIO = 8.27             # ギア比
SPEED_UNIT = 'm/s'            # 速度単位 ('m/s', 'km/h', 'mph')

# ============================================================================
# オプティカルフローセンサー設定
# ============================================================================
# 出力速度の単位（内部計算はmm、最終出力をこの単位に変換）
OPTICAL_FLOW_SPEED_UNIT = 'm/s'
# ピクセル→m変換係数（路面から30mmの位置にセンサー設置で0.0001程度）
POSITION_SCALING_FACTOR = 0.0001
# MTF-01 シリアルポート設定
MTF01_PORT = "/dev/ttyTHS1"
MTF01_BAUD = 115200
MTF01_FLOW_SCALE = 4200.0     # フローカウント→ラジアン変換係数

# ============================================================================
# Speed PID制御設定（速度フィードバック制御）
# ============================================================================
# モデル出力をthrottleではなくtarget_speed(m/s)として扱い、
# RPM/オプティカルフローから算出した現在速度との差分をPID制御でthrottleに変換する。
USE_SPEED_CONTROL = False

# 速度ソース: 'rpm', 'optical_flow', 'fused'
# rpm: RPMセンサーからの速度 (m/s)
# optical_flow: オプティカルフローのvy (m/s)
# fused: RPM優先、RPM=0時はoptical_flowにフォールバック
SPEED_SOURCE = 'rpm'

# PIDゲイン
SPEED_PID_KP = 0.5            # 比例ゲイン
SPEED_PID_KI = 0.1            # 積分ゲイン
SPEED_PID_KD = 0.05           # 微分ゲイン
SPEED_PID_INTEGRAL_LIMIT = 1.0  # 積分項の上限（ワインドアップ防止）

# 速度の最大値 (m/s) — モデル出力の正規化に使用
# model_output[1] ∈ [-1, 1] → target_speed = model_output[1] * MAX_SPEED
MAX_SPEED = 3.0

# ============================================================================
# 出力・モニタリング設定
# ============================================================================
# ターミナルへの出力
TERMINAL_PRINT = True

# 走行中のデータ確認用WEBアプリ
MONITOR = True
MONITOR_PORT = 8000

# ============================================================================
# 走行記録設定
# ============================================================================
RECORD_FILE_NAME = "record"
RECORDS_DIRECTORY = "records"
RECORDS_DIRECTORY_ULTRASONIC_TEST = "records/ultrasonic_test.csv"
SAVE_FORMAT = "csv"  # "csv", "ndjson", "donkeycar"
IMAGES_DIRECTORY = "images"
AUTO_ZIP_ON_EXIT = True    # 終了時に記録フォルダを自動zip圧縮
AUTO_VIDEO_ON_EXIT = False # 終了時に走行画像から動画を自動生成
AUTO_VIDEO_FPS = 20        # 自動生成動画のフレームレート
AUTO_VIDEO_PREFIX = "cam"  # 動画化する画像のプレフィックス（"cam", "lidar", "cam0" 等）

# ============================================================================
# シミュレーションモード
# ============================================================================
SIM_MODE = False

# ============================================================================
# 位置推論とモデル切り替え設定
# ============================================================================
# annotation_training_d2j で学習した位置推論モデルによる自動運転モデル切り替え
USE_POSITION_SWITCHING = False
POSITION_MODEL_NAME = None                     # 位置推論モデルのファイル名
POSITION_MODEL_TYPE = "resnet18_location"      # モデルアーキテクチャ（donkey_location, resnet18_location）
POSITION_NUM_CLASSES = 8                       # 位置クラス数
POSITION_MODEL_INPUT_IMAGE = "cam1/image_array"

# 位置ごとのモデルマッピング（位置クラスID → 自動運転モデルファイル名）
POSITION_MODELS_MAP = {
    0: "model_position0.pth",
    1: "model_position1.pth",
    2: "model_position2.pth",
    3: "model_position3.pth",
    # 必要に応じて位置4-7も追加
}

# 位置クラスの名前（ログ表示用）
POSITION_CLASS_NAMES = [
    "Position0", "Position1", "Position2", "Position3",
    "Position4", "Position5", "Position6", "Position7"
]

# 位置推論の実行間隔（フレーム数）
POSITION_INFERENCE_INTERVAL = 5

# デフォルトモデル（推論できない場合に使用、None の場合は MODEL_NAME を使用）
POSITION_DEFAULT_MODEL = None

# ============================================================================
# YOLO物体検知設定
# ============================================================================
# YOLOモデルで物体を検知し、検知結果に応じて制御値を修正/モデルを切り替える
USE_YOLO_DETECTION = False

# YOLOモデル設定
YOLO_MODEL_PATH = "models/yolov8n.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.5   # 検知信頼度閾値 (0.0-1.0)
YOLO_IOU_THRESHOLD = 0.45         # NMS の IoU 閾値
YOLO_INPUT_SIZE = 640             # 入力画像サイズ
YOLO_DETECTION_INTERVAL = 3      # 検知実行間隔（フレーム数）

# 検知結果に基づく制御修正ルール（yolo_detection.py で使用）
YOLO_CONTROL_RULES = {
    0: {  # car
        "steering_offset": 0.0,
        "throttle_scale": 0.5,
        "priority": 8,
        "description": "Car detected - Reduce speed"
    },
    1: {  # route
        "steering_offset": 0.0,
        "throttle_scale": 1.0,
        "priority": 5,
        "description": "Route detected - Normal speed"
    },
    2: {  # signal
        "steering_offset": 0.0,
        "throttle_scale": 0.6,
        "priority": 9,
        "description": "Signal detected - Prepare to stop"
    },
    3: {  # stop
        "steering_offset": 0.0,
        "throttle_scale": 0.0,
        "priority": 10,
        "description": "Stop sign - Full stop"
    },
    4: {  # park
        "steering_offset": 0.0,
        "throttle_scale": 0.3,
        "priority": 7,
        "description": "Parking area - Slow down"
    },
}

# 検知結果に基づくモデル切り替え設定
YOLO_MODEL_SWITCHING = {
    0: "car_traffic_model.pth",
    1: "route_following_model.pth",
    2: "signal_aware_model.pth",
    3: "stop_zone_model.pth",
    4: "parking_model.pth",
}

# 検知対象クラスのフィルタリング（None = 全クラス検知）
YOLO_TARGET_CLASSES = None

# 検知結果の表示設定
YOLO_DISPLAY_DETECTIONS = True        # ターミナルに表示
YOLO_SAVE_ANNOTATED_IMAGES = False    # 画像に描画して保存

# YOLO物体追従制御設定
USE_YOLO_OBJECT_TRACKING = False
YOLO_TRACKING_TARGET_CLASSES = [4]         # 追従対象クラスID
YOLO_TRACKING_STEERING_GAIN = 0.8          # ステアリング補正ゲイン (0.0-2.0推奨)
YOLO_TRACKING_CENTER_DEADZONE = 0.1        # 中心不感帯（画像幅比, 0.0-0.3推奨）

# YOLO障害物回避制御設定
USE_YOLO_OBSTACLE_AVOIDANCE = False
YOLO_OBSTACLE_CLASSES = [0]                # 回避対象クラスID
YOLO_OBSTACLE_AVOIDANCE_GAIN = 1.2         # 回避ステアリングゲイン (0.5-2.0推奨)
YOLO_OBSTACLE_SIZE_THRESHOLD = 0.15        # 回避判定する物体サイズ閾値（画像面積比）
YOLO_OBSTACLE_CENTER_ZONE = 0.4            # 中央エリア幅（画像幅比）

# YOLOクラス名（カスタム学習用、学習データに合わせて設定）
YOLO_CLASS_NAMES = {
    0: "car",
    1: "route",
    2: "signal",
    3: "stop",
    4: "park",
}

# COCO dataset を使用する場合は上記を以下に差し替え:
# YOLO_CLASS_NAMES = {
#     0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane",
#     5: "bus", 6: "train", 7: "truck", 8: "boat", 9: "traffic light",
#     10: "fire hydrant", 11: "stop sign", 12: "parking meter", 13: "bench",
#     14: "bird", 15: "cat", 16: "dog", 17: "horse", 18: "sheep", 19: "cow"
#     # 以下省略（80クラス）
# }
