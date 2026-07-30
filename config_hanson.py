
# # config_hanson.pyW
# #必要に応じてコメントアウトを外して使用してください。
# # coding:utf-8
# # machine type: m1-12 for hanson
PERCEPTION_ENABLED = True
PERCEPTION_APPLY_CORRECTION = True
PERCEPTION_INPUT_IMAGE = "cam0"
PERCEPTION_DEBUG = True
# # device: Raspberry Pi 4B, ydlidar tfmini plus, Raspberry Pi Camera Module V3

# # 使用するセンサー
ACTIVE_SENSORS = ["ultrasonic","camera_0"]
#ACTIVE_SENSORS = ["ultrasonic"]
#ACTIVE_SENSORS = ["uturtleltrasonic"]

# # 走行プラン（判断モード）選択
#PLAN = "right_left_3"
#PLAN = "right_left_3"      # バンバン
#PLAN = "wall_follow"       # 壁沿いON-OFF（右手法/左手法）
#PLAN = "wall_follow_pid"   # 壁沿いPID
#PLAN = "center_follow_pid" # 中央維持
#PLAN = "gap_follow"        # 開いた方へ（基本選択肢）
PLAN = "racer"             # やり込み（上級）
#PLAN = "donkeycar"
#PLAN = "go_straight"

STOP_RANGE = 250       # 停止判断に使用する距離

# # ステアリングのPWM値
STEERING_CENTER_PWM = 373
STEERING_WIDTH_PWM = 120
#STEERING_RIGHT_PWM = STEERING_CENTER_PWM - STEERING_WIDTH_PWM
#STEERING_LEFT_PWM = STEERING_CENTER_PWM + STEERING_WIDTH_PWM

# # スロットルのPWM値
# # モーターの回転音を聞き、音が変わらないところが最大/最小値とする
# # ニュートラル付近の値を入力している状態でスイッチをONにするとピッピッピッと上がり調子の音がする。
THROTTLE_STOPPED_PWM = 370   # めやす: 370〜400
THROTTLE_FORWARD_PWM = 475
THROTTLE_REVERSE_PWM = 280



MODEL_DIR = "models" #hason用
MODEL_NAME = "donkeycar_20260404_211906.pth"
MODEL_PATH = f"{MODEL_DIR}/{MODEL_NAME}"

# # 走行プラン（判断モード）選択
# PLAN = "nn"
# # 機械学習モデル設定（NN/CNN）
# MODEL_DIR = "data_viewer/models" #hason用
# MODEL_NAME = "nn_20260320_234209_5_64_64_64_2.pth"
# INFERENCE_ENGINE = "pytorch"
\
# # ボタン定数スロットル出力
# FORWARD_STRAIGHT = 0.6  # デフォルトXボタン
# FORWARD_CORNER = 0.3    # デフォルトAボタン

# # ジョイスティックの軸割り当て
# JOYSTICK_AXIS_LEFT = 0   # ステアリング（左右、左スティック用）
#B # JOYSTICK_AXIS_LEFT = 3  # ステアリング（左右、右スティック用）
# JOYSTICK_AXIS_RIGHT = 4  # スロットル（上下、右スティック用）
# # JOYSTICK_AXIS_RIGHT = 1  # スロットル（上下、左スティック用）

# # ============================================================================
# # カメラ設定
# # ============================================================================
IMAGE_W = 224
IMAGE_H = 126 #224
IMAGE_DEPTH = 3          # RGB=3, モノクロ=1
# CAMERA_FRAMERATE = 60

CAMERA_TUNING_FILE = "/home/pi/togikaidrive-dev/setup/imx219_200d.json"

# # ============================================================================
# B# 共通（速度・検知距離。全ルールで参照）
# # ============================================================================
FORWARD_STRAIGHT = 0.8 # 直線の速度。大きいほど速い
FORWARD_CORNER   = 0.3  # コーナーの速度。大きいほど曲がりながら速い
DETECTION_RANGE  = 150   # 前壁を検知し始める距離[mm]（RL3 / gap_follow 等）
RIGHT_LEFT_RANGE = 400   # 左右壁で旋回判定する距離[mm]（RL3）

# # ============================================================================
# # wall_follow / wall_follow_pid（右手法・左手法は HAND_SIDE で切替）
# # ============================================================================
HAND_SIDE = "left"            # 追従する壁。"right"=右手法 / "left"=左手法
TARGET_RANGE            = 180  # 壁との目標距離[mm]。小さいほど壁に寄る
TARGET_RANGE_ADJUSTMENT = 50   # 目標±この幅[mm]は直進扱い(不感帯)。大きいほど鈍く安定
ALL_FOLLOW_USE_ALIGNMENT = True  # 壁の傾き補正ON/OFF。まずFalseで斜め走行を体感→Trueで改善
WALL_FOLLOW_K_ANGLE       = 0.5   # 角度補正の強さ。USE_ALIGNMENT=True時のみ有効(目安0〜1)

# # ============================================================================
# # wall_follow_pid 専用（PID。PDまで推奨・Iは0から）
# # ============================================================================
#K_P = 0.005   # 偏差への反応の強さ。大きいほど機敏だが振動しやすい
#K_I = 0.0     # 偏差の蓄積補正。通常0。残留ズレが取れない時だけ少し
#K_D = 0.001  # 変化を抑える。大きいほど行き過ぎ・蛇行を抑制

# # ============================================================================
# # center_follow_pid（左右の中央を維持）
# # ============================================================================
CENTER_K_P = 0.01            # 中央維持の反応の強さ
CENTER_K_I = 0.0              # 蓄積補正（通常0）
CENTER_K_D = 0.00075           # 行き過ぎ抑制
CENTER_FALLBACK_RANGE = 1000   # 片側の壁がなくなった時に代替として使う距離[mm]

# # ============================================================================
# # gap_follow / racer
# # ============================================================================

# --- gap_follow（基本選択肢：開いている方へ進む）---
GAP_STEER_GAIN      = 4.0    # 操舵の鋭さ。大きいほど少しの左右差で強く曲がる(0=曲がらない/1=標準/2〜3でバンバンに近づく)
GAP_FOLLOW_BRAKE_DIST = 600  # 前方がこの距離[mm]より近いと CORNER 速度に落とす。STOP_RANGE(250)より大きい値にすること

# --- racer（やり込み枠：速さ重視。4つは相互依存）---
RACER_STEER_GAIN     = 3.0    # 操舵の鋭さ。大きいほど鋭く曲がるが上げ過ぎると蛇行(目安0〜2)
RACER_SPEED_GAIN     = 1.0    # 直線の伸び。大きいほど速いがコーナー進入が破綻しやすい(目安0〜1.5)
RACER_BRAKE_DIST     = 600    # 減速開始距離[mm]。前方がこの距離より近いと減速を始める。小さいほど突っ込む
RACER_STEER_SLOWDOWN = 0.8    # 曲げ時の減速の強さ。大きいほどカーブで安全に減速、小さいと曲がりきれない(0〜1)
RACER_SPEED_CEIL     = 0.9    # 速度上限(安全側・固定推奨)。どれだけ攻めてもこの値で頭打ち

# # カメラ0のフリップ設定
CAMERA_0_VFLIP = True
CAMERA_0_HFLIP = True

# # カメラスロット設定
# # TYPE: None=プラットフォーム自動検出, "jetson", "pi", "sv125", "usb_generic", "lidar"
# # DEVICE_ID: None=自動検出, 整数=手動指定（CSI: sensor-id, USB: /dev/videoN のN）
CAMERA_0_TYPE = None
CAMERA_0_DEVICE_ID = 0

# # ============================================================================
# # 走行記録設定
# # ============================================================================
# RECORD_FILE_NAME = "record"
# RECORDS_DIRECTORY = "records"
# RECORDS_DIRECTORY_ULTRASONIC_TEST = "records/ultrasonic_test.csv"
# SAVE_FORMAT = "donkeycar"  # "csv", "ndjson", "donkeycar"
# IMAGES_DIRECTORY = "images"
# AUTO_ZIP_ON_EXIT = False    # 終了時に記録フォルダを自動zip圧縮
# AUTO_VIDEO_ON_EXIT = False # 終了時に走行画像から動画を自動生成
# AUTO_VIDEO_FPS = 20        # 自動生成動画のフレームレート
# AUTO_VIDEO_PREFIX = "cam"  # 動画化する画像のプレフィックス（"cam", "lidar", "cam0" 等）
