# run.py
# coding:utf-8

# libcameraのログを抑制（importより前に設定）
import os
os.environ['LIBCAMERA_LOG_LEVELS'] = 'ERROR'

import shutil

# config.pyが存在しない場合、config_default.pyからコピー
_config_path = os.path.join(os.path.dirname(__file__), 'config.py')
_default_path = os.path.join(os.path.dirname(__file__), 'config_default.py')
if not os.path.exists(_config_path):
    if os.path.exists(_default_path):
        shutil.copy2(_default_path, _config_path)
        print("config_default.py から config.py を作成しました")
    else:
        raise FileNotFoundError("config_default.py が見つかりません")

import config

# config_hanson.py が存在する場合、config の値を上書き
_hanson_path = os.path.join(os.path.dirname(__file__), 'config_hanson.py')
if os.path.exists(_hanson_path):
    import importlib.util
    _spec = importlib.util.spec_from_file_location("config_hanson", _hanson_path)
    _hanson = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_hanson)
    _overridden = []
    for _name in dir(_hanson):
        if not _name.startswith('_'):
            setattr(config, _name, getattr(_hanson, _name))
            _overridden.append(_name)
    if _overridden:
        print("config_hanson.py でパラメータを上書き実施しました。")

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.info("ライブラリの初期化に数秒かかります...")
import time
from datetime import datetime
from pytz import timezone
jst = timezone('Asia/Tokyo')
import sys
import signal
import atexit
import gc


class SpikeFilter:
    THRESHOLD_MM = 300
    CONFIRM_COUNT = 2
    _AGREE_RATIO = 3
    _CUTOFF_CONFIRM_RATIO = 3

    def __init__(self):
        self._prev = {}
        self._pending = {}
        self._pending_count = {}
        self._cutoff_count = {}

    def update(self, name, value):
        if value is None:
            return self._prev.get(name, 0)

        prev = self._prev.get(name)

        if value == config.CUTOFF_RANGE:
            count = self._cutoff_count.get(name, 0) + 1
            self._cutoff_count[name] = count
            self._pending.pop(name, None)
            self._pending_count.pop(name, None)

            if (
                prev is None
                or count >= self.CONFIRM_COUNT * self._CUTOFF_CONFIRM_RATIO
            ):
                self._prev[name] = value
                return value

            return prev

        self._cutoff_count.pop(name, None)

        if prev is None or abs(value - prev) <= self.THRESHOLD_MM:
            self._prev[name] = value
            self._pending.pop(name, None)
            self._pending_count.pop(name, None)
            return value

        pending = self._pending.get(name)
        agree_threshold_mm = self.THRESHOLD_MM / self._AGREE_RATIO

        if (
            pending is not None
            and abs(value - pending) <= agree_threshold_mm
        ):
            count = self._pending_count.get(name, 1) + 1
        else:
            pending = value
            count = 1

        self._pending[name] = pending
        self._pending_count[name] = count

        if count >= self.CONFIRM_COUNT:
            self._prev[name] = value
            self._pending.pop(name, None)
            self._pending_count.pop(name, None)
            return value

        return prev


# togikaidriveのモジュール
if config.SIM_MODE:
    import motor_sim as motor
else:
    import motor

from planner import DefaultPlanner, SpeedPIDController, estimate_speed
import monitor
from joystick import Joystick, KeyboardController
from pwm_controller import PWMController
from record_manager import RecordManager
from data_aggregator import DataAggregator
from sensor_buffer_manager import (
    init_buffer_manager,
    add_sensor_to_buffer,
    get_sensor_data_with_statistics,
)

# 以下はconfig.pyでの設定によりimport
if "ultrasonic" in config.ACTIVE_SENSORS:
    if config.SIM_MODE:
        import ultrasonic_sim as ultrasonic
    else:
        import ultrasonic

if any(s.startswith("camera_") for s in config.ACTIVE_SENSORS):
    if config.SIM_MODE:
        import camera_sim as camera
    else:
        import camera

if "imu" in config.ACTIVE_SENSORS:
    if config.SIM_MODE:
        import imu_sim as imu
    else:
        import imu

if "lidar" in config.ACTIVE_SENSORS:
    import lidar

if "optical_flow" in config.ACTIVE_SENSORS:
    import opticalflow

if "rpm" in config.ACTIVE_SENSORS:
    from rpm_sensor import RPMSensor

# AIモデルのチェック、利用したいモデルがあればリストに追加する
SEQUENCE_MODEL_PLANS = ["gru", "tcn", "causal_cnn"]
if config.PLAN in [
    "nn",
    "donkeycar",
    "resnet18",
    "mobilevit_xxs",
    "edgenext_xx_small",
] + SEQUENCE_MODEL_PLANS:
    from train_pytorch import (
        NeuralNetwork,
        FlexibleNeuralNetwork,
        ConvolutionalNeuralNetwork,
        load_model,
        get_model_from_catalog,
    )
    import torch

if config.PLAN in SEQUENCE_MODEL_PLANS:
    from train_pytorch import load_sequence_model

# 位置推論モデルのインポート（必要な場合）
if config.USE_POSITION_SWITCHING:
    try:
        from position_inference import (
            load_position_model,
            load_position_specific_models,
            infer_position,
        )
        submodule_path = os.path.join(
            os.path.dirname(__file__),
            'annotation_training_d2j',
        )
        if submodule_path not in sys.path:
            sys.path.insert(0, submodule_path)
        from model_catalog import get_model as get_location_model
        logger.info("位置推論モデルモジュールをインポートしました")
    except ImportError as e:
        logger.error(f"位置推論モジュールのインポートに失敗: {e}")
        config.USE_POSITION_SWITCHING = False

# YOLO物体検知のインポート（必要な場合）
if config.USE_YOLO_DETECTION:
    try:
        from yolo_detection import (
            load_yolo_model,
            load_yolo_specific_models,
            detect_objects,
            apply_detection_control_modification,
            select_model_by_detection,
        )
        from ultralytics import YOLO
        logger.info("YOLOモジュールをインポートしました")
    except ImportError as e:
        logger.error(f"YOLOモジュールのインポートに失敗: {e}")
        logger.error("Ultralyticsをインストールしてください: pip install ultralytics")
        config.USE_YOLO_DETECTION = False


# センサー値やパラメータをブラウザで変更できるmonitorを利用
monitor_thread = None
if config.MONITOR:
    import threading
    monitor_thread = threading.Thread(
        target=monitor.run,
        kwargs={
            "host": "0.0.0.0",
            "port": config.MONITOR_PORT,
            "debug": False,
        },
        daemon=True,
    )
    monitor_thread.start()


# --- 初期化 ---
def initialize_system():
    """
    各種設定値の確認、モジュールのインスタンスを初期化する。
    """
    from device_detection import detect_device

    device_info = detect_device()
    config.DEVICE_TYPE = device_info.device_type
    config.PLATFORM_NAME = device_info.platform_name
    config.GPIO_BACKEND = device_info.gpio_backend
    config.I2C_BUS = device_info.i2c_bus

    logger.info(
        f"Platform detected: {config.PLATFORM_NAME}, I2C Bus: {config.I2C_BUS}"
    )

    # 選択したプランチェック
    logger.info(f"PLAN: {config.PLAN}")

    if config.PLAN not in config.PLAN_LIST:
        logger.error("Please set plan from %s", config.PLAN_LIST)
        sys.exit()

    # モジュールの初期化
    motor_instance = motor.Motor()

    # 有効なセンサーインスタンスを作成
    active_sensor_instances = {}

    if "ultrasonic" in config.ACTIVE_SENSORS:
        active_sensor_instances.update(
            {
                sensor_name: ultrasonic.Ultrasonic(
                    sensor_name=sensor_name
                )
                for sensor_name in config.ULTRASONIC_SENSOR_LIST
            }
        )

    if "imu" in config.ACTIVE_SENSORS:
        imu_type = getattr(
            config,
            'IMU_TYPE',
            'AUTO',
        )

        if imu_type == "AUTO":
            imu_type = imu.detect_imu_type(config)
            logger.info(
                f"IMU type: {imu_type} (自動検出)"
            )
        else:
            logger.info(
                f"IMU type: {imu_type} (config指定)"
            )

        if imu_type == "BNO085":
            active_sensor_instances["imu"] = imu.BNO085()
        else:
            active_sensor_instances["imu"] = imu.BNO055()

    # RPMセンサー初期化
    if "rpm" in config.ACTIVE_SENSORS:
        rpm_mode = getattr(
            config,
            'RPM_MODE',
            'i2c',
        )

        rpm_sensor = RPMSensor(
            mode=rpm_mode,
            i2c_bus=getattr(
                config,
                'RPM_I2C_BUS',
                config.I2C_BUS,
            ),
            i2c_address=getattr(
                config,
                'RPM_I2C_ADDRESS',
                0x08,
            ),
            gpio_pin=getattr(
                config,
                'RPM_GPIO_PIN',
                4,
            ),
            motor_pole_pairs=getattr(
                config,
                'RPM_MOTOR_POLE_PAIRS',
                1,
            ),
            tire_diameter_mm=getattr(
                config,
                'TIRE_DIAMETER_MM',
                64.0,
            ),
            gear_ratio=getattr(
                config,
                'GEAR_RATIO',
                8.27,
            ),
            speed_unit=getattr(
                config,
                'SPEED_UNIT',
                'm/s',
            ),
        )

        if rpm_sensor.enabled:
            active_sensor_instances["rpm"] = rpm_sensor
        else:
            logger.warning(
                "RPMセンサーの初期化に失敗しました"
            )

    # オプティカルフローセンサー初期化
    if "optical_flow" in config.ACTIVE_SENSORS:
        print("\n--- オプティカルフロー初期化開始 ---")

        try:
            of_sensor = opticalflow.detect_opticalflow()

            if of_sensor and of_sensor.is_active.value:
                active_sensor_instances["optical_flow"] = of_sensor
                logger.info(
                    "オプティカルフローセンサー初期化完了"
                )
            else:
                logger.warning(
                    "オプティカルフローセンサーの初期化に失敗しました"
                )

        except Exception as e:
            logger.error(
                f"オプティカルフローセンサー初期化エラー: {e}"
            )

        print("--- オプティカルフロー初期化完了 ---\n")

    # LiDAR初期化
    lidar_instance = None

    if "lidar" in config.ACTIVE_SENSORS:
        detected = lidar.detect_lidar(config)

        if detected is not None:
            logger.info(
                f"LiDAR detected: {detected}"
            )
        else:
            logger.info(
                f"LiDAR type: {config.LIDAR_TYPE} (config指定)"
            )

        if config.LIDAR_TYPE == "NONE":
            logger.warning(
                "LiDAR is NONE — skipping LiDAR initialization"
            )
        else:
            print("\n--- LiDAR初期化開始 ---")
            lidar_instance = lidar.create_lidar(
                lidar_type=config.LIDAR_TYPE
            )
            active_sensor_instances["lidar"] = lidar_instance
            print("--- LiDAR初期化完了 ---")
            print("LiDARのスキャン開始を待機中...")
            time.sleep(3)
            print("--- LiDAR準備完了 ---\n")

    # カメラ初期化
    active_camera_count = sum(
        1
        for i in range(4)
        if f"camera_{i}" in config.ACTIVE_SENSORS
    )

    if (
        active_camera_count >= 4
        and config.CAMERA_FRAMERATE > 30
    ):
        logger.warning(
            f"カメラ4台構成のためフレームレートを "
            f"{config.CAMERA_FRAMERATE}fps → 30fps に制限 "
            f"（GPUメモリ保護）"
        )
        config.CAMERA_FRAMERATE = 30

    for cam_idx in range(4):
        cam_name = f"camera_{cam_idx}"

        if cam_name in config.ACTIVE_SENSORS:
            print(
                f"\n--- カメラ初期化開始 ({cam_name}) ---"
            )

            active_sensor_instances[cam_name] = camera.create_camera(
                device_id=getattr(
                    config,
                    f'CAMERA_{cam_idx}_DEVICE_ID',
                    cam_idx,
                ),
                camera_type=getattr(
                    config,
                    f'CAMERA_{cam_idx}_TYPE',
                    None,
                ),
                lidar_instance=lidar_instance,
            )

            print(
                f"--- カメラ初期化完了 ({cam_name}) ---\n"
            )

    # プランナーの初期化
    planner_instance = DefaultPlanner()

    # モデルの初期ロード
    model = reload_model()

    model_required_plans = [
        "nn",
        "donkeycar",
        "resnet18",
        "mobilevit_xxs",
        "edgenext_xx_small",
    ] + SEQUENCE_MODEL_PLANS

    if (
        model is None
        and config.PLAN in model_required_plans
    ):
        logger.critical(
            "モデルのロードに失敗しました。起動を停止します。"
            f" PLAN={config.PLAN}, MODEL_NAME={config.MODEL_NAME}"
        )
        sys.exit(1)

    # 位置推論システム
    position_model = None
    position_models_dict = {}

    if config.USE_POSITION_SWITCHING:
        position_model = load_position_model()
        position_models_dict = load_position_specific_models()

        if position_model is None:
            logger.warning(
                "位置推論モデルのロードに失敗しました。通常モードで動作します"
            )
            config.USE_POSITION_SWITCHING = False

    # YOLO物体検知システム
    yolo_model = None
    yolo_models_dict = {}

    if config.USE_YOLO_DETECTION:
        yolo_model = load_yolo_model()
        yolo_models_dict = load_yolo_specific_models()

        if yolo_model is None:
            logger.warning(
                "YOLOモデルのロードに失敗しました。通常モードで動作します"
            )
            config.USE_YOLO_DETECTION = False

    logger.info("System initialized.")

    return (
        motor_instance,
        active_sensor_instances,
        planner_instance,
        model,
        position_model,
        position_models_dict,
        yolo_model,
        yolo_models_dict,
    )


def _detect_inference_engine(model_path):
    """ファイル拡張子から推論エンジンを自動判定する。"""
    if model_path.endswith('.engine'):
        base = model_path.replace('.engine', '')

        for suffix in ['_tensorrt', '']:
            if base.endswith(suffix):
                pth_path = (
                    base[:-len(suffix)] + '.pth'
                    if suffix
                    else base + '.pth'
                )

                if os.path.exists(pth_path):
                    return "tensorrt", pth_path

        return "tensorrt", None

    elif model_path.endswith('.xml'):
        base = model_path.replace('.xml', '')

        for suffix in ['_openvino', '']:
            if base.endswith(suffix):
                pth_path = (
                    base[:-len(suffix)] + '.pth'
                    if suffix
                    else base + '.pth'
                )

                if os.path.exists(pth_path):
                    return "openvino", pth_path

        return "openvino", None

    else:
        return "pytorch", None


def reload_model(model_path=None):
    """モデルを再ロードする。"""
    model = None

    if not model_path:
        if not config.MODEL_NAME:
            logger.warning(
                f"MODEL_NAME is not set or empty. "
                f"Skipping model loading. (PLAN: {config.PLAN})"
            )
            return None

        model_path = os.path.join(
            config.MODEL_DIR,
            config.MODEL_NAME,
        )

    inference_engine, pth_path = _detect_inference_engine(
        model_path
    )

    logger.info(
        f"Loading model: PLAN={config.PLAN}, "
        f"path={model_path}, engine={inference_engine}"
    )

    if not os.path.exists(model_path):
        logger.error(
            f"モデルファイルが見つかりません: {model_path}"
        )
        logger.error(
            f"config.py の MODEL_NAME を確認してください "
            f"（現在: {config.MODEL_NAME}）"
        )
        return None

    if config.PLAN in [
        "nn",
        "donkeycar",
        "resnet18",
        "mobilevit_xxs",
        "edgenext_xx_small",
    ] + SEQUENCE_MODEL_PLANS:
        device = torch.device(
            'cuda'
            if torch.cuda.is_available()
            else 'cpu'
        )

        if torch.cuda.is_available():
            logger.info(
                f"GPU detected: "
                f"{torch.cuda.get_device_name(0)}, "
                f"CUDA: {torch.version.cuda}"
            )
        else:
            logger.info("Running on CPU")
    else:
        device = None

    if config.PLAN == "nn":
        try:
            checkpoint = torch.load(
                model_path,
                weights_only=False,
                map_location='cpu',
            )

            if 'hidden_layers' in checkpoint:
                input_size = checkpoint.get(
                    'input_size',
                    len(config.ULTRASONIC_SENSOR_LIST),
                )
                output_size = checkpoint.get(
                    'output_size',
                    2,
                )
                hidden_layers = checkpoint['hidden_layers']
                use_dropout = checkpoint.get(
                    'use_dropout',
                    False,
                )
                dropout_rate = checkpoint.get(
                    'dropout_rate',
                    0.2,
                )

                model = FlexibleNeuralNetwork(
                    input_size,
                    output_size,
                    hidden_layers,
                    use_dropout,
                    dropout_rate,
                )

                model.load_state_dict(
                    checkpoint['model_state_dict']
                )

                model._normalization_params = checkpoint.get(
                    'normalization_params'
                )
                model._selected_sensors = checkpoint.get(
                    'selected_sensors',
                    [],
                )

                logger.info(
                    f"NN model loaded (data_viewer format): "
                    f"hidden={hidden_layers}, "
                    f"dropout={use_dropout}, "
                    f"sensors={model._selected_sensors}"
                )

            else:
                input_dim = len(
                    config.ULTRASONIC_SENSOR_LIST
                )
                output_dim = 2

                model = NeuralNetwork(
                    input_dim,
                    output_dim,
                    config.HIDDEN_DIM,
                    config.NUM_HIDDEN_LAYERS,
                )

                model.load_state_dict(
                    checkpoint['model_state_dict']
                )

                model._normalization_params = None
                model._selected_sensors = None

                logger.info(
                    "NN model loaded (train_pytorch format)"
                )

        except Exception as e:
            logger.error(
                f"Failed to load NN model: {e}"
            )
            import traceback
            traceback.print_exc()
            return None

        if device is not None:
            model = model.to(device)

        logger.info(
            f"NeuralNetwork model on {device}"
        )

    elif config.PLAN in [
        "donkeycar",
        "resnet18",
        "mobilevit_xxs",
        "edgenext_xx_small",
    ]:
        try:
            if inference_engine == "tensorrt":
                from train_pytorch import TensorRTModel
                model = TensorRTModel(
                    model_path
                )
                logger.info(
                    f"{config.PLAN} TensorRT engine loaded: {model_path}"
                )

            else:
                detected_input_size = None

                try:
                    checkpoint = torch.load(
                        model_path,
                        weights_only=False,
                        map_location='cpu',
                    )

                    if (
                        isinstance(checkpoint, dict)
                        and 'input_size' in checkpoint
                    ):
                        detected_input_size = tuple(
                            checkpoint['input_size']
                        )
                        logger.info(
                            f"チェックポイントから入力サイズを検出: "
                            f"{detected_input_size}"
                        )

                except Exception as e:
                    logger.warning(
                        f"入力サイズの検出に失敗（デフォルトを使用）: {e}"
                    )

                if detected_input_size is not None:
                    config_size = (
                        config.IMAGE_H,
                        config.IMAGE_W,
                    )

                    if detected_input_size != config_size:
                        raise ValueError(
                            "モデルの入力サイズとconfig.pyの画像サイズが一致しません。\n"
                            f"  チェックポイントの入力サイズ (H, W): {detected_input_size}\n"
                            f"  config.pyの画像サイズ (IMAGE_H, IMAGE_W): {config_size}\n"
                            "config.pyの IMAGE_H, IMAGE_W を学習時と同じ値に修正してください。"
                        )

                model = get_model_from_catalog(
                    config.PLAN,
                    for_training=False,
                    input_size=detected_input_size,
                )

                if model is not None:
                    load_model(
                        model,
                        model_path=model_path,
                    )

                    if device is not None:
                        model = model.to(device)

                    logger.info(
                        f"{config.PLAN} model loaded from model_catalog: "
                        f"{model_path} on {device}"
                    )
                else:
                    logger.warning(
                        f"Could not create model for PLAN: {config.PLAN}"
                    )
                    model = None

            if hasattr(
                config,
                'MODEL_INPUT_IMAGE',
            ):
                logger.info(
                    f"Inference will use camera based on MODEL_INPUT_IMAGE: "
                    f"{config.MODEL_INPUT_IMAGE}"
                )

        except Exception as e:
            logger.error(
                f"Failed to load model from catalog for {config.PLAN}: {e}"
            )
            model = None

    elif config.PLAN in SEQUENCE_MODEL_PLANS:
        try:
            if inference_engine == "tensorrt":
                if pth_path is None:
                    logger.error(
                        "TensorRT engine requires corresponding .pth for config: "
                        f"{model_path.replace('.engine', '.pth')}"
                    )
                    return None

                checkpoint = torch.load(
                    pth_path,
                    map_location='cpu',
                    weights_only=False,
                )

                seq_config = checkpoint['config']
                selected_sources = checkpoint.get(
                    'selected_sources',
                    [],
                )

                checkpoint_arch = seq_config.get(
                    'model_arch',
                    checkpoint.get(
                        'model_arch',
                        '',
                    ),
                )

                del checkpoint

                if checkpoint_arch != config.PLAN:
                    logger.error(
                        f"PLAN='{config.PLAN}' とモデルのアーキテクチャ "
                        f"'{checkpoint_arch}' が一致しません。"
                    )
                    return None

                from train_pytorch import TensorRTModel

                model = TensorRTModel(
                    model_path
                )
                model._sequence_config = seq_config
                model._selected_sources = selected_sources

                logger.info(
                    f"Sequence TensorRT engine ({checkpoint_arch}) loaded: {model_path}"
                )

            else:
                model, seq_config, selected_sources = load_sequence_model(
                    model_path,
                    device,
                )

                checkpoint_arch = seq_config.get(
                    'model_arch',
                    '',
                )

                if checkpoint_arch != config.PLAN:
                    logger.error(
                        f"PLAN='{config.PLAN}' とモデルのアーキテクチャ "
                        f"'{checkpoint_arch}' が一致しません。"
                    )
                    return None

                model._sequence_config = seq_config
                model._selected_sources = selected_sources

            logger.info(
                f"Sequence model ({config.PLAN}) loaded: "
                f"seq_len={seq_config['seq_len']}, "
                f"pred_horizon={seq_config['pred_horizon']}, "
                f"img_size={seq_config.get('img_size', (128, 128))}"
            )

        except Exception as e:
            logger.error(
                f"Failed to load sequence model: {e}"
            )
            import traceback
            traceback.print_exc()
            model = None

    else:
        logger.warning(
            "PLAN is not supported for model loading. No model reloaded."
        )

    if model is not None:
        model.eval()
        logger.info(
            "Model set to eval mode for inference"
        )

    if (
        model is not None
        and getattr(
            config,
            'USE_FP16',
            False,
        )
    ):
        model.half()
        logger.info(
            f"Model converted to FP16 (half precision) on {device}"
        )

    if (
        model is not None
        and device is not None
        and torch.cuda.is_available()
    ):
        allocated = (
            torch.cuda.memory_allocated(0)
            / 1024**2
        )

        reserved = (
            torch.cuda.memory_reserved(0)
            / 1024**2
        )

        logger.info(
            f"GPU Memory - Allocated: {allocated:.1f}MB, "
            f"Reserved: {reserved:.1f}MB"
        )

    return model


# --- 終了処理 ---
def cleanup_system(
    motor_instance,
    planner_instance,
    active_sensor_instances_dict,
    controller_instance=None,
):
    import time

    logger.info(
        "終了処理を開始..."
    )

    if controller_instance is not None:
        if (
            hasattr(
                controller_instance,
                'close',
            )
            and callable(
                controller_instance.close
            )
        ):
            try:
                logger.info(
                    "コントローラーをクリーンアップ中..."
                )
                controller_instance.close()
            except Exception as e:
                logger.error(
                    f"Error during controller cleanup: {e}"
                )

    if motor_instance is not None:
        try:
            motor_instance.set_steering_pwm_value(0)
            motor_instance.set_throttle_pwm_value(0)
            time.sleep(0.1)
            motor_instance.cleanup()
        except Exception as e:
            logger.error(
                f"Error during motor cleanup: {e}"
            )

    if active_sensor_instances_dict:
        camera_sensors = [
            name
            for name in active_sensor_instances_dict.keys()
            if name.startswith('camera_')
        ]

        for camera_name in camera_sensors:
            if camera_name in active_sensor_instances_dict:
                sensor = active_sensor_instances_dict[camera_name]

                if (
                    hasattr(
                        sensor,
                        'cleanup',
                    )
                    and callable(
                        sensor.cleanup
                    )
                ):
                    logger.info(
                        f"Cleaning up {camera_name}..."
                    )

                    try:
                        sensor.cleanup()
                        time.sleep(0.1)
                    except Exception as e:
                        logger.error(
                            f"Error cleaning up {camera_name}: {e}"
                        )

        for sensor_name, sensor in active_sensor_instances_dict.items():
            if not sensor_name.startswith('camera_'):
                if (
                    hasattr(
                        sensor,
                        'shutdown',
                    )
                    and callable(
                        sensor.shutdown
                    )
                ):
                    try:
                        sensor.shutdown()
                    except Exception as e:
                        logger.error(
                            f"Error shutting down {sensor_name}: {e}"
                        )

                elif (
                    hasattr(
                        sensor,
                        'cleanup',
                    )
                    and callable(
                        sensor.cleanup
                    )
                ):
                    try:
                        sensor.cleanup()
                    except Exception as e:
                        logger.error(
                            f"Error cleaning up {sensor_name}: {e}"
                        )

    if planner_instance is not None:
        try:
            planner_instance.cleanup()
        except Exception as e:
            logger.error(
                f"Error cleaning up planner: {e}"
            )

    if (
        config.MONITOR
        and monitor_thread
        and monitor_thread.is_alive()
    ):
        logger.info(
            "monitorスレッドの終了を待機中..."
        )
        monitor.shutdown_signal = True
        monitor_thread.join(
            timeout=1.0
        )

    gc.collect()

    logger.info(
        "System cleanup complete."
    )


def _zip_record_folder(record_manager):
    if not getattr(
        config,
        'AUTO_ZIP_ON_EXIT',
        False,
    ):
        return

    if not hasattr(
        record_manager,
        'records',
    ) or len(
        record_manager.records
    ) == 0:
        logger.info(
            "保存レコードがゼロのため、zip圧縮をスキップ"
        )
        return

    try:
        if config.SAVE_FORMAT == "donkeycar":
            target_dir = record_manager.record_directory
        else:
            target_dir = record_manager.image_directory

        if (
            not target_dir
            or not os.path.exists(target_dir)
        ):
            logger.warning(
                f"zip対象フォルダが存在しません: {target_dir}"
            )
            return

        zip_path = target_dir + '.zip'
        base_name = os.path.basename(
            target_dir
        )

        import zipfile

        files = []

        for root, dirs, filenames in os.walk(
            target_dir
        ):
            for f in filenames:
                files.append(
                    os.path.join(
                        root,
                        f,
                    )
                )

        total = len(files)

        print(
            f"zip圧縮開始: {base_name} ({total}ファイル)"
        )

        with zipfile.ZipFile(
            zip_path,
            'w',
            zipfile.ZIP_DEFLATED,
        ) as zf:
            for i, filepath in enumerate(
                files,
                1,
            ):
                arcname = os.path.join(
                    base_name,
                    os.path.relpath(
                        filepath,
                        target_dir,
                    ),
                )

                zf.write(
                    filepath,
                    arcname,
                )

                if i % 50 == 0 or i == total:
                    print(
                        f"\r  zip圧縮中... {i}/{total} "
                        f"({i*100//total}%)",
                        end="",
                        flush=True,
                    )

        print(
            f"\nzip圧縮完了: {zip_path}"
        )
        logger.info(
            f"zip圧縮完了: {zip_path}"
        )

    except Exception as e:
        logger.error(
            f"zip圧縮中にエラー: {e}"
        )


def _generate_video(record_manager):
    if not getattr(
        config,
        'AUTO_VIDEO_ON_EXIT',
        False,
    ):
        return

    if not hasattr(
        record_manager,
        'records',
    ) or len(
        record_manager.records
    ) == 0:
        return

    try:
        if config.SAVE_FORMAT == "donkeycar":
            target_dir = record_manager.record_directory
        else:
            target_dir = record_manager.image_directory

        if (
            not target_dir
            or not os.path.exists(target_dir)
        ):
            return

        from tools.images_to_video import images_to_video

        fps = getattr(
            config,
            'AUTO_VIDEO_FPS',
            20,
        )

        prefix = getattr(
            config,
            'AUTO_VIDEO_PREFIX',
            'cam',
        )

        images_to_video(
            target_dir,
            fps=fps,
            prefix=prefix,
        )

    except Exception as e:
        logger.error(
            f"動画生成中にエラー: {e}"
        )


def _cleanup_empty_record_folders(record_manager):
    try:
        if config.SAVE_FORMAT == "donkeycar":
            if (
                hasattr(
                    record_manager,
                    'record_directory',
                )
                and record_manager.record_directory
            ):
                if os.path.exists(
                    record_manager.record_directory
                ):
                    files = os.listdir(
                        record_manager.record_directory
                    )

                    if len(files) <= 1:
                        shutil.rmtree(
                            record_manager.record_directory
                        )

                        logger.info(
                            f"空の記録フォルダを削除: "
                            f"{record_manager.record_directory}"
                        )

                        record_manager._current_session_dir = None

        else:
            if (
                hasattr(
                    record_manager,
                    'image_directory',
                )
                and record_manager.image_directory
            ):
                if os.path.exists(
                    record_manager.image_directory
                ):
                    files = os.listdir(
                        record_manager.image_directory
                    )

                    if len(files) == 0:
                        os.rmdir(
                            record_manager.image_directory
                        )

                        logger.info(
                            f"空の画像フォルダを削除: "
                            f"{record_manager.image_directory}"
                        )

                        record_manager._current_images_dir = None

    except Exception as e:
        logger.warning(
            f"空フォルダ削除中にエラー: {e}"
        )


# グローバル変数（シグナルハンドラ用）
motor_instance = None
active_sensor_instances = None
planner_instance = None
record_manager = None
joystick = None
cleanup_done = False


def signal_handler(
    sig,
    frame,
):
    global cleanup_done

    if cleanup_done:
        return

    cleanup_done = True

    logger.info(
        "\nシグナルを受信しました。終了処理を実行中..."
    )

    if record_manager:
        if (
            hasattr(
                record_manager,
                'records',
            )
            and len(
                record_manager.records
            ) > 0
        ):
            record_manager.save_data()
            _generate_video(record_manager)
            _zip_record_folder(record_manager)

        else:
            _cleanup_empty_record_folders(
                record_manager
            )

    if (
        motor_instance
        and planner_instance
        and active_sensor_instances
    ):
        cleanup_system(
            motor_instance,
            planner_instance,
            active_sensor_instances,
            joystick,
        )

    sys.exit(0)


# ============================================================================
# 自動走行のメイン関数
# ============================================================================
if __name__ == "__main__":
    signal.signal(
        signal.SIGINT,
        signal_handler,
    )

    signal.signal(
        signal.SIGTERM,
        signal_handler,
    )

    # センサー設定チェック
    has_distance_sensors = (
        "ultrasonic" in config.ACTIVE_SENSORS
        or "lidar" in config.ACTIVE_SENSORS
    )

    has_camera = any(
        s.startswith("camera_")
        for s in config.ACTIVE_SENSORS
    )

    distance_required_plans = [
        "right_left_3",
        "right_left_3_records",
        "wall_follow",
        "wall_follow_pid",
        "center_follow_pid",
        "go_straight",
        "gap_follow",
        "racer",
        "follow_the_gap",
        "nn",
    ]

    camera_required_plans = [
        "donkeycar",
        "resnet18",
        "mobilevit_xxs",
        "edgenext_xx_small",
    ] + SEQUENCE_MODEL_PLANS

    both_sensors_required_plans = []
    manual_plans = ["manual"]

    try:
        (
            motor_instance,
            active_sensor_instances,
            planner_instance,
            model,
            position_model,
            position_models_dict,
            yolo_model,
            yolo_models_dict,
        ) = initialize_system()

        data_aggregator = DataAggregator(
            sensor_instances=active_sensor_instances,
            max_history=10,
        )

        spike_filter = SpikeFilter()

        # Speed PID制御の初期化
        speed_pid = None

        if getattr(
            config,
            'USE_SPEED_CONTROL',
            False,
        ):
            speed_pid = SpeedPIDController()

            logger.info(
                f"Speed PID制御有効: source={config.SPEED_SOURCE}, "
                f"KP={config.SPEED_PID_KP}, "
                f"KI={config.SPEED_PID_KI}, "
                f"KD={config.SPEED_PID_KD}, "
                f"MAX_SPEED={config.MAX_SPEED} m/s"
            )

        # コントローラーの選択
        controller_type = getattr(
            config,
            'CONTROLLER_TYPE',
            'joystick',
        ).lower()

        if controller_type == "pwm":
            print(
                "PWMコントローラーモードを使用"
            )
            joystick = PWMController()

        elif controller_type == "keyboard":
            print(
                "キーボードコントローラーモードを使用"
            )
            joystick = KeyboardController()

        elif config.HAVE_JOYSTICK:
            print(
                "ジョイスティックモードを使用"
            )
            joystick = Joystick()

        else:
            if config.PLAN in manual_plans:
                print(
                    "マニュアルプラン検出: キーボード操作モードを使用"
                )
                joystick = KeyboardController()
            else:
                print(
                    "デフォルト: ジョイスティックモードを使用"
                )
                joystick = Joystick()

        record_manager = RecordManager()
        is_recording = False
        recording_start_time = None

        if joystick.HAVE_CONTROLLER:
            print(
                f"{controller_type.upper()}コントローラー検出: "
                "userモードで即座に開始"
            )

        else:
            print(
                f"{controller_type.upper()}コントローラー未接続... "
                "autoモードで開始します"
            )

            config.HAVE_JOYSTICK = False

            can_auto_drive = False
            missing_sensors = []

            if config.PLAN in distance_required_plans:
                if has_distance_sensors:
                    can_auto_drive = True
                else:
                    missing_sensors.append(
                        "測距センサー（ultrasonic/lidar）"
                    )

            elif config.PLAN in camera_required_plans:
                if has_camera:
                    can_auto_drive = True
                else:
                    missing_sensors.append(
                        "カメラ（camera_0/camera_1）"
                    )

            elif config.PLAN in both_sensors_required_plans:
                if has_distance_sensors and has_camera:
                    can_auto_drive = True
                else:
                    if not has_distance_sensors:
                        missing_sensors.append(
                            "測距センサー（ultrasonic/lidar）"
                        )
                    if not has_camera:
                        missing_sensors.append(
                            "カメラ（camera_0/camera_1）"
                        )

            elif config.PLAN in manual_plans:
                print(
                    "マニュアルプラン: 手動操作専用モード"
                )
                can_auto_drive = False
                mode = "user"
                auto_mode_disabled = True

            else:
                print(
                    f"⚠️  警告: 不明なプラン '{config.PLAN}' です、"
                    "手動操作（userモード）のみ使用可能です"
                )
                mode = "user"
                auto_mode_disabled = True

            if (
                not can_auto_drive
                and config.PLAN not in manual_plans
            ):
                print(
                    f"⚠️  プラン '{config.PLAN}' に必要なセンサー: "
                    f"{', '.join(missing_sensors)}"
                )
                print(
                    "⚠️  手動操作（userモード）のみ使用可能です、"
                    "- Sボタンでのautoモード切り替えを無効化しています"
                )
                mode = "user"
                auto_mode_disabled = True

            elif config.PLAN in manual_plans:
                pass

            else:
                print(
                    "Sボタンでモード切り替え、"
                    "Yボタンで記録開始/停止"
                )
                mode = "user"
                auto_mode_disabled = False

            started = True

        if not config.HAVE_JOYSTICK:
            if config.PLAN in manual_plans:
                print(
                    "マニュアルプラン: 手動操作専用モード"
                )
                print(
                    "Rキーで記録開始/停止"
                )
                mode = "user"
                started = True
                auto_mode_disabled = True

            else:
                can_auto_drive = False

                if (
                    config.PLAN in distance_required_plans
                    and has_distance_sensors
                ):
                    can_auto_drive = True

                elif (
                    config.PLAN in camera_required_plans
                    and has_camera
                ):
                    can_auto_drive = True

                if not can_auto_drive:
                    print(
                        "❌ エラー: 自動走行に必要なセンサーが無効です"
                    )

                    if config.PLAN in distance_required_plans:
                        print(
                            f"❌ プラン '{config.PLAN}' には"
                            "測距センサー（ultrasonic/lidar）が必要です"
                        )

                    elif config.PLAN in camera_required_plans:
                        print(
                            f"❌ プラン '{config.PLAN}' にはカメラが必要です"
                        )

                    print(
                        "❌ ジョイスティックもないため操作不可能です"
                    )
                    print("解決方法:")
                    print(
                        "1. 必要なセンサーをconfig.pyのACTIVE_SENSORSに追加"
                    )
                    print(
                        "2. HAVE_JOYSTICKをTrueにしてジョイスティックを使用"
                    )
                    print(
                        "3. PLAN='manual'にしてキーボード操作を使用"
                    )
                    sys.exit(1)

                print(
                    "ジョイスティックなし: Enterキー待機中..."
                )
                input(
                    "Enterを押して走行開始！"
                )

                mode = "auto"
                started = True
                auto_mode_disabled = False

                is_recording = True
                recording_start_time = time.time()
                print(
                    "Recording started"
                )

        # PWM記録制御用カウンター
        pwm_active_count = 0
        pwm_inactive_count = 0
        pwm_consecutive = getattr(
            config,
            'PWM_RECORDING_CONSECUTIVE_COUNT',
            3,
        )

        # バッファマネージャーの初期化
        init_buffer_manager()

        # ====================================================================
        # メインループ
        # ====================================================================
        start_time = time.time()
        loop_fps = None
        loop_time_prev = time.time()
        _print_interval = 0.2
        _print_last = 0.0
        _monitor_interval = 0.05
        _monitor_last = 0.0

        while True:
            # モニター処理
            if config.MONITOR:
                if monitor.set_config_reload:
                    logger.info(
                        "Set Config detected. Reinitializing system..."
                    )
                    time.sleep(0.1)
                    model = reload_model()
                    monitor.set_config_reload = False
                    continue

                if monitor.realtime_data["pause_drive"]:
                    logger.info(
                        "メインループを一時停止中..."
                    )
                    motor_instance.set_steering_pwm_value(0)
                    motor_instance.set_throttle_pwm_value(0)
                    time.sleep(0.1)
                    continue

            # =================================================================
            # 認知
            # =================================================================
            if config.HAVE_JOYSTICK:
                joystick.poll()
                mode = joystick.mode[0]

            _now = time.time()

            need_lidar_image_for_record = (
                is_recording
                and getattr(
                    config,
                    'SAVE_LIDAR_IMAGES',
                    False,
                )
            )

            need_lidar_image_for_monitor = (
                config.MONITOR
                and (
                    _now - _monitor_last
                    >= _monitor_interval
                )
            )

            data_aggregator.lidar_generate_image = (
                need_lidar_image_for_record
                or need_lidar_image_for_monitor
            )

            # センサー値更新
            data_aggregator.update_sensors()
            sensor_data = (
                data_aggregator.get_latest_all_sensors()
            )

            add_sensor_to_buffer(
                sensor_data
            )

            # IMUデータ
            imu_raw = sensor_data.pop(
                "imu",
                None,
            )

            if (
                imu_raw
                and isinstance(
                    imu_raw,
                    (list, tuple),
                )
                and len(imu_raw) >= 2
            ):
                acceleration = imu_raw[0]
                angular_velocity = imu_raw[1]

                for axis in [
                    "x",
                    "y",
                    "z",
                ]:
                    ax_val = (
                        acceleration.get(axis)
                        if isinstance(
                            acceleration,
                            dict,
                        )
                        else None
                    )

                    if ax_val is not None:
                        ax_val = float(
                            ax_val[-1]
                            if hasattr(
                                ax_val,
                                '__getitem__',
                            )
                            else ax_val
                        )

                    sensor_data[
                        f"imu_acl_{axis}"
                    ] = (
                        round(
                            ax_val,
                            4,
                        )
                        if ax_val is not None
                        else 0.0
                    )

                    gyr_val = (
                        angular_velocity.get(axis)
                        if isinstance(
                            angular_velocity,
                            dict,
                        )
                        else None
                    )

                    if gyr_val is not None:
                        gyr_val = float(
                            gyr_val[-1]
                            if hasattr(
                                gyr_val,
                                '__getitem__',
                            )
                            else gyr_val
                        )

                    sensor_data[
                        f"imu_gyr_{axis}"
                    ] = (
                        round(
                            gyr_val,
                            4,
                        )
                        if gyr_val is not None
                        else 0.0
                    )

            # 測距センサー
            ranges = {}

            if "ultrasonic" in config.ACTIVE_SENSORS:
                for us_name in config.ULTRASONIC_SENSOR_LIST:
                    raw = data_aggregator.get_latest_sensor_value(
                        us_name
                    )

                    ranges[us_name] = spike_filter.update(
                        us_name,
                        raw,
                    )

                    sensor_data[us_name] = ranges[us_name]

            elif "lidar" in config.ACTIVE_SENSORS:
                lidar_data_latest = (
                    data_aggregator.get_latest_sensor_value(
                        "lidar"
                    )
                )

                if (
                    lidar_data_latest
                    and isinstance(
                        lidar_data_latest,
                        dict,
                    )
                ):
                    zone_distances = lidar_data_latest.get(
                        'zone_distances',
                        [],
                    )

                    for i, zone_name in enumerate(
                        config.ULTRASONIC_SENSOR_LIST
                    ):
                        if i < len(zone_distances):
                            zone_value = int(
                                zone_distances[i]
                            )
                            ranges[zone_name] = zone_value
                            sensor_data[zone_name] = zone_value
                        else:
                            ranges[zone_name] = 0
                            sensor_data[zone_name] = 0

                else:
                    lidar_sensor = active_sensor_instances.get(
                        "lidar"
                    )

                    if (
                        lidar_sensor
                        and hasattr(
                            lidar_sensor,
                            'zone_distances',
                        )
                    ):
                        for i, zone_name in enumerate(
                            config.ULTRASONIC_SENSOR_LIST
                        ):
                            if i < len(
                                lidar_sensor.zone_distances
                            ):
                                zone_value = int(
                                    lidar_sensor.zone_distances[i]
                                )
                                ranges[zone_name] = zone_value
                                sensor_data[zone_name] = zone_value
                            else:
                                ranges[zone_name] = 0
                                sensor_data[zone_name] = 0

            # LiDAR全点群
            lidar_data_dict = None

            if "lidar" in config.ACTIVE_SENSORS:
                lidar_data_dict = sensor_data.pop(
                    "lidar",
                    None,
                )

                if (
                    getattr(
                        config,
                        'SAVE_LIDAR_DATA',
                        False,
                    )
                    and lidar_data_dict
                    and isinstance(
                        lidar_data_dict,
                        dict,
                    )
                ):
                    measurements = lidar_data_dict.get(
                        'measurements'
                    )

                    if (
                        measurements is not None
                        and len(measurements) > 0
                    ):
                        sensor_data[
                            "lidar_distance_array"
                        ] = measurements

                if getattr(
                    config,
                    'SAVE_LIDAR_IMAGES',
                    False,
                ):
                    lidar_sensor = active_sensor_instances.get(
                        "lidar"
                    )

                    if (
                        lidar_sensor is not None
                        and lidar_sensor.latest_image is not None
                    ):
                        sensor_data[
                            "lidar_image"
                        ] = lidar_sensor.latest_image

            # RPM
            rpm_value = 0
            rpm_speed = 0.0

            rpm_data = sensor_data.pop(
                "rpm",
                None,
            )

            if (
                rpm_data
                and isinstance(
                    rpm_data,
                    dict,
                )
            ):
                rpm_value = int(
                    rpm_data.get(
                        "rpm",
                        0,
                    )
                )

                rpm_speed = round(
                    rpm_data.get(
                        "speed",
                        0.0,
                    ),
                    2,
                )

                sensor_data[
                    "rpm_value"
                ] = rpm_value

                sensor_data[
                    "rpm_speed"
                ] = rpm_speed

            # オプティカルフロー
            of_speed = 0.0

            of_data = sensor_data.pop(
                "optical_flow",
                None,
            )

            if (
                of_data
                and isinstance(
                    of_data,
                    dict,
                )
            ):
                of_speed = of_data.get(
                    "vy",
                    0.0,
                )

                sensor_data[
                    "of_vx"
                ] = of_data.get(
                    "vx",
                    0.0,
                )

                sensor_data[
                    "of_vy"
                ] = of_data.get(
                    "vy",
                    0.0,
                )

            # 現在速度
            current_speed = estimate_speed(
                rpm_speed,
                of_speed,
                source=getattr(
                    config,
                    'SPEED_SOURCE',
                    'rpm',
                ),
            )

            sensor_data[
                "speed"
            ] = round(
                current_speed,
                3,
            )

            # ================================================================
            # 全カメラ画像を取得
            # ================================================================
            camera_images = {}

            for _s in config.ACTIVE_SENSORS:
                if _s.startswith("camera_"):
                    camera_images[_s] = (
                        data_aggregator.get_latest_sensor_value(
                            _s
                        )
                    )

            # ================================================================
            # ★変更箇所: カメラ画像選択
            #
            # 画像AIプランだけではなく、Perceptionでもcamera_0を使える。
            # ================================================================
            inference_camera_image = None

            camera_based_plans = [
                "donkeycar",
                "resnet18",
                "mobilevit_xxs",
                "edgenext_xx_small",
            ] + SEQUENCE_MODEL_PLANS

            # ① 画像AIプラン
            if config.PLAN in camera_based_plans:
                if hasattr(
                    config,
                    'MODEL_INPUT_IMAGE',
                ):
                    for ci in range(4):
                        if (
                            f"cam{ci}"
                            in config.MODEL_INPUT_IMAGE
                        ):
                            inference_camera_image = (
                                camera_images.get(
                                    f"camera_{ci}"
                                )
                            )
                            break

            # ② Perception用カメラ
            elif getattr(
                config,
                "PERCEPTION_ENABLED",
                True,
            ):
                perception_input_image = getattr(
                    config,
                    "PERCEPTION_INPUT_IMAGE",
                    "cam0",
                )

                for ci in range(4):
                    if (
                        f"cam{ci}"
                        in perception_input_image
                    ):
                        inference_camera_image = (
                            camera_images.get(
                                f"camera_{ci}"
                            )
                        )
                        break

            # ③ 最終フォールバック
            if inference_camera_image is None:
                inference_camera_image = (
                    camera_images.get(
                        "camera_0"
                    )
                )

            # =================================================================
            # 判断
            # =================================================================
            if mode == "user":
                steering_value = joystick.steering
                throttle_value = joystick.throttle

            else:
                steering_value, throttle_value = (
                    planner_instance.planning_sequence(
                        mode,
                        config.PLAN,
                        data_aggregator,
                        model=(
                            model
                            if config.PLAN in [
                                "nn",
                                "donkeycar",
                                "resnet18",
                                "mobilevit_xxs",
                                "edgenext_xx_small",
                            ] + SEQUENCE_MODEL_PLANS
                            else None
                        ),
                        inference_camera_image=inference_camera_image,
                        position_model=position_model,
                        position_models_dict=position_models_dict,
                        yolo_model=yolo_model,
                        yolo_models_dict=yolo_models_dict,
                        camera_images=camera_images,
                        ranges=ranges,
                        lidar_data=lidar_data_dict,
                    )
                )

            if mode == "auto_str":
                throttle_value = joystick.throttle

            # Speed PID制御
            target_speed = 0.0

            if (
                speed_pid is not None
                and mode != "user"
            ):
                max_speed = getattr(
                    config,
                    'MAX_SPEED',
                    3.0,
                )

                num_outputs = getattr(
                    config,
                    'NUM_OUTPUTS',
                    2,
                )

                if num_outputs >= 3:
                    target_speed = (
                        planner_instance.speed
                        * max_speed
                    )
                else:
                    target_speed = (
                        throttle_value
                        * max_speed
                    )

                throttle_value = speed_pid.compute(
                    target_speed,
                    current_speed,
                )

                sensor_data[
                    "target_speed"
                ] = round(
                    target_speed,
                    3,
                )

            # =================================================================
            # 操作
            # =================================================================
            motor_instance.set_steering_pwm_value(
                steering_value
            )

            motor_instance.set_throttle_pwm_value(
                throttle_value
            )

            # =================================================================
            # 記録
            # =================================================================
            data_aggregator.add_control_data(
                steering_value,
                throttle_value,
            )

            timestamp = datetime.now(
                jst
            ).strftime(
                "%Y%m%d%H%M%S%f"
            )

            if controller_type == "pwm":
                throttle_active = (
                    abs(joystick.throttle)
                    >= getattr(
                        config,
                        'PWM_DEADZONE_THROTTLE',
                        0.05,
                    )
                )

                if throttle_active:
                    pwm_active_count += 1
                    pwm_inactive_count = 0

                    if (
                        pwm_active_count
                        >= pwm_consecutive
                        and not is_recording
                    ):
                        is_recording = True
                        recording_start_time = time.time()
                        print(
                            "*** Recording started (PWM throttle active) ***"
                        )

                else:
                    pwm_inactive_count += 1
                    pwm_active_count = 0

                    if (
                        pwm_inactive_count
                        >= pwm_consecutive
                        and is_recording
                    ):
                        is_recording = False
                        recording_start_time = None
                        print(
                            "*** Recording stopped (PWM throttle inactive) ***"
                        )

                if is_recording:
                    sensor_data_with_stats = (
                        get_sensor_data_with_statistics(
                            sensor_data
                        )
                    )

                    record_manager.record_data(
                        timestamp,
                        mode,
                        sensor_data_with_stats,
                        steering_value,
                        throttle_value,
                    )

            elif (
                (
                    config.HAVE_JOYSTICK
                    and config.CONTROLLER_TYPE == "joystick"
                )
                or config.PLAN in manual_plans
            ):
                if joystick.recording and not is_recording:
                    is_recording = True
                    recording_start_time = time.time()
                    print(
                        "*** Recording started/resumed ***"
                    )

                elif (
                    not joystick.recording
                    and is_recording
                ):
                    is_recording = False
                    recording_start_time = None
                    print(
                        "*** Recording stopped ***"
                    )

                if is_recording:
                    sensor_data_with_stats = (
                        get_sensor_data_with_statistics(
                            sensor_data
                        )
                    )

                    record_manager.record_data(
                        timestamp,
                        mode,
                        sensor_data_with_stats,
                        steering_value,
                        throttle_value,
                    )

            else:
                if is_recording:
                    sensor_data_with_stats = (
                        get_sensor_data_with_statistics(
                            sensor_data
                        )
                    )

                    record_manager.record_data(
                        timestamp,
                        mode,
                        sensor_data_with_stats,
                        steering_value,
                        throttle_value,
                    )

            # FPS
            loop_time_now = time.time()
            loop_dt = (
                loop_time_now
                - loop_time_prev
            )

            loop_fps = (
                round(
                    1.0 / loop_dt,
                    1,
                )
                if loop_dt > 0
                else 0
            )

            loop_time_prev = loop_time_now

            # ターミナル出力
            record_count = len(
                record_manager.records
            )

            if (
                config.TERMINAL_PRINT
                and loop_time_now
                - _print_last
                >= _print_interval
            ):
                _print_last = loop_time_now

                elapsed_time = ""

                if recording_start_time:
                    elapsed_seconds = int(
                        time.time()
                        - recording_start_time
                    )
                    minutes = (
                        elapsed_seconds
                        // 60
                    )
                    seconds = (
                        elapsed_seconds
                        % 60
                    )
                    elapsed_time = (
                        f" {minutes:02d}:{seconds:02d}"
                    )

                position_info = ""

                if (
                    config.USE_POSITION_SWITCHING
                    and planner_instance.current_position_id is not None
                ):
                    position_name = (
                        config.POSITION_CLASS_NAMES[
                            planner_instance.current_position_id
                        ]
                        if planner_instance.current_position_id
                        < len(
                            config.POSITION_CLASS_NAMES
                        )
                        else f"Pos{planner_instance.current_position_id}"
                    )
                    position_info = (
                        f" [{position_name}]"
                    )

                detection_info = ""

                if (
                    config.USE_YOLO_DETECTION
                    and config.YOLO_DISPLAY_DETECTIONS
                    and planner_instance.current_detections
                ):
                    detected_objects = ", ".join(
                        [
                            d["class_name"]
                            for d in planner_instance.current_detections[:3]
                        ]
                    )
                    detection_info = (
                        f" [Det: {detected_objects}]"
                    )

                if (
                    is_recording
                    and hasattr(
                        joystick,
                        'is_braking',
                    )
                    and joystick.is_braking
                ):
                    status = (
                        f"[BRK:{record_count}{elapsed_time}]"
                    )

                elif is_recording:
                    status = (
                        f"[REC:{record_count}{elapsed_time}]"
                    )

                else:
                    status = (
                        f"[STP:{record_count}]"
                    )

                sensor_str = ", ".join(
                    [
                        f"{k}:{v:>5}"
                        for k, v in ranges.items()
                    ]
                )

                rpm_str = (
                    f", RPM:{rpm_value:>6}"
                    if "rpm" in active_sensor_instances
                    else ""
                )

                flow_str = (
                    f", Flow:{of_speed:>5}"
                    if "optical_flow" in active_sensor_instances
                    else ""
                )

                speed_str = (
                    f", Spd:{current_speed:>5.2f}m/s"
                    if speed_pid is not None
                    else ""
                )

                tgt_speed_str = (
                    f"->Tgt:{target_speed:>5.2f}"
                    if speed_pid is not None
                    and mode != "user"
                    else ""
                )

                perception_str = ""

                if (
                    getattr(
                        config,
                        "PERCEPTION_ENABLED",
                        False,
                    )
                    and planner_instance.perception is not None
                    and mode != "user"
                ):
                    try:
                        p = planner_instance.perception.result
                        perception_str = (
                            f", PDir:{p.course.direction:>5.2f}"
                            f", PDng:{p.safety.danger_level:>4.2f}"
                        )
                    except Exception:
                        perception_str = ""

                print(
                    f"{status}"
                    f"{position_info}"
                    f"{detection_info} "
                    f"Mode:{mode}, "
                    f"St:{steering_value:>6.2f}, "
                    f"Th:{throttle_value:>5.2f}, "
                    f"{sensor_str}"
                    f"{rpm_str}"
                    f"{flow_str}"
                    f"{speed_str}"
                    f"{tgt_speed_str}"
                    f"{perception_str}  "
                    f"FPS:{loop_fps}"
                )

            # モニター出力
            if (
                config.MONITOR
                and loop_time_now
                - _monitor_last
                >= _monitor_interval
            ):
                _monitor_last = loop_time_now

                ftg_info = None

                if config.PLAN == "follow_the_gap":
                    from follow_the_gap import get_follow_the_gap_instance
                    ftg_inst = get_follow_the_gap_instance()
                    ftg_info = ftg_inst._ftg_info

                imu_yaw_rate = None
                imu_accel = None

                if "imu" in active_sensor_instances:
                    gyr_z = sensor_data.get(
                        "imu_gyr_z"
                    )

                    if gyr_z is not None:
                        imu_yaw_rate = round(
                            float(gyr_z),
                            2,
                        )

                    acl_x = sensor_data.get(
                        "imu_acl_x"
                    )
                    acl_y = sensor_data.get(
                        "imu_acl_y"
                    )

                    if (
                        acl_x is not None
                        or acl_y is not None
                    ):
                        imu_accel = {
                            "x": (
                                round(
                                    float(acl_x),
                                    2,
                                )
                                if acl_x is not None
                                else 0
                            ),
                            "y": (
                                round(
                                    float(acl_y),
                                    2,
                                )
                                if acl_y is not None
                                else 0
                            ),
                        }

                monitor.update_data(
                    mode=mode,
                    steering_value=steering_value,
                    throttle_value=throttle_value,
                    ranges=ranges,
                    timestamp=timestamp,
                    camera_image_0=camera_images.get(
                        "camera_0"
                    ),
                    camera_image_1=camera_images.get(
                        "camera_1"
                    ),
                    camera_image_2=camera_images.get(
                        "camera_2"
                    ),
                    camera_image_3=camera_images.get(
                        "camera_3"
                    ),
                    lidar_measurements=(
                        lidar_data_dict.get('measurements')
                        if lidar_data_dict
                        else None
                    ),
                    ftg_info=ftg_info,
                    imu_yaw_rate=imu_yaw_rate,
                    imu_accel=imu_accel,
                    rpm=rpm_value,
                    optical_flow_speed=of_speed,
                    record_count=record_count,
                    fps=loop_fps,
                )

    except KeyboardInterrupt:
        logger.info(
            "終了処理を実行中..."
        )

    finally:
        if not cleanup_done:
            if record_manager:
                if len(
                    record_manager.records
                ) > 0:
                    record_manager.save_data()
                    _generate_video(record_manager)
                    _zip_record_folder(record_manager)
                else:
                    _cleanup_empty_record_folders(
                        record_manager
                    )

            cleanup_system(
                motor_instance,
                planner_instance,
                active_sensor_instances,
                joystick,
            )
