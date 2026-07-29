# yolo_detection.py
# YOLO物体検知と制御修正機能
import os
import logging
import torch
import config
from model_inference import load_model_with_engine

logger = logging.getLogger(__name__)


def load_yolo_model():
    """
    YOLO物体検知モデルを読み込む
    推論エンジン（PyTorch, TensorRT, OpenVINO）に対応

    Returns:
        yolo_model: YOLOモデルまたはNone
    """
    if not config.USE_YOLO_DETECTION:
        return None

    if not os.path.exists(config.YOLO_MODEL_PATH):
        logger.error(f"YOLOモデルが見つかりません: {config.YOLO_MODEL_PATH}")
        logger.info("YOLOv8モデルをダウンロードしてください:")
        logger.info("  wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt")
        return None

    try:
        # 統一的なモデルローダーを使用
        yolo_model = load_model_with_engine(
            model_path=config.YOLO_MODEL_PATH,
            model_type="yolo",
            inference_engine=getattr(config, 'INFERENCE_ENGINE', 'pytorch')
        )

        if yolo_model is not None:
            logger.info(f"YOLOモデルをロードしました: {config.YOLO_MODEL_PATH} ({getattr(config, 'INFERENCE_ENGINE', 'pytorch')})")
            return yolo_model
        else:
            logger.error("YOLOモデルのロードに失敗")
            return None

    except Exception as e:
        logger.error(f"YOLOモデルのロードに失敗: {e}")
        import traceback
        traceback.print_exc()
        return None


def load_yolo_specific_models():
    """
    検知結果に応じた自動運転モデルを読み込む

    Returns:
        models_dict (dict): {class_id: model} の辞書
    """
    if not config.USE_YOLO_DETECTION or not config.YOLO_MODEL_SWITCHING:
        return {}

    from train_pytorch import get_model_from_catalog

    models_dict = {}
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    for class_id, model_name in config.YOLO_MODEL_SWITCHING.items():
        model_path = os.path.join(config.MODEL_DIR, model_name)

        if not os.path.exists(model_path):
            logger.warning(f"クラス{class_id}用モデルが見つかりません: {model_path}")
            continue

        try:
            # 自動運転モデルのアーキテクチャを取得
            model = get_model_from_catalog(config.PLAN)

            if model is not None:
                checkpoint = torch.load(model_path, map_location=device, weights_only=False)
                if 'model_state_dict' in checkpoint:
                    model.load_state_dict(checkpoint['model_state_dict'])
                else:
                    model.load_state_dict(checkpoint)

                model = model.to(device)
                model.eval()

                models_dict[class_id] = model
                class_name = config.YOLO_CLASS_NAMES.get(class_id, f"Class{class_id}")
                logger.info(f"クラス{class_id}({class_name})用モデルをロード: {model_name}")

        except Exception as e:
            logger.error(f"クラス{class_id}用モデルのロードに失敗: {e}")

    return models_dict


def detect_objects(yolo_model, camera_image):
    """
    カメラ画像から物体を検知

    Args:
        yolo_model: YOLOモデル
        camera_image: カメラ画像（numpy array, RGB）

    Returns:
        detections: 検知結果のリスト [{class_id, confidence, bbox, class_name}, ...]
    """
    if yolo_model is None or camera_image is None:
        return []

    try:
        # YOLO推論実行
        results = yolo_model.predict(
            camera_image,
            conf=config.YOLO_CONFIDENCE_THRESHOLD,
            iou=config.YOLO_IOU_THRESHOLD,
            imgsz=config.YOLO_INPUT_SIZE,
            classes=config.YOLO_TARGET_CLASSES,
            verbose=False
        )

        detections = []
        if results and len(results) > 0:
            result = results[0]

            # 検知結果を解析
            if result.boxes is not None and len(result.boxes) > 0:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]

                    class_name = config.YOLO_CLASS_NAMES.get(class_id, f"class_{class_id}")

                    detections.append({
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": confidence,
                        "bbox": bbox,
                    })

        return detections

    except Exception as e:
        logger.error(f"YOLO物体検知エラー: {e}")
        return []


def apply_detection_control_modification(detections, steering, throttle):
    """
    検知結果に基づいてステアリング・スロットル値を修正

    Args:
        detections: 検知結果のリスト
        steering: 元のステアリング値
        throttle: 元のスロットル値

    Returns:
        modified_steering: 修正後のステアリング値
        modified_throttle: 修正後のスロットル値
        applied_rule: 適用されたルール（ログ用）
    """
    if not detections:
        return steering, throttle, None

    # 最も優先度の高いルールを選択
    best_rule = None
    best_priority = -1
    best_detection = None

    for detection in detections:
        class_id = detection["class_id"]
        if class_id in config.YOLO_CONTROL_RULES:
            rule = config.YOLO_CONTROL_RULES[class_id]
            priority = rule.get("priority", 0)

            if priority > best_priority:
                best_priority = priority
                best_rule = rule
                best_detection = detection

    # ルールを適用
    if best_rule:
        modified_steering = steering + best_rule.get("steering_offset", 0.0)
        modified_throttle = throttle * best_rule.get("throttle_scale", 1.0)

        # 値の範囲制限
        modified_steering = max(-1.0, min(1.0, modified_steering))
        modified_throttle = max(-1.0, min(1.0, modified_throttle))

        return modified_steering, modified_throttle, {
            "class_name": best_detection["class_name"],
            "confidence": best_detection["confidence"],
            "description": best_rule.get("description", ""),
        }

    return steering, throttle, None


def select_model_by_detection(detections, yolo_models_dict, default_model):
    """
    検知結果に基づいて最適なモデルを選択

    Args:
        detections: 検知結果のリスト
        yolo_models_dict: クラスID→モデルの辞書
        default_model: デフォルトモデル

    Returns:
        selected_model: 選択されたモデル
        detected_class: 検知されたクラス情報（ログ用）
    """
    if not detections or not yolo_models_dict:
        return default_model, None

    # 最も信頼度の高い検知結果を選択
    best_detection = max(detections, key=lambda d: d["confidence"])
    class_id = best_detection["class_id"]

    if class_id in yolo_models_dict:
        return yolo_models_dict[class_id], {
            "class_id": class_id,
            "class_name": best_detection["class_name"],
            "confidence": best_detection["confidence"],
        }

    return default_model, None


def calculate_object_tracking_steering(detections, image_width):
    """
    検知物体に向かって進むためのステアリング補正値を計算

    Args:
        detections: 検知結果のリスト [{class_id, confidence, bbox, class_name}, ...]
        image_width: 画像の幅（ピクセル）

    Returns:
        steering_offset: ステアリング補正値（-1.0 ~ 1.0）
        target_detection: 追従対象の検知情報（ログ用）
    """
    if not detections or not config.USE_YOLO_OBJECT_TRACKING:
        return 0.0, None

    # 追従対象クラスのみをフィルタリング
    target_detections = [
        d for d in detections
        if d["class_id"] in config.YOLO_TRACKING_TARGET_CLASSES
    ]

    if not target_detections:
        return 0.0, None

    # 最も信頼度の高い対象を選択
    best_detection = max(target_detections, key=lambda d: d["confidence"])
    bbox = best_detection["bbox"]  # [x1, y1, x2, y2]

    # バウンディングボックスの中心X座標を計算
    bbox_center_x = (bbox[0] + bbox[2]) / 2.0

    # 画像中心からのずれを計算（-1.0 ~ 1.0に正規化）
    image_center_x = image_width / 2.0
    offset_normalized = (bbox_center_x - image_center_x) / image_center_x

    # 不感帯を適用（中心付近のノイズを除去）
    deadzone = config.YOLO_TRACKING_CENTER_DEADZONE
    if abs(offset_normalized) < deadzone:
        offset_normalized = 0.0

    # ゲインを適用してステアリング補正値を計算
    steering_offset = offset_normalized * config.YOLO_TRACKING_STEERING_GAIN

    # 範囲制限（-1.0 ~ 1.0）
    steering_offset = max(-1.0, min(1.0, steering_offset))

    target_info = {
        "class_name": best_detection["class_name"],
        "confidence": best_detection["confidence"],
        "bbox_center_x": bbox_center_x,
        "image_center_x": image_center_x,
        "offset": offset_normalized,
        "steering_offset": steering_offset,
    }

    return steering_offset, target_info


def calculate_obstacle_avoidance_steering(detections, image_width, image_height):
    """
    障害物を回避するためのステアリング補正値を計算

    Args:
        detections: 検知結果のリスト [{class_id, confidence, bbox, class_name}, ...]
        image_width: 画像の幅（ピクセル）
        image_height: 画像の高さ（ピクセル）

    Returns:
        steering_offset: ステアリング補正値（-1.0 ~ 1.0）
        obstacle_info: 回避対象の障害物情報（ログ用）
    """
    if not detections or not config.USE_YOLO_OBSTACLE_AVOIDANCE:
        return 0.0, None

    # 回避対象クラスのみをフィルタリング
    obstacle_detections = [
        d for d in detections
        if d["class_id"] in config.YOLO_OBSTACLE_CLASSES
    ]

    if not obstacle_detections:
        return 0.0, None

    # 画像面積を計算
    image_area = image_width * image_height

    # サイズ閾値以上の障害物のみを対象とする
    significant_obstacles = []
    for obstacle in obstacle_detections:
        bbox = obstacle["bbox"]  # [x1, y1, x2, y2]
        bbox_width = bbox[2] - bbox[0]
        bbox_height = bbox[3] - bbox[1]
        bbox_area = bbox_width * bbox_height
        area_ratio = bbox_area / image_area

        if area_ratio >= config.YOLO_OBSTACLE_SIZE_THRESHOLD:
            obstacle["bbox_area"] = bbox_area
            obstacle["area_ratio"] = area_ratio
            significant_obstacles.append(obstacle)

    if not significant_obstacles:
        return 0.0, None

    # 最も大きい（=最も近い可能性が高い）障害物を選択
    largest_obstacle = max(significant_obstacles, key=lambda d: d["bbox_area"])
    bbox = largest_obstacle["bbox"]  # [x1, y1, x2, y2]

    # バウンディングボックスの中心X座標を計算
    bbox_center_x = (bbox[0] + bbox[2]) / 2.0

    # 画像中心座標
    image_center_x = image_width / 2.0

    # 中央エリアの範囲を計算
    center_zone_half_width = (image_width * config.YOLO_OBSTACLE_CENTER_ZONE) / 2.0
    center_zone_left = image_center_x - center_zone_half_width
    center_zone_right = image_center_x + center_zone_half_width

    # 障害物が中央エリアにあるかチェック
    if center_zone_left <= bbox_center_x <= center_zone_right:
        # 中央エリアにある場合、回避方向を決定
        # 障害物の位置に基づいて逆方向にステアリング
        offset_from_center = bbox_center_x - image_center_x

        # 障害物とは逆方向に回避（-1倍）
        avoidance_direction = -offset_from_center / center_zone_half_width

        # ゲインを適用
        steering_offset = avoidance_direction * config.YOLO_OBSTACLE_AVOIDANCE_GAIN

        # 範囲制限（-1.0 ~ 1.0）
        steering_offset = max(-1.0, min(1.0, steering_offset))

        obstacle_info = {
            "class_name": largest_obstacle["class_name"],
            "confidence": largest_obstacle["confidence"],
            "bbox_center_x": bbox_center_x,
            "image_center_x": image_center_x,
            "area_ratio": largest_obstacle["area_ratio"],
            "steering_offset": steering_offset,
            "avoidance_direction": "left" if steering_offset < 0 else "right",
        }

        return steering_offset, obstacle_info

    # 中央エリア外の障害物は回避不要
    return 0.0, None
