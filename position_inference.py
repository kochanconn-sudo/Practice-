# position_inference.py
# 位置推論とモデル切り替え機能
import os
import logging
import torch
import config
from model_inference import load_model_with_engine, ModelInference

logger = logging.getLogger(__name__)


def load_position_model():
    """
    annotation_training_d2jで学習した位置推論用の分類モデルを読み込む
    推論エンジン（PyTorch, TensorRT, OpenVINO）に対応

    Returns:
        position_model: 位置推論モデルまたはNone
    """
    if not config.USE_POSITION_SWITCHING or not config.POSITION_MODEL_NAME:
        return None

    position_model_path = os.path.join(config.MODEL_DIR, config.POSITION_MODEL_NAME)

    if not os.path.exists(position_model_path):
        logger.error(f"位置推論モデルが見つかりません: {position_model_path}")
        return None

    try:
        # 統一的なモデルローダーを使用
        position_model = load_model_with_engine(
            model_path=position_model_path,
            model_type="position",
            inference_engine=getattr(config, 'INFERENCE_ENGINE', 'pytorch'),
            model_name=config.POSITION_MODEL_TYPE,
            num_classes=config.POSITION_NUM_CLASSES
        )

        if position_model is not None:
            logger.info(f"位置推論モデルをロードしました: {config.POSITION_MODEL_NAME} ({config.POSITION_MODEL_TYPE}, {getattr(config, 'INFERENCE_ENGINE', 'pytorch')})")
            return position_model
        else:
            logger.error("位置推論モデルのロードに失敗")
            return None

    except Exception as e:
        logger.error(f"位置推論モデルのロードに失敗: {e}")
        import traceback
        traceback.print_exc()
        return None


def load_position_specific_models():
    """
    各位置専用の自動運転モデルを読み込む

    Returns:
        models_dict (dict): {position_id: model} の辞書
    """
    if not config.USE_POSITION_SWITCHING:
        return {}

    from train_pytorch import get_model_from_catalog

    models_dict = {}
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    for position_id, model_name in config.POSITION_MODELS_MAP.items():
        model_path = os.path.join(config.MODEL_DIR, model_name)

        if not os.path.exists(model_path):
            logger.warning(f"位置{position_id}用モデルが見つかりません: {model_path}")
            continue

        try:
            # 自動運転モデルのアーキテクチャを取得（config.PLANに基づく）
            model = get_model_from_catalog(config.PLAN)

            if model is not None:
                checkpoint = torch.load(model_path, map_location=device, weights_only=False)
                if 'model_state_dict' in checkpoint:
                    model.load_state_dict(checkpoint['model_state_dict'])
                else:
                    model.load_state_dict(checkpoint)

                model = model.to(device)
                model.eval()

                models_dict[position_id] = model
                position_name = config.POSITION_CLASS_NAMES[position_id] if position_id < len(config.POSITION_CLASS_NAMES) else f"Position{position_id}"
                logger.info(f"位置{position_id}({position_name})用モデルをロード: {model_name}")

        except Exception as e:
            logger.error(f"位置{position_id}用モデルのロードに失敗: {e}")

    # デフォルトモデルも追加
    if config.POSITION_DEFAULT_MODEL:
        default_model_path = os.path.join(config.MODEL_DIR, config.POSITION_DEFAULT_MODEL)
        if os.path.exists(default_model_path):
            try:
                default_model = get_model_from_catalog(config.PLAN)
                if default_model is not None:
                    checkpoint = torch.load(default_model_path, map_location=device, weights_only=False)
                    if 'model_state_dict' in checkpoint:
                        default_model.load_state_dict(checkpoint['model_state_dict'])
                    else:
                        default_model.load_state_dict(checkpoint)
                    default_model = default_model.to(device)
                    default_model.eval()
                    models_dict['default'] = default_model
                    logger.info(f"デフォルトモデルをロード: {config.POSITION_DEFAULT_MODEL}")
            except Exception as e:
                logger.error(f"デフォルトモデルのロードに失敗: {e}")

    return models_dict


def infer_position(position_model, camera_image):
    """
    カメラ画像から現在の位置を推論
    推論エンジン（PyTorch, TensorRT, OpenVINO）に対応

    Args:
        position_model: 位置推論モデル
        camera_image: カメラ画像（numpy array, RGB）

    Returns:
        position_id (int): 推論された位置クラスID
        confidence (float): 推論の信頼度
    """
    if position_model is None or camera_image is None:
        return None, 0.0

    try:
        from torchvision import transforms
        from PIL import Image

        # 画像の前処理
        transform = transforms.Compose([
            transforms.Resize((config.IMAGE_H, config.IMAGE_W)),
            transforms.ToTensor(),
        ])

        # numpy配列をPIL Imageに変換
        pil_image = Image.fromarray(camera_image)
        input_tensor = transform(pil_image).unsqueeze(0)

        # 統一的な推論インターフェースを使用
        inference_wrapper = ModelInference(position_model, getattr(config, 'INFERENCE_ENGINE', 'pytorch'))

        # 推論エンジンに応じて適切なデバイスに配置
        if getattr(config, 'INFERENCE_ENGINE', 'pytorch') == "pytorch" or getattr(config, 'INFERENCE_ENGINE', 'pytorch') == "tensorrt":
            device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            input_tensor = input_tensor.to(device)

        # 推論実行
        output = inference_wrapper.infer(input_tensor)

        # 出力の処理
        if isinstance(output, tuple):
            output = output[0]

        if not isinstance(output, torch.Tensor):
            output = torch.from_numpy(output)

        probabilities = torch.softmax(output, dim=1)
        confidence, position_id = torch.max(probabilities, dim=1)

        return position_id.item(), confidence.item()

    except Exception as e:
        logger.error(f"位置推論エラー: {e}")
        import traceback
        traceback.print_exc()
        return None, 0.0
