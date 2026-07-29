# model_inference.py
# 統一的なモデル推論ユーティリティ
import os
import logging
import torch
import numpy as np
from PIL import Image
import config

logger = logging.getLogger(__name__)


class ModelInference:
    """
    統一的なモデル推論クラス
    PyTorch, TensorRT, OpenVINOの推論を統一的に扱う
    """

    def __init__(self, model, inference_engine=None):
        """
        Args:
            model: モデルオブジェクト
            inference_engine: 推論エンジン ("pytorch", "tensorrt", "openvino")
                             Noneの場合はgetattr(config, 'INFERENCE_ENGINE', 'pytorch')を使用
        """
        self.model = model
        self.inference_engine = inference_engine or getattr(config, 'INFERENCE_ENGINE', 'pytorch')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        logger.info(f"ModelInference initialized with engine: {self.inference_engine}")

    def infer(self, input_data):
        """
        統一的な推論インターフェース

        Args:
            input_data: 入力データ（numpy array, PIL Image, torch.Tensor）

        Returns:
            output: モデル出力
        """
        if self.inference_engine == "tensorrt":
            return self._tensorrt_inference(input_data)
        elif self.inference_engine == "openvino":
            return self._openvino_inference(input_data)
        else:  # pytorch (default)
            return self._pytorch_inference(input_data)

    def _pytorch_inference(self, input_data):
        """PyTorchモデルでの推論"""
        try:
            # model_catalogのrunメソッドがある場合
            if hasattr(self.model, 'run'):
                # numpy配列を期待
                if isinstance(input_data, torch.Tensor):
                    input_data = input_data.cpu().numpy()
                return self.model.run(input_data)

            # 通常のPyTorchモデル
            if isinstance(input_data, np.ndarray):
                input_data = torch.from_numpy(input_data).float()

            if not isinstance(input_data, torch.Tensor):
                raise TypeError(f"Expected torch.Tensor or numpy.ndarray, got {type(input_data)}")

            input_data = input_data.to(self.device)

            with torch.no_grad():
                output = self.model(input_data)

            return output

        except Exception as e:
            logger.error(f"PyTorch inference error: {e}")
            raise

    def _tensorrt_inference(self, input_data):
        """TensorRTモデルでの推論"""
        try:
            # model_catalogのrunメソッドがある場合（TensorRT対応モデル）
            if hasattr(self.model, 'run'):
                if isinstance(input_data, torch.Tensor):
                    input_data = input_data.cpu().numpy()
                return self.model.run(input_data)

            # TensorRTモデルの直接実行
            if isinstance(input_data, np.ndarray):
                input_data = torch.from_numpy(input_data).float()

            input_data = input_data.to(self.device)

            with torch.no_grad():
                output = self.model(input_data)

            return output

        except Exception as e:
            logger.warning(f"TensorRT inference failed, falling back to PyTorch: {e}")
            return self._pytorch_inference(input_data)

    def _openvino_inference(self, input_data):
        """OpenVINOモデルでの推論"""
        try:
            # model_catalogのrunメソッドがある場合（OpenVINO対応モデル）
            if hasattr(self.model, 'run'):
                if isinstance(input_data, torch.Tensor):
                    input_data = input_data.cpu().numpy()
                return self.model.run(input_data)

            # OpenVINO推論エンジンを使用
            from openvino.runtime import Core

            # OpenVINOモデルの場合
            if hasattr(self.model, 'infer_new_request'):
                # OpenVINOコンパイル済みモデル
                if isinstance(input_data, torch.Tensor):
                    input_data = input_data.cpu().numpy()

                results = self.model.infer_new_request({0: input_data})
                return list(results.values())[0]

            # フォールバック: PyTorch推論
            logger.warning("OpenVINO direct inference not available, falling back to PyTorch")
            return self._pytorch_inference(input_data)

        except ImportError:
            logger.warning("OpenVINO not available, falling back to PyTorch")
            return self._pytorch_inference(input_data)
        except Exception as e:
            logger.warning(f"OpenVINO inference failed, falling back to PyTorch: {e}")
            return self._pytorch_inference(input_data)


def load_model_with_engine(model_path, model_type=None, inference_engine=None, **kwargs):
    """
    推論エンジンに応じてモデルを読み込む

    Args:
        model_path: モデルファイルパス
        model_type: モデルタイプ（"driving", "position", "yolo"）
        inference_engine: 推論エンジン ("pytorch", "tensorrt", "openvino")
        **kwargs: モデル固有のパラメータ

    Returns:
        model: 読み込まれたモデル
    """
    inference_engine = inference_engine or getattr(config, 'INFERENCE_ENGINE', 'pytorch')

    if not os.path.exists(model_path):
        logger.error(f"Model file not found: {model_path}")
        return None

    try:
        if inference_engine == "tensorrt":
            return _load_tensorrt_model(model_path, model_type, **kwargs)
        elif inference_engine == "openvino":
            return _load_openvino_model(model_path, model_type, **kwargs)
        else:  # pytorch
            return _load_pytorch_model(model_path, model_type, **kwargs)

    except Exception as e:
        logger.error(f"Failed to load model with {inference_engine}: {e}")
        # フォールバック: PyTorchで読み込み
        if inference_engine != "pytorch":
            logger.info("Falling back to PyTorch model loading...")
            return _load_pytorch_model(model_path, model_type, **kwargs)
        return None


def _load_pytorch_model(model_path, model_type, **kwargs):
    """PyTorchモデルを読み込む"""
    if model_type == "driving":
        from train_pytorch import get_model_from_catalog
        model = get_model_from_catalog(kwargs.get('plan', 'nn'))
        if model is not None:
            checkpoint = torch.load(model_path, map_location='cuda' if torch.cuda.is_available() else 'cpu', weights_only=False)
            if 'model_state_dict' in checkpoint:
                model.load_state_dict(checkpoint['model_state_dict'])
            else:
                model.load_state_dict(checkpoint)
            model.eval()
        return model

    elif model_type == "position":
        # annotation_training_d2jのmodel_catalogを使用
        import sys
        annotation_path = os.path.join(os.path.dirname(__file__), 'annotation_training_d2j')
        if annotation_path not in sys.path:
            sys.path.insert(0, annotation_path)

        from model_catalog import get_model as get_location_model
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # model_typeパラメータを使用（例: "donkey_location", "resnet18_location"）
        model_name = kwargs.get('model_name', 'resnet18_location')
        model = get_location_model(
            model_type=model_name,
            pretrained=False
        )

        checkpoint = torch.load(model_path, map_location=device, weights_only=False)
        if 'model_state_dict' in checkpoint:
            model.load_state_dict(checkpoint['model_state_dict'])
        else:
            model.load_state_dict(checkpoint)
        model = model.to(device)
        model.eval()
        return model

    elif model_type == "yolo":
        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            return model
        except ImportError:
            logger.error("Ultralytics YOLO is not installed. Install with: pip install ultralytics")
            return None

    else:
        # 汎用PyTorchモデル
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        model = torch.load(model_path, map_location=device, weights_only=False)
        if isinstance(model, dict) and 'model_state_dict' in model:
            # state_dictのみの場合はアーキテクチャが必要
            logger.warning("Model architecture required for state_dict loading")
            return None
        model.eval()
        return model


def _load_tensorrt_model(model_path, model_type, **kwargs):
    """TensorRTモデルを読み込む"""
    try:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        if model_type == "yolo":
            # Ultralytics YOLO TensorRT/ONNX
            try:
                from ultralytics import YOLO

                # 優先順位: .engine > .onnx > .pt
                # 1. TensorRTエンジンファイル（.engine）を確認
                engine_path = model_path.replace('.pt', '.engine')
                if os.path.exists(engine_path):
                    model = YOLO(engine_path, task='detect')
                    logger.info(f"Loaded YOLO TensorRT engine: {engine_path}")
                    return model

                # 2. ONNXファイル（.onnx）を確認（Jetson推奨）
                onnx_path = model_path.replace('.pt', '.onnx')
                if os.path.exists(onnx_path):
                    model = YOLO(onnx_path, task='detect')
                    logger.info(f"Loaded YOLO ONNX model: {onnx_path}")
                    return model

                # 3. .engineと.onnxがない場合、PyTorchモデルにフォールバック
                logger.warning(f"YOLO TensorRT engine not found: {engine_path}")
                logger.warning(f"YOLO ONNX model not found: {onnx_path}")
                logger.info(f"Falling back to PyTorch model: {model_path}")
                return _load_pytorch_model(model_path, model_type, **kwargs)
            except ImportError:
                logger.error("Ultralytics YOLO is not installed")
                return _load_pytorch_model(model_path, model_type, **kwargs)
        else:
            # 自動運転モデル・位置推論モデルのTensorRT (.engine)
            from train_pytorch import TensorRTModel

            if model_path.endswith('.pth'):
                engine_path = model_path.replace('.pth', '.engine')
            elif model_path.endswith('.pt'):
                engine_path = model_path.replace('.pt', '.engine')
            else:
                engine_path = model_path + '.engine'

            if not os.path.exists(engine_path):
                logger.warning(f"TensorRT engine not found: {engine_path}")
                logger.info("Falling back to PyTorch model. To create TensorRT engine, run: python tools/torch2trt_converter.py")
                return _load_pytorch_model(model_path, model_type, **kwargs)

            model_trt = TensorRTModel(engine_path)
            logger.info(f"Loaded TensorRT engine: {engine_path}")
            return model_trt

    except Exception as e:
        logger.error(f"TensorRT model loading failed: {e}")
        import traceback
        traceback.print_exc()
        return _load_pytorch_model(model_path, model_type, **kwargs)


def _load_openvino_model(model_path, model_type, **kwargs):
    """OpenVINOモデルを読み込む"""
    # OpenVINO形式のモデルパスを確認
    if model_path.endswith('.pth'):
        openvino_xml = model_path.replace('.pth', '_openvino.xml')
    elif model_path.endswith('.pt'):
        openvino_xml = model_path.replace('.pt', '_openvino.xml')
    else:
        openvino_xml = model_path + '_openvino.xml'

    if not os.path.exists(openvino_xml):
        logger.warning(f"OpenVINO model not found: {openvino_xml}")
        logger.info("Falling back to PyTorch model. To create OpenVINO model, use annotation_training_d2j/tools/pytorch_to_openvino.py")
        return _load_pytorch_model(model_path, model_type, **kwargs)

    try:
        from openvino.runtime import Core

        if model_type == "yolo":
            # Ultralytics YOLO OpenVINO/ONNX
            from ultralytics import YOLO

            # 優先順位: OpenVINO > ONNX > PyTorch
            # 1. OpenVINOモデルディレクトリを確認
            openvino_dir = model_path.replace('.pt', '_openvino_model')
            if os.path.exists(openvino_dir):
                model = YOLO(openvino_dir, task='detect')
                logger.info(f"Loaded YOLO OpenVINO model: {openvino_dir}")
                return model

            # 2. ONNXファイルを確認（OpenVINOがない場合のフォールバック）
            onnx_path = model_path.replace('.pt', '.onnx')
            if os.path.exists(onnx_path):
                logger.info(f"OpenVINO model not found, using ONNX: {onnx_path}")
                model = YOLO(onnx_path, task='detect')
                logger.info(f"Loaded YOLO ONNX model: {onnx_path}")
                return model

            # 3. OpenVINOもONNXもない場合、PyTorchモデルにフォールバック
            logger.warning(f"YOLO OpenVINO model not found: {openvino_dir}")
            logger.warning(f"YOLO ONNX model not found: {onnx_path}")
            return _load_pytorch_model(model_path, model_type, **kwargs)
        else:
            # 自動運転モデル・位置推論モデルのOpenVINO
            ie = Core()
            model = ie.compile_model(openvino_xml, "CPU")
            logger.info(f"Loaded OpenVINO model: {openvino_xml}")
            return model

    except ImportError:
        logger.error("OpenVINO not installed")
        return _load_pytorch_model(model_path, model_type, **kwargs)
    except Exception as e:
        logger.error(f"OpenVINO model loading failed: {e}")
        return _load_pytorch_model(model_path, model_type, **kwargs)
