#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PyTorchモデルをTensorRTエンジン (.engine) に変換するツール

全モデルタイプ（単一入力CNN / 時系列GRU・TCN等）を ONNX経由で統一変換。
変換後にPyTorch vs TensorRTのベンチマーク比較を自動実行。

Usage:
    python tools/torch2trt_converter.py
    python tools/torch2trt_converter.py --no-benchmark
    python tools/torch2trt_converter.py --benchmark-iterations 100
"""

import os
import sys

# TensorRTのPythonバインディングにアクセスするためのパス追加
sys.path.insert(0, '/usr/lib/python3.10/dist-packages')

import torch
import argparse
import logging
import glob
import time
import numpy as np

# ロギング設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def _build_engine_from_onnx(onnx_path, engine_path, fp16_mode=True, max_workspace_size=1<<25):
    """ONNXファイルからTensorRTエンジンをビルドして保存する（共通処理）"""
    import tensorrt as trt

    trt_logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(trt_logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, trt_logger)

    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                logger.error(f"ONNX parse error: {parser.get_error(i)}")
            return None

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, max_workspace_size)
    if fp16_mode:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            logger.info("FP16モードを有効にしました。")
        else:
            logger.warning("このプラットフォームではFP16が高速ではありません。FP32で変換します。")

    serialized_engine = builder.build_serialized_network(network, config)
    if serialized_engine is None:
        logger.error("TensorRTエンジンのビルドに失敗しました。")
        return None

    with open(engine_path, 'wb') as f:
        f.write(serialized_engine)
    logger.info(f"TensorRTエンジンを保存しました: {engine_path}")
    return engine_path


def convert_pytorch_to_tensorrt(model, input_size=(224, 224), batch_size=1,
                               fp16_mode=True, max_workspace_size=1<<25,
                               save_path=None, device='cuda'):
    """
    単一入力PyTorchモデルをONNX経由でTensorRTエンジンに変換する

    Args:
        model: 変換するPyTorchモデル
        input_size: 入力画像サイズ (height, width)
        batch_size: バッチサイズ
        fp16_mode: FP16を使用するか
        max_workspace_size: TensorRTワークスペースサイズ
        save_path: 保存パス (.engine)
        device: デバイス

    Returns:
        エンジンファイルパス (str) or None
    """
    if not torch.cuda.is_available():
        logger.error("CUDA が利用できないため、TensorRTへの変換ができません。")
        return None

    model = model.to(device)
    model.eval()

    x = torch.ones((batch_size, 3, input_size[0], input_size[1])).to(device)

    # ONNX エクスポート
    engine_path = save_path or 'model.engine'
    onnx_path = engine_path.replace('.engine', '.onnx')
    logger.info(f"ONNXにエクスポートしています... → {onnx_path}")
    try:
        with torch.no_grad():
            torch.onnx.export(
                model, x, onnx_path,
                input_names=['input'],
                output_names=['output'],
                dynamic_axes=None,
                opset_version=17,
                do_constant_folding=True,
            )
        logger.info("ONNXエクスポート完了。")
    except Exception as e:
        logger.error(f"ONNXエクスポートに失敗しました: {e}")
        return None

    # TensorRT エンジンビルド
    logger.info(f"TensorRTエンジンをビルドしています... → {engine_path}")
    result = _build_engine_from_onnx(onnx_path, engine_path, fp16_mode, max_workspace_size)

    # ONNX中間ファイルは残す（デバッグ用）
    logger.info(f"ONNX中間ファイル: {onnx_path}")
    return result


def convert_sequence_to_tensorrt(model, seq_len=8, num_image_sources=1,
                                  img_size=(128, 128), batch_size=1,
                                  fp16_mode=True, max_workspace_size=1<<25,
                                  save_path=None, device='cuda'):
    """
    時系列モデル（GRU/TCN/CausalCNN）をONNX経由でTensorRTエンジンに変換する

    Args:
        model: BaseSequenceModelのサブクラス
        seq_len: シーケンス長
        num_image_sources: 画像ソース数
        img_size: 入力画像サイズ (height, width)
        batch_size: バッチサイズ
        fp16_mode: FP16を使用するか
        max_workspace_size: TensorRTワークスペースサイズ
        save_path: 保存パス (.engine)
        device: デバイス

    Returns:
        エンジンファイルパス (str) or None
    """
    if not torch.cuda.is_available():
        logger.error("CUDA が利用できないため、TensorRTへの変換ができません。")
        return None

    model = model.to(device)
    model.eval()

    # ダミー入力: images (B, T, S, C, H, W), ego_states (B, T, 5)
    images = torch.ones((batch_size, seq_len, num_image_sources, 3, img_size[0], img_size[1])).to(device)
    ego_states = torch.zeros((batch_size, seq_len, 5)).to(device)

    # ONNX エクスポート
    engine_path = save_path or 'model_sequence.engine'
    onnx_path = engine_path.replace('.engine', '.onnx')
    logger.info(f"ONNXにエクスポートしています... → {onnx_path}")
    try:
        with torch.no_grad():
            torch.onnx.export(
                model,
                (images, ego_states),
                onnx_path,
                input_names=['images', 'ego_states'],
                output_names=['trajectory'],
                dynamic_axes=None,
                opset_version=17,
                do_constant_folding=True,
            )
        logger.info("ONNXエクスポート完了。")
    except Exception as e:
        logger.error(f"ONNXエクスポートに失敗しました: {e}")
        return None

    # TensorRT エンジンビルド
    logger.info(f"TensorRTエンジンをビルドしています... → {engine_path}")
    result = _build_engine_from_onnx(onnx_path, engine_path, fp16_mode, max_workspace_size)
    logger.info(f"ONNX中間ファイル: {onnx_path}")
    return result


def benchmark_inference(model, num_iterations=50, warmup=5, label="Model", inputs=None):
    """
    モデルの推論速度を計測（単一入力・複数入力対応）

    Args:
        model: PyTorchモデルまたはTensorRTModel
        num_iterations: 測定回数
        warmup: ウォームアップ回数
        label: 表示ラベル
        inputs: 入力テンソルのリスト（Noneの場合は単一テンソル前提）
    """
    if inputs is None:
        raise ValueError("inputs must be provided")

    is_multi = isinstance(inputs, (list, tuple))

    def run_once():
        if is_multi:
            return model(*inputs)
        else:
            return model(inputs)

    if hasattr(model, 'eval'):
        model.eval()

    with torch.no_grad():
        for _ in range(warmup):
            run_once()

    if torch.cuda.is_available():
        torch.cuda.synchronize()

    times = []
    with torch.no_grad():
        for _ in range(num_iterations):
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            start = time.perf_counter()
            run_once()
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            times.append((time.perf_counter() - start) * 1000)

    times = np.array(times)
    logger.info(f"{label} 推論時間 ({num_iterations}回): "
                f"平均={times.mean():.2f}ms, 中央値={np.median(times):.2f}ms, "
                f"最小={times.min():.2f}ms, FPS={1000.0/times.mean():.1f}")
    return times.mean()


def load_model_weights(model, weights_path, device):
    """モデルに重みを読み込む（チェックポイント形式か通常の形式かを自動判定）"""
    checkpoint = torch.load(weights_path, map_location=device)
    if isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
        model.load_state_dict(checkpoint['model_state_dict'])
        logger.info("チェックポイント形式のモデルを読み込みました。")
    else:
        model.load_state_dict(checkpoint)
        logger.info("state_dict形式のモデルを読み込みました。")
    return model


def find_pytorch_models(models_dir):
    """指定されたディレクトリ内のPyTorchモデル（.pthファイル）を探す"""
    if not os.path.exists(models_dir):
        logger.error(f"ディレクトリ {models_dir} が見つかりません。")
        return []
    pth_files = glob.glob(os.path.join(models_dir, "**", "*.pth"), recursive=True)
    pth_files = [f for f in pth_files if "_trt" not in f]
    return pth_files


def get_available_model_types():
    """annotation_training_d2j/model_catalog.pyから利用可能なモデルタイプを取得"""
    try:
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        annotation_path = os.path.join(project_root, 'annotation_training_d2j')
        sys.path.insert(0, annotation_path)
        from model_catalog import list_all_available_models, SEQUENCE_ARCHITECTURES

        all_models = list_all_available_models()
        sequence_names = set(SEQUENCE_ARCHITECTURES.keys())
        categorized = {"driving": [], "position": [], "waypoint": [], "sequence": []}

        for model_name in all_models:
            if model_name in sequence_names:
                categorized["sequence"].append(model_name)
            elif model_name.endswith("_location"):
                categorized["position"].append(model_name)
            elif model_name.endswith("_waypoint"):
                categorized["waypoint"].append(model_name)
            else:
                categorized["driving"].append(model_name)

        return categorized
    except Exception as e:
        logger.warning(f"モデルカタログの読み込みに失敗: {e}")
        return {
            "driving": ["donkeycar", "resnet18", "resnet34", "mobilevit_xxs", "edgenext_xx_small"],
            "position": ["donkey_location", "resnet18_location"],
            "waypoint": ["donkey_waypoint", "resnet18_waypoint"],
            "sequence": ["gru", "tcn", "causal_cnn"]
        }


def infer_model_type_from_filename(model_path, available_models=None):
    """ファイル名からモデルタイプを推測する"""
    if available_models is None:
        available_models = get_available_model_types()

    basename = os.path.basename(model_path).lower()

    all_model_names = (
        available_models.get("sequence", []) +
        available_models["position"] +
        available_models["waypoint"] +
        available_models["driving"]
    )
    sorted_models = sorted(all_model_names, key=len, reverse=True)

    for model_name in sorted_models:
        model_name_lower = model_name.lower()
        variants = [
            model_name_lower,
            model_name_lower.replace('_', '-'),
            model_name_lower.replace('_', '')
        ]

        for variant in variants:
            if variant in basename:
                if model_name in available_models.get("sequence", []):
                    return model_name, "sequence"
                elif model_name in available_models["position"]:
                    return model_name, "position"
                elif model_name in available_models["waypoint"]:
                    return model_name, "waypoint"
                else:
                    return model_name, "driving"

    if "location" in basename:
        return ("resnet18_location" if "resnet18" in basename else "donkey_location"), "position"
    elif "waypoint" in basename:
        return ("resnet18_waypoint" if "resnet18" in basename else "donkey_waypoint"), "waypoint"
    else:
        return "donkeycar", "driving"


def main():
    parser = argparse.ArgumentParser(description='PyTorchモデルをTensorRTエンジン (.engine) に変換するツール')
    parser.add_argument('--models_dir', type=str, default='models', help='PyTorchモデルを含むディレクトリ')
    parser.add_argument('--model_type', type=str, default=None, help='モデルタイプ (例: resnet18)')
    parser.add_argument('--width', type=int, default=224, help='入力画像の幅')
    parser.add_argument('--height', type=int, default=224, help='入力画像の高さ')
    parser.add_argument('--batch_size', type=int, default=1, help='バッチサイズ')
    parser.add_argument('--fp16', action='store_true', default=True, help='FP16モードを有効にする（デフォルト: 有効）')
    parser.add_argument('--fp32', action='store_true', help='FP32モードで変換（FP16を無効化）')
    parser.add_argument('--no-benchmark', action='store_true', help='変換後のベンチマーク比較をスキップ')
    parser.add_argument('--benchmark-iterations', type=int, default=50, help='ベンチマーク測定回数（デフォルト: 50）')

    args = parser.parse_args()

    if args.fp32:
        args.fp16 = False

    if not torch.cuda.is_available():
        logger.error("CUDA が利用できないため、TensorRTへの変換ができません。")
        return

    try:
        import tensorrt as trt
        logger.info(f"TensorRT version: {trt.__version__}")
    except ImportError:
        logger.error("tensorrt がインストールされていません。")
        return

    # PyTorchモデルを検索
    pth_files = find_pytorch_models(args.models_dir)

    if not pth_files:
        logger.error(f"{args.models_dir} 内にPyTorchモデル（.pthファイル）が見つかりませんでした。")
        return

    pth_files.sort(key=lambda f: os.path.getmtime(f), reverse=True)
    default_idx = 1

    print("\n=== 変換可能なPyTorchモデル ===")
    for i, model_path in enumerate(pth_files):
        print(f"{i+1}. {model_path}")

    while True:
        try:
            choice = input(f"\n変換するモデルの番号を入力してください (デフォルト: {default_idx}, qで終了): ").strip()
            if choice.lower() == 'q':
                return
            if choice == '':
                idx = default_idx - 1
            else:
                idx = int(choice) - 1
            if 0 <= idx < len(pth_files):
                selected_model_path = pth_files[idx]
                break
            else:
                print("有効な番号を入力してください。")
        except ValueError:
            print("数字または 'q' を入力してください。")

    # モデルタイプ推測
    available_models = get_available_model_types()
    inferred_model_type, category = infer_model_type_from_filename(selected_model_path, available_models)

    def _classify_model(name, avail):
        if name in avail.get('sequence', []):
            return 'sequence'
        elif name in avail['position']:
            return 'position'
        elif name in avail['waypoint']:
            return 'waypoint'
        else:
            return 'driving'

    if args.model_type is None:
        print(f"\n推測されたモデルタイプ: {inferred_model_type} ({category}モデル)")
        confirm = input(f"このモデルタイプで変換しますか？ (Y/n または別のモデル名を入力): ").strip()
        if confirm.lower() in ('y', ''):
            model_type = inferred_model_type
        elif confirm.lower() == 'n':
            print("\n利用可能なモデルタイプ:")
            print(f"  自動運転: {', '.join(available_models['driving'])}")
            print(f"  時系列: {', '.join(available_models.get('sequence', []))}")
            print(f"  位置推論: {', '.join(available_models['position'])}")
            print(f"  ウェイポイント: {', '.join(available_models['waypoint'])}")
            model_type = input("\nモデルタイプを入力してください: ")
            category = _classify_model(model_type, available_models)
        else:
            model_type = confirm
            category = _classify_model(model_type, available_models)
    else:
        model_type = args.model_type
        category = _classify_model(model_type, available_models)

    # モデルの読み込みと変換
    try:
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        device = torch.device('cuda')

        # 出力パス（.engine に統一）
        engine_path = selected_model_path.replace('.pth', '.engine')

        if category == 'sequence':
            sys.path.insert(0, project_root)
            from train_pytorch import load_sequence_model
            logger.info(f"時系列モデルを読み込みます: {model_type}")
            model, seq_cfg, selected_sources = load_sequence_model(selected_model_path, device)
            seq_len = seq_cfg.get('seq_len', 8)
            num_image_sources = seq_cfg.get('num_image_sources', 1)
            img_size = seq_cfg.get('img_size', (args.height, args.width))
            if isinstance(img_size, (list, tuple)):
                img_h, img_w = img_size[0], img_size[1]
            else:
                img_h, img_w = img_size, img_size
        elif category in ['position', 'waypoint']:
            annotation_path = os.path.join(project_root, 'annotation_training_d2j')
            sys.path.insert(0, annotation_path)
            from model_catalog import get_model
            logger.info(f"{category}モデルを読み込みます: {model_type}")
            model = get_model(model_type, pretrained=False, input_size=(args.height, args.width))
        else:
            sys.path.insert(0, project_root)
            from train_pytorch import get_model_from_catalog
            logger.info(f"自動運転モデルを読み込みます: {model_type}")
            model = get_model_from_catalog(model_type)

        if model is None:
            logger.error(f"モデルタイプ '{model_type}' の読み込みに失敗しました")
            return

        if category != 'sequence':
            model = load_model_weights(model, selected_model_path, device)
        model = model.to(device)
        model.eval()

        # 変換内容の確認
        print(f"\n選択したモデル: {selected_model_path}")
        print(f"変換後のモデル: {engine_path}")
        if category == 'sequence':
            print(f"入力サイズ: 画像={img_h}x{img_w}, seq_len={seq_len}, sources={num_image_sources}")
        else:
            print(f"入力サイズ: 高さ={args.height}, 幅={args.width}")
        print(f"FP16モード: {'有効' if args.fp16 else '無効'}")

        confirm = input("\nこの設定でモデルを変換しますか？ (Y/n): ").strip().lower()
        if confirm not in ('y', ''):
            print("変換をキャンセルしました。")
            return

        # 変換実行
        if category == 'sequence':
            result = convert_sequence_to_tensorrt(
                model,
                seq_len=seq_len,
                num_image_sources=num_image_sources,
                img_size=(img_h, img_w),
                batch_size=args.batch_size,
                fp16_mode=args.fp16,
                save_path=engine_path,
                device=device
            )
        else:
            result = convert_pytorch_to_tensorrt(
                model,
                input_size=(args.height, args.width),
                batch_size=args.batch_size,
                fp16_mode=args.fp16,
                save_path=engine_path,
                device=device
            )

        if result is not None:
            print(f"\nモデルの変換に成功しました: {result}")

            # ベンチマーク比較
            if not args.no_benchmark:
                print(f"\n{'='*50}")
                print("変換前後の推論速度を比較しています...")
                print(f"{'='*50}")

                sys.path.insert(0, project_root)
                from train_pytorch import TensorRTModel
                trt_model = TensorRTModel(result)

                if category == 'sequence':
                    images = torch.ones((args.batch_size, seq_len, num_image_sources, 3, img_h, img_w)).to(device)
                    ego_states = torch.zeros((args.batch_size, seq_len, 5)).to(device)
                    bench_inputs = [images, ego_states]
                else:
                    x = torch.ones((args.batch_size, 3, args.height, args.width)).to(device)
                    bench_inputs = x

                pytorch_time = benchmark_inference(model, num_iterations=args.benchmark_iterations, label="PyTorch", inputs=bench_inputs)
                tensorrt_time = benchmark_inference(trt_model, num_iterations=args.benchmark_iterations, label="TensorRT", inputs=bench_inputs)
                if pytorch_time > 0 and tensorrt_time > 0:
                    print(f"\n高速化率: {pytorch_time / tensorrt_time:.2f}x")
        else:
            print("\nモデルの変換に失敗しました。")

    except ImportError as e:
        logger.error(f"モジュールをインポートできませんでした: {e}")
    except Exception as e:
        logger.error(f"エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
