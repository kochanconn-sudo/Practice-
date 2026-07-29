#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOモデルをTensorRTに変換するツール

Usage:
    python tools/yolo2trt_converter.py models/yolo11n_20251105_222010/weights/best.pt
    python tools/yolo2trt_converter.py --all
    python tools/yolo2trt_converter.py --model yolo11n.pt --imgsz 640 --half
"""

import os
import sys
import argparse
import glob

# プロジェクトルートを追加
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)


def find_yolo_models(models_dir="models"):
    """
    指定されたディレクトリ内のYOLOモデル（.ptファイル）を探す

    Args:
        models_dir: 検索するディレクトリ

    Returns:
        見つかったYOLOモデルのリスト
    """
    if not os.path.exists(models_dir):
        print(f"❌ ディレクトリ {models_dir} が見つかりません。")
        return []

    # .ptファイルを検索（YOLOモデル）
    pt_files = glob.glob(os.path.join(models_dir, "**", "*.pt"), recursive=True)

    # _tensorrtやbest.ptを含むファイルのみ（YOLO特有）
    # すでに変換済み（.engine）は除外
    yolo_files = []
    for f in pt_files:
        if "_tensorrt" not in f and not f.endswith('.engine'):
            # YOLOモデルかどうか判定（yolo, best.pt, weights/ を含むパス）
            basename = os.path.basename(f).lower()
            dirname = os.path.dirname(f).lower()

            if 'yolo' in basename or 'yolo' in dirname or basename == 'best.pt' or 'weights' in dirname:
                yolo_files.append(f)

    return yolo_files


def convert_yolo_to_tensorrt(model_path, imgsz=640, half=True, batch=1, device=0, workspace=4, verbose=False):
    """
    YOLOモデルをTensorRTに変換

    Args:
        model_path: YOLOモデルのパス (.pt)
        imgsz: 入力画像サイズ (default: 640)
        half: FP16モードを使用するか (default: True)
        batch: バッチサイズ (default: 1)
        device: 使用するGPUデバイス (default: 0)
        workspace: TensorRTワークスペースサイズ (GB) (default: 4)
        verbose: 詳細ログを表示するか (default: False)

    Returns:
        bool: 変換成功したかどうか
    """
    if not os.path.exists(model_path):
        print(f"❌ エラー: モデルファイルが見つかりません: {model_path}")
        return False

    try:
        from ultralytics import YOLO
    except ImportError:
        print("❌ エラー: ultralytics がインストールされていません")
        print("   pip install ultralytics でインストールしてください")
        return False

    # 出力パスの生成（.pt → .engine）
    if model_path.endswith('.pt'):
        engine_path = model_path.replace('.pt', '.engine')
    else:
        engine_path = model_path + '.engine'

    print(f"\n{'='*70}")
    print(f"YOLOモデル TensorRT変換")
    print(f"{'='*70}")
    print(f"入力モデル: {model_path}")
    print(f"出力モデル: {engine_path}")
    print(f"入力サイズ: {imgsz}x{imgsz}")
    print(f"FP16モード: {half}")
    print(f"バッチサイズ: {batch}")
    print(f"ワークスペース: {workspace} GB")
    print(f"{'='*70}\n")

    try:
        print(f"📥 YOLOモデル読み込み中...")
        model = YOLO(model_path)
        print("✅ モデル読み込み完了\n")

        print("🔄 TensorRT変換中...")
        print("   （初回は時間がかかる場合があります）")

        # TensorRTにエクスポート
        model.export(
            format="engine",
            imgsz=imgsz,
            half=half,
            batch=batch,
            device=device,
            workspace=workspace,
            verbose=verbose
        )

        # ファイルサイズ確認
        if os.path.exists(engine_path):
            file_size_mb = os.path.getsize(engine_path) / (1024 * 1024)

            print(f"\n✅ 変換完了\n")
            print(f"{'='*70}")
            print(f"TensorRTモデル: {engine_path}")
            print(f"ファイルサイズ: {file_size_mb:.1f} MB")
            print(f"{'='*70}")

            # 変換後のモデルをテスト
            print(f"\n🧪 変換後のモデルをテスト中...")
            try:
                tensorrt_model = YOLO(engine_path)
                print("✅ TensorRTモデルの読み込みに成功しました")

                # 簡単な推論テスト（ダミーデータ）
                import numpy as np
                test_image = np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=np.uint8)
                results = tensorrt_model(test_image, verbose=False)
                print("✅ TensorRT推論テスト成功")
                print(f"   検出数: {len(results[0].boxes) if results and len(results) > 0 else 0}")

            except Exception as e:
                print(f"⚠️  テスト中にエラー: {e}")

            return True
        else:
            print(f"❌ エラー: 変換後のファイルが見つかりません: {engine_path}")
            return False

    except Exception as e:
        print(f"❌ エラー: 変換中に問題が発生しました")
        print(f"   {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description='YOLOモデルをTensorRTに変換',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
例:
    # 基本的な使用法
    python tools/yolo2trt_converter.py models/yolo11n_20251105_222010/weights/best.pt

    # 全YOLOモデルを変換
    python tools/yolo2trt_converter.py --all

    # カスタム設定で変換
    python tools/yolo2trt_converter.py models/best.pt --imgsz 640 --half --batch 1

    # FP32モードで変換（高精度）
    python tools/yolo2trt_converter.py models/best.pt --fp32
        """
    )

    parser.add_argument(
        'model_path',
        nargs='?',
        help='変換するYOLOモデルのパス (.pt)'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='modelsフォルダ内の全YOLOモデルを変換'
    )
    parser.add_argument(
        '--models-dir',
        default='models',
        help='モデルディレクトリ (デフォルト: models)'
    )
    parser.add_argument(
        '--imgsz',
        type=int,
        default=640,
        help='入力画像サイズ (デフォルト: 640)'
    )
    parser.add_argument(
        '--half',
        action='store_true',
        default=True,
        help='FP16モードで変換 (デフォルト: 有効)'
    )
    parser.add_argument(
        '--fp32',
        action='store_true',
        help='FP32モードで変換（FP16を無効化）'
    )
    parser.add_argument(
        '--batch',
        type=int,
        default=1,
        help='バッチサイズ (デフォルト: 1)'
    )
    parser.add_argument(
        '--device',
        type=int,
        default=0,
        help='使用するGPUデバイス (デフォルト: 0)'
    )
    parser.add_argument(
        '--workspace',
        type=int,
        default=4,
        help='TensorRTワークスペースサイズ (GB) (デフォルト: 4)'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='詳細ログを表示'
    )

    args = parser.parse_args()

    # FP32フラグが指定された場合、halfを無効化
    if args.fp32:
        args.half = False

    # 変換対象モデルを決定
    models_to_convert = []

    if args.all:
        # 全YOLOモデルを変換
        yolo_models = find_yolo_models(args.models_dir)
        if not yolo_models:
            print(f"❌ {args.models_dir} 内にYOLOモデルが見つかりません")
            return 1
        models_to_convert = yolo_models
        print(f"\n📋 {len(models_to_convert)}個のYOLOモデルを変換します")
        for i, model_path in enumerate(models_to_convert, 1):
            print(f"   {i}. {model_path}")

    elif args.model_path:
        # 指定されたモデルを変換
        if not os.path.exists(args.model_path):
            print(f"❌ モデルが見つかりません: {args.model_path}")
            return 1
        models_to_convert = [args.model_path]

    else:
        # 対話的にモデルを選択
        yolo_models = find_yolo_models(args.models_dir)
        if not yolo_models:
            print(f"❌ {args.models_dir} 内にYOLOモデルが見つかりません")
            return 1

        print("\n=== 変換可能なYOLOモデル ===")
        for i, model_path in enumerate(yolo_models, 1):
            # .engineが存在するかチェック
            engine_path = model_path.replace('.pt', '.engine')
            status = "✅ 変換済み" if os.path.exists(engine_path) else ""
            print(f"{i}. {model_path:60s} {status}")

        while True:
            try:
                choice = input("\n変換するモデルの番号を入力してください（aで全て、qで終了）: ")
                if choice.lower() == 'q':
                    return 0
                elif choice.lower() == 'a':
                    models_to_convert = yolo_models
                    break
                else:
                    idx = int(choice) - 1
                    if 0 <= idx < len(yolo_models):
                        models_to_convert = [yolo_models[idx]]
                        break
                    else:
                        print("有効な番号を入力してください。")
            except ValueError:
                print("数字、'a'、または 'q' を入力してください。")

    # 変換実行
    print(f"\n🚀 YOLOモデルTensorRT変換開始")
    print(f"   合計: {len(models_to_convert)}個のモデル\n")

    success_count = 0
    failed_models = []

    for i, model_path in enumerate(models_to_convert, 1):
        print(f"\n[{i}/{len(models_to_convert)}] 変換中: {model_path}")

        success = convert_yolo_to_tensorrt(
            model_path=model_path,
            imgsz=args.imgsz,
            half=args.half,
            batch=args.batch,
            device=args.device,
            workspace=args.workspace,
            verbose=args.verbose
        )

        if success:
            success_count += 1
        else:
            failed_models.append(model_path)

    # 結果サマリー
    print("\n\n" + "="*70)
    print("変換結果サマリー")
    print("="*70)
    print(f"成功: {success_count}/{len(models_to_convert)}")

    if failed_models:
        print(f"\n失敗したモデル:")
        for model_path in failed_models:
            print(f"  ❌ {model_path}")

    print("="*70)

    return 0 if success_count == len(models_to_convert) else 1


if __name__ == "__main__":
    sys.exit(main())
