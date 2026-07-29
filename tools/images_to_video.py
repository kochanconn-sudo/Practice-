# tools/images_to_video.py
# coding:utf-8
#
# 走行データの画像を動画（mp4）に変換し、対象のdataフォルダ内に保存する。
# catalogデータがあれば、ステアリング・スロットル・センサー値を動画下部にオーバーレイ表示する。
#
# 使い方:
#   python tools/images_to_video.py                                    （一覧から選択）
#   python tools/images_to_video.py data/data_20260314_011703
#   python tools/images_to_video.py data/data_20260314_011703 --fps 30 --prefix cam
#   python tools/images_to_video.py data/data_20260314_011703 --prefix lidar
#   python tools/images_to_video.py data/data_20260314_011703 --no-overlay

import argparse
import glob
import json
import os
import re
import sys
import cv2
import numpy as np


def collect_images(images_dir, prefix="cam"):
    """images_dir 内の画像を番号順に収集する"""
    pattern = re.compile(rf'^(\d+)_{re.escape(prefix)}_image_array_\.jpg$')
    files = []
    for fname in os.listdir(images_dir):
        m = pattern.match(fname)
        if m:
            files.append((int(m.group(1)), os.path.join(images_dir, fname)))
    files.sort(key=lambda x: x[0])
    return [path for _, path in files]


def load_catalog(data_dir):
    """catalogファイルを読み込み、_indexをキーにした辞書を返す"""
    catalog_files = sorted(glob.glob(os.path.join(data_dir, "catalog_*.catalog")))
    records = {}
    for cat_file in catalog_files:
        with open(cat_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                    idx = rec.get("_index")
                    if idx is not None:
                        records[idx] = rec
                except json.JSONDecodeError:
                    continue
    return records


OVERLAY_HEIGHT = 40
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.4
FONT_COLOR = (255, 255, 255)
FONT_THICKNESS = 1
BG_COLOR = (30, 30, 30)


def draw_overlay(frame, record, frame_w):
    """フレーム下部にcatalogデータのオーバーレイを描画して返す"""
    overlay = np.full((OVERLAY_HEIGHT, frame_w, 3), BG_COLOR, dtype=np.uint8)

    # タイムスタンプ
    ts_ms = record.get("_timestamp_ms")
    if ts_ms is not None:
        from datetime import datetime
        dt = datetime.fromtimestamp(ts_ms / 1000.0)
        time_str = dt.strftime("%H:%M:%S.") + f"{dt.microsecond // 1000:03d}"
    else:
        time_str = ""

    mode = record.get("user/mode", "?")
    st = record.get("user/angle", 0.0)
    th = record.get("user/throttle", 0.0)

    # センサー値を収集（lidar/ or ultrasonic/ のキー）
    sensor_parts = []
    for key in sorted(record.keys()):
        if key.startswith("lidar/") and key != "lidar/image_array":
            name = key.split("/", 1)[1]
            val = record[key]
            if isinstance(val, (int, float)):
                sensor_parts.append(f"{name}:{int(val):>5}")
        elif key.startswith("ultrasonic/"):
            name = key.split("/", 1)[1]
            val = record[key]
            if isinstance(val, (int, float)):
                sensor_parts.append(f"{name}:{int(val):>5}")

    # 1行目: time, mode, steering, throttle
    line1 = f"{time_str}  Mode:{mode}  St:{st:>6.2f}  Th:{th:>5.2f}"
    # 2行目: センサー値
    line2 = "  ".join(sensor_parts) if sensor_parts else ""

    cv2.putText(overlay, line1, (5, 14), FONT, FONT_SCALE, FONT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
    if line2:
        cv2.putText(overlay, line2, (5, 32), FONT, FONT_SCALE, (200, 200, 200), FONT_THICKNESS, cv2.LINE_AA)

    return np.vstack([frame, overlay])


def extract_index_from_path(path):
    """画像パスからインデックス番号を抽出する"""
    basename = os.path.basename(path)
    m = re.match(r'^(\d+)_', basename)
    return int(m.group(1)) if m else None


def images_to_video(data_dir, fps=20, prefix="cam", overlay=True):
    """
    dataフォルダ内の画像から動画を生成する。

    Args:
        data_dir: dataフォルダのパス
        fps: 動画のフレームレート
        prefix: 画像のプレフィックス
        overlay: catalogデータのオーバーレイを描画するか

    Returns:
        生成された動画ファイルのパス、または失敗時 None
    """
    images_dir = os.path.join(data_dir, "images")
    if not os.path.isdir(images_dir):
        print(f"画像フォルダが見つかりません: {images_dir}")
        return None

    image_paths = collect_images(images_dir, prefix)
    if not image_paths:
        print(f"画像が見つかりません（prefix='{prefix}'）: {images_dir}")
        return None

    # 最初の画像からサイズを取得
    first = cv2.imread(image_paths[0])
    if first is None:
        print(f"画像を読み込めません: {image_paths[0]}")
        return None
    h, w = first.shape[:2]

    # catalogデータの読み込み
    catalog = {}
    if overlay:
        catalog = load_catalog(data_dir)
        if not catalog:
            print("catalogデータなし: オーバーレイなしで生成します")
            overlay = False

    # 出力サイズ（オーバーレイありの場合は高さを追加）
    out_h = h + OVERLAY_HEIGHT if overlay else h
    out_w = w

    # 出力ファイルパス
    dir_name = os.path.basename(os.path.normpath(data_dir))
    output_name = f"{dir_name}_{prefix}.mp4"
    output_path = os.path.join(data_dir, output_name)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (out_w, out_h))

    total = len(image_paths)
    print(f"動画生成開始: {total}フレーム, {out_w}x{out_h}, {fps}fps → {output_path}")
    for i, path in enumerate(image_paths, 1):
        frame = cv2.imread(path)
        if frame is not None:
            if frame.shape[:2] != (h, w):
                frame = cv2.resize(frame, (w, h))
            if overlay:
                idx = extract_index_from_path(path)
                record = catalog.get(idx, {})
                frame = draw_overlay(frame, record, w)
            writer.write(frame)
        if i % 50 == 0 or i == total:
            print(f"\r  処理中... {i}/{total} ({i*100//total}%)", end="", flush=True)

    writer.release()
    print(f"\n動画生成完了: {output_path}")
    return output_path


def list_data_folders(base_dir="data"):
    """dataフォルダ内の走行データを一覧表示し、番号選択させる"""
    if not os.path.isdir(base_dir):
        print(f"dataフォルダが見つかりません: {base_dir}")
        return None

    # data_で始まるフォルダを日付順にソート
    folders = sorted([
        d for d in os.listdir(base_dir)
        if os.path.isdir(os.path.join(base_dir, d)) and d.startswith("data_")
    ])
    if not folders:
        print(f"{base_dir} 内に走行データがありません")
        return None

    print(f"\n{'No':>3}  {'フォルダ名':<30}  {'画像数':>6}")
    print("-" * 50)
    for i, name in enumerate(folders, 1):
        images_dir = os.path.join(base_dir, name, "images")
        if os.path.isdir(images_dir):
            count = sum(1 for f in os.listdir(images_dir) if f.endswith(".jpg"))
        else:
            count = 0
        print(f"{i:>3}  {name:<30}  {count:>6}")

    print()
    try:
        choice = input("番号を選択（Enter で最新）: ").strip()
    except (KeyboardInterrupt, EOFError):
        print()
        return None

    if choice == "":
        idx = len(folders) - 1
    elif choice.isdigit():
        idx = int(choice) - 1
        if idx < 0 or idx >= len(folders):
            print("無効な番号です")
            return None
    else:
        # フォルダ名の直接入力（部分一致）
        matches = [f for f in folders if choice in f]
        if len(matches) == 1:
            return os.path.join(base_dir, matches[0])
        elif len(matches) > 1:
            print(f"複数一致しました: {', '.join(matches)}")
            return None
        else:
            print(f"一致するフォルダがありません: {choice}")
            return None

    return os.path.join(base_dir, folders[idx])


def main():
    parser = argparse.ArgumentParser(description="走行データの画像を動画に変換")
    parser.add_argument("data_dir", nargs="?", default=None, help="dataフォルダのパス（省略時は一覧から選択）")
    parser.add_argument("--fps", type=int, default=20, help="フレームレート（デフォルト: 20）")
    parser.add_argument("--prefix", default="cam", help="画像プレフィックス（デフォルト: cam）")
    parser.add_argument("--no-overlay", action="store_true", help="データオーバーレイを無効化")
    args = parser.parse_args()

    if args.data_dir is None:
        args.data_dir = list_data_folders()
        if args.data_dir is None:
            sys.exit(1)

    if not os.path.isdir(args.data_dir):
        print(f"フォルダが見つかりません: {args.data_dir}")
        sys.exit(1)

    result = images_to_video(args.data_dir, fps=args.fps, prefix=args.prefix, overlay=not args.no_overlay)
    if result is None:
        sys.exit(1)


if __name__ == "__main__":
    main()
