# coding:utf-8

"""
test_camera_wall_detector.py

実際のRaspberry Pi Cameraから画像を取得し、
camera_wall_detector.pyで画像解析を行うテストプログラム。

このプログラムでは車を動かさない。

処理の流れ:

    Raspberry Pi Camera
            ↓
        camera.py
            ↓
        camera.read()
            ↓
     camera_wall_detector.py
            ↓
       壁・コーナー・障害物候補
            ↓
         ターミナル表示

目的:
    1. Raspberry Pi Cameraが正常に動くか確認
    2. カメラ画像が取得できるか確認
    3. CameraWallDetectorが正常に解析できるか確認
    4. 実際のコースで判定値を確認
    5. 後でperception.pyへ接続するための土台にする

重要:
    このテストではモーターやESCには一切触れない。
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import Optional

import config
import camera

from camera_wall_detector import CameraWallDetector


# ============================================================
# ログ設定
# ============================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
)

logger = logging.getLogger(__name__)


# ============================================================
# テスト設定
# ============================================================

DEFAULT_FRAMES = 0
DEFAULT_INTERVAL = 0.10

# 何フレームごとに表示するか
PRINT_EVERY = 1


# ============================================================
# 引数
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi Camera + CameraWallDetector test"
    )

    parser.add_argument(
        "--frames",
        type=int,
        default=DEFAULT_FRAMES,
        help="テストするフレーム数。0ならCtrl+Cまで継続",
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL,
        help="表示間隔・ループ待機時間（秒）",
    )

    parser.add_argument(
        "--camera-id",
        type=int,
        default=getattr(
            config,
            "CAMERA_0_DEVICE_ID",
            0,
        ),
        help="カメラデバイスID",
    )

    parser.add_argument(
        "--quiet",
        action="store_true",
        help="フレームごとの詳細表示を抑制",
    )

    return parser.parse_args()


# ============================================================
# 結果表示
# ============================================================

def print_result(
    result: dict,
    frame_number: int,
) -> None:
    """
    CameraWallDetectorの結果を見やすく表示する。
    """

    print()
    print("=" * 72)

    print(
        f"FRAME: {frame_number}"
    )

    print("-" * 72)

    # --------------------------------------------------------
    # 壁
    # --------------------------------------------------------

    print("【壁認識】")

    print(
        f"  左壁   : "
        f"{result.get('left_wall', False)}"
        f"  "
        f"(confidence="
        f"{result.get('left_confidence', 0.0):.2f})"
    )

    print(
        f"  前方壁 : "
        f"{result.get('front_wall', False)}"
        f"  "
        f"(confidence="
        f"{result.get('front_confidence', 0.0):.2f})"
    )

    print(
        f"  右壁   : "
        f"{result.get('right_wall', False)}"
        f"  "
        f"(confidence="
        f"{result.get('right_confidence', 0.0):.2f})"
    )

    # --------------------------------------------------------
    # 方向
    # --------------------------------------------------------

    print()
    print("【方向】")

    print(
        f"  direction              : "
        f"{result.get('direction', 0.0):+.3f}"
    )

    print(
        f"  direction_confidence   : "
        f"{result.get('direction_confidence', 0.0):.3f}"
    )

    print(
        f"  center_offset          : "
        f"{result.get('center_offset', 0.0):+.3f}"
    )

    # --------------------------------------------------------
    # コーナー
    # --------------------------------------------------------

    print()
    print("【コーナー】")

    corner = result.get(
        "corner",
        None,
    )

    if corner is None:
        corner_text = "なし"
    else:
        corner_text = str(corner)

    print(
        f"  corner          : "
        f"{corner_text}"
    )

    print(
        f"  confidence      : "
        f"{result.get('corner_confidence', 0.0):.3f}"
    )

    # --------------------------------------------------------
    # 障害物
    # --------------------------------------------------------

    print()
    print("【障害物候補】")

    print(
        f"  obstacle        : "
        f"{result.get('obstacle', False)}"
    )

    print(
        f"  confidence      : "
        f"{result.get('obstacle_confidence', 0.0):.3f}"
    )

    print(
        f"  type            : "
        f"{result.get('obstacle_type', None)}"
    )

    # --------------------------------------------------------
    # 相対距離
    # --------------------------------------------------------

    print()
    print("【カメラ相対距離】")

    print(
        f"  left            : "
        f"{format_optional_float(result.get('left_distance'))}"
    )

    print(
        f"  front           : "
        f"{format_optional_float(result.get('front_distance'))}"
    )

    print(
        f"  right           : "
        f"{format_optional_float(result.get('right_distance'))}"
    )

    # --------------------------------------------------------
    # デバッグ
    # --------------------------------------------------------

    debug = result.get(
        "debug",
        {},
    )

    print()
    print("【デバッグ】")

    if isinstance(debug, dict):

        for key, value in debug.items():

            if isinstance(value, float):
                print(
                    f"  {key:<24}: "
                    f"{value:.4f}"
                )

            else:
                print(
                    f"  {key:<24}: "
                    f"{value}"
                )

    print("=" * 72)


def format_optional_float(
    value,
) -> str:
    if value is None:
        return "None"

    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return str(value)


# ============================================================
# カメラ設定表示
# ============================================================

def print_camera_config(
    camera_id: int,
) -> None:

    print()
    print("=" * 72)
    print("Camera Wall Detector Test")
    print("=" * 72)

    print(
        f"カメラID        : "
        f"{camera_id}"
    )

    print(
        f"カメラタイプ    : "
        f"{getattr(config, 'CAMERA_0_TYPE', None)}"
    )

    print(
        f"解像度          : "
        f"{getattr(config, 'IMAGE_W', '?')}"
        f"x"
        f"{getattr(config, 'IMAGE_H', '?')}"
    )

    print(
        f"VFLIP           : "
        f"{getattr(config, 'CAMERA_0_VFLIP', None)}"
    )

    print(
        f"HFLIP           : "
        f"{getattr(config, 'CAMERA_0_HFLIP', None)}"
    )

    print("=" * 72)
    print()


# ============================================================
# カメラ作成
# ============================================================

def create_test_camera(
    camera_id: int,
):
    """
    test用のカメラを作る。

    Raspberry Pi Cameraを明示的に指定する。

    camera.pyのcreate_camera()は
    camera_type='pi'をサポートしている。
    """

    logger.info(
        "Raspberry Pi Cameraを初期化します..."
    )

    cam = camera.create_camera(
        device_id=camera_id,
        use_multiprocess=False,
        camera_type="pi",
        lidar_instance=None,
    )

    logger.info(
        "Raspberry Pi Camera 初期化完了"
    )

    return cam


# ============================================================
# 1フレームの取得
# ============================================================

def capture_frame(
    cam,
):
    """
    カメラから1フレーム取得する。
    """

    try:
        ret, frame = cam.read()
    except Exception as exc:
        logger.exception(
            "カメラ読み取りエラー: %s",
            exc,
        )
        return False, None

    if not ret:
        return False, None

    if frame is None:
        return False, None

    return True, frame


# ============================================================
# メインテスト
# ============================================================

def run_test(
    frames: int,
    interval: float,
    camera_id: int,
    quiet: bool,
) -> int:

    cam = None
    detector = None

    frame_number = 0

    successful_frames = 0
    failed_frames = 0

    start_time = time.perf_counter()

    try:

        # ----------------------------------------------------
        # 設定表示
        # ----------------------------------------------------

        print_camera_config(
            camera_id
        )

        # ----------------------------------------------------
        # Detector
        # ----------------------------------------------------

        logger.info(
            "CameraWallDetectorを初期化します..."
        )

        detector = (
            CameraWallDetector()
        )

        logger.info(
            "CameraWallDetector 初期化完了"
        )

        # ----------------------------------------------------
        # Camera
        # ----------------------------------------------------

        cam = create_test_camera(
            camera_id
        )

        # ----------------------------------------------------
        # 最初のフレーム取得
        # ----------------------------------------------------

        logger.info(
            "最初のフレームを取得しています..."
        )

        first_ok = False

        for attempt in range(20):

            ret, frame = capture_frame(
                cam
            )

            if ret and frame is not None:

                first_ok = True

                logger.info(
                    "カメラフレーム取得成功 "
                    "shape=%s",
                    frame.shape,
                )

                break

            logger.warning(
                "フレーム取得失敗 "
                "(%d/20)",
                attempt + 1,
            )

            time.sleep(
                0.1
            )

        if not first_ok:

            logger.error(
                "カメラからフレームを取得できませんでした。"
            )

            return 1

        print()
        print(
            "カメラ認識テスト開始"
        )
        print(
            "車は動きません。"
        )
        print(
            "終了するには Ctrl+C"
        )

        if frames > 0:
            print(
                f"指定フレーム数: "
                f"{frames}"
            )

        print()

        # ----------------------------------------------------
        # メインループ
        # ----------------------------------------------------

        while True:

            frame_number += 1

            ret, frame = capture_frame(
                cam
            )

            if not ret or frame is None:

                failed_frames += 1

                logger.warning(
                    "Frame %d: "
                    "画像取得失敗",
                    frame_number,
                )

                time.sleep(
                    interval
                )

                if (
                    frames > 0
                    and frame_number >= frames
                ):
                    break

                continue

            successful_frames += 1

            # ------------------------------------------------
            # カメラ画像解析
            # ------------------------------------------------

            result = detector.analyze(
                frame
            )

            # ------------------------------------------------
            # 表示
            # ------------------------------------------------

            if not quiet:

                if (
                    frame_number
                    % PRINT_EVERY
                    == 0
                ):
                    print_result(
                        result,
                        frame_number,
                    )

            # ------------------------------------------------
            # フレーム数制限
            # ------------------------------------------------

            if (
                frames > 0
                and frame_number >= frames
            ):
                break

            # ------------------------------------------------
            # 待機
            # ------------------------------------------------

            if interval > 0:

                time.sleep(
                    interval
                )

    except KeyboardInterrupt:

        print()
        print(
            "Ctrl+Cを検出しました。"
        )

    except Exception as exc:

        logger.exception(
            "テスト中に予期しないエラー: %s",
            exc,
        )

        return 1

    finally:

        # ----------------------------------------------------
        # カメラ解放
        # ----------------------------------------------------

        if cam is not None:

            logger.info(
                "カメラを解放しています..."
            )

            try:
                cam.release()

            except Exception as exc:

                logger.warning(
                    "カメラ解放時エラー: %s",
                    exc,
                )

        # ----------------------------------------------------
        # 結果
        # ----------------------------------------------------

        elapsed = (
            time.perf_counter()
            - start_time
        )

        print()
        print("=" * 72)
        print("テスト終了")
        print("=" * 72)

        print(
            f"総フレーム数   : "
            f"{frame_number}"
        )

        print(
            f"成功            : "
            f"{successful_frames}"
        )

        print(
            f"失敗            : "
            f"{failed_frames}"
        )

        print(
            f"経過時間        : "
            f"{elapsed:.2f} 秒"
        )

        if elapsed > 0:

            print(
                f"実効FPS         : "
                f"{successful_frames / elapsed:.2f}"
            )

        print("=" * 72)

    return 0


# ============================================================
# メイン
# ============================================================

def main() -> int:

    args = parse_args()

    # --------------------------------------------------------
    # 引数チェック
    # --------------------------------------------------------

    if args.frames < 0:

        print(
            "--frames は0以上にしてください。"
        )

        return 1

    if args.interval < 0:

        print(
            "--interval は0以上にしてください。"
        )

        return 1

    # --------------------------------------------------------
    # 実行
    # --------------------------------------------------------

    return run_test(
        frames=args.frames,
        interval=args.interval,
        camera_id=args.camera_id,
        quiet=args.quiet,
    )


if __name__ == "__main__":
    sys.exit(
        main()
    )
