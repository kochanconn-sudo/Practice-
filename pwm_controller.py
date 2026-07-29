#!/usr/bin/env python3
# coding:utf-8
"""
PWMコントローラークラス
プロポからのPWM信号をI2C経由で読み取り、joystickと同じインターフェースで操作値を提供

対応ファームウェア: firmware/esp32s3/esp32s3.ino (v1.3.2以降)
  - XIAO ESP32S3にArduino IDEで書き込んで使用
  - I2Cレジスタ0x01で16バイト送信: CH1(4) + CH2(4) + CH3(4) + RPM(4)

使用方法:
    from pwm_controller import PWMController
    controller = PWMController()
    controller.poll()  # PWM信号を読み取って更新
    steering = controller.steering  # -1.0 ~ 1.0
    throttle = controller.throttle  # -1.0 ~ 1.0
"""

import config
import os
import fcntl
import time

# I2C ioctl定数
I2C_SLAVE = 0x0703


class PWMController:
    """
    PWM信号を読み取ってジョイスティックと同じインターフェースで操作値を提供するクラス
    """

    def __init__(self):
        """初期化"""
        self.HAVE_CONTROLLER = False

        # joystickと同じインターフェースの属性
        self.steering = 0.0
        self.throttle = 0.0
        self.mode = ["user", "auto_str", "auto"]  # mode[0]が現在のモード
        self.mode_sequence = ["user", "auto", "auto_str"]
        self.mode_index = 0
        self.recording = False
        self.is_braking = False

        # PWM設定
        self.i2c_bus = config.PWM_I2C_BUS
        self.i2c_addr = config.PWM_I2C_ADDRESS
        self.i2c_fd = None

        # キャリブレーション値を読み込み
        self.ch1_left = config.PWM_CH1_LEFT_RAW
        self.ch1_center = config.PWM_CH1_CENTER_RAW
        self.ch1_right = config.PWM_CH1_RIGHT_RAW

        self.ch2_forward = config.PWM_CH2_FORWARD_RAW
        self.ch2_neutral = config.PWM_CH2_NEUTRAL_RAW
        self.ch2_reverse = config.PWM_CH2_REVERSE_RAW

        # デッドゾーン設定（config.pyから読み込み）
        self.deadzone_steering = getattr(config, 'PWM_DEADZONE_STEERING', 0.03)
        self.deadzone_throttle = getattr(config, 'PWM_DEADZONE_THROTTLE', 0.03)

        # PWM有効範囲（pulseInLongの戻り値はマイクロ秒、通常RC信号は500〜2500μs）
        self.pwm_min_valid = 500
        self.pwm_max_valid = 2500

        # CH3（モード切替信号）- エッジ検出でモードローテーション
        self.rc_mode = "unknown"  # "ai", "rc", "no_signal"
        self.prev_rc_mode = "unknown"

        # RPMセンサー（transfer5: 上位16bit=パルス数, 下位16bit=経過ms）
        self.rpm = 0
        self.pulse_count = 0
        self.pulse_elapsed_ms = 0
        self.rpm_min_pulse = 2  # この値未満のパルスカウントはノイズとして除外
        self.motor_pole_pairs = getattr(config, 'RPM_MOTOR_POLE_PAIRS', 1)

        # RAW値の保持（monitor等で二重読み取りを防ぐ）
        self.raw1 = 0
        self.raw2 = 0
        self.raw3 = 0
        self.raw4 = 0

        # 前回の値（スムージング用）
        self.prev_steering = 0.0
        self.prev_throttle = 0.0
        self.smoothing = 0.3  # 0.0=即座に変化、1.0=変化なし

        # I2Cリトライ設定（ESP32S3はpulseInLong/delay中に応答できないため）
        self.i2c_max_retries = 5
        self.i2c_retry_delay = 0.1  # 秒

        # I2C接続試行（デバイスファイルのオープン+アドレス設定のみで接続確認）
        try:
            self.i2c_fd = os.open(f"/dev/i2c-{self.i2c_bus}", os.O_RDWR)
            fcntl.ioctl(self.i2c_fd, I2C_SLAVE, self.i2c_addr)
            self.HAVE_CONTROLLER = True
            print(f"PWMコントローラー接続成功 (I2C: bus={self.i2c_bus}, addr=0x{self.i2c_addr:02X})")
            print(f"キャリブレーション値:")
            print(f"  CH1 (ステアリング): LEFT={self.ch1_left}, CENTER={self.ch1_center}, RIGHT={self.ch1_right}")
            print(f"  CH2 (スロットル):   FORWARD={self.ch2_forward}, NEUTRAL={self.ch2_neutral}, REVERSE={self.ch2_reverse}")
        except Exception as e:
            self.HAVE_CONTROLLER = False
            print(f"PWMコントローラー接続失敗: {e}")
            print("キーボード操作に切り替えます")

    def is_valid_pwm(self, raw_value):
        """
        PWM値が有効範囲内かチェック
        pulseInLongはタイムアウト時に0を返すため、0や範囲外の値を除外する

        Args:
            raw_value: 生のPWM値（マイクロ秒）

        Returns:
            bool: 有効な値ならTrue
        """
        return self.pwm_min_valid <= raw_value <= self.pwm_max_valid

    def read_raw_values(self):
        """
        PWM生値を読み取る
        write(レジスタ設定)とread(データ取得)を分離したトランザクションで通信

        Returns:
            tuple: (raw1, raw2, raw3, raw4) - CH1/CH2/CH3/RPMの生値、エラー時は(None, None, None, None)
        """
        if not self.HAVE_CONTROLLER:
            return None, None, None, None

        for attempt in range(self.i2c_max_retries):
            try:
                # writeトランザクション: レジスタアドレス送信 → ESP32S3のonReceive発火
                os.write(self.i2c_fd, bytes([0x01]))
                # readトランザクション: 16バイト読み取り → ESP32S3のonRequest発火
                # CH1(4) + CH2(4) + CH3(4) + RPM(4) = 16バイト
                data = os.read(self.i2c_fd, 16)

                if len(data) != 16:
                    continue

                # 4バイトずつ32ビット値に変換（ビッグエンディアン: a,b,c,d = MSB→LSB）
                raw1 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
                raw2 = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]
                raw3 = data[8] << 24 | data[9] << 16 | data[10] << 8 | data[11]
                raw4 = data[12] << 24 | data[13] << 16 | data[14] << 8 | data[15]

                return raw1, raw2, raw3, raw4
            except OSError:
                if attempt < self.i2c_max_retries - 1:
                    time.sleep(self.i2c_retry_delay)
        return None, None, None, None

    def raw_to_normalized(self, raw_value, min_val, center_val, max_val):
        """
        RAW値を-1.0~1.0の範囲に正規化（デッドゾーンはpoll()で適用）

        Args:
            raw_value: 生のPWM値
            min_val: 最小値（左または前進）
            center_val: 中央値（ニュートラル）
            max_val: 最大値（右または後退）

        Returns:
            float: -1.0 ~ 1.0の正規化された値
        """
        if raw_value < center_val:
            # 中央より小さい側（左/前進）
            if min_val == center_val:
                return 0.0
            normalized = (raw_value - center_val) / (center_val - min_val)
        else:
            # 中央より大きい側（右/後退）
            if max_val == center_val:
                return 0.0
            normalized = (raw_value - center_val) / (max_val - center_val)

        # -1.0 ~ 1.0 にクリップ
        normalized = max(-1.0, min(1.0, normalized))

        return normalized

    def apply_smoothing(self, new_value, prev_value):
        """
        値のスムージング（急激な変化を抑える）

        Args:
            new_value: 新しい値
            prev_value: 前回の値

        Returns:
            float: スムージング適用後の値
        """
        return prev_value * self.smoothing + new_value * (1 - self.smoothing)

    def poll(self):
        """
        PWM信号を読み取って操作値を更新
        joystick.poll()と同じインターフェース
        """
        if not self.HAVE_CONTROLLER:
            return

        # PWM生値を読み取り
        raw1, raw2, raw3, raw4 = self.read_raw_values()

        if raw1 is None or raw2 is None:
            return

        # RAW値を保持（monitor/calibrate表示用）
        self.raw1 = raw1
        self.raw2 = raw2
        self.raw3 = raw3 if raw3 is not None else 0
        self.raw4 = raw4 if raw4 is not None else 0

        # CH3（モード切替信号）の判定 + エッジ検出でモードローテーション
        if raw3 is not None:
            if raw3 > 1500:
                self.rc_mode = "ai"
            elif raw3 >= 100:
                self.rc_mode = "rc"
            else:
                self.rc_mode = "no_signal"

            # エッジ検出: RC→AIに切り替わった瞬間にモードを切り替え
            if self.rc_mode == "ai" and self.prev_rc_mode != "ai":
                self.mode_index = (self.mode_index + 1) % len(self.mode_sequence)
                self.mode[0] = self.mode_sequence[self.mode_index]
                print("Mode:", self.mode[0])
            self.prev_rc_mode = self.rc_mode

        # RPMセンサー生データの展開と計算
        # transfer5: 上位16bit=パルス数, 下位16bit=経過時間(ms)
        if raw4 is not None:
            self.pulse_count = (raw4 >> 16) & 0xFFFF
            self.pulse_elapsed_ms = raw4 & 0xFFFF
            if self.pulse_count >= self.rpm_min_pulse and self.pulse_elapsed_ms > 0:
                self.rpm = (self.pulse_count // self.motor_pole_pairs) * (60000 // self.pulse_elapsed_ms)
            else:
                self.rpm = 0

        # 無信号バリデーション: pulseInLongはタイムアウト時に0を返す
        # 有効範囲外の値は無視し、前回値を保持する
        if not self.is_valid_pwm(raw1) or not self.is_valid_pwm(raw2):
            return

        # 正規化
        raw_steering = self.raw_to_normalized(raw1, self.ch1_left, self.ch1_center, self.ch1_right)
        raw_throttle = self.raw_to_normalized(raw2, self.ch2_forward, self.ch2_neutral, self.ch2_reverse)

        # スムージング適用
        self.steering = self.apply_smoothing(raw_steering, self.prev_steering)
        self.throttle = self.apply_smoothing(raw_throttle, self.prev_throttle)

        # デッドゾーン適用（スムージング後に適用し、微小残留値を除去）
        if abs(self.steering) < self.deadzone_steering:
            self.steering = 0.0
        if abs(self.throttle) < self.deadzone_throttle:
            self.throttle = 0.0

        # 前回値を更新
        self.prev_steering = self.steering
        self.prev_throttle = self.throttle

        # ブレーキ判定（スロットルが後退側に大きく倒れている場合）
        self.is_braking = self.throttle < -0.8

    def calibrate(self, interval=0.05):
        """
        PWM信号のキャリブレーションモード
        プロポを操作して最大値・最小値を記録

        Args:
            interval: 読み取り間隔（秒）
        """
        if not self.HAVE_CONTROLLER:
            print("エラー: コントローラーが接続されていません")
            return

        print("\n" + "=" * 80)
        print("PWMキャリブレーションモード")
        print("=" * 80)
        print("\n指示に従ってプロポを操作してください：")
        print("  1. ステアリングを左右に最大まで動かす")
        print("  2. スロットルを前進・後退に最大まで動かす")
        print("  3. 各位置で1〜2秒間保持してください")
        print("\nCtrl+Cで終了すると、測定した最大値・最小値が表示されます")
        print("-" * 80)

        # 最大値・最小値の初期化
        ch1_min = float('inf')
        ch1_max = float('-inf')
        ch1_center = None
        ch2_min = float('inf')
        ch2_max = float('-inf')
        ch2_neutral = None
        ch3_min = float('inf')
        ch3_max = float('-inf')

        # 初期値取得用のカウンター
        stable_count = 0
        required_stable = 10  # 10回連続で安定した値を取得

        print(f"\n{'時刻':<12} {'CH1(RAW)':>10} {'CH2(RAW)':>10} {'CH3(RAW)':>10} {'状態':<30}")
        print("-" * 90)

        try:
            while True:
                raw1, raw2, raw3, raw4 = self.read_raw_values()

                if raw1 is not None and raw2 is not None and self.is_valid_pwm(raw1) and self.is_valid_pwm(raw2):
                    # 最大値・最小値の更新
                    ch1_min = min(ch1_min, raw1)
                    ch1_max = max(ch1_max, raw1)
                    ch2_min = min(ch2_min, raw2)
                    ch2_max = max(ch2_max, raw2)

                    # CH3の最大値・最小値を更新（有効値のみ）
                    if raw3 is not None and raw3 >= 100:
                        ch3_min = min(ch3_min, raw3)
                        ch3_max = max(ch3_max, raw3)

                    # 中央値・ニュートラル値の推定（初期10回の平均）
                    if stable_count < required_stable:
                        if ch1_center is None:
                            ch1_center = raw1
                            ch2_neutral = raw2
                        else:
                            ch1_center = (ch1_center * stable_count + raw1) / (stable_count + 1)
                            ch2_neutral = (ch2_neutral * stable_count + raw2) / (stable_count + 1)
                        stable_count += 1

                    # 状態判定
                    status = []
                    if abs(raw1 - ch1_min) < 50:
                        status.append("ステア:左MAX")
                    elif abs(raw1 - ch1_max) < 50:
                        status.append("ステア:右MAX")
                    elif ch1_center and abs(raw1 - ch1_center) < 50:
                        status.append("ステア:中央")

                    if abs(raw2 - ch2_min) < 50:
                        status.append("スロ:前進MAX")
                    elif abs(raw2 - ch2_max) < 50:
                        status.append("スロ:後退MAX")
                    elif ch2_neutral and abs(raw2 - ch2_neutral) < 50:
                        status.append("スロ:中立")

                    # CH3の状態判定
                    if raw3 is not None and raw3 >= 100:
                        if raw3 > 1500:
                            status.append("CH3:AI")
                        else:
                            status.append("CH3:RC")
                    elif raw3 is not None:
                        status.append("CH3:無信号")

                    status_str = " / ".join(status) if status else "操作中..."

                    timestamp = time.strftime("%H:%M:%S")
                    ch3_str = f"{raw3:>10}" if raw3 is not None else f"{'---':>10}"
                    print(f"{timestamp:<12} {raw1:>10} {raw2:>10} {ch3_str} {status_str:<30}")

                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n" + "=" * 80)
            print("キャリブレーション結果")
            print("=" * 80)
            print("\n【CH1: ステアリング】")
            print(f"  左最大   (LEFT):   {ch1_min:>10} RAW")
            print(f"  中央     (CENTER): {int(ch1_center):>10} RAW" if ch1_center else "  中央値: 未測定")
            print(f"  右最大   (RIGHT):  {ch1_max:>10} RAW")

            print("\n【CH2: スロットル】")
            print(f"  前進最大 (FORWARD): {ch2_min:>10} RAW")
            print(f"  中立     (NEUTRAL): {int(ch2_neutral):>10} RAW" if ch2_neutral else "  中立値: 未測定")
            print(f"  後退最大 (REVERSE): {ch2_max:>10} RAW")

            print("\n【CH3: モード切替】")
            if ch3_min != float('inf'):
                print(f"  最小値:             {ch3_min:>10} RAW")
                print(f"  最大値:             {ch3_max:>10} RAW")
                print(f"  閾値: >1500=AI, 100-1500=RC, <100=無信号")
            else:
                print("  信号未検出")

            print("\n" + "=" * 80)
            print("config.pyに設定する値:")
            print("=" * 80)
            print(f"PWM_CH1_LEFT_RAW = {ch1_min}")
            print(f"PWM_CH1_CENTER_RAW = {int(ch1_center) if ch1_center else 0}")
            print(f"PWM_CH1_RIGHT_RAW = {ch1_max}")
            print(f"PWM_CH2_FORWARD_RAW = {ch2_min}")
            print(f"PWM_CH2_NEUTRAL_RAW = {int(ch2_neutral) if ch2_neutral else 0}")
            print(f"PWM_CH2_REVERSE_RAW = {ch2_max}")
            print("=" * 80)

    def monitor(self, interval=0.05):
        """
        PWM信号を連続監視

        Args:
            interval: 読み取り間隔（秒）
        """
        if not self.HAVE_CONTROLLER:
            print("エラー: コントローラーが接続されていません")
            return

        print("\nPWM信号監視開始 (Ctrl+Cで終了)")
        print("-" * 110)
        print(f"{'時刻':<12} {'Steering':>10} {'Throttle':>10} {'RAW1':>8} {'RAW2':>8} {'RAW3':>8} {'RPM':>8} {'Mode':<12}")
        print("-" * 110)

        try:
            while True:
                self.poll()
                timestamp = time.strftime("%H:%M:%S")

                if self.raw1 > 0:
                    print(f"{timestamp:<12} {self.steering:>10.3f} {self.throttle:>10.3f} {self.raw1:>8} {self.raw2:>8} {self.raw3:>8} {self.rpm:>8} {self.rc_mode:<12}")
                else:
                    print(f"{timestamp:<12} {'---':>10} {'---':>10} {'---':>8} {'---':>8} {'---':>8} {'---':>8} {'---':<12}")

                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n監視を終了しました")

    def close(self):
        """I2C接続を閉じる"""
        if self.i2c_fd is not None:
            try:
                os.close(self.i2c_fd)
                self.i2c_fd = None
                print("PWMコントローラー接続を閉じました")
            except:
                pass


# テスト用のメイン関数
if __name__ == "__main__":
    import sys

    print("=" * 80)
    print("PWMコントローラー")
    print("=" * 80)

    # コマンドライン引数でモード選択
    mode = "calibrate"  # デフォルトはキャリブレーションモード
    if len(sys.argv) > 1:
        if sys.argv[1] in ["monitor", "m"]:
            mode = "monitor"
        elif sys.argv[1] in ["calibrate", "c", "calib"]:
            mode = "calibrate"
        elif sys.argv[1] in ["test", "t"]:
            mode = "test"
        else:
            print(f"\n使用方法: {sys.argv[0]} [calibrate|monitor|test]")
            print("  calibrate (c): キャリブレーションモード（デフォルト）")
            print("  monitor (m):   監視モード（正規化された値＋RAW値）")
            print("  test (t):      簡易テストモード（正規化された値のみ）")
            sys.exit(1)

    # --- ROS2 モード ---
    if '--ros' in sys.argv:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            from geometry_msgs.msg import Twist

            class PWMControllerNode(Node):
                def __init__(self):
                    super().__init__('pwm_controller_node')
                    self.controller = PWMController()
                    if not self.controller.HAVE_CONTROLLER:
                        self.get_logger().error("PWMコントローラーに接続できません")
                        return
                    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
                    self.mode_pub = self.create_publisher(String, '/joy/mode', 10)
                    self.timer = self.create_timer(0.02, self.publish_data)
                    self.get_logger().info("PWM Controller node started")

                def publish_data(self):
                    self.controller.poll()
                    mode = self.controller.mode[0]
                    self.mode_pub.publish(String(data=mode))
                    if mode == "user":
                        twist = Twist()
                        twist.linear.x = float(self.controller.throttle)
                        twist.angular.z = float(self.controller.steering)
                        self.cmd_vel_pub.publish(twist)

            def main_ros(args=None):
                rclpy.init(args=args)
                node = PWMControllerNode()
                try:
                    rclpy.spin(node)
                except KeyboardInterrupt:
                    pass
                finally:
                    if hasattr(node, 'controller'):
                        node.controller.close()
                    node.destroy_node()
                    if rclpy.ok():
                        rclpy.shutdown()

            main_ros()
            sys.exit(0)
        except ImportError:
            print("ROS2が利用できません")
            sys.exit(1)

    controller = PWMController()

    if not controller.HAVE_CONTROLLER:
        print("エラー: PWMコントローラーに接続できませんでした")
        print("\n確認事項:")
        print("  1. I2Cデバイスが接続されているか")
        print("  2. I2Cバス番号が正しいか (i2cdetect -y -r 7)")
        print("  3. 適切な権限があるか (sudo usermod -aG i2c $USER)")
        sys.exit(1)

    try:
        if mode == "calibrate":
            controller.calibrate(interval=0.1)
        elif mode == "monitor":
            controller.monitor(interval=0.1)
        elif mode == "test":
            # 簡易テストモード（正規化された値のみ表示）
            print("\nPWMコントローラー簡易テスト")
            print("\nプロポを操作してください（Ctrl+Cで終了）")
            print("-" * 80)
            print(f"{'時刻':<12} {'Steering':>10} {'Throttle':>10} {'Braking':<8}")
            print("-" * 80)

            while True:
                controller.poll()

                timestamp = time.strftime("%H:%M:%S")
                braking_str = "BRAKE" if controller.is_braking else ""

                print(f"{timestamp:<12} {controller.steering:>10.3f} {controller.throttle:>10.3f} {braking_str:<8}")

                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n終了")
    finally:
        controller.close()
