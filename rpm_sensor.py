#!/usr/bin/env python3
"""
DonkeyCar Part for Hobbywing RPM Sensor (統合版)
XIAO ESP32S3 経由のI2Cモードと、RasPi GPIO直接計測モードの両対応

【モード選択】
  mode='i2c'  : XIAO ESP32S3 がパルスカウント→RPM計算し I2C送信（推奨）
                88,000RPM / 高速モーターに対応
  mode='gpio' : RasPi GPIO で直接パルスポーリング（後方互換・低速用途向け）

【I2C接続構成】
  XIAO ESP32S3 (I2Cスレーブ 0x08)
      ↑ D7ピンで割り込みパルスカウント → RPM計算
      ↓ I2C (SDA/SCL)
  Raspberry Pi (I2Cマスター) ← このモジュールで読み取り

【I2Cレジスタマップ (アドレス 0x08, レジスタ 0x01)】
  Byte  0- 3 : ステアリング PWM値 (uint32 big-endian)
  Byte  4- 7 : スロットル PWM値  (uint32 big-endian)
  Byte  8-11 : 切り替え信号      (uint32 big-endian)
  Byte 12-15 : モーターRPM値     (uint32 big-endian)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Usage in myconfig.py:
    RPM_SENSOR_ENABLED   = True
    RPM_MODE             = 'i2c'     # 'i2c' or 'gpio'

    # I2Cモード用
    RPM_I2C_BUS          = 1
    RPM_I2C_ADDRESS      = 0x08

    # GPIOモード用（後方互換）
    RPM_GPIO_PIN         = 4
    RPM_MOTOR_POLE_PAIRS = 1         # 2極モーター=1, 4極モーター=2

    # 共通
    TIRE_DIAMETER_MM     = 64.0
    GEAR_RATIO           = 8.27
    SPEED_UNIT           = 'm/s'

Usage in manage.py:
    from rpm_sensor import RPMSensor

    if cfg.RPM_SENSOR_ENABLED:
        rpm_sensor = RPMSensor(
            mode=cfg.RPM_MODE,
            # I2Cモード用
            i2c_bus=cfg.RPM_I2C_BUS,
            i2c_address=cfg.RPM_I2C_ADDRESS,
            # GPIOモード用
            gpio_pin=cfg.RPM_GPIO_PIN,
            motor_pole_pairs=cfg.RPM_MOTOR_POLE_PAIRS,
            # 共通
            tire_diameter_mm=cfg.TIRE_DIAMETER_MM,
            gear_ratio=cfg.GEAR_RATIO,
            speed_unit=cfg.SPEED_UNIT,
        )
        V.add(rpm_sensor, outputs=['rpm', 'rps', 'speed'], threaded=True)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import time
import struct
import threading
from multiprocessing import Process, Value

# ── 外部ライブラリ（インポート失敗しても片方のモードだけ無効化） ──────────────
try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    try:
        import smbus as smbus2
        SMBUS_AVAILABLE = True
    except ImportError:
        print("WARNING: smbus2 not found. I2C mode disabled.  → pip install smbus2")
        SMBUS_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("WARNING: RPi.GPIO not found. GPIO mode disabled.")
    GPIO_AVAILABLE = False
    GPIO = None

# ── I2C定数 ────────────────────────────────────────────────────────────────
_I2C_REG_DATA       = 0x01   # ステアリング/スロットル/切替/RPM
_I2C_REG_VERSION    = 0x00   # ファームウェアバージョン
_I2C_DATA_BYTES     = 16
_I2C_RPM_OFFSET     = 12     # Byte12-15 = パルス数(上位16bit) + 経過ms(下位16bit) big-endian


# ══════════════════════════════════════════════════════════════════════════════
class RPMSensor:
    """
    DonkeyCar Part for Hobbywing RPM Sensor (統合版)

    mode='i2c'  : XIAO ESP32S3 からI2C経由でRPMを受信（推奨）
    mode='gpio' : RasPi GPIO でパルスを直接ポーリング（後方互換）
    """

    def __init__(self,
                 mode='i2c',
                 # ── I2Cモード用 ──────────────────────
                 i2c_bus=1,
                 i2c_address=0x08,
                 poll_interval=0.02,
                 # ── GPIOモード用 ─────────────────────
                 gpio_pin=4,
                 motor_pole_pairs=1,
                 sample_interval=0.02,
                 debounce_us=100,
                 # ── 共通 ─────────────────────────────
                 smoothing_enabled=True,
                 ema_alpha=0.2,
                 tire_diameter_mm=64.0,
                 gear_ratio=8.27,
                 speed_unit='m/s'):
        """
        Args:
            mode              : 'i2c' or 'gpio'
            i2c_bus           : I2Cバス番号 (通常1)
            i2c_address       : XIAO のI2Cスレーブアドレス (0x08)
            poll_interval     : I2Cポーリング周期 (秒) [i2cモード]
            gpio_pin          : GPIO BCMピン番号 [gpioモード]
            motor_pole_pairs  : ポールペア数 (2極=1, 4極=2)
            sample_interval   : RPM計算周期 (秒) [gpioモード]
            debounce_us       : デバウンス時間 (µs) [gpioモード]
            smoothing_enabled : EMAスムージング有効/無効
            ema_alpha         : EMA係数 (小さいほど平滑化強い)
            tire_diameter_mm  : タイヤ径 (mm)
            gear_ratio        : ギア比 (モーターRPM / ホイールRPM)
            speed_unit        : 速度単位 ('m/s', 'km/h', 'mph')
        """
        self.enabled   = False
        self.mode      = mode
        self.motor_pole_pairs = motor_pole_pairs
        self.speed_unit = speed_unit
        self.smoothing_enabled = smoothing_enabled
        self.ema_alpha = ema_alpha
        self.tire_diameter_mm = tire_diameter_mm
        self.gear_ratio = gear_ratio
        self.tire_circumference_m = (tire_diameter_mm * 3.14159265) / 1000.0

        if mode == 'i2c':
            self._init_i2c(i2c_bus, i2c_address, poll_interval)
        elif mode == 'gpio':
            self._init_gpio(gpio_pin, motor_pole_pairs, sample_interval, debounce_us)
        else:
            print(f"RPMSensor: 不明なmode '{mode}'. 'i2c' または 'gpio' を指定してください。")

    # ══════════════════════════════════════════════════════════════════════════
    # I2Cモード 初期化
    # ══════════════════════════════════════════════════════════════════════════
    def _init_i2c(self, i2c_bus, i2c_address, poll_interval):
        if not SMBUS_AVAILABLE:
            print("RPMSensor [i2c]: smbus2 が利用できないため無効化されました。")
            return

        self.i2c_bus      = i2c_bus
        self.i2c_address  = i2c_address
        self.poll_interval = poll_interval

        try:
            self.bus = smbus2.SMBus(i2c_bus)
        except Exception as e:
            print(f"RPMSensor [i2c]: I2Cバス{i2c_bus}のオープンに失敗: {e}")
            return

        # スレッド間共有値
        self._lock        = threading.Lock()
        self._rpm         = 0.0
        self._rps         = 0.0
        self._speed       = 0.0
        self._max_rpm     = 0.0
        self._max_speed   = 0.0
        self._total_reads = 0
        self._error_count = 0
        self._ema_rpm     = 0.0

        # バックグラウンドスレッド起動
        self._running = True
        self._thread  = threading.Thread(target=self._i2c_poll_loop, daemon=True)
        self._thread.start()

        self.enabled = True
        print(f"RPMSensor [i2c] initialized")
        print(f"  I2C Bus: {i2c_bus}, Address: 0x{i2c_address:02X}")
        print(f"  Poll rate: {1.0/poll_interval:.1f} Hz")
        print(f"  Gear ratio: {self.gear_ratio:.2f}:1, "
              f"Tire: {self.tire_diameter_mm:.0f}mm")
        print(f"  Smoothing: {'Enabled (EMA α={:.2f})'.format(self.ema_alpha) if self.smoothing_enabled else 'Disabled (Raw)'}")

    def _i2c_read_rpm(self):
        """
        XIAO ESP32S3 からI2C経由でRPM生データを読み取り、RPMを計算する。
        レジスタ0x01の Byte12-15: 上位16bit=パルス数, 下位16bit=経過時間(ms)

        Returns:
            int or None: RPM値。読み取り失敗時は None
        """
        try:
            data = self.bus.read_i2c_block_data(
                self.i2c_address, _I2C_REG_DATA, _I2C_DATA_BYTES
            )
            if len(data) < _I2C_DATA_BYTES:
                return None
            raw = struct.unpack_from('>I', bytes(data), _I2C_RPM_OFFSET)[0]
            pulse_count = (raw >> 16) & 0xFFFF
            elapsed_ms = raw & 0xFFFF
            # ノイズフィルタ: パルス2回未満は停止とみなす
            if pulse_count < 2 or elapsed_ms == 0:
                return 0
            return (pulse_count // self.motor_pole_pairs) * (60000 // elapsed_ms)
        except Exception:
            return None

    def _i2c_poll_loop(self):
        """I2Cポーリングループ（バックグラウンドスレッド）"""
        while self._running:
            t0 = time.time()

            raw_rpm = self._i2c_read_rpm()

            if raw_rpm is not None:
                current_rpm   = self._apply_ema(float(raw_rpm))
                current_rps   = current_rpm / 60.0
                current_speed = self._rpm_to_speed(current_rpm)

                with self._lock:
                    self._rpm   = current_rpm
                    self._rps   = current_rps
                    self._speed = current_speed
                    self._total_reads += 1
                    if current_rpm   > self._max_rpm:   self._max_rpm   = current_rpm
                    if current_speed > self._max_speed: self._max_speed = current_speed
            else:
                with self._lock:
                    self._error_count += 1

            elapsed = time.time() - t0
            sleep_t = self.poll_interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def read_version(self):
        """
        XIAO のファームウェアバージョンを読み取る (I2Cモード専用)

        Returns:
            tuple: (major, minor, patch, firmware_id) or None
        """
        if self.mode != 'i2c' or not self.enabled:
            return None
        try:
            data = self.bus.read_i2c_block_data(
                self.i2c_address, _I2C_REG_DATA, _I2C_DATA_BYTES
            )
            if len(data) < 4:
                return None
            return data[0], data[1], data[2], data[3]
        except Exception as e:
            print(f"Version read error: {e}")
            return None

    # ══════════════════════════════════════════════════════════════════════════
    # GPIOモード 初期化（後方互換）
    # ══════════════════════════════════════════════════════════════════════════
    def _init_gpio(self, gpio_pin, motor_pole_pairs, sample_interval, debounce_us):
        if not GPIO_AVAILABLE:
            print("RPMSensor [gpio]: RPi.GPIO が利用できないため無効化されました。")
            return

        self.gpio_pin         = gpio_pin
        self.motor_pole_pairs = motor_pole_pairs
        self.sample_interval  = sample_interval
        self.debounce_us      = debounce_us

        # multiprocessing 共有メモリ
        self.on                  = Value('b', 1)
        self.shared_rpm          = Value('f', 0.0)
        self.shared_rps          = Value('f', 0.0)
        self.shared_speed        = Value('f', 0.0)
        self.shared_total_pulses = Value('i', 0)
        self.shared_max_rpm      = Value('f', 0.0)
        self.shared_max_speed    = Value('f', 0.0)

        self.p = Process(target=self._gpio_process, args=(
            self.shared_rpm, self.shared_rps, self.shared_speed,
            self.shared_total_pulses, self.shared_max_rpm, self.shared_max_speed
        ))
        self.p.daemon = True
        self.p.start()

        self.enabled = True
        print(f"RPMSensor [gpio] initialized on GPIO{gpio_pin}")
        print(f"  Update rate: {1.0/sample_interval:.1f} Hz")
        print(f"  Gear ratio: {self.gear_ratio:.2f}:1, "
              f"Tire: {self.tire_diameter_mm:.0f}mm")
        print(f"  Smoothing: {'Enabled (EMA)' if self.smoothing_enabled else 'Disabled (Raw)'}")

    def _gpio_process(self, shared_rpm, shared_rps, shared_speed,
                      shared_total_pulses, shared_max_rpm, shared_max_speed):
        """GPIOパルスカウントプロセス（独立プロセスで高速ポーリング）"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

            pulse_count      = 0
            last_pulse_time  = 0
            total_pulses     = 0
            ema_rpm          = 0.0
            last_gpio_state  = GPIO.input(self.gpio_pin)
            last_calc_time   = time.time()

            # 割り込みコールバック（マルチプロセスでは動作しない場合あり、ポーリングが主）
            def pulse_callback(channel):
                nonlocal pulse_count, last_pulse_time, total_pulses
                now_us = time.time() * 1_000_000
                if last_pulse_time > 0 and (now_us - last_pulse_time) < self.debounce_us:
                    return
                pulse_count   += 1
                total_pulses  += 1
                last_pulse_time = now_us

            bouncetime_ms = max(1, int(self.debounce_us / 1000))
            try:
                GPIO.add_event_detect(
                    self.gpio_pin, GPIO.RISING,
                    callback=pulse_callback, bouncetime=bouncetime_ms
                )
            except Exception:
                pass  # ポーリングにフォールバック

            while self.on.value:
                # 立ち上がりエッジをポーリング検出
                cur_state = GPIO.input(self.gpio_pin)
                if cur_state == 1 and last_gpio_state == 0:
                    now_us = time.time() * 1_000_000
                    if last_pulse_time == 0 or (now_us - last_pulse_time) > self.debounce_us:
                        pulse_count  += 1
                        total_pulses += 1
                        last_pulse_time = now_us
                last_gpio_state = cur_state

                # 一定周期でRPM計算
                now = time.time()
                if now - last_calc_time >= self.sample_interval:
                    last_calc_time = now
                    pulses      = pulse_count
                    pulse_count = 0

                    rpm = (pulses / self.motor_pole_pairs) * (60.0 / self.sample_interval)

                    # EMAスムージング
                    if self.smoothing_enabled:
                        if ema_rpm == 0.0 and rpm > 0:
                            ema_rpm = rpm
                        else:
                            ema_rpm = self.ema_alpha * rpm + (1.0 - self.ema_alpha) * ema_rpm
                        current_rpm = ema_rpm
                    else:
                        current_rpm = rpm

                    current_rps   = current_rpm / 60.0
                    current_speed = self._rpm_to_speed(current_rpm)

                    # 統計更新
                    if current_rpm   > shared_max_rpm.value:   shared_max_rpm.value   = current_rpm
                    if current_speed > shared_max_speed.value: shared_max_speed.value = current_speed

                    # 停止判定（500RPM未満でリセット）
                    if current_rpm < 500:
                        ema_rpm = 0.0
                        current_rpm = current_rps = current_speed = 0.0

                    shared_rpm.value          = current_rpm
                    shared_rps.value          = current_rps
                    shared_speed.value        = current_speed
                    shared_total_pulses.value = total_pulses

        except Exception as e:
            print(f"ERROR in RPMSensor [gpio] process: {e}")
            import traceback; traceback.print_exc()
            self.on.value = False
        finally:
            try:
                GPIO.remove_event_detect(self.gpio_pin)
                GPIO.cleanup(self.gpio_pin)
            except Exception:
                pass

    # ══════════════════════════════════════════════════════════════════════════
    # 共通ユーティリティ
    # ══════════════════════════════════════════════════════════════════════════
    def _apply_ema(self, raw_rpm):
        """EMAスムージング（I2Cモード用）。停止時はリセット。"""
        if not self.smoothing_enabled:
            self._ema_rpm = 0.0
            return raw_rpm

        if raw_rpm == 0:
            self._ema_rpm = 0.0
            return 0.0

        if self._ema_rpm == 0.0:
            self._ema_rpm = raw_rpm
        else:
            self._ema_rpm = self.ema_alpha * raw_rpm + (1.0 - self.ema_alpha) * self._ema_rpm

        return self._ema_rpm

    def _rpm_to_speed(self, rpm):
        """モーターRPM → 走行速度 (指定単位) に変換"""
        speed_ms = (rpm / self.gear_ratio * self.tire_circumference_m) / 60.0
        if self.speed_unit == 'km/h':
            return speed_ms * 3.6
        elif self.speed_unit == 'mph':
            return speed_ms * 2.23694
        return speed_ms  # m/s

    # ══════════════════════════════════════════════════════════════════════════
    # DonkeyCar パートインターフェース（共通）
    # ══════════════════════════════════════════════════════════════════════════
    def run_threaded(self):
        """
        DonkeyCar スレッドモード用。現在の (rpm, rps, speed) を返す。
        """
        if not self.enabled:
            return 0.0, 0.0, 0.0

        if self.mode == 'i2c':
            with self._lock:
                return self._rpm, self._rps, self._speed
        else:  # gpio
            return (self.shared_rpm.value,
                    self.shared_rps.value,
                    self.shared_speed.value)

    def run(self):
        """非スレッドモード用（run_threaded と同じ）"""
        return self.run_threaded()

    def update(self):
        """DonkeyCar パートインターフェース（バックグラウンド処理のため何もしない）"""
        pass

    def shutdown(self):
        """DonkeyCar パートインターフェース - シャットダウン時クリーンアップ"""
        if not self.enabled:
            return

        if self.mode == 'i2c':
            self._running = False
            self._thread.join(timeout=1.0)
            try:
                self.bus.close()
            except Exception:
                pass
            with self._lock:
                total  = self._total_reads
                errors = self._error_count
                max_r  = self._max_rpm
                max_s  = self._max_speed
            if total > 0:
                print(f"\nRPMSensor [i2c] Statistics:")
                print(f"  Total I2C reads : {total}")
                print(f"  I2C errors      : {errors} ({100*errors/(total+errors):.1f}%)")
                print(f"  Max RPM         : {int(max_r)}")
                print(f"  Max Speed       : {max_s:.2f} {self.speed_unit}")

        else:  # gpio
            self.on.value = False
            time.sleep(0.2)
            if self.p is not None:
                self.p.terminate()
                self.p.join(timeout=1.0)
            if self.shared_total_pulses.value > 0:
                print(f"\nRPMSensor [gpio] Statistics:")
                print(f"  Total pulses : {self.shared_total_pulses.value}")
                print(f"  Max RPM      : {int(self.shared_max_rpm.value)}")
                print(f"  Max Speed    : {self.shared_max_speed.value:.2f} {self.speed_unit}")

    def get_stats(self):
        """現在の統計情報を辞書で返す（デバッグ用）"""
        if not self.enabled:
            return {}
        if self.mode == 'i2c':
            with self._lock:
                return {
                    'mode': 'i2c',
                    'rpm': self._rpm, 'rps': self._rps, 'speed': self._speed,
                    'max_rpm': self._max_rpm, 'max_speed': self._max_speed,
                    'total_reads': self._total_reads, 'error_count': self._error_count,
                }
        else:
            return {
                'mode': 'gpio',
                'rpm': self.shared_rpm.value,
                'rps': self.shared_rps.value,
                'speed': self.shared_speed.value,
                'max_rpm': self.shared_max_rpm.value,
                'max_speed': self.shared_max_speed.value,
                'total_pulses': self.shared_total_pulses.value,
            }


# ══════════════════════════════════════════════════════════════════════════════
# スタンドアロンテスト
# ══════════════════════════════════════════════════════════════════════════════
def test_rpm_sensor(mode='i2c'):
    """
    スタンドアロン動作確認
    Args:
        mode: 'i2c' or 'gpio'
    """
    print("=" * 70)
    print(f"RPMSensor Test  [mode={mode}]")
    print("=" * 70)

    sensor = RPMSensor(
        mode=mode,
        # I2Cモード用
        i2c_bus=7,                # Jetson Orin Nano: 7, RPi: 1
        i2c_address=0x08,
        poll_interval=0.02,
        # GPIOモード用
        gpio_pin=4,
        motor_pole_pairs=1,       # 2極=1 / 4極=2
        sample_interval=0.02,
        debounce_us=100,
        # 共通
        smoothing_enabled=True,
        ema_alpha=0.2,
        tire_diameter_mm=64.0,
        gear_ratio=8.27,
        speed_unit='m/s',
    )

    if not sensor.enabled:
        print("Sensor not enabled!")
        return

    # I2Cモード: バージョン確認
    if mode == 'i2c':
        ver = sensor.read_version()
        if ver:
            print(f"XIAO Firmware: v{ver[0]}.{ver[1]}.{ver[2]} (FW ID: {ver[3]})")
        else:
            print("WARNING: バージョン情報を読み取れませんでした")

    # GPIOモード: プロセス起動確認
    if mode == 'gpio':
        time.sleep(0.5)
        if not sensor.p.is_alive():
            print("ERROR: GPIOプロセスの起動に失敗しました")
            return

    print(f"\nRPM/RPS/Speed 計測中... (Ctrl+C で停止)")
    print("-" * 75)
    print(f"{'Time':>8}  {'RPM':>8}  {'RPS':>6}  {'Speed':>12}  Status")
    print("-" * 75)

    try:
        start = time.time()
        while True:
            rpm, rps, speed = sensor.run_threaded()
            elapsed   = time.time() - start
            status    = "Running" if rpm > 0 else "Stopped"
            speed_str = f"{speed:.3f} {sensor.speed_unit}"
            print(f"{elapsed:>8.2f}s  {int(rpm):>8}  {rps:>6.2f}  {speed_str:>12}  {status}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print()

    finally:
        sensor.shutdown()


# ── ROS2 ノード実装 ────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Float32

    class RPMNode(Node):
        def __init__(self, mode='i2c'):
            super().__init__('rpm_node')
            import config as _cfg
            # RPMSensorインスタンスを作成（configから設定を取得）
            self.sensor = RPMSensor(
                mode=mode,
                i2c_bus=getattr(_cfg, 'RPM_I2C_BUS', 7),
                i2c_address=getattr(_cfg, 'RPM_I2C_ADDRESS', 0x08),
                poll_interval=0.02,
                gpio_pin=getattr(_cfg, 'RPM_GPIO_PIN', 4),
                motor_pole_pairs=getattr(_cfg, 'RPM_MOTOR_POLE_PAIRS', 1),
                sample_interval=0.02,
                debounce_us=100,
                smoothing_enabled=True,
                ema_alpha=0.2,
                tire_diameter_mm=getattr(_cfg, 'TIRE_DIAMETER_MM', 64.0),
                gear_ratio=getattr(_cfg, 'GEAR_RATIO', 8.27),
                speed_unit='m/s',
            )
            # パブリッシャー
            self.speed_pub = self.create_publisher(TwistStamped, '/rpm/speed', 10)
            self.rpm_pub = self.create_publisher(Float32, '/rpm/value', 10)
            self.frame_id = 'base_link'
            # 0.1秒周期でpublish
            self.timer = self.create_timer(0.1, self.publish_rpm)
            self.get_logger().info(f"RPM Node Initialized (mode={mode})")

        def publish_rpm(self):
            try:
                rpm, rps, speed = self.sensor.run_threaded()

                # 速度 (m/s) — TwistStamped.twist.linear.x
                speed_msg = TwistStamped()
                speed_msg.header.stamp = self.get_clock().now().to_msg()
                speed_msg.header.frame_id = self.frame_id
                speed_msg.twist.linear.x = float(speed)
                self.speed_pub.publish(speed_msg)

                # RPM生値
                rpm_msg = Float32()
                rpm_msg.data = float(rpm)
                self.rpm_pub.publish(rpm_msg)
            except Exception as e:
                self.get_logger().error(f"Error while publishing RPM data: {e}")

        def destroy_node(self):
            self.sensor.shutdown()
            super().destroy_node()

    def main_ros(mode='i2c', args=None):
        """ROSモード実行"""
        rclpy.init(args=args)
        node = RPMNode(mode=mode)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

except ImportError:
    rclpy = None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run RPM sensor as ROS2 node or standalone")
    parser.add_argument('--ros', action='store_true', help="Run as ROS2 node")
    parser.add_argument('--mode', default='i2c', choices=['i2c', 'gpio'], help="Sensor mode (default: i2c)")
    args = parser.parse_args()

    if args.ros and rclpy:
        print("Starting in ROS2 mode...")
        main_ros(mode=args.mode)
    else:
        print("Starting in standalone mode...")
        test_rpm_sensor(args.mode)
