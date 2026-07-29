# coding:utf-8
import config
from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import numpy as np

class Joystick(object):
    def __init__(self, dev_fn=config.JOYSTICK_DEVICE_FILE):
        self.HAVE_CONTROLLER = True
        self.stick_left = config.JOYSTICK_AXIS_LEFT
        self.stick_right = config.JOYSTICK_AXIS_RIGHT
        self.buttons = {
            "Y": config.JOYSTICK_Y,
            "X": config.JOYSTICK_X,
            "A": config.JOYSTICK_A,
            "B": config.JOYSTICK_B,
            "S": config.JOYSTICK_S,
        }
        self.steering = 0.0 #changed from steer
        self.throttle = 0.0 #changed from accel
        self.throttle_const_0 = config.FORWARD_STRAIGHT
        self.throttle_const_1 = config.FORWARD_CORNER
        self.throttle_stop_on = False
        self.button_states = {name: False for name in self.buttons.keys()}
        self.previous_button_states = {name: False for name in self.buttons.keys()}
        self.mode = ["user", "auto_str", "auto"]
        self.recording = False  # 初期状態は記録OFF
        self.is_braking = False  # ブレーキ状態フラグ
        self.previous_braking = False  # 前回のブレーキ状態

        # pygameの初期化
        pygame.init()
        pygame.joystick.init()

        # ジョイスティックの初期化
        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print('ジョイスティックの名前:', self.joystick.get_name())
            print('ボタン数 :', self.joystick.get_numbuttons())
        except pygame.error:
            self.HAVE_CONTROLLER = False
            print('ジョイスティックが接続されていません。ジョイスティックをOFFにします。')

    def poll(self):
        # ジョイスティックが接続されていない場合は何もしない
        if not self.HAVE_CONTROLLER:
            return
            
        # まずイベントを取得
        for e in pygame.event.get():
            pass  # イベントの中身を特に使わない場合はパスでもOK
        
        # 前回の状態を保存
        self.previous_button_states = self.button_states.copy()
        
        # ボタン状態を更新（押されているかどうか）
        for name, button_id in self.buttons.items():
            self.button_states[name] = bool(self.joystick.get_button(button_id))
        
        # 軸の値を取得
        self.steering = round(self.joystick.get_axis(self.stick_left), 2)
        # ひとまずスティックから throttle を取る
        current_throttle = round(self.joystick.get_axis(self.stick_right), 2)
        
        # ボタン押下に応じて throttle を上書き
        if self.button_states["X"]:
            self.throttle = self.throttle_const_0 * -1 #コントローラの軸は逆
            self.is_braking = False
        elif self.button_states["B"]:
            self.throttle = self.throttle_const_1 * -1 #コントローラの軸は逆
            self.is_braking = False
        elif self.button_states["A"]:
            self.throttle = -1.0
            self.is_braking = True  # Aボタンはブレーキ
        else:
            self.throttle = current_throttle
            self.is_braking = False

        # ジョイスティックのスケーリングと軸方向合わせ（ブレーキ時は除く）
        if not self.is_braking:
            self.throttle = self.throttle * config.JOYSTICK_THROTTLE_SCALE
        self.steering = self.steering * config.JOYSTICK_STEERING_SCALE

        # Sボタンが押された瞬間を検出（エッジ検出）
        if self.button_states["S"] and not self.previous_button_states["S"]:
            self.mode = np.roll(self.mode, 1)
            print("Mode:", self.mode[0])

        # Yボタンが押された瞬間を検出
        if self.button_states["Y"] and not self.previous_button_states["Y"]:
            self.recording = not self.recording
            print(f"Recording: {self.recording}")

        # ブレーキ終了時の記録停止処理
        if self.previous_braking and not self.is_braking:
            # ブレーキから離された瞬間
            if self.recording:
                self.recording = False
                print("*** Recording stopped after brake ***")

        # ブレーキ状態を保存
        self.previous_braking = self.is_braking

class KeyboardController(object):
    """キーボード操作でJoystickクラスと同等の機能を提供"""
    def __init__(self):
        self.HAVE_CONTROLLER = True
        self.steering = 0.0
        self.throttle = 0.0
        self.throttle_const_0 = config.FORWARD_STRAIGHT
        self.throttle_const_1 = config.FORWARD_CORNER
        self.throttle_stop_on = False
        self.button_states = {
            "Y": False,
            "X": False,
            "A": False,
            "B": False,
            "S": False,
        }
        self.previous_button_states = {name: False for name in self.button_states.keys()}
        self.mode = ["user", "auto_str", "auto"]
        self.recording = False
        self.is_braking = False  # ブレーキ状態フラグ
        self.previous_braking = False  # 前回のブレーキ状態

        # pygameの初期化
        pygame.init()
        pygame.display.set_mode((400, 300))  # 小さなウィンドウを作成（フォーカス用）
        pygame.display.set_caption("Manual Control - Use WASD keys, Space=throttle, R=record, M=mode")
        
        print("キーボード操作モード:")
        print("  W/↑: スロットル前進")
        print("  S/↓: スロットル後退")
        print("  A/←: ステアリング左")
        print("  D/→: ステアリング右")
        print("  Space: ブレーキ/停止")
        print("  R: 記録開始/停止")
        print("  M: モード切替")
        print("  ESC: 終了")

    def poll(self):
        """キーボード入力をポーリング"""
        # 前回の状態を保存
        self.previous_button_states = self.button_states.copy()
        
        # イベント処理
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                import sys
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                # キー押下時の処理
                if event.key == pygame.K_r:
                    self.button_states["Y"] = True
                elif event.key == pygame.K_m:
                    self.button_states["S"] = True
                elif event.key == pygame.K_SPACE:
                    self.button_states["A"] = True
                elif event.key == pygame.K_ESCAPE:
                    import sys
                    sys.exit()
            elif event.type == pygame.KEYUP:
                # キー離した時の処理
                if event.key == pygame.K_r:
                    self.button_states["Y"] = False
                elif event.key == pygame.K_m:
                    self.button_states["S"] = False
                elif event.key == pygame.K_SPACE:
                    self.button_states["A"] = False

        # 現在押されているキーを取得
        keys = pygame.key.get_pressed()
        
        # ステアリング制御 (A/D キーまたは←/→矢印キー)
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            self.steering = 1.0  # 左
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            self.steering = -1.0  # 右
        else:
            self.steering = 0.0  # 中央
        
        # スロットル制御 (W/S キーまたは↑/↓矢印キー)
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            self.throttle = self.throttle_const_0  # 前進
            self.is_braking = False
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            self.throttle = self.throttle_const_1 * -1  # 後退
            self.is_braking = False
        elif keys[pygame.K_SPACE]:
            self.throttle = -1.0  # ブレーキ
            self.is_braking = True  # Spaceキーはブレーキ
        else:
            self.throttle = 0.0  # 停止
            self.is_braking = False
        
        # ボタンエッジ検出
        # Rキー: 記録開始/停止
        if self.button_states["Y"] and not self.previous_button_states["Y"]:
            self.recording = not self.recording
            print(f"Recording: {self.recording}")
        
        # Mキー: モード切替
        if self.button_states["S"] and not self.previous_button_states["S"]:
            self.mode = np.roll(self.mode, 1)
            print("Mode:", self.mode[0])

        # ブレーキ終了時の記録停止処理
        if self.previous_braking and not self.is_braking:
            # ブレーキから離された瞬間
            if self.recording:
                self.recording = False
                print("*** Recording stopped after brake ***")

        # ブレーキ状態を保存
        self.previous_braking = self.is_braking

# ROS2の有無を判定してインポート
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Bool, String
    from geometry_msgs.msg import Twist

    class JoystickROSNode(Node):
        def __init__(self, joystick, queue_size, timer_period):
            super().__init__('joystick_node')
            self.joystick = joystick

            # 統一インターフェース
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', queue_size)
            self.joy_mode_pub = self.create_publisher(String, '/joy/mode', queue_size)

            # 互換用: 個別トピック
            self.steering_pub = self.create_publisher(Float32, '/joystick/steering', queue_size)
            self.throttle_pub = self.create_publisher(Float32, '/joystick/throttle', queue_size)
            self.mode_pub = self.create_publisher(String, '/joystick/mode', queue_size)
            self.recording_pub = self.create_publisher(Bool, '/joystick/recording', queue_size)
            self.button_pubs = {
                name: self.create_publisher(Bool, f'/joystick/button/{name.lower()}', queue_size)
                for name in joystick.buttons.keys()
            }

            self.timer = self.create_timer(timer_period, self.publish_data)

        def publish_data(self):
            self.joystick.poll()
            mode = self.joystick.mode[0]

            # 統一トピック: userモード時のみ/cmd_velを発行
            if mode == "user":
                twist = Twist()
                twist.linear.x = float(self.joystick.throttle)
                twist.angular.z = float(self.joystick.steering)
                self.cmd_vel_pub.publish(twist)
            self.joy_mode_pub.publish(String(data=mode))

            # 互換用トピック
            self.steering_pub.publish(Float32(data=self.joystick.steering))
            self.throttle_pub.publish(Float32(data=self.joystick.throttle))
            self.mode_pub.publish(String(data=mode))
            self.recording_pub.publish(Bool(data=self.joystick.recording))

            for name, state in self.joystick.button_states.items():
                self.button_pubs[name].publish(Bool(data=state))
        
        def spin(self):
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                if rclpy.ok():
                    self.get_logger().info("ROS2 node stopped by user.")
            finally:
                if rclpy.ok():
                    self.destroy_node()
                    rclpy.shutdown()            

    def main_ros(queue_size=10, timer_period=0.016):
        rclpy.init()
        joystick = Joystick()
        if not joystick.HAVE_CONTROLLER:
            return
        joystick_node = JoystickROSNode(joystick, queue_size, timer_period)

        try:
            joystick_node.spin()
        except  KeyboardInterrupt:
            print("Shutting down Joystick node...")

except ImportError:
    # print("ROS2関連ライブラリがインストールされていません。ROS2モードは無効です。")
    rclpy = None

# raw data 確認用
def main_pygame():
    joystick = Joystick()
    if not joystick.HAVE_CONTROLLER:
        return
    while True:
        for e in pygame.event.get():
            print(e)        

# def main_pygame():
#     import time
#     joystick = Joystick()
#     if not joystick.HAVE_CONTROLLER:
#         return
    
#     print("ジョイスティックが正常に初期化されました。テストを開始します...")
#     time.sleep(0.5) 
    
#     try:
#         while True:
#             joystick.poll()
#             print(
#                 f"Steering: {joystick.steering}, Throttle: {joystick.throttle}, "
#                 f"Mode: {joystick.mode[0]}, Recording: {joystick.recording}, "
#                 f"Buttons: {joystick.button_states}"
#             )
#             time.sleep(0.1)  # 100ms間隔で表示
#     except KeyboardInterrupt:
#         print("\nプログラムを終了します...")
    
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Joystick with or without ROS2")
    parser.add_argument('--ros', action='store_true', help="Run with ROS2 node")
    parser.add_argument('--queue_size', type=int, default=10, help="Queue size for ROS2 publishers")
    parser.add_argument('--timer_period', type=float, default=0.1, help="Timer period for ROS2 publishers in seconds")
    args = parser.parse_args()

    if args.ros and rclpy is not None:
        main_ros(queue_size=args.queue_size, timer_period=args.timer_period)
    else:
        main_pygame()
