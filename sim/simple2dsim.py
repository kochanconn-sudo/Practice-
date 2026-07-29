#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import time
import matplotlib.patches as patches
import json
import sys
from scipy.interpolate import splprep, splev
from matplotlib.colors import ListedColormap


# --- キュービックスプライン補間関数 ---
def cubic_spline_smoothing(path, smoothing_factor=0, num_points=100):
    if len(path) < 4:
        return path
    x = [pt[0] for pt in path]
    y = [pt[1] for pt in path]
    tck, u = splprep([x, y], s=smoothing_factor)
    unew = np.linspace(0, 1, num_points)
    out = splev(unew, tck)
    smoothed_path = list(zip(out[0], out[1]))
    return smoothed_path

# --- シミュレーション環境の定義 ---
class GridMap:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=int)

    def add_obstacle_rect(self, x0, y0, w, h):
        self.grid[y0:y0+h, x0:x0+w] = 1

    def in_bounds(self, pos):
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, pos):
        x, y = pos
        return self.grid[y, x] == 0

    def neighbors(self, pos):
        (x, y) = pos
        results = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        results = filter(self.in_bounds, results)
        results = filter(self.is_free, results)
        return list(results)

    def inflate_obstacles(self, inflation_radius=1):
        """
        元の障害物は 1、インフレーションで追加されたセルは 2 としてマークする
        """
        # inflated に現在のグリッドのコピーを作成（既存障害物は 1 のまま）
        inflated = self.grid.copy()
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 1:
                    # 周囲のセルをチェック
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            if self.in_bounds((nx, ny)) and inflated[ny, nx] == 0:
                                inflated[ny, nx] = 2  # インフレーションセルを 2 とする
        self.grid = inflated

# --- A*プランナー ---
class AStarPlanner:
    def __init__(self, gridmap):
        self.map = gridmap

    def heuristic(self, a, b):
        (x1, y1) = a; (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def clearance(self, pos):
        x, y = pos
        best = float('inf')
        for iy in range(self.map.height):
            for ix in range(self.map.width):
                if self.map.grid[iy, ix] == 1:
                    d = math.hypot(ix - x, iy - y)
                    if d < best:
                        best = d
        return best if best != float('inf') else 9999

    def plan(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        safe_distance = 2.0
        penalty_factor = 10
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for nxt in self.map.neighbors(current):
                base_cost = 1
                clear = self.clearance(nxt)
                extra_cost = (safe_distance - clear) * penalty_factor if clear < safe_distance else 0
                new_cost = cost_so_far[current] + base_cost + extra_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self.heuristic(goal, nxt)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
        if goal not in came_from:
            return []
        current = goal
        path = []
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

class HybridAStarPlanner:
    def __init__(self, gridmap, steering_angles=[-0.3, 0, 0.3], dt=0.2, step_size=1.0):
        self.map = gridmap
        self.steering_angles = steering_angles  # 使用するステアリング角の離散集合（ラジアン）
        self.dt = dt
        self.step_size = step_size

    def heuristic(self, state, goal):
        # state: (x, y, theta), goal: (x, y)
        (x, y, _) = state
        (gx, gy) = goal
        return math.hypot(gx - x, gy - y)

    def plan(self, start, goal):
        """
        start: (x, y, theta)
        goal: (x, y)
        出力は (x, y) 座標のリストとして返す
        """
        # 開始状態に heading を含める
        start_state = (start[0], start[1], start[2])  # 例として
        frontier = []
        heapq.heappush(frontier, (0, start_state))
        came_from = {}
        cost_so_far = {}
        came_from[start_state] = None
        cost_so_far[start_state] = 0

        while frontier:
            _, current = heapq.heappop(frontier)
            (cx, cy, ctheta) = current
            if math.hypot(goal[0] - cx, goal[1] - cy) < 1.0:  # goal 到達の閾値
                break
            # 遷移の生成：各 steering 入力に対して状態遷移をシミュレーション
            for steer in self.steering_angles:
                # 車両の運動モデルに基づく次状態
                new_theta = ctheta + (self.step_size / 1.0) * math.tan(steer)
                new_x = cx + self.step_size * math.cos(new_theta)
                new_y = cy + self.step_size * math.sin(new_theta)
                next_state = (new_x, new_y, new_theta)
                # ここで障害物判定やコスト計算を行う（簡易化のためここでは距離コストのみ）
                new_cost = cost_so_far[current] + self.step_size + abs(steer)  # 旋回コストを加える
                if not self._is_state_valid(next_state):
                    continue
                if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                    cost_so_far[next_state] = new_cost
                    priority = new_cost + self.heuristic(next_state, goal)
                    heapq.heappush(frontier, (priority, next_state))
                    came_from[next_state] = current

        # 経路復元
        # 最終状態の候補を探索（goal に近い状態）
        end_state = min(cost_so_far.keys(), key=lambda s: math.hypot(goal[0]-s[0], goal[1]-s[1]))
        path_states = []
        current = end_state
        while current is not None:
            path_states.append(current)
            current = came_from.get(current)
        path_states.reverse()
        # (x, y) 座標のみ抽出して返す
        path = [(state[0], state[1]) for state in path_states]
        return path

    def _is_state_valid(self, state):
        (x, y, _) = state
        ix, iy = int(round(x)), int(round(y))
        return self.map.in_bounds((ix, iy)) and self.map.is_free((ix, iy))

# --- 車両モデル ---
class Car:
    def __init__(self, x, y, heading=0.0, sensor_offset=0.0):
        self.x = x          # 位置 (m)
        self.y = y
        self.heading = heading  # ラジアン
        self.speed = 0.0        # 速度 (m/s)
        self.sensor_offset = sensor_offset  # センサー位置（車両前方からのオフセット）
        self.wheelbase = 1.0    # ホイールベース (m)
        self.length = 1.0       # 車両の全長 (m)　※描画用
        self.width = 0.5        # 車両の幅 (m)　※描画用

    def pose(self):
        return (self.x, self.y)

    def update(self, steering, throttle, dt=0.1):
        """
        バイシクルモデルによる車両の更新
        steering: 前輪のステアリング角 (ラジアン)
        throttle: 速度（正は前進、負は後退）
        dt: タイムステップ
        """
        L = self.wheelbase
        # アッカーマン（バイシクル）モデルの更新式
        # heading の更新： v / L * tan(δ)
        self.heading += (throttle / L) * math.tan(steering) * dt
        self.x += throttle * math.cos(self.heading) * dt
        self.y += throttle * math.sin(self.heading) * dt

    def get_grid_pos(self):
        return (int(round(self.x)), int(round(self.y)))

    def get_polygon(self):
        """
        車両を三角形（ポリゴン）として返す
        車両のローカル座標系で、先端: (length/2, 0), 左後: (-length/2, width/2), 右後: (-length/2, -width/2)
        を定義し、車両の heading に従って回転・平行移動した結果を返す。
        """
        L = self.length
        W = self.width
        # ローカル座標の頂点
        pts_local = np.array([
            [L/2, 0],
            [-L/2, W/2],
            [-L/2, -W/2]
        ])
        # 回転行列
        cos_theta = math.cos(self.heading)
        sin_theta = math.sin(self.heading)
        R = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        pts_world = []
        for pt in pts_local:
            rotated = R.dot(pt)
            pts_world.append((self.x + rotated[0], self.y + rotated[1]))
        return pts_world

    def ultrasonic_sensor_multi_wedge(self, gridmap, max_range=10, 
                                      sensor_configs=[{"offset": math.radians(30), "fov": math.radians(30)},
                                                      {"offset": 0.0, "fov": math.radians(30)},
                                                      {"offset": -math.radians(30), "fov": math.radians(30)}],
                                      num_samples=5):
        readings = {}  # {offset: (min_distance, fov)}
        for config in sensor_configs:
            offset = config["offset"]
            fov = config["fov"]
            sensor_angle = self.heading + offset
            # センサー起点（車両先端位置）
            origin_x = self.x + self.sensor_offset * math.cos(sensor_angle)
            origin_y = self.y + self.sensor_offset * math.sin(sensor_angle)
            half_fov = fov / 2.0
            if num_samples <= 1:
                angles = [sensor_angle]
            else:
                angles = np.linspace(sensor_angle - half_fov, sensor_angle + half_fov, num_samples)
            min_dist = max_range
            for ang in angles:
                dist = 0.0
                step = 0.2
                while dist < max_range:
                    test_x = origin_x + dist * math.cos(ang)
                    test_y = origin_y + dist * math.sin(ang)
                    ix, iy = int(round(test_x)), int(round(test_y))
                    if not gridmap.in_bounds((ix, iy)):
                        break
                    # もしそのセルが「元の障害物」（値 1）の場合だけ、距離計測を止める
                    if gridmap.grid[iy, ix] == 1:
                        break
                    dist += step
                if dist < min_dist:
                    min_dist = dist
            readings[offset] = (min_dist, fov)
        return readings

class LocalPlanner:
    def __init__(self, lookahead_distance=1.0):
        self.lookahead_distance = lookahead_distance

    def get_target_point(self, smooth_path, current_pos, current_heading):
        """
        平滑化された経路 smooth_path から、車両の現在位置 current_pos と heading current_heading
        に基づいて、前方にある候補点の中からルックアヘッド距離に近い目標点を返す。
        """
        # smooth_path が空の場合は、現在の位置を返す（または適当なデフォルト値を返す）
        if not smooth_path:
            return current_pos
        
        front_candidates = []
        for pt in smooth_path:
            # 各点をセル中心に補正
            target_x = pt[0] + 0.5
            target_y = pt[1] + 0.5
            dx = target_x - current_pos[0]
            dy = target_y - current_pos[1]
            # 前方向の単位ベクトル
            forward_vector = (math.cos(current_heading), math.sin(current_heading))
            # 内積を計算
            dot = dx * forward_vector[0] + dy * forward_vector[1]
            # 内積が正なら前方にあると判断
            if dot > 0:
                dist = math.hypot(dx, dy)
                front_candidates.append((pt, dist))
        
        if not front_candidates:
            # 前方に候補がなければ、平滑経路の最後の点を返す
            return smooth_path[-1]
        
        # ルックアヘッド距離に近い候補を選ぶ
        target_point = min(front_candidates, key=lambda item: abs(item[1] - self.lookahead_distance))[0]
        return target_point

    def compute_control(self, current_pos, current_heading, target_point):
        """
        Pure Pursuit 制御に基づく steering コマンドを計算する
        """
        target_x = target_point[0] + 0.5
        target_y = target_point[1] + 0.5
        dx = target_x - current_pos[0]
        dy = target_y - current_pos[1]
        angle_to_target = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(angle_to_target - current_heading), math.cos(angle_to_target - current_heading))
        steering = 2.0 * angle_error  # ゲインは適宜調整
        return steering

# --- Behavior Tree（シンプル実装） ---
class BehaviorTree:
    def __init__(self, car, gridmap, planner, waypoints):
        self.car = car
        self.gridmap = gridmap
        self.planner = planner
        # 複数のウェイポイントをリストとして保持
        self.waypoints = waypoints  
        self.waypoint_index = 0      # 現在目指すウェイポイントのインデックス
        self.lap_count = 0           # 周回数
        self.path = []           # 計画された経路（整数セルのリスト）
        self.smooth_path = []    # 平滑化された経路（連続値）
        self.path_index = 0
        self.ultrasonic_threshold = 1.5
        self.recovery_steps = 0
        self.max_recovery_steps = 10
        self.reached_goal = False

        # ローカルプランナーのインスタンス（ルックアヘッド距離は 1.0 など）
        self.local_planner = LocalPlanner(lookahead_distance=1.0)

    def tick(self):
        if self.reached_goal:
            return 0.0, 0.0

        # センサー取得および障害物リカバリー処理
        sensor_readings = self.car.ultrasonic_sensor_multi_wedge(self.gridmap, max_range=10)
        central = sensor_readings.get(0.0, (10, math.radians(30)))[0]
        if central < self.ultrasonic_threshold:
            print(f"Recovery: obstacle too close (central dist={central:.2f})")
            if self.recovery_steps < self.max_recovery_steps:
                steering = 1.0  # 逆走時の steering（例：1.0 rad）
                throttle = -0.3  # 逆走速度
                self.recovery_steps += 1
                self.path = []
                self.smooth_path = []
                return steering, throttle
            else:
                self.recovery_steps = 0

        # 現在のグローバル目標ウェイポイントを選択
        current_goal = self.waypoints[self.waypoint_index]

        # グローバルウェイポイントへの到達判定（グローバルな位置としてセル中心）
        goal_center_x = current_goal[0] + 0.5
        goal_center_y = current_goal[1] + 0.5
        dx_goal = goal_center_x - self.car.x
        dy_goal = goal_center_y - self.car.y
        goal_distance = math.hypot(dx_goal, dy_goal)
        if goal_distance < 1.0:
            print(f"Reached global waypoint {current_goal}")
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.waypoint_index = 0
                self.lap_count += 1
                print(f"Lap {self.lap_count} completed!")
            self.path = []
            self.smooth_path = []
            self.path_index = 0
            return 0.0, 0.0

        # 経路計画が未実行または完了している場合に再計画
        if not self.path or self.path_index >= len(self.smooth_path):
            start = self.car.get_grid_pos()
            goal = (int(round(current_goal[0])), int(round(current_goal[1])))
            self.path = self.planner.plan(start, goal)
            self.path_index = 0
            if not self.path:
                print("No path found!")
                return 0.0, 0.0
            raw_path = [(float(x), float(y)) for (x, y) in self.path]
            self.smooth_path = cubic_spline_smoothing(raw_path, smoothing_factor=1, num_points=100)
            print("New smooth path planned:", self.smooth_path)

        # LocalPlanner でルックアヘッド目標点を取得（車両の現在位置と heading を考慮）
        target_point = self.local_planner.get_target_point(self.smooth_path, (self.car.x, self.car.y), self.car.heading)

        # 【追加】次の target_point との距離チェック
        target_center_x = target_point[0] + 0.5
        target_center_y = target_point[1] + 0.5
        dx_target = target_center_x - self.car.x
        dy_target = target_center_y - self.car.y
        target_distance = math.hypot(dx_target, dy_target)
        if target_distance > 2.0:
            print(f"Large deviation from next target point: {target_distance:.2f} cells. Replanning from a point 1 cell ahead.")
            # 車両の現在の heading に沿って、1セル先の位置を新たなスタート地点とする
            new_start_x = self.car.x + math.cos(self.car.heading)
            new_start_y = self.car.y + math.sin(self.car.heading)
            new_start = (int(round(new_start_x)), int(round(new_start_y)))
            # 再計画実施
            goal = (int(round(current_goal[0])), int(round(current_goal[1])))
            self.path = self.planner.plan(new_start, goal)
            self.path_index = 0
            if not self.path:
                print("No path found on replanning!")
                return 0.0, 0.0
            raw_path = [(float(x), float(y)) for (x, y) in self.path]
            self.smooth_path = cubic_spline_smoothing(raw_path, smoothing_factor=1, num_points=100)
            print("Replanned smooth path:", self.smooth_path)
            # 更新後、再度 target_point を取得
            target_point = self.local_planner.get_target_point(self.smooth_path, (self.car.x, self.car.y), self.car.heading)

        # 【オプション】前方判定や向き修正の処理もここで必要なら追加（省略可）

        # 通常の Pure Pursuit 追従制御
        steering = self.local_planner.compute_control((self.car.x, self.car.y), self.car.heading, target_point)
        throttle = 0.5

        # 追従判定：target_point との距離が十分近い場合
        error_x = target_point[0] + 0.5 - self.car.x
        error_y = target_point[1] + 0.5 - self.car.y
        distance = math.hypot(error_x, error_y)
        if distance < 0.5:
            print(f"Reached target point {target_point}, next index {self.path_index+1}")
            self.path_index += 1
            if self.path_index >= len(self.smooth_path):
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self.waypoint_index = 0
                    self.lap_count += 1
                    print(f"Lap {self.lap_count} completed!")
                self.path = []
                self.smooth_path = []
                self.path_index = 0

        return steering, throttle
        
# --- コース作成ツール ---
def create_course():
    """
    マウス操作で障害物矩形、ウェイポイント、及び車両のスタート位置と向きを作成するツール。
    - 左クリックドラッグ: 障害物（始点と終点）を入力
    - 右クリック:
         * 初回: スタート位置を設定（単にクリック）  
         * 以降: ウェイポイントを追加
    - Shift+右クリックドラッグ: すでに設定されたスタート位置の向きを指定する
    - 's' キー: コースを保存
    - 'q' キー: 終了（保存せず）
    """
    import matplotlib.patches as patches
    import json

    course = {"obstacles": [], "waypoints": [], "start": None}  # "start": {"pos": (x, y), "heading": θ}
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_title("Course Creation Tool\nLeft-drag: Draw obstacle; Right-click: Set start / Add waypoint; Shift+Right-drag: Set start orientation\n's': save, 'q': quit")
    
    grid_width, grid_height = 20, 20
    wall_thickness = 1

    # 初期の境界壁
    obstacle_patches = []
    # 上壁
    upper_wall = patches.Rectangle((0, grid_height - wall_thickness), grid_width, wall_thickness, fill=True, color='gray', alpha=0.5)
    obstacle_patches.append(upper_wall)
    course["obstacles"].append({"x": 0, "y": grid_height - wall_thickness, "w": grid_width, "h": wall_thickness})
    # 下壁
    lower_wall = patches.Rectangle((0, 0), grid_width, wall_thickness, fill=True, color='gray', alpha=0.5)
    obstacle_patches.append(lower_wall)
    course["obstacles"].append({"x": 0, "y": 0, "w": grid_width, "h": wall_thickness})
    # 左壁
    left_wall = patches.Rectangle((0, 0), wall_thickness, grid_height, fill=True, color='gray', alpha=0.5)
    obstacle_patches.append(left_wall)
    course["obstacles"].append({"x": 0, "y": 0, "w": wall_thickness, "h": grid_height})
    # 右壁
    right_wall = patches.Rectangle((grid_width - wall_thickness, 0), wall_thickness, grid_height, fill=True, color='gray', alpha=0.5)
    obstacle_patches.append(right_wall)
    course["obstacles"].append({"x": grid_width - wall_thickness, "y": 0, "w": wall_thickness, "h": grid_height})

    waypoints = []
    additional_obstacle_patches = []
    start_position = None  # スタート位置 (x, y)
    start_heading = None   # スタート向き（ラジアン）
    # 右クリックでスタート向き設定中かどうかのフラグ
    setting_start_orientation = False
    # スタート向き指定用の一時線アーティスト
    start_orientation_line = None

    current_start_point = None
    current_rect = None

    def redraw():
        ax.clear()
        ax.set_title("Course Creation Tool\nLeft-drag: Draw obstacle; Right-click: Set start / Add waypoint; Shift+Right-drag: Set start orientation\n's': save, 'q': quit")
        ax.set_xlim(0, grid_width)
        ax.set_ylim(0, grid_height)
        for patch in obstacle_patches:
            ax.add_patch(patch)
        for patch in additional_obstacle_patches:
            ax.add_patch(patch)
        for wp in waypoints:
            ax.plot(wp[0], wp[1], 'gx', markersize=10)
        if start_position is not None:
            ax.plot(start_position[0], start_position[1], 'ro', markersize=10)
            ax.text(start_position[0], start_position[1], "Start", color='r', fontsize=12, fontweight='bold')
        # スタート向きの表示：スタート位置から矢印で向きを表示
        if start_position is not None and start_heading is not None:
            arrow_length = 2.0
            end_x = start_position[0] + arrow_length * math.cos(start_heading)
            end_y = start_position[1] + arrow_length * math.sin(start_heading)
            ax.arrow(start_position[0], start_position[1],
                     end_x - start_position[0], end_y - start_position[1],
                     head_width=0.3, head_length=0.5, fc='r', ec='r')
        fig.canvas.draw_idle()

    def on_press(event):
        nonlocal current_start_point, current_rect, start_position, setting_start_orientation, start_heading, start_orientation_line
        if event.inaxes != ax:
            return
        if event.button == 1:  # 左クリック: 障害物描画開始
            current_start_point = (event.xdata, event.ydata)
            current_rect = patches.Rectangle(current_start_point, 0, 0, fill=True, color='gray', alpha=0.5)
            ax.add_patch(current_rect)
            fig.canvas.draw_idle()
        elif event.button == 3:
            if event.key == 'shift' or (event.guiEvent and event.guiEvent.modifiers() & 0x0004):  
                # ここでは、Shift キー押下時（環境により異なります）または、GUI イベントで Shift が検出できればスタート向き設定モード
                if start_position is not None:
                    setting_start_orientation = True
                    # スタート向き設定のため、始点はスタート位置
                    start_orientation_line = ax.plot([start_position[0], event.xdata], [start_position[1], event.ydata], 'r--')[0]
                    fig.canvas.draw_idle()
            else:
                # 通常の右クリック：スタート位置未設定なら設定、既にあればウェイポイント追加
                if start_position is None:
                    start_position = (event.xdata, event.ydata)
                    course["start"] = {"pos": start_position, "heading": None}
                else:
                    waypoints.append((event.xdata, event.ydata))
                redraw()

    def on_motion(event):
        nonlocal current_rect, start_orientation_line
        if current_start_point is not None and current_rect is not None and event.inaxes == ax:
            width = event.xdata - current_start_point[0]
            height = event.ydata - current_start_point[1]
            current_rect.set_width(width)
            current_rect.set_height(height)
            fig.canvas.draw_idle()
        if setting_start_orientation and start_position is not None and event.inaxes == ax:
            # 更新スタート向き表示線
            if start_orientation_line is not None:
                start_orientation_line.remove()
            start_orientation_line = ax.plot([start_position[0], event.xdata], [start_position[1], event.ydata], 'r--')[0]
            print(event.xdata,event.ydata)
            fig.canvas.draw_idle()

    def on_release(event):
        nonlocal current_start_point, current_rect, setting_start_orientation, start_heading, start_orientation_line
        if event.xdata is None or event.ydata is None:
            current_start_point = None
            current_rect = None
            return
        if event.button == 1 and current_start_point is not None:
            end_point = (event.xdata, event.ydata)
            rect = patches.Rectangle((min(current_start_point[0], end_point[0]), min(current_start_point[1], end_point[1])),
                                     abs(end_point[0]-current_start_point[0]), abs(end_point[1]-current_start_point[1]),
                                     fill=True, color='gray', alpha=0.5)
            additional_obstacle_patches.append(rect)
            course["obstacles"].append({
                "x": min(current_start_point[0], end_point[0]),
                "y": min(current_start_point[1], end_point[1]),
                "w": abs(end_point[0]-current_start_point[0]),
                "h": abs(end_point[1]-current_start_point[1])
            })
            current_start_point = None
            current_rect = None
            redraw()
        elif event.button == 3 and setting_start_orientation:
            # Shift+右クリックドラッグ終了: スタート向きの決定
            dx = event.xdata - start_position[0]
            dy = event.ydata - start_position[1]
            start_heading = math.atan2(dy, dx)
            course["start"]["heading"] = start_heading
            setting_start_orientation = False
            if start_orientation_line is not None:
                start_orientation_line.remove()
                start_orientation_line = None
            redraw()

    def on_key(event):
        if event.key == 's':
            course["waypoints"] = waypoints
            with open("course.json", "w") as f:
                json.dump(course, f, indent=2)
            print("Course saved to course.json")
            plt.close()
        elif event.key == 'q':
            plt.close()

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('key_press_event', on_key)
    redraw()
    plt.show()


# --- コース読み込み関数 ---
def load_course(filename):
    with open(filename, "r") as f:
        course = json.load(f)
    return course

# --- シミュレーションメイン ---
def run_simulation(course):
    grid_width = 20
    grid_height = 20
    gridmap = GridMap(grid_width, grid_height)
    # course["obstacles"] は各障害物の辞書リスト（x, y, w, h）
    for obs in course.get("obstacles", []):
        # 座標を整数に丸めるなど、必要に応じて変換
        gridmap.add_obstacle_rect(int(obs["x"]), int(obs["y"]), int(obs["w"]), int(obs["h"]))
    gridmap.inflate_obstacles(inflation_radius=1)

    # 複数のウェイポイントを取得（各ウェイポイントは [x, y] として保存されていると仮定）
    if course.get("waypoints"):
        waypoints = [(float(pt[0]), float(pt[1])) for pt in course["waypoints"]]
    else:
        waypoints = [(18, 2)]

        # スタート位置と heading の設定
    if course.get("start") is not None and "pos" in course["start"]:
        start_pos = course["start"]["pos"]
        # heading が存在していれば使い、なければ 0.0 とする
        start_heading = course["start"]["heading"] if course["start"].get("heading") is not None else math.radians(90)
    else:
        # デフォルトのスタート位置、heading
        start_pos = (2.0, 2.0)
        start_heading = 0.0
    
    # 車両を初期化（スタート位置と heading を反映）
    car = Car(start_pos[0], start_pos[1], heading=start_heading, sensor_offset=0.0)
    
    planner = AStarPlanner(gridmap)
    bt = BehaviorTree(car, gridmap, planner, waypoints)

    # LocalPlanner のインスタンス（ルックアヘッド距離を1.0とする例）
    local_planner = LocalPlanner(lookahead_distance=1.0)

    # ListedColormap: 0 → white, 1 → black, 2 → light red
    cmap = ListedColormap(["white", "black", "#ffcccc"])

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))
    sensor_configs = [
        {"offset": math.radians(30), "fov": math.radians(30)},
        {"offset": 0.0, "fov": math.radians(30)},
        {"offset": -math.radians(30), "fov": math.radians(30)}
    ]
    for step in range(10000):
        steering, throttle = bt.tick()
        car.update(steering, throttle, dt=0.2)
        sensor_readings = car.ultrasonic_sensor_multi_wedge(gridmap, max_range=10, 
                                                            sensor_configs=sensor_configs, num_samples=5)
        colors = {
            math.radians(30): 'g',
            0.0: 'b',
            -math.radians(30): 'r'
        }
        ax.clear()
        #ax.imshow(gridmap.grid, cmap=cmap, origin='lower', extent=[0, gridmap.width, 0, gridmap.height])
        # 画面表示のため、0.5調整
        ax.imshow(gridmap.grid, cmap=cmap, origin='lower', extent=[-0.5, gridmap.width-0.5, -0.5, gridmap.height-0.5])
        if bt.path:
            path_x = [cell[0] + 0.5 for cell in bt.path]
            path_y = [cell[1] + 0.5 for cell in bt.path]
            ax.plot(path_x, path_y, 'b.-', label='Planned Path')
        if bt.smooth_path:
            smooth_x = [pt[0] + 0.5 for pt in bt.smooth_path]
            smooth_y = [pt[1] + 0.5 for pt in bt.smooth_path]
            ax.plot(smooth_x, smooth_y, 'g.-', label='Smooth Path')


        # 車両描画：三角形
        car_poly = car.get_polygon()
        car_patch = patches.Polygon(car_poly, closed=True, color='r')
        ax.add_patch(car_patch)
        #ax.plot(waypoints[0][0] + 0.5, waypoints[0][1] + 0.5, 'gx', markersize=10, label='Waypoint')
        # 複数ウェイポイントをすべて表示
        for wp in waypoints:
            ax.plot(wp[0] + 0.5, wp[1] + 0.5, 'gx', markersize=10)

        # 各センサーの描画：扇形で表示
        for config in sensor_configs:
            offset = config["offset"]
            fov = config["fov"]
            reading, _ = sensor_readings.get(offset, (10, fov))
            sensor_angle = car.heading + offset
            origin_x = car.x + car.sensor_offset * math.cos(sensor_angle)
            origin_y = car.y + car.sensor_offset * math.sin(sensor_angle)
            half_fov_deg = math.degrees(fov/2)
            theta1 = math.degrees(sensor_angle) - half_fov_deg
            theta2 = theta1 + math.degrees(fov)
            color = colors.get(offset, 'k')
            sensor_wedge = patches.Wedge((origin_x, origin_y), reading, theta1, theta2,
                                           facecolor=color, alpha=0.3, edgecolor=color)
            ax.add_patch(sensor_wedge)
            line_end_x = origin_x + reading * math.cos(sensor_angle)
            line_end_y = origin_y + reading * math.sin(sensor_angle)
            ax.plot([origin_x, line_end_x], [origin_y, line_end_y],
                    color=color, linewidth=2, label=f"Sensor {math.degrees(offset):.0f}°: {reading:.1f}")
            ax.text(origin_x, origin_y, f"{reading:.1f}", color=color, fontsize=12, fontweight='bold')
        
        # --- Pure Pursuit 可視化 ---
        # 現在の平滑化経路から、LocalPlanner を使ってルックアヘッド目標点を取得
        target_point = local_planner.get_target_point(bt.smooth_path, (car.x, car.y), car.heading)
        # 車両の現在位置を中心としたルックアヘッド円を描画
        lookahead_circle = patches.Circle((car.x, car.y), local_planner.lookahead_distance, fill=False, color='m', linestyle='--', linewidth=1.5)
        ax.add_patch(lookahead_circle)
        # ルックアヘッド目標点を描画
        ax.plot(target_point[0] + 0.5, target_point[1] + 0.5, 'mo', markersize=8, label='LocalTarget')        
        
        # グローバルウェイポイント（現在の目標）の距離を計算（セル中心として補正）
        current_wp = waypoints[bt.waypoint_index]
        dx_wp = (current_wp[0] + 0.5) - car.x
        dy_wp = (current_wp[1] + 0.5) - car.y
        wp_distance = math.hypot(dx_wp, dy_wp)
        # 距離を表示（例として、画面上部中央付近に表示）
        ax.text(grid_width/2 - 2, grid_height - 1, f"Distance to WP: {wp_distance:.2f} m", 
                color='k', fontsize=12, fontweight='bold')

        ax.set_title(f"Step {step}")
        ax.set_xlim(0, gridmap.width)
        ax.set_ylim(0, gridmap.height)
        ax.legend(loc='upper right')
        plt.pause(0.001)
        if bt.reached_goal:
            break
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    # コマンドライン引数によりモードを選択
    # 例: "python script.py create" でコース作成モード、"python script.py simulate" でシミュレーションモード
    if len(sys.argv) >= 2 and sys.argv[1] == "create":
        create_course()
    elif len(sys.argv) >= 2 and sys.argv[1] == "simulate":
        # シミュレーションでは course.json を読み込む前提
        course = load_course("course.json")
        run_simulation(course)
    else:
        print("Usage: python script.py [create|simulate]")
