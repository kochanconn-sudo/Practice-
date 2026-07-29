# 独自ルールを作成してみよう

right_left_3モードを基に、独自の走行ルールを実装する手順を学びます。

## 基本的なright_left_3モードの理解

まず、既存のright_left_3モードの動作を確認しましょう。

### planner.py内のRight_Left_3_Plannerクラス

```python
def plan(self, ultrasonic_sensors):
    """3つのセンサー（左前・前・右前）で障害物回避"""
    # センサー値の取得
    left_distance = ultrasonic_sensors.get("FrLH", config.CUTOFF_RANGE)
    front_distance = ultrasonic_sensors.get("FrFR", config.CUTOFF_RANGE)
    right_distance = ultrasonic_sensors.get("FrRH", config.CUTOFF_RANGE)

    # 基本的な判断ロジック
    if front_distance < config.STOP_RANGE:
        # 前方に障害物 → 左右を比較
        if left_distance > right_distance:
            return config.LEFT, config.FORWARD_CORNER
        else:
            return config.RIGHT, config.FORWARD_CORNER
    else:
        # 前方クリア → 直進
        return config.NEUTRAL, config.FORWARD_STRAIGHT
```

---

## 独自ルールの実装手順

### 手順1: 新しいPlannerクラスを作成

```python
class My_Custom_Planner(Base_Planner):
    def __init__(self):
        super().__init__()
        self.name = "my_custom"

    def plan(self, ultrasonic_sensors):
        """独自の走行ルール"""
        # センサー値の取得
        left_distance = ultrasonic_sensors.get("FrLH", config.CUTOFF_RANGE)
        front_distance = ultrasonic_sensors.get("FrFR", config.CUTOFF_RANGE)
        right_distance = ultrasonic_sensors.get("FrRH", config.CUTOFF_RANGE)

        # 独自ルール例：より保守的な走行
        safety_margin = 50  # 安全マージン(mm)

        if front_distance < (config.STOP_RANGE + safety_margin):
            # より早めに回避動作
            if left_distance > right_distance + safety_margin:
                return config.LEFT, config.FORWARD_CORNER
            elif right_distance > left_distance + safety_margin:
                return config.RIGHT, config.FORWARD_CORNER
            else:
                # 左右両方とも近い場合は停止
                return config.NEUTRAL, config.STOP
        else:
            return config.NEUTRAL, config.FORWARD_STRAIGHT
```

### 手順2: config.pyに新しいモードを追加

```python
PLAN_LIST = [
    "go_straight",
    "right_left_3",
    "my_custom",     # 追加
    "wall_follow",
    # ...
]

PLAN = "my_custom"  # 新しいモードを選択
```

### 手順3: planner.pyのファクトリ関数に追加

```python
def create_planner(plan_name):
    if plan_name == "go_straight":
        return Go_Straight_Planner()
    elif plan_name == "right_left_3":
        return Right_Left_3_Planner()
    elif plan_name == "my_custom":     # 追加
        return My_Custom_Planner()
    # ...
```

---

## カスタマイズのアイデア

### 1. 速度制御の改良

```python
# 障害物の距離に応じて速度を調整
if min(left_distance, front_distance, right_distance) < 300:
    throttle = config.FORWARD_CORNER * 0.7  # 減速
else:
    throttle = config.FORWARD_STRAIGHT
```

### 2. 複数センサーの活用

```python
# 後方センサーも使用してより安全な走行
rear_left = ultrasonic_sensors.get("RrLH", config.CUTOFF_RANGE)
rear_right = ultrasonic_sensors.get("RrRH", config.CUTOFF_RANGE)

# 後方が狭い場合は前進を優先
if min(rear_left, rear_right) < 200:
    return config.NEUTRAL, config.FORWARD_CORNER
```

### 3. 状態記憶の追加

```python
class My_Custom_Planner(Base_Planner):
    def __init__(self):
        super().__init__()
        self.last_direction = config.NEUTRAL
        self.stuck_counter = 0

    def plan(self, ultrasonic_sensors):
        # 前回の方向を考慮した判断
        # 同じ場所で振動することを防ぐ
        ...
```

---

## テストと調整

1. `python run.py`で新しいルールをテスト
2. パラメータを調整して最適化
3. `python tools/graph.py`で走行データを分析
4. さらなる改良を実施

---

## 課題

### 初級: 安全マージンの追加

- 停止距離に安全マージンを追加
- 左右判断にもマージンを追加

### 中級: 速度の動的制御

- 障害物との距離に応じて速度を変更
- 滑らかな減速を実現

### 上級: スタック検出と回復

- 同じ場所で停滞していることを検出
- 自動的にバックして回復する機能を追加
