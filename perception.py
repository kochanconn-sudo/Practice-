# perception.py

import config_hanson


class Perception:

    def __init__(self):
        print("Perception 起動")


    def update(
        self,
        sensor_data=None,
        camera_data=None,
        lidar_data=None
    ):

        result = {

            # 距離
            "left_distance": None,
            "right_distance": None,
            "front_distance": None,

            # 壁
            "left_wall": False,
            "right_wall": False,
            "front_wall": False,

            # 状況
            "corner": None,
            "obstacle": False,
            "danger_level": 0,

            # 元データ
            "sensor": sensor_data,
            "camera": camera_data,
            "lidar": lidar_data
        }


        if sensor_data is None:
            return result


        ultrasonic = {}

        for i in range(5):

            key = f"ultrasonic/zone_{i}"

            if key in sensor_data:
                ultrasonic[i] = sensor_data[key]


        # ======================
        # センサー配置
        #
        # zone0 : 右
        # zone1 : 右前
        # zone2 : 前
        # zone3 : 左前
        # zone4 : 左
        # ======================


        right = ultrasonic.get(0)
        right_front = ultrasonic.get(1)
        front = ultrasonic.get(2)
        left_front = ultrasonic.get(3)
        left = ultrasonic.get(4)


        result["right_distance"] = right
        result["front_distance"] = front
        result["left_distance"] = left



        # ======================
        # 壁判定
        # ======================

        WALL_DISTANCE = 700
        FRONT_LIMIT = 500


        if left is not None:
            if left < WALL_DISTANCE:
                result["left_wall"] = True


        if right is not None:
            if right < WALL_DISTANCE:
                result["right_wall"] = True


        if front is not None:
            if front < FRONT_LIMIT:
                result["front_wall"] = True
                result["obstacle"] = True



        # ======================
        # 危険度
        # ======================

        if front is not None:

            if front < 300:
                result["danger_level"] = 3

            elif front < 600:
                result["danger_level"] = 2

            else:
                result["danger_level"] = 0



        return result