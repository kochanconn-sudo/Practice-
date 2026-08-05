# ============================================================
# planner.py
#
# Next Generation Planner
# Toyota Engineering Society AI Car
#
# Version : 2.0
# ============================================================

import time
import logging
from enum import Enum
from typing import Dict, Any, Optional

import numpy as np

logger = logging.getLogger(__name__)


# ============================================================
# Driving Mode
# ============================================================

class DriveMode(Enum):
    USER = "user"
    AUTO = "auto"


# ============================================================
# Planning Mode
# ============================================================

class PlanMode(Enum):

    WALL_FOLLOW = "wall_follow"

    WALL_FOLLOW_PID = "wall_follow_pid"

    CENTER_FOLLOW = "center_follow"

    RACER = "racer"

    GAP_FOLLOW = "gap_follow"

    FOLLOW_THE_GAP = "follow_the_gap"

    NN = "nn"

    CAMERA = "camera"

    YOLO = "yolo"

    CUSTOM = "custom"


# ============================================================
# Planner
# ============================================================

class Planner:

    """
    次世代Planner

    全ての走行方法をここで管理する司令塔

    役割

        update()

            ↓

        perception()

            ↓

        planning_sequence()

            ↓

        execute_plan()

            ↓

        steering
        throttle
    """

    # --------------------------------------------------------

    def __init__(self):

        logger.info("Planner Start")

        ###############################################
        # 出力
        ###############################################

        self.steering = 0.0
        self.throttle = 0.0

        ###############################################
        # 状態
        ###############################################

        self.mode = DriveMode.AUTO

        self.plan = PlanMode.RACER

        ###############################################
        # Sensor
        ###############################################

        self.ranges = {}

        self.camera = None

        self.lidar = None

        self.yolo = None

        ###############################################
        # Recovery
        ###############################################

        self.in_recovery = False

        ###############################################
        # Timer
        ###############################################

        self.last_time = time.perf_counter()

        ###############################################
        # Debug
        ###############################################

        self.debug = {}

        logger.info("Planner Ready")


    # ========================================================
    # Main Update
    # ========================================================

    def update(

        self,

        ranges,

        camera=None,

        lidar=None,

        yolo=None,

    ):

        """
        Planner入口

        run.pyから毎フレーム呼ばれる
        """

        self.ranges = ranges

        self.camera = camera

        self.lidar = lidar

        self.yolo = yolo

        return self.planning_sequence()


    # ========================================================
    # Planning Sequence
    # ========================================================

    def planning_sequence(self):

        """
        判断順番

        Recovery

            ↓

        Sensor

            ↓

        Perception

            ↓

        Planning

            ↓

        Safety

            ↓

        Output
        """

        ###############################################

        if self.in_recovery:

            return self.recovery()

        ###############################################

        self.perception()

        ###############################################

        self.execute_plan()

        ###############################################

        self.safety()

        ###############################################

        return self.steering, self.throttle


    # ========================================================
    # Perception
    # ========================================================

    def perception(self):

        """
        ここでは

        ・超音波

        ・カメラ

        ・LiDAR

        ・YOLO

        を統合する

        Part2で実装
        """

        pass


    # ========================================================
    # Execute Plan
    # ========================================================

    def execute_plan(self):

        """
        PlanModeによって

        使用するアルゴリズムを切り替える
        """

        if self.plan == PlanMode.WALL_FOLLOW:

            self.wall_follow()

        elif self.plan == PlanMode.WALL_FOLLOW_PID:

            self.wall_follow_pid()

        elif self.plan == PlanMode.CENTER_FOLLOW:

            self.center_follow()

        elif self.plan == PlanMode.RACER:

            self.racer()

        elif self.plan == PlanMode.GAP_FOLLOW:

            self.gap_follow()

        elif self.plan == PlanMode.FOLLOW_THE_GAP:

            self.follow_the_gap()

        elif self.plan == PlanMode.NN:

            self.nn()

        elif self.plan == PlanMode.CAMERA:

            self.camera_drive()

        elif self.plan == PlanMode.YOLO:

            self.yolo_drive()

        elif self.plan == PlanMode.CUSTOM:

            self.custom()

        else:

            self.steering = 0.0

            self.throttle = 0.0   
    # ========================================================
    # Safety
    # ========================================================

    def safety(self):

        """
        最終安全チェック

        どのアルゴリズムを使用していても
        最後に必ずここを通る。

        ・衝突防止
        ・速度制限
        ・ステアリング制限
        """

        # ----------------------------
        # Front Collision
        # ----------------------------

        front = self.ranges.get("FrFR", 9999)

        if front < 120:

            self.steering = 0.0
            self.throttle = 0.0

            logger.warning("Emergency Stop")

            return

        # ----------------------------
        # Steering Clamp
        # ----------------------------

        self.steering = max(
            -1.0,
            min(
                1.0,
                self.steering,
            ),
        )

        # ----------------------------
        # Throttle Clamp
        # ----------------------------

        self.throttle = max(
            -1.0,
            min(
                1.0,
                self.throttle,
            ),
        )


    # ========================================================
    # Recovery
    # ========================================================

    def recovery(self):

        """
        スタック時の復帰

        Part3で強化予定
        """

        logger.info("Recovery")

        self.steering = 0.0
        self.throttle = -0.4

        return (
            self.steering,
            self.throttle,
        )


    # ========================================================
    # Wall Follow
    # ========================================================

    def wall_follow(self):

        left = self.ranges.get("FrLH",300)
        front = self.ranges.get("FrFR",300)
        right = self.ranges.get("FrRH",300)

        target = 300

        error = target-right

        gain = 0.004

        steering = gain*error

        steering=max(-1,min(1,steering))

        if front<250:

            throttle=0.30

        else:

            throttle=0.60

        self.steering=steering
        self.throttle=throttle


    # ========================================================
    # Wall Follow PID
    # ========================================================

    def wall_follow_pid(self):

        left = self.ranges.get("FrLH",300)
        right = self.ranges.get("FrRH",300)
        front = self.ranges.get("FrFR",300)

        target = 300

        now=time.perf_counter()

        dt=now-self.last_time

        self.last_time=now

        if dt<=0:

            dt=0.01

        if not hasattr(self,"pid_i"):

            self.pid_i=0

        if not hasattr(self,"pid_last"):

            self.pid_last=0

        error=target-right

        self.pid_i+=error*dt

        d=(error-self.pid_last)/dt

        self.pid_last=error

        kp=0.003
        ki=0.0001
        kd=0.001

        steering=kp*error+ki*self.pid_i+kd*d

        steering=max(-1,min(1,steering))

        if front<250:

            throttle=0.35

        else:

            throttle=0.65

        self.steering=steering
        self.throttle=throttle
            # ========================================================
    # Center Follow
    # ========================================================

    def center_follow(self):

        left = self.ranges.get("FrLH", 300)
        right = self.ranges.get("FrRH", 300)
        front = self.ranges.get("FrFR", 300)

        corridor_center = (left - right)

        gain = 0.0035

        steering = -corridor_center * gain

        steering = max(-1.0, min(1.0, steering))

        if front < 220:

            throttle = 0.30

        elif abs(steering) > 0.6:

            throttle = 0.45

        else:

            throttle = 0.70

        self.steering = steering
        self.throttle = throttle


    # ========================================================
    # Gap Follow
    # ========================================================

    def gap_follow(self):

        left = self.ranges.get("FrLH", 300)
        front = self.ranges.get("FrFR", 300)
        right = self.ranges.get("FrRH", 300)

        sensors = np.array([
            left,
            front,
            right,
        ])

        gap = np.argmax(sensors)

        if gap == 0:

            steering = -0.8

        elif gap == 1:

            steering = 0.0

        else:

            steering = 0.8

        if front < 180:

            throttle = 0.25

        elif front < 350:

            throttle = 0.45

        else:

            throttle = 0.70

        self.steering = steering
        self.throttle = throttle


    # ========================================================
    # Racer
    # ========================================================

    def racer(self):

        left = self.ranges.get("FrLH", 300)
        front = self.ranges.get("FrFR", 300)
        right = self.ranges.get("FrRH", 300)

        error = left - right

        steering = -(error * 0.0025)

        steering = max(-1.0, min(1.0, steering))

        speed = front / 500.0

        speed = max(0.25, min(1.0, speed))

        throttle = 0.25 + speed * 0.55

        if abs(steering) > 0.7:

            throttle *= 0.75

        if front < 180:

            throttle = 0.20

        self.steering = steering
        self.throttle = throttle


    # ========================================================
    # Follow The Gap
    # ========================================================

    def follow_the_gap(self):

        if self.lidar is None:

            self.steering = 0.0
            self.throttle = 0.0
            return

        ranges = np.array(self.lidar)

        index = np.argmax(ranges)

        center = len(ranges) // 2

        steering = (index - center) / center

        steering = max(-1.0, min(1.0, steering))

        throttle = 0.70

        self.steering = steering
        self.throttle = throttle
        
